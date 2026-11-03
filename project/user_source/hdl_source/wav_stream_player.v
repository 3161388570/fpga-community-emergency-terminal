`timescale 1ns/1ps

// Stream a prepared AUDIO.PCM file from a FAT32 volume to the existing
// 48 kHz HDMI audio path.  The PC-side conversion produces signed,
// little-endian, 16-bit, mono PCM, so the FPGA does not need a WAV parser.
//
// The write side requests one SD sector at a time and follows the FAT32
// cluster chain.  A dual-clock circular buffer separates the bursty SD
// interface from the constant-rate audio consumer.  When the buffer is
// temporarily empty, a zero sample is emitted instead of stopping the HDMI
// audio packet stream.
module wav_stream_player #(
    parameter integer ADDR_WIDTH = 12,
    parameter integer SAMPLES_PER_SECTOR = 256,
    parameter integer HEADER_BYTES = 44      // canonical WAV header, skipped
) (
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        wr_abort,
    input  wire        play_enable,
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size_bytes,
    input  wire [31:0] fat_start_sector,
    input  wire [31:0] data_start_sector,
    input  wire [7:0]  sectors_per_cluster,

    output reg         sd_req,
    output reg  [31:0] sd_lba,
    input  wire        sd_grant,
    input  wire        sd_busy,
    input  wire        sd_valid,
    input  wire [7:0]  sd_byte,
    input  wire [8:0]  sd_byte_index,
    input  wire        sd_done,

    input  wire        rd_clk,
    input  wire        rd_rst_n,
    input  wire        sample_tick,
    output reg signed [15:0] sample_out,
    output reg        sample_valid,
    input  wire        sample_ready,

    output reg [31:0] sample_counter,
    output wire        stream_active,
    output wire        stream_end,
    output wire        buffer_low
);

localparam integer BUFFER_DEPTH = (1 << ADDR_WIDTH);
localparam [3:0] WS_IDLE     = 4'd0;
localparam [3:0] WS_DATA_REQ = 4'd1;
localparam [3:0] WS_DATA_FILL = 4'd2;
localparam [3:0] WS_FAT_REQ  = 4'd3;
localparam [3:0] WS_FAT_FILL = 4'd4;
localparam [3:0] WS_END      = 4'd5;
localparam [3:0] WS_ERROR    = 4'd6;
localparam [3:0] WS_DATA_PREP = 4'd7;

reg [3:0] wstate;
reg [31:0] current_cluster;
reg [7:0] sector_in_cluster;
reg [31:0] sectors_read;
reg [5:0] hdr_words;
reg [7:0] low_byte;
reg       low_byte_pending;
reg [31:0] fat_entry;
reg [31:0] fat_byte_offset;
reg [31:0] data_sector_lba;
reg [31:0] data_cluster_offset;
reg [2:0] wstate_sync1;
reg [2:0] wstate_sync2;
reg abort_toggle;
reg abort_sync1;
reg abort_sync2;
reg abort_seen;
reg playback_started;
(* async_reg = "true" *) reg play_enable_sync1;
(* async_reg = "true" *) reg play_enable_sync2;

wire [ADDR_WIDTH-1:0] fifo_wrusedw;
wire [ADDR_WIDTH-1:0] fifo_rdusedw;
wire [15:0] fifo_dout;
wire        fifo_valid;
wire        fifo_full;
wire        fifo_empty;
wire        fifo_write;
wire        fifo_read;
wire        fifo_reset;
wire [31:0] total_sectors;
wire [31:0] total_samples;
wire [31:0] fat_next_cluster;
wire        buffer_has_sector_space;
wire        stream_done_in_wr;
wire        stream_done_in_rd;
wire        play_enable_rd;

function [31:0] cluster_sector_offset;
    input [31:0] cluster;
    input [7:0]  cluster_sectors;
    begin
        case (cluster_sectors)
            8'd1:   cluster_sector_offset = cluster - 32'd2;
            8'd2:   cluster_sector_offset = (cluster - 32'd2) << 1;
            8'd4:   cluster_sector_offset = (cluster - 32'd2) << 2;
            8'd8:   cluster_sector_offset = (cluster - 32'd2) << 3;
            8'd16:  cluster_sector_offset = (cluster - 32'd2) << 4;
            8'd32:  cluster_sector_offset = (cluster - 32'd2) << 5;
            8'd64:  cluster_sector_offset = (cluster - 32'd2) << 6;
            8'd128: cluster_sector_offset = (cluster - 32'd2) << 7;
            default: cluster_sector_offset = 32'd0;
        endcase
    end
endfunction

assign total_sectors = (file_size_bytes + 32'd511) >> 9;
assign total_samples = (file_size_bytes - HEADER_BYTES) >> 1;
wire skip_word = (sectors_read == 32'd0) && (hdr_words < (HEADER_BYTES/2));
assign fat_next_cluster = fat_entry & 32'h0fff_ffff;
assign buffer_has_sector_space = fifo_wrusedw <= (BUFFER_DEPTH - SAMPLES_PER_SECTOR);
assign stream_done_in_wr = (wstate == WS_END) || (wstate == WS_ERROR);
assign stream_done_in_rd = (wstate_sync2 == WS_END) || (wstate_sync2 == WS_ERROR);
assign play_enable_rd = play_enable_sync2;
assign fifo_reset = !wr_rst_n || !rd_rst_n || wr_abort || !play_enable;
assign fifo_write = (wstate == WS_DATA_FILL) && sd_valid && low_byte_pending && !fifo_full && !skip_word;
assign fifo_read = sample_tick && sample_ready && playback_started &&
                   !fifo_empty && (sample_counter < total_samples);

assign buffer_low = (fifo_rdusedw < (BUFFER_DEPTH >> 2));
assign stream_active = play_enable_rd && playback_started;
assign stream_end = play_enable_rd && stream_done_in_rd &&
                   ((sample_counter >= total_samples) || fifo_empty);

rfifo_32_32_512 #(
    .DATA_WIDTH_W(16),
    .DATA_WIDTH_R(16),
    .ADDR_WIDTH_W(ADDR_WIDTH),
    .ADDR_WIDTH_R(ADDR_WIDTH),
    .AL_FULL_NUM(BUFFER_DEPTH - 4),
    .AL_EMPTY_NUM(2),
    .SHOW_AHEAD_EN(1'b0),
    .OUTREG_EN("NOREG")
) pcm_fifo (
    .rst(fifo_reset),
    .clkw(wr_clk),
    .clkr(rd_clk),
    .we(fifo_write),
    .di({sd_byte, low_byte}),
    .re(fifo_read),
    .dout(fifo_dout),
    .valid(fifo_valid),
    .full_flag(fifo_full),
    .empty_flag(fifo_empty),
    .afull(),
    .aempty(),
    .wrusedw(fifo_wrusedw),
    .rdusedw(fifo_rdusedw)
);

always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        wstate           <= WS_IDLE;
        current_cluster  <= 32'd0;
        sector_in_cluster <= 8'd0;
        sectors_read     <= 32'd0;
        hdr_words <= 6'd0;
        low_byte         <= 8'd0;
        low_byte_pending <= 1'b0;
        fat_entry        <= 32'd0;
        fat_byte_offset  <= 32'd0;
        data_sector_lba  <= 32'd0;
        data_cluster_offset <= 32'd0;
        sd_req            <= 1'b0;
        sd_lba            <= 32'd0;
        abort_toggle     <= 1'b0;
    end else begin
        if (wr_abort) begin
            abort_toggle      <= ~abort_toggle;
            wstate            <= WS_IDLE;
            current_cluster   <= 32'd0;
            sector_in_cluster <= 8'd0;
            sectors_read      <= 32'd0;
            hdr_words <= 6'd0;
            low_byte_pending  <= 1'b0;
            fat_entry         <= 32'd0;
            data_sector_lba   <= 32'd0;
            data_cluster_offset <= 32'd0;
            sd_req            <= 1'b0;
        end else if (!play_enable) begin
            wstate            <= WS_IDLE;
            current_cluster   <= 32'd0;
            sector_in_cluster <= 8'd0;
            sectors_read      <= 32'd0;
            hdr_words <= 6'd0;
            low_byte_pending  <= 1'b0;
            fat_entry         <= 32'd0;
            data_sector_lba   <= 32'd0;
            data_cluster_offset <= 32'd0;
            sd_req            <= 1'b0;
        end else begin
            case (wstate)
                WS_IDLE: begin
                    if ((first_cluster >= 32'd2) &&
                        (file_size_bytes != 0) &&
                        (sectors_per_cluster != 0) &&
                        (total_sectors != 0)) begin
                        current_cluster   <= first_cluster;
                        sector_in_cluster <= 8'd0;
                        sectors_read      <= 32'd0;
                        hdr_words <= 6'd0;
                        low_byte_pending  <= 1'b0;
                        data_cluster_offset <= cluster_sector_offset(first_cluster, sectors_per_cluster);
                        sd_req            <= 1'b0;
                        wstate            <= WS_DATA_PREP;
                    end
                end

                WS_DATA_PREP: begin
                    data_sector_lba <= data_start_sector + data_cluster_offset;
                    wstate <= WS_DATA_REQ;
                end

                WS_DATA_REQ: begin
                    if (!sd_req && !sd_busy && buffer_has_sector_space) begin
                        sd_lba <= data_sector_lba;
                        sd_req <= 1'b1;
                    end
                    if (sd_req && sd_grant) begin
                        sd_req <= 1'b0;
                        wstate <= WS_DATA_FILL;
                    end
                end

                WS_DATA_FILL: begin
                    if (sd_valid) begin
                        if (!low_byte_pending) begin
                            low_byte         <= sd_byte;
                            low_byte_pending <= 1'b1;
                        end else begin
                            low_byte_pending      <= 1'b0;
                            if (skip_word) hdr_words <= hdr_words + 1'b1;
                        end
                    end

                    if (sd_done) begin
                        if (sectors_read + 1 >= total_sectors) begin
                            wstate <= WS_END;
                        end else if (sector_in_cluster + 1 < sectors_per_cluster) begin
                            sector_in_cluster <= sector_in_cluster + 1'b1;
                            sectors_read      <= sectors_read + 1'b1;
                            data_sector_lba   <= data_sector_lba + 1'b1;
                            wstate             <= WS_DATA_REQ;
                        end else begin
                            sectors_read     <= sectors_read + 1'b1;
                            fat_byte_offset  <= (current_cluster[6:0] << 2);
                            fat_entry        <= 32'd0;
                            sd_req            <= 1'b0;
                            wstate            <= WS_FAT_REQ;
                        end
                    end
                end

                WS_FAT_REQ: begin
                    if (!sd_req && !sd_busy) begin
                        sd_lba <= fat_start_sector + (current_cluster >> 7);
                        sd_req <= 1'b1;
                    end
                    if (sd_req && sd_grant) begin
                        sd_req <= 1'b0;
                        wstate <= WS_FAT_FILL;
                    end
                end

                WS_FAT_FILL: begin
                    if (sd_valid) begin
                        if (sd_byte_index == fat_byte_offset[8:0])
                            fat_entry[7:0] <= sd_byte;
                        else if (sd_byte_index == fat_byte_offset[8:0] + 1'b1)
                            fat_entry[15:8] <= sd_byte;
                        else if (sd_byte_index == fat_byte_offset[8:0] + 2'd2)
                            fat_entry[23:16] <= sd_byte;
                        else if (sd_byte_index == fat_byte_offset[8:0] + 2'd3)
                            fat_entry[31:24] <= sd_byte;
                    end

                    if (sd_done) begin
                        if ((fat_next_cluster >= 32'd2) &&
                            (fat_next_cluster < 32'h0fff_fff0)) begin
                            current_cluster   <= fat_next_cluster;
                            sector_in_cluster <= 8'd0;
                            data_cluster_offset <= cluster_sector_offset(fat_next_cluster, sectors_per_cluster);
                            wstate            <= WS_DATA_PREP;
                        end else begin
                            wstate <= WS_ERROR;
                        end
                    end
                end

                WS_END: begin
                    if (!play_enable)
                        wstate <= WS_IDLE;
                end

                WS_ERROR: begin
                    if (!play_enable)
                        wstate <= WS_IDLE;
                end

                default: wstate <= WS_IDLE;
            endcase
        end
    end
end

always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        wstate_sync1   <= 3'd0;
        wstate_sync2   <= 3'd0;
        abort_sync1    <= 1'b0;
        abort_sync2    <= 1'b0;
        abort_seen     <= 1'b0;
        play_enable_sync1 <= 1'b0;
        play_enable_sync2 <= 1'b0;
        sample_out     <= 16'sd0;
        sample_valid   <= 1'b0;
        sample_counter <= 32'd0;
        playback_started <= 1'b0;
    end else begin
        wstate_sync1  <= wstate[2:0];
        wstate_sync2  <= wstate_sync1;
        abort_sync1   <= abort_toggle;
        abort_sync2   <= abort_sync1;
        play_enable_sync1 <= play_enable;
        play_enable_sync2 <= play_enable_sync1;
        sample_valid  <= 1'b0;

        if (!play_enable_rd || (abort_sync2 != abort_seen)) begin
            abort_seen     <= abort_sync2;
            sample_out     <= 16'sd0;
            sample_counter <= 32'd0;
            playback_started <= 1'b0;
        end else begin
            if (!playback_started &&
                ((fifo_rdusedw >= SAMPLES_PER_SECTOR) ||
                 (stream_done_in_rd && !fifo_empty)))
                playback_started <= 1'b1;
            else if (playback_started && stream_done_in_rd &&
                     (fifo_empty || (sample_counter >= total_samples)) && !fifo_valid)
                playback_started <= 1'b0;

            if (fifo_valid) begin
                sample_out     <= fifo_dout;
                sample_counter <= sample_counter + 1'b1;
                sample_valid   <= 1'b1;
            end else begin
                sample_out <= 16'sd0;
            end
        end
    end
end

endmodule
