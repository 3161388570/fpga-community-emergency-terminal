`timescale 1ns/1ps
// Main-image SDRAM read engine rebuilt from lecture-8 media_frame_reader
// (lecture 8 four-bank frame cache): reservation, inflight window, burst gap, and independent
// request/return counting. Adapted to Anlogic App_rd_en/Sdr_rd_en and to
// this project's read_req/ack plus rd_grant isolation.
module media_sdram_read_engine #(
    parameter MEM_DATA_BITS       = 32,
    parameter ADDR_BITS           = 21,
    parameter BURST_BITS          = 9,
    parameter FIFO_DEPTH          = 1024,
    parameter BURST_SIZE          = 128,
    parameter FIFO_MARGIN         = 8,
    parameter BURST_GAP_CYCLES    = 12,
    parameter MAX_INFLIGHT_BURSTS = 2,
    parameter READ_LOW_WATERMARK  = 704
) (
    input                       rst,
    input                       mem_clk,
    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       Sdr_rd_en,
    input                       App_wr_busy,
    output                      O_rd_busy,
    output                      read_priority,
    input                       rd_grant,
    output                      rd_data_we,
    output                      App_rd_en,
    output [ADDR_BITS-1:0]      App_rd_addr,
    input                       read_req,
    output reg                  read_req_ack,
    output                      read_finish,
    input  [ADDR_BITS-1:0]      read_addr_0,
    input  [ADDR_BITS-1:0]      read_addr_1,
    input  [ADDR_BITS-1:0]      read_addr_2,
    input  [ADDR_BITS-1:0]      read_addr_3,
    input  [1:0]                read_addr_index,
    input  [ADDR_BITS-1:0]      read_len,
    output reg                  fifo_aclr,
    input  [9:0]                wrusedw
);
    localparam [ADDR_BITS-1:0] ZERO = {ADDR_BITS{1'b0}};
    localparam integer INFLIGHT_LIMIT = MAX_INFLIGHT_BURSTS * BURST_SIZE;
    localparam integer INFLIGHT_W = 9;
    localparam integer BURST_W = BURST_BITS;

    reg        read_req_d0;
    reg        read_req_d1;
    reg        read_req_d2;
    reg [1:0]  read_addr_index_d0;
    reg [1:0]  read_addr_index_d1;
    reg [ADDR_BITS-1:0] read_len_d0;
    reg [ADDR_BITS-1:0] read_len_d1;
    reg [ADDR_BITS-1:0] read_len_latch;
    reg [ADDR_BITS-1:0] issued_count;
    reg [ADDR_BITS-1:0] returned_count;
    reg [ADDR_BITS-1:0] App_rd_addr_r;
    reg        App_rd_en_r;
    reg        reader_busy;
    reg        reload_pending;
    reg        burst_active;
    reg [BURST_W-1:0] burst_issued;
    reg [BURST_W-1:0] burst_target;
    reg [7:0]  burst_gap_count;
    reg [2:0]  fifo_reset_count;
    reg [INFLIGHT_W-1:0] inflight_count;
    reg        inflight_empty;
    reg        inflight_window_ok_q;
    reg        fifo_reservation_ok_q;
    reg        frame_done_pulse;

    wire unused_sdr_busy = Sdr_busy;
    wire unused_mem_bits = (MEM_DATA_BITS == 32);

    wire [ADDR_BITS-1:0] remaining_words = read_len_latch - issued_count;
    wire [ADDR_BITS-1:0] next_burst_words_d =
        (remaining_words > BURST_SIZE[ADDR_BITS-1:0]) ?
        BURST_SIZE[ADDR_BITS-1:0] : remaining_words;
    // Registered copy: issued_count only changes while a burst is being
    // presented and a new burst cannot start for BURST_GAP_CYCLES / the
    // post-reload fifo_reset_count afterwards, so one cycle of lag is
    // invisible to start_burst and the admission checks.
    reg [ADDR_BITS-1:0] next_burst_words;
    always @(posedge mem_clk)
        next_burst_words <= next_burst_words_d;

    wire [INFLIGHT_W:0] inflight_plus_burst =
        {1'b0, inflight_count} + next_burst_words[INFLIGHT_W:0];
    wire [11:0] reserved_words =
        {2'b00, wrusedw} + {{(12-INFLIGHT_W){1'b0}}, inflight_count};
    wire [11:0] reserved_after_burst =
        reserved_words + {1'b0, next_burst_words[10:0]};

    wire inflight_window_ok = (MAX_INFLIGHT_BURSTS == 1) ? inflight_empty :
        (inflight_plus_burst <= INFLIGHT_LIMIT[INFLIGHT_W:0]);
    wire fifo_reservation_ok =
        reserved_after_burst <= (FIFO_DEPTH - FIFO_MARGIN);

    wire grant_run = rd_grant && Sdr_init_done;
    wire start_burst = grant_run && reader_busy &&
                       (fifo_reset_count == 3'd0) &&
                       !reload_pending &&
                       !burst_active &&
                       (burst_gap_count == 8'd0) &&
                       (issued_count != read_len_latch) &&
                       !App_wr_busy &&
                       !Sdr_init_ref_vld &&
                       inflight_window_ok_q &&
                       fifo_reservation_ok_q;

    wire last_present = App_rd_en_r && rd_grant && !Sdr_init_ref_vld &&
                      (burst_issued + 1'b1 == burst_target);
    wire issue_fire = grant_run && reader_busy && burst_active &&
                      (fifo_reset_count == 3'd0) &&
                      !reload_pending &&
                      !Sdr_init_ref_vld &&
                      (issued_count != read_len_latch) &&
                      (burst_issued != burst_target) &&
                      !last_present;

    // Present-cycle request: registered intent gated by this-cycle refresh
    // so a request scheduled just before refresh is retried, not counted.
    wire app_rd_present = App_rd_en_r && rd_grant && !Sdr_init_ref_vld;
    wire rd_return = Sdr_rd_en && rd_grant && !fifo_aclr &&
                     (!inflight_empty || app_rd_present);

    assign App_rd_en   = app_rd_present;
    assign App_rd_addr = App_rd_addr_r;
    assign rd_data_we  = rd_return;
    assign read_finish = frame_done_pulse;
    assign O_rd_busy = reader_busy &&
                       (burst_active || !inflight_empty || (burst_gap_count != 8'd0) || app_rd_present);
    assign read_priority = reader_busy && (wrusedw <= READ_LOW_WATERMARK[9:0]);

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            read_req_d0 <= 1'b0;
            read_req_d1 <= 1'b0;
            read_req_d2 <= 1'b0;
            read_len_d0 <= ZERO;
            read_len_d1 <= ZERO;
            read_addr_index_d0 <= 2'b00;
            read_addr_index_d1 <= 2'b00;
        end else begin
            read_req_d0 <= read_req;
            read_req_d1 <= read_req_d0;
            read_req_d2 <= read_req_d1;
            read_len_d0 <= read_len;
            read_len_d1 <= read_len_d0;
            read_addr_index_d0 <= read_addr_index;
            read_addr_index_d1 <= read_addr_index_d0;
        end
    end

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            read_req_ack <= 1'b0;
            fifo_aclr <= 1'b1;
            reader_busy <= 1'b0;
            reload_pending <= 1'b0;
            burst_active <= 1'b0;
            burst_issued <= {BURST_W{1'b0}};
            burst_target <= {BURST_W{1'b0}};
            burst_gap_count <= 8'd0;
            fifo_reset_count <= 3'd0;
            issued_count <= ZERO;
            returned_count <= ZERO;
            inflight_count <= {INFLIGHT_W{1'b0}};
            inflight_empty <= 1'b1;
            inflight_window_ok_q <= 1'b1;
            fifo_reservation_ok_q <= 1'b1;
            App_rd_en_r <= 1'b0;
            App_rd_addr_r <= ZERO;
            read_len_latch <= ZERO;
            frame_done_pulse <= 1'b0;
        end else if (!rd_grant) begin
            read_req_ack <= 1'b0;
            fifo_aclr <= 1'b1;
            reader_busy <= 1'b0;
            reload_pending <= 1'b0;
            burst_active <= 1'b0;
            burst_issued <= {BURST_W{1'b0}};
            burst_target <= {BURST_W{1'b0}};
            burst_gap_count <= 8'd0;
            fifo_reset_count <= 3'd0;
            issued_count <= ZERO;
            returned_count <= ZERO;
            inflight_count <= {INFLIGHT_W{1'b0}};
            inflight_empty <= 1'b1;
            App_rd_en_r <= 1'b0;
            frame_done_pulse <= 1'b0;
        end else begin
            inflight_window_ok_q <= inflight_window_ok;
            fifo_reservation_ok_q <= fifo_reservation_ok;
            frame_done_pulse <= 1'b0;
            App_rd_en_r <= issue_fire;

            if (fifo_reset_count != 3'd0)
                fifo_reset_count <= fifo_reset_count - 1'b1;
            if (burst_gap_count != 8'd0)
                burst_gap_count <= burst_gap_count - 1'b1;

            case ({app_rd_present, Sdr_rd_en})
                2'b10: begin
                    inflight_count <= inflight_count + 1'b1;
                    inflight_empty <= 1'b0;
                end
                2'b01: begin
                    if (!inflight_empty) begin
                        inflight_count <= inflight_count - 1'b1;
                        if (inflight_count == {{(INFLIGHT_W-1){1'b0}}, 1'b1})
                            inflight_empty <= 1'b1;
                    end
                end
                default: begin
                    inflight_count <= inflight_count;
                    inflight_empty <= inflight_empty;
                end
            endcase

            if (read_req_d2 && !reload_pending && grant_run) begin
                read_req_ack <= 1'b1;
                reload_pending <= 1'b1;
                reader_busy <= 1'b0;
                burst_active <= 1'b0;
                App_rd_en_r <= 1'b0;
                fifo_aclr <= 1'b1;
            end else begin
                if (read_req_ack && !read_req_d2)
                    read_req_ack <= 1'b0;

                if (reload_pending && inflight_empty && !read_req_d2 &&
                    !app_rd_present) begin
                    reload_pending <= 1'b0;
                    reader_busy <= 1'b1;
                    issued_count <= ZERO;
                    returned_count <= ZERO;
                    inflight_count <= {INFLIGHT_W{1'b0}};
                    inflight_empty <= 1'b1;
                    burst_active <= 1'b0;
                    burst_issued <= {BURST_W{1'b0}};
                    burst_target <= {BURST_W{1'b0}};
                    burst_gap_count <= 8'd0;
                    fifo_reset_count <= 3'd4;
                    fifo_aclr <= 1'b1;
                    read_len_latch <= read_len_d1;
                    if (read_addr_index_d1 == 2'd0)
                        App_rd_addr_r <= read_addr_0;
                    else if (read_addr_index_d1 == 2'd1)
                        App_rd_addr_r <= read_addr_1;
                    else if (read_addr_index_d1 == 2'd2)
                        App_rd_addr_r <= read_addr_2;
                    else
                        App_rd_addr_r <= read_addr_3;
                end else if (fifo_reset_count == 3'd1) begin
                    fifo_aclr <= 1'b0;
                end else if (!reload_pending && (fifo_reset_count == 3'd0)) begin
                    fifo_aclr <= 1'b0;
                end
            end

            if (start_burst) begin
                burst_active <= 1'b1;
                burst_issued <= {BURST_W{1'b0}};
                burst_target <= next_burst_words[BURST_W-1:0];
            end

            if (app_rd_present) begin
                issued_count <= issued_count + 1'b1;
                burst_issued <= burst_issued + 1'b1;
                App_rd_addr_r <= App_rd_addr_r + 1'b1;
                if (burst_issued + 1'b1 == burst_target) begin
                    burst_active <= 1'b0;
                    burst_gap_count <= BURST_GAP_CYCLES[7:0];
                end
            end

            if (rd_return) begin
                returned_count <= returned_count + 1'b1;
                if (returned_count == (read_len_latch - 1'b1)) begin
                    reader_busy <= 1'b0;
                    burst_active <= 1'b0;
                    frame_done_pulse <= 1'b1;
                end
            end
        end
    end
endmodule

