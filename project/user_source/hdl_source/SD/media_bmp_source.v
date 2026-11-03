module media_bmp_source #(
    parameter [31:0] SCAN_START_SECTOR = 0,
    parameter [31:0] SCAN_MAX_SECTOR = 131071,
    parameter [2:0] SCAN_TARGET_COUNT = 4,
    parameter integer USE_FAT32 = 1
)(
    input clk, input rst, input transport_reset,
    input [15:0] bmp_width, input [15:0] bmp_height,
    input audio_sd_req, input [31:0] audio_sd_addr,
    output audio_sd_grant, output audio_sd_busy, output [7:0] audio_sd_data,
    output audio_sd_data_valid, output [8:0] audio_sd_byte_index,
    output audio_sd_end,
    input [2:0] audio_select, output [4:0] audio_found_mask, output [159:0] audio_sizes,
    output audio_file_found, output [31:0] audio_first_cluster,
    output [31:0] audio_file_size, output [31:0] audio_fat_start_sector,
    output [31:0] audio_data_start_sector, output [7:0] audio_sectors_per_cluster,
    input scan_start_pulse, input load_start_pulse,
    input load_abort, input [31:0] load_sector,
    input write_acknowledged,
    output sd_init_done, output bmp_ready, output decode_error,
    output [2:0] decode_error_code, output [3:0] state_code_i,
    output scan_done, output scan_found_valid, output [31:0] scan_found_sector,
    output sd_sec_read, output sd_sec_read_data_valid, output sd_sec_read_end,
    output write_req, output write_en, output [31:0] write_data,
    output SD_nCS, output SD_DCLK, output SD_MOSI, input SD_MISO
);
wire [31:0] sd_sec_read_addr;
wire [7:0] sd_sec_read_data;
wire bmp_sd_sec_read;
wire [31:0] bmp_sd_sec_read_addr;
wire [7:0] sd_card_read_data;
wire sd_card_read_data_valid;
wire sd_card_read_end;
reg sd_owner_valid;
reg sd_owner_audio;
reg [8:0] sd_byte_index;
wire sd_mux_read = sd_owner_valid;
wire [31:0] sd_mux_read_addr = sd_owner_audio ? audio_sd_addr : bmp_sd_sec_read_addr;
wire bmp_read_data_valid = sd_card_read_data_valid && sd_owner_valid && !sd_owner_audio;
wire audio_read_data_valid = sd_card_read_data_valid && sd_owner_valid && sd_owner_audio;
wire bmp_read_end = sd_card_read_end && sd_owner_valid && !sd_owner_audio;
wire audio_read_end = sd_card_read_end && sd_owner_valid && sd_owner_audio;
wire [2:0] scan_found_total;
wire bmp_data_wr_en;
wire [23:0] bmp_data;
assign write_en = bmp_data_wr_en;
assign write_data = {bmp_data, 8'b0};
media_compatible_bmp_reader #(.USE_FAT32(USE_FAT32)) bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),
    .decode_error           (decode_error),
    .error_code             (decode_error_code),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),
    .audio_select           (audio_select),
    .audio_found_mask       (audio_found_mask),
    .audio_sizes            (audio_sizes),
    .audio_file_found       (audio_file_found),
    .audio_first_cluster    (audio_first_cluster),
    .audio_file_size        (audio_file_size),
    .audio_fat_start_sector (audio_fat_start_sector),
    .audio_data_start_sector(audio_data_start_sector),
    .audio_sectors_per_cluster(audio_sectors_per_cluster),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code_i),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_acknowledged),
    .sd_sec_read            (bmp_sd_sec_read),
    .sd_sec_read_addr       (bmp_sd_sec_read_addr),
    .sd_sec_read_data       (sd_card_read_data),
    .sd_sec_read_data_valid (bmp_read_data_valid),
    .sd_sec_read_end        (bmp_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

assign sd_sec_read = bmp_sd_sec_read;
assign sd_sec_read_addr = bmp_sd_sec_read_addr;
assign sd_sec_read_data = sd_card_read_data;
assign sd_sec_read_data_valid = bmp_read_data_valid;
assign sd_sec_read_end = bmp_read_end;
assign audio_sd_grant = !sd_owner_valid && !bmp_sd_sec_read && audio_sd_req;
assign audio_sd_busy = sd_owner_valid || bmp_sd_sec_read || sd_card_read_end;
assign audio_sd_data = sd_card_read_data;
assign audio_sd_data_valid = audio_read_data_valid;
assign audio_sd_byte_index = sd_byte_index;
assign audio_sd_end = audio_read_end;

always @(posedge clk or posedge transport_reset) begin
    if (transport_reset) begin
        sd_owner_valid <= 1'b0;
        sd_owner_audio <= 1'b0;
        sd_byte_index <= 9'd0;
    end else begin
        if (!sd_owner_valid) begin
            if (bmp_sd_sec_read) begin
                sd_owner_valid <= 1'b1;
                sd_owner_audio <= 1'b0;
                sd_byte_index <= 9'd0;
            end else if (audio_sd_grant) begin
                sd_owner_valid <= 1'b1;
                sd_owner_audio <= 1'b1;
                sd_byte_index <= 9'd0;
            end
        end else begin
            if (sd_card_read_data_valid)
                sd_byte_index <= sd_byte_index + 1'b1;
            if (sd_card_read_end)
                sd_owner_valid <= 1'b0;
        end
    end
end

sd_card_top #(.SPI_HIGH_SPEED_DIV(2)) sd_card_top_m0(
    .clk                    (clk),
    .rst                    (transport_reset),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_mux_read),
    .sd_sec_read_addr       (sd_mux_read_addr),
    .sd_sec_read_data       (sd_card_read_data),
    .sd_sec_read_data_valid (sd_card_read_data_valid),
    .sd_sec_read_end        (sd_card_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'h00),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module media_compatible_bmp_reader #(
    parameter integer USE_FAT32 = 1
)(
    input                       clk,
    input                       rst,
    output                      ready,
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output                      scan_done,
    output                      scan_found_valid,
    output [31:0]               scan_found_sector,
    output [2:0]                scan_found_total,
    input  [2:0]                audio_select,
    output [4:0]                audio_found_mask,
    output [159:0]              audio_sizes,
    output                      audio_file_found,
    output [31:0]               audio_first_cluster,
    output [31:0]               audio_file_size,
    output [31:0]               audio_fat_start_sector,
    output [31:0]               audio_data_start_sector,
    output [7:0]                audio_sectors_per_cluster,
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,
    input                       sd_init_done,
    output [3:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output                      write_req,
    input                       write_req_ack,
    output                      sd_sec_read,
    output [31:0]               sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,
    output                      bmp_data_wr_en,
    output [23:0]               bmp_data,
    output                      decode_error,
    output [2:0]                error_code
);

reg fallback_active;
reg fallback_scan_start;
reg fat_scan_in_progress;

wire fat_ready;
wire fat_scan_done;
wire fat_scan_found_valid;
wire [31:0] fat_scan_found_sector;
wire [2:0] fat_scan_found_total;
wire [3:0] fat_state_code;
wire fat_write_req;
wire fat_sd_sec_read;
wire [31:0] fat_sd_sec_read_addr;
wire fat_bmp_data_wr_en;
wire [23:0] fat_bmp_data;
wire fat_filesystem_valid;
wire fat_decode_error;
wire [2:0] fat_error_code;
wire [4:0] fat_audio_found_mask;
wire [159:0] fat_audio_sizes;
wire fat_audio_file_found;
wire [31:0] fat_audio_first_cluster;
wire [31:0] fat_audio_file_size;
wire [31:0] fat_audio_fat_start_sector;
wire [31:0] fat_audio_data_start_sector;
wire [7:0] fat_audio_sectors_per_cluster;

wire raw_ready;
wire raw_scan_done;
wire raw_scan_found_valid;
wire [31:0] raw_scan_found_sector;
wire [2:0] raw_scan_found_total;
wire [3:0] raw_state_code;
wire raw_write_req;
wire raw_sd_sec_read;
wire [31:0] raw_sd_sec_read_addr;
wire raw_bmp_data_wr_en;
wire [23:0] raw_bmp_data;
wire raw_decode_error;
wire [2:0] raw_error_code;

assign ready             = fallback_active ? raw_ready             : fat_ready;
assign scan_done         = fallback_active ? raw_scan_done         : (fat_scan_done && !fat_scan_in_progress);
assign scan_found_valid  = fallback_active ? raw_scan_found_valid  : fat_scan_found_valid;
assign scan_found_sector = fallback_active ? raw_scan_found_sector : fat_scan_found_sector;
assign scan_found_total  = fallback_active ? raw_scan_found_total  : fat_scan_found_total;
assign audio_found_mask  = fallback_active ? 5'd0 : fat_audio_found_mask;
assign audio_sizes       = fallback_active ? 160'd0 : fat_audio_sizes;
assign audio_file_found  = fallback_active ? 1'b0 : fat_audio_file_found;
assign audio_first_cluster = fallback_active ? 32'd0 : fat_audio_first_cluster;
assign audio_file_size = fallback_active ? 32'd0 : fat_audio_file_size;
assign audio_fat_start_sector = fallback_active ? 32'd0 : fat_audio_fat_start_sector;
assign audio_data_start_sector = fallback_active ? 32'd0 : fat_audio_data_start_sector;
assign audio_sectors_per_cluster = fallback_active ? 8'd0 : fat_audio_sectors_per_cluster;
assign state_code        = fallback_active ? (USE_FAT32 ? {1'b1, raw_state_code[2:0]} : raw_state_code) : fat_state_code;
assign write_req         = fallback_active ? raw_write_req         : fat_write_req;
assign sd_sec_read       = fallback_active ? raw_sd_sec_read       : fat_sd_sec_read;
assign sd_sec_read_addr  = fallback_active ? raw_sd_sec_read_addr  : fat_sd_sec_read_addr;
assign bmp_data_wr_en    = fallback_active ? raw_bmp_data_wr_en    : fat_bmp_data_wr_en;
assign bmp_data          = fallback_active ? raw_bmp_data          : fat_bmp_data;
assign decode_error      = fallback_active ? raw_decode_error : fat_decode_error;
assign error_code        = fallback_active ? raw_error_code : fat_error_code;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fallback_active     <= !USE_FAT32;
        fallback_scan_start <= 1'b0;
        fat_scan_in_progress <= 1'b0;
    end else if (!sd_init_done) begin
        fallback_active     <= !USE_FAT32;
        fallback_scan_start <= 1'b0;
        fat_scan_in_progress <= 1'b0;
    end else begin
        fallback_scan_start <= 1'b0;

        if (scan_start) begin
            fallback_active      <= !USE_FAT32;
            fat_scan_in_progress <= USE_FAT32 != 0;
        end else if (fat_scan_in_progress && fat_scan_done) begin
            fat_scan_in_progress <= 1'b0;
            if (!fat_filesystem_valid) begin
                fallback_active     <= 1'b1;
                fallback_scan_start <= 1'b1;
            end
        end
    end
end

fat32_bmp_reader u_fat32_reader(
    .clk(clk), .rst(rst), .ready(fat_ready),
    .scan_start(scan_start && USE_FAT32 != 0), .scan_start_sector(scan_start_sector),
    .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
    .scan_done(fat_scan_done), .scan_found_valid(fat_scan_found_valid),
    .scan_found_sector(fat_scan_found_sector), .scan_found_total(fat_scan_found_total),
    .audio_select(audio_select), .audio_found_mask(fat_audio_found_mask), .audio_sizes(fat_audio_sizes),
    .audio_file_found(fat_audio_file_found), .audio_first_cluster(fat_audio_first_cluster),
    .audio_file_size(fat_audio_file_size), .audio_fat_start_sector(fat_audio_fat_start_sector),
    .audio_data_start_sector(fat_audio_data_start_sector),
    .audio_sectors_per_cluster(fat_audio_sectors_per_cluster),
    .load_start(load_start && !fallback_active), .load_abort(load_abort),
    .load_sector(load_sector), .sd_init_done(sd_init_done), .state_code(fat_state_code),
    .bmp_width(bmp_width), .bmp_height(bmp_height),
    .write_req(fat_write_req), .write_req_ack(write_req_ack && !fallback_active),
    .sd_sec_read(fat_sd_sec_read), .sd_sec_read_addr(fat_sd_sec_read_addr),
    .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
    .sd_sec_read_end(sd_sec_read_end), .bmp_data_wr_en(fat_bmp_data_wr_en),
    .bmp_data(fat_bmp_data), .filesystem_valid(fat_filesystem_valid),
    .decode_error(fat_decode_error), .error_code(fat_error_code)
);

bmp_read u_raw_reader(
    .clk(clk), .rst(rst), .ready(raw_ready),
    .scan_start(fallback_scan_start || (scan_start && USE_FAT32 == 0)), .scan_start_sector(scan_start_sector),
    .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
    .scan_done(raw_scan_done), .scan_found_valid(raw_scan_found_valid),
    .scan_found_sector(raw_scan_found_sector), .scan_found_total(raw_scan_found_total),
    .load_start(load_start && fallback_active), .load_abort(load_abort),
    .load_sector(load_sector), .sd_init_done(sd_init_done), .state_code(raw_state_code),
    .bmp_width(bmp_width), .bmp_height(bmp_height),
    .write_req(raw_write_req), .write_req_ack(write_req_ack && fallback_active),
    .sd_sec_read(raw_sd_sec_read), .sd_sec_read_addr(raw_sd_sec_read_addr),
    .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
    .sd_sec_read_end(sd_sec_read_end), .bmp_data_wr_en(raw_bmp_data_wr_en),
    .bmp_data(raw_bmp_data), .decode_error(raw_decode_error), .error_code(raw_error_code)
);

endmodule


