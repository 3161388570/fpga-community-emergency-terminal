`timescale 1ns/1ps

module media_frame_loader #(
    parameter MEM_DATA_BITS = 32,
    parameter READ_DATA_BITS = 32,
    parameter WRITE_DATA_BITS = 32,
    parameter ADDR_BITS = 21,
    parameter BURST_BITS = 9,
    parameter BURST_SIZE = 256,
    parameter WRITE_V_FLIP = 1,
    parameter FRAME_WIDTH = 640,
    parameter FRAME_HEIGHT = 480,
    parameter THUMBNAIL_ENABLE = 0,
    parameter integer THUMBNAIL_SAMPLE_X_STEP = 8,
    parameter integer THUMBNAIL_SAMPLE_Y_STEP = 4,
    parameter integer THUMBNAIL_OUTPUT_X_SCALE = 4,
    parameter integer THUMBNAIL_OUTPUT_Y_SCALE = 2,
    parameter [ADDR_BITS-1:0] THUMBNAIL_BASE_ADDRESS = 21'd1228800
) (
    input rst,
    input mem_clk,
    input Sdr_init_done,
    input Sdr_init_ref_vld,
    input Sdr_busy,
    output App_wr_en,
    output [ADDR_BITS-1:0] App_wr_addr,
    output [MEM_DATA_BITS-1:0] App_wr_din,
    output [3:0] App_wr_dm,
    input write_clk,
    input write_req,
    input write_frame_start,
    output write_req_ack,
    output write_finish,
    input [ADDR_BITS-1:0] write_addr_0,
    input [ADDR_BITS-1:0] write_addr_1,
    input [ADDR_BITS-1:0] write_addr_2,
    input [ADDR_BITS-1:0] write_addr_3,
    input [1:0] write_addr_index,
    input [ADDR_BITS-1:0] write_len,
    input write_en,
    input [WRITE_DATA_BITS-1:0] write_data,
    input reader_busy,
    output loader_busy,
    output thumbnail_ready,
    output thumbnail_overflow,
    output [3:0] thumbnail_mask
);

wire [8:0] rdusedw;
wire write_fifo_aclr;
wire main_writer_busy;
wire main_app_wr_en;
wire [ADDR_BITS-1:0] main_app_wr_addr;
wire [MEM_DATA_BITS-1:0] main_app_wr_din;
wire thumbnail_writer_busy;
wire thumbnail_writer_pending;
wire thumbnail_mem_write_en;
wire [ADDR_BITS-1:0] thumbnail_mem_write_addr;
wire [31:0] thumbnail_mem_write_data;
wire thumbnail_mem_write_grant;
wire App_rd_busy = reader_busy || thumbnail_writer_busy;

assign App_wr_dm = 4'b0000;

wfifo_32_32_512 #(
    .ADDR_WIDTH_W(9),
    .ADDR_WIDTH_R(9),
    .AL_FULL_NUM(509)
) write_buf (
    .clkr(mem_clk),
    .clkw(write_clk),
    .rst(write_fifo_aclr),
    .we(write_en),
    .re(main_app_wr_en),
    .di(write_data),
    .empty_flag(),
    .full_flag(),
    .wrusedw(),
    .rdusedw(rdusedw),
    .dout(main_app_wr_din)
);

frame_fifo_write #(
    .MEM_DATA_BITS(MEM_DATA_BITS),
    .ADDR_BITS(ADDR_BITS),
    .BURST_BITS(BURST_BITS),
    .BURST_SIZE(BURST_SIZE),
    .WRITE_V_FLIP(WRITE_V_FLIP),
    .FRAME_WIDTH(FRAME_WIDTH),
    .FRAME_HEIGHT(FRAME_HEIGHT)
) frame_fifo_write_m0 (
    .rst(rst),
    .mem_clk(mem_clk),
    .Sdr_init_done(Sdr_init_done),
    .Sdr_init_ref_vld(Sdr_init_ref_vld),
    .Sdr_busy(Sdr_busy),
    .App_rd_busy(App_rd_busy),
    .O_wr_busy(main_writer_busy),
    .App_wr_en(main_app_wr_en),
    .App_wr_addr(main_app_wr_addr),
    .write_req(write_req),
    .write_req_ack(write_req_ack),
    .write_finish(write_finish),
    .write_addr_0(write_addr_0),
    .write_addr_1(write_addr_1),
    .write_addr_2(write_addr_2),
    .write_addr_3(write_addr_3),
    .write_addr_index(write_addr_index),
    .write_len(write_len),
    .fifo_aclr(write_fifo_aclr),
    .rdusedw(rdusedw)
);

generate
if (THUMBNAIL_ENABLE != 0) begin : g_thumbnail
    // The thumbnail writer waits until the SDRAM port has been quiet for a while (a write that
    // follows a read too closely is lost; the display reader leaves only ~12 clocks between
    // its bursts), then writes one montage row as a continuous burst while the reader and the
    // main writer see the port as busy.
    reg [6:0] quiet_cnt;
    always @(posedge mem_clk or posedge rst) begin
        if (rst)
            quiet_cnt <= 7'd0;
        else if (!Sdr_init_done || reader_busy || main_writer_busy || main_app_wr_en)
            quiet_cnt <= 7'd0;
        else if (quiet_cnt != 7'd127)
            quiet_cnt <= quiet_cnt + 1'b1;
    end
    wire gen_go_ok = (quiet_cnt >= 7'd48);

    media_thumbnail_generator #(
        .ADDR_BITS(ADDR_BITS),
        .FRAME_WIDTH(FRAME_WIDTH),
        .FRAME_HEIGHT(FRAME_HEIGHT),
        .SAMPLE_X_STEP(THUMBNAIL_SAMPLE_X_STEP),
        .SAMPLE_Y_STEP(THUMBNAIL_SAMPLE_Y_STEP),
        .OUTPUT_X_SCALE(THUMBNAIL_OUTPUT_X_SCALE),
        .OUTPUT_Y_SCALE(THUMBNAIL_OUTPUT_Y_SCALE),
        .INPUT_BOTTOM_UP(WRITE_V_FLIP),
        .BASE_ADDRESS(THUMBNAIL_BASE_ADDRESS)
    ) thumbnail_generator (
        .rst(rst),
        .write_clk(write_clk),
        .write_frame_start(write_frame_start),
        .image_bank(write_addr_index),
        .write_en(write_en),
        .write_data(write_data),
        .mem_clk(mem_clk),
        .go_ok(gen_go_ok),
        .writer_busy(thumbnail_writer_busy),
        .writer_pending(thumbnail_writer_pending),
        .mem_write_en(thumbnail_mem_write_en),
        .mem_write_addr(thumbnail_mem_write_addr),
        .mem_write_data(thumbnail_mem_write_data),
        .thumbnail_ready(thumbnail_ready),
        .thumbnail_overflow(thumbnail_overflow),
        .thumbnail_mask(thumbnail_mask)
    );

    assign thumbnail_mem_write_grant = thumbnail_mem_write_en;
    assign App_wr_en = main_app_wr_en || thumbnail_mem_write_en;
    assign App_wr_addr = main_app_wr_en ? main_app_wr_addr : thumbnail_mem_write_addr;
    assign App_wr_din = main_app_wr_en ? main_app_wr_din : thumbnail_mem_write_data;
    assign loader_busy = main_writer_busy || thumbnail_writer_busy;
end else begin : g_no_thumbnail
    assign thumbnail_writer_busy = 1'b0;
    assign thumbnail_writer_pending = 1'b0;
    assign thumbnail_mem_write_en = 1'b0;
    assign thumbnail_mem_write_grant = 1'b0;
    assign thumbnail_mem_write_addr = {ADDR_BITS{1'b0}};
    assign thumbnail_mem_write_data = 32'd0;
    assign thumbnail_ready = 1'b0;
    assign thumbnail_overflow = 1'b0;
    assign thumbnail_mask = 4'd0;
    assign App_wr_en = main_app_wr_en;
    assign App_wr_addr = main_app_wr_addr;
    assign App_wr_din = main_app_wr_din;
    assign loader_busy = main_writer_busy;
end
endgenerate

endmodule

module image_downsample_2x2 (
    input wire clk,
    input wire rst,
    input wire frame_start,
    input wire in_en,
    input wire [31:0] in_data,
    output wire out_en,
    output wire [31:0] out_data
);

reg [9:0] x_count;
reg [8:0] y_count;

wire take_pixel = in_en && ~x_count[0] && ~y_count[0];

always @(posedge clk or posedge rst) begin
    if (rst) begin
        x_count <= 10'd0;
        y_count <= 9'd0;
    end else if (frame_start) begin
        x_count <= 10'd0;
        y_count <= 9'd0;
    end else if (in_en) begin
        if (x_count == 10'd639) begin
            x_count <= 10'd0;
            if (y_count == 9'd479)
                y_count <= 9'd0;
            else
                y_count <= y_count + 1'b1;
        end else begin
            x_count <= x_count + 1'b1;
        end
    end
end

assign out_en = take_pixel;
assign out_data = in_data;

endmodule
