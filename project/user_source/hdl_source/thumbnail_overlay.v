`timescale 1ns/1ps

module thumbnail_overlay #(
    parameter integer DIAG_EN = 0,          // test builds: show thumbnail writer status squares
    parameter integer DISPLAY_WIDTH = 640,
    parameter integer DISPLAY_HEIGHT = 480,
    parameter integer TILE_WIDTH = DISPLAY_WIDTH / 2,
    parameter integer TILE_HEIGHT = DISPLAY_HEIGHT / 2,
    parameter integer BORDER_WIDTH = 2
) (
    input wire clk,
    input wire rst,
    input wire mode,
    input wire [1:0] focus_index,
    input wire vsync,
    input wire de,
    input wire [23:0] rgb_in,
    input wire test_pattern,
    input wire [5:0] diag,      // test builds: 6 status squares top-left (green = 1)
    output wire [23:0] rgb_out,
    output wire border_active
);

reg [9:0] pixel_x;
reg [9:0] pixel_y;
reg de_q;
reg vs_q;
wire frame_start = vs_q && !vsync;
localparam [9:0] DISPLAY_WIDTH_10 = DISPLAY_WIDTH;
localparam [9:0] DISPLAY_HEIGHT_10 = DISPLAY_HEIGHT;
localparam [9:0] TILE_WIDTH_10 = TILE_WIDTH;
localparam [9:0] TILE_HEIGHT_10 = TILE_HEIGHT;
localparam [9:0] TILE_RIGHT_INNER_10 = TILE_WIDTH - BORDER_WIDTH;
localparam [9:0] TILE_BOTTOM_INNER_10 = TILE_HEIGHT - BORDER_WIDTH;
wire right_tile = pixel_x >= TILE_WIDTH_10;
wire lower_tile = pixel_y >= TILE_HEIGHT_10;
wire [9:0] local_x = right_tile ? pixel_x - TILE_WIDTH_10 : pixel_x;
wire [9:0] local_y = lower_tile ? pixel_y - TILE_HEIGHT_10 : pixel_y;
wire [1:0] tile_index = {lower_tile, right_tile};
wire inside_display = pixel_x < DISPLAY_WIDTH_10 && pixel_y < DISPLAY_HEIGHT_10;
wire tile_border = local_x < BORDER_WIDTH || local_x >= TILE_RIGHT_INNER_10 ||
                   local_y < BORDER_WIDTH || local_y >= TILE_BOTTOM_INNER_10;
wire border_pixel = mode && de && inside_display && tile_border;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        pixel_x <= 10'd0;
        pixel_y <= 10'd0;
        de_q <= 1'b0;
        vs_q <= 1'b1;
    end else begin
        de_q <= mode && de;
        vs_q <= vsync;
        if (frame_start) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
        end else if (mode) begin
            if (!de)
                pixel_x <= 10'd0;
            else if (pixel_x < DISPLAY_WIDTH_10 - 1'b1)
                pixel_x <= pixel_x + 1'b1;
            if (de_q && !de && pixel_y < DISPLAY_HEIGHT_10 - 1'b1)
                pixel_y <= pixel_y + 1'b1;
        end
    end
end

assign border_active = border_pixel;

// ---------------------------------------------------------------------
// Diagnostic tile (bring-up only)
// The upper-left tile is replaced by a pattern generated from the display
// position itself.  The pattern never passes through the SDRAM reader,
// the FIFO or the upscaler, so whatever a camera sees in it comes from
// the overlay inputs onwards: the transition mux, the HDMI serialiser,
// the monitor or the lens.  Two-pixel border, 4-pixel black/white stripes
// at the very start of the line, a solid red block at x 32..47 and a
// 32-pixel white grid - any blur, blend or shift at the left edge is
// immediately visible, and the other three tiles keep their real image
// data for comparison.
// ---------------------------------------------------------------------
wire [23:0] test_rgb =
    (local_x < 10'd32) ? (local_x[2] ? 24'hffffff : 24'h000000) :
    (local_x < 10'd48) ? 24'hff0000 :
    ((local_x[4:0] == 5'd0) || (local_y[4:0] == 5'd0)) ? 24'hffffff : 24'h000040;
wire test_pixel = mode && test_pattern && de && inside_display &&
                  (tile_index == 2'b00);

wire [9:0] dsq = (pixel_x - 10'd8) >> 4;
wire diag_pixel = (DIAG_EN != 0) && mode && de && pixel_y >= 10'd8 && pixel_y < 10'd24 && pixel_x >= 10'd8 && pixel_x < 10'd104;
wire [23:0] diag_rgb = diag[dsq[2:0]] ? 24'h00ff00 : 24'hff0000;
assign rgb_out = diag_pixel ? diag_rgb : border_pixel ?
                 ((tile_index == focus_index) ? 24'h60ff90 : 24'hb0b8b8) :
                 (test_pixel ? test_rgb : rgb_in);

endmodule
