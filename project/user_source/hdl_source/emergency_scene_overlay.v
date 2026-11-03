`timescale 1ns/1ps

module emergency_scene_overlay #(
    parameter integer H_ACTIVE = 640,
    parameter integer V_ACTIVE = 480
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        emergency_mode_async,
    input  wire        thumbnail_mode_async,
    input  wire [1:0]  image_index_async,
    input  wire        display_valid_async,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    output reg         hs_out,
    output reg         vs_out,
    output reg         de_out,
    output reg  [23:0] rgb_out,
    output wire        emergency_active
);

localparam integer BAND_TOP = 416;
localparam integer TEXT_Y = 432;        // 32 px glyph row inside the 56 px band
localparam integer GLYPH = 32;          // 16x16 font shown at 2x
localparam integer GAP = 640;           // blank run before a message re-enters

(* async_reg = "true" *) reg [1:0] emergency_sync;
(* async_reg = "true" *) reg [1:0] valid_sync;
(* async_reg = "true" *) reg [1:0] thumbnail_sync;
(* async_reg = "true" *) reg [1:0] image_sync_0;
reg [1:0] image_sync_1;
reg emergency_frame;
reg display_valid_frame;
reg thumbnail_frame;
reg [1:0] image_frame;
reg vs_previous;
reg de_previous;
reg [9:0] pixel_x;
reg [9:0] pixel_y;
reg [11:0] scroll_pos;
reg       scroll_divider;
reg       emergency_previous;

wire frame_start = vs_previous && !vs_in;
// The font ROM is synchronous, so text lookups run one pixel ahead.
wire [9:0] next_x = de_in ? ((pixel_x < H_ACTIVE-1) ? pixel_x + 1'b1 : pixel_x) : 10'd0;
wire [6:0] span_length;
wire [7:0] span_code_unused;
wire [11:0] scroll_span = span_length * GLYPH + GAP;
subtitle_text_rom u_span_rom (.msg(emergency_frame ? 4'd4 : {2'b00, image_frame}),
                              .index(7'd0), .code(span_code_unused), .length(span_length));
wire frame_border = pixel_x < 8 || pixel_x >= H_ACTIVE-8 ||
                    pixel_y < 8 || pixel_y >= V_ACTIVE-8;
wire emergency_header = pixel_y >= 24 && pixel_y < 88;
wire subtitle_band = pixel_y >= BAND_TOP && pixel_y < V_ACTIVE-8;
wire warning_phase = scroll_pos[5];
wire overlay_enable = emergency_frame || (display_valid_frame && !thumbnail_frame);
assign emergency_active = emergency_frame;

// One font ROM and one text ROM serve every text region: the regions never
// overlap, so the region decoder below picks the message, cell and glyph row.
reg [3:0]  text_msg;
reg [6:0]  text_index;
reg [3:0]  text_row;
reg [3:0]  text_col;
reg        text_region;
reg        text_yellow;
reg        text_in_range;
wire [7:0]  text_code;
wire [6:0]  text_length;
wire [15:0] glyph_bits;
subtitle_text_rom u_text_rom (.msg(text_msg), .index(text_index),
                              .code(text_code), .length(text_length));
subtitle_font_rom u_font_rom (.clk(clk), .code(text_code), .row(text_row), .bits(glyph_bits));
// region/column decoded for the pixel whose glyph bits arrive next cycle
reg       lk_valid;
reg [1:0] lk_kind;     // 0 subtitle, 1 title, 2 body
reg [3:0] lk_col;
reg       lk_yellow;
always @(posedge clk) begin
    lk_valid  <= text_in_range;
    lk_kind   <= (text_msg == 4'd10) ? 2'd1 : (text_msg >= 4'd5) ? 2'd2 : 2'd0;
    lk_col    <= text_col;
    lk_yellow <= text_yellow;
end
wire glyph_on = lk_valid && glyph_bits[15-lk_col];

integer stream_x;
integer body_line;
integer body_y;
integer body_x;
integer title_left;
reg text_pixel;
reg title_pixel;
reg body_pixel;
reg arrow_pixel;
reg [23:0] composed_rgb;

always @* begin
    text_msg = 4'd0;
    text_index = 7'd0;
    text_row = 4'd0;
    text_col = 4'd0;
    text_region = 1'b0;
    text_yellow = 1'b0;
    stream_x = 0;
    body_line = 0;
    body_y = 0;
    body_x = 0;
    title_left = 224;

    if (pixel_y >= TEXT_Y && pixel_y < TEXT_Y+GLYPH) begin
        // scrolling subtitle: message enters from the right edge
        stream_x = next_x + scroll_pos - GAP;
        text_msg = emergency_frame ? 4'd4 : {2'b00, image_frame};
        text_region = stream_x >= 0;
        text_index = stream_x >> 5;
        text_col = (stream_x >> 1) & 15;
        text_row = (pixel_y - TEXT_Y) >> 1;
        text_yellow = emergency_frame && text_index < 7;
    end else if (emergency_frame && pixel_y >= 40 && pixel_y < 72 &&
                 next_x >= title_left && next_x < title_left+192) begin
        text_msg = 4'd10;
        text_region = 1'b1;
        text_index = (next_x - title_left) >> 5;
        text_col = ((next_x - title_left) >> 1) & 15;
        text_row = (pixel_y - 40) >> 1;
    end else if (emergency_frame && pixel_y >= 112 && pixel_y < 372 &&
                 next_x >= 48 && next_x < 400) begin
        body_y = pixel_y - 112;
        body_line = body_y / 52;
        body_x = next_x - 48;
        text_msg = 4'd5 + body_line;
        text_region = (body_y - body_line*52) < GLYPH;
        text_index = body_x >> 5;
        text_col = (body_x >> 1) & 15;
        text_row = (body_y - body_line*52) >> 1;
    end

    text_in_range = text_region && text_index < text_length && stream_x < 4096;
    text_pixel = overlay_enable && glyph_on && lk_kind == 2'd0 && !lk_yellow;
    title_pixel = glyph_on && lk_kind == 2'd1;
    body_pixel = glyph_on && lk_kind == 2'd2;

    arrow_pixel = emergency_frame && pixel_x >= 488 && pixel_x < 576 &&
                  pixel_y >= 188 && pixel_y < 276 &&
                  ((pixel_y >= 220 && pixel_y < 244) ||
                   (pixel_x >= 536 && ((pixel_y-188) >= (pixel_x-536)) &&
                                      ((276-pixel_y) >= (pixel_x-536))));

    composed_rgb = rgb_in;
    if (emergency_frame) begin
        composed_rgb = ((pixel_x[5] ^ pixel_y[5]) ? 24'h401018 : 24'h281018);
        if (pixel_y >= 104 && pixel_y < 392 && pixel_x >= 32 && pixel_x < 608)
            composed_rgb = 24'h202830;
        if (emergency_header)
            composed_rgb = 24'hB01820;
        if (arrow_pixel)
            composed_rgb = 24'hFFD32A;
        if (frame_border)
            composed_rgb = warning_phase ? 24'hFFCC20 : 24'hD01018;
    end
    if (overlay_enable && subtitle_band)
        composed_rgb = emergency_frame ? 24'h680C14 : 24'h102840;
    if (title_pixel || body_pixel || text_pixel)
        composed_rgb = 24'hFFFFFF;
    else if (overlay_enable && glyph_on && lk_kind == 2'd0 && lk_yellow)
        composed_rgb = 24'hFFD32A;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        emergency_sync <= 2'b00;
        valid_sync <= 2'b00;
        thumbnail_sync <= 2'b00;
        image_sync_0 <= 2'b00;
        image_sync_1 <= 2'b00;
        emergency_frame <= 1'b0;
        display_valid_frame <= 1'b0;
        thumbnail_frame <= 1'b0;
        image_frame <= 2'b00;
        emergency_previous <= 1'b0;
        vs_previous <= 1'b0;
        de_previous <= 1'b0;
        pixel_x <= 10'd0;
        pixel_y <= 10'd0;
        scroll_pos <= 12'd0;
        scroll_divider <= 1'b0;
        hs_out <= 1'b0;
        vs_out <= 1'b0;
        de_out <= 1'b0;
        rgb_out <= 24'd0;
    end else begin
        emergency_sync <= {emergency_sync[0], emergency_mode_async};
        valid_sync <= {valid_sync[0], display_valid_async};
        thumbnail_sync <= {thumbnail_sync[0], thumbnail_mode_async};
        image_sync_0 <= image_index_async;
        image_sync_1 <= image_sync_0;
        vs_previous <= vs_in;
        de_previous <= de_in;

        if (frame_start) begin
            emergency_frame <= emergency_sync[1];
            display_valid_frame <= valid_sync[1];
            thumbnail_frame <= thumbnail_sync[1];
            image_frame <= image_sync_1;
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            emergency_previous <= emergency_sync[1];
            if ((emergency_sync[1] != emergency_previous) || (!emergency_sync[1] && image_sync_1 != image_frame)) begin
                scroll_pos <= 12'd0;
                scroll_divider <= 1'b0;
            end else if (scroll_divider) begin
                scroll_divider <= 1'b0;
                scroll_pos <= (scroll_pos >= scroll_span - 1) ? 12'd0 : scroll_pos + 1'b1;
            end else begin
                scroll_divider <= 1'b1;
            end
        end else begin
            if (!de_in)
                pixel_x <= 10'd0;
            else if (pixel_x < H_ACTIVE-1)
                pixel_x <= pixel_x + 1'b1;
            if (de_previous && !de_in && pixel_y < V_ACTIVE-1)
                pixel_y <= pixel_y + 1'b1;
        end

        hs_out <= hs_in;
        vs_out <= vs_in;
        de_out <= de_in;
        if (!de_in)
            rgb_out <= 24'd0;
        else if (emergency_frame || (display_valid_frame && !thumbnail_frame))
            rgb_out <= composed_rgb;
        else
            rgb_out <= rgb_in;
    end
end

endmodule
