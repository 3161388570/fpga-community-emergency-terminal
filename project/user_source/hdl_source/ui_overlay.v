`timescale 1ns/1ps

// Subtitle band, emergency screen (hard cut in, 12-frame dissolve out) and the
// picture-quality panel.  One 16x16 font ROM serves every text region: the
// regions never overlap, so a single region decoder picks message/cell/row.
//
// Pipeline (4 clocks after the pixel counters):
//   s1 compose emergency (E) and normal (N) pictures + panel element
//   s2 dissolve E/N
//   s3 panel composite (semi-transparent fill, opaque borders/bars/text)
//   s4 panel fade (1.5 s fade-out after leaving the quality page)
module ui_overlay #(
    parameter integer H_ACTIVE = 640,
    parameter integer V_ACTIVE = 480
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        emergency_mode_async,
    input  wire        thumbnail_mode_async,
    input  wire [1:0]  image_index_async,
    input  wire        display_valid_async,
    input  wire        settings_mode_async,
    input  wire        selected_param_async,
    input  wire [2:0]  brightness_level,
    input  wire [2:0]  contrast_level,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    output wire        hs_out,
    output wire        vs_out,
    output wire        de_out,
    output wire [23:0] rgb_out,
    output wire        emergency_active,
    output wire        osd_active
);

localparam integer BAND_TOP = 416;
localparam integer TEXT_Y   = 432;
localparam integer GLYPH    = 32;
localparam integer GAP      = 640;

// panel geometry
localparam integer PX0 = 90, PY0 = 93, PW = 460, PH = 294;
localparam integer BAR_X = 232;

(* async_reg = "true" *) reg [1:0] emergency_sync;
(* async_reg = "true" *) reg [1:0] valid_sync;
(* async_reg = "true" *) reg [1:0] thumbnail_sync;
(* async_reg = "true" *) reg [1:0] settings_sync;
(* async_reg = "true" *) reg [1:0] image_sync_0;
reg [1:0] image_sync_1;
reg emergency_frame;
reg display_valid_frame;
reg thumbnail_frame;
reg [1:0] image_frame;
reg [3:0] exit_cnt;        // 12..1 while the emergency screen dissolves away
reg       panel_on;
reg [6:0] panel_hold;
reg [4:0] panel_fade;      // 16 = opaque
reg [2:0] panel_div;
reg       sel_param_frame;
reg [2:0] bright_frame, contrast_frame;
reg [5:0] frame_phase;     // 0..59, one second per cycle
reg vs_previous;
reg de_previous;
reg [9:0] pixel_x;
reg [9:0] pixel_y;
reg [11:0] scroll_pos;
reg [19:0] scroll_acc;      // scroll position in 1/256 pixel, advanced once per frame

reg       emergency_previous;

wire frame_start = vs_previous && !vs_in;
wire emerg_render = emergency_frame || (exit_cnt != 4'd0);
wire normal_enable = display_valid_frame && !thumbnail_frame;
wire overlay_enable = emerg_render || normal_enable;
wire panel_show = panel_on && !emerg_render;
assign emergency_active = emergency_frame;
assign osd_active = panel_on;

wire [9:0] next_x = de_in ? ((pixel_x < H_ACTIVE-1) ? pixel_x + 1'b1 : pixel_x) : 10'd0;
wire [6:0] span_length;
wire [7:0] span_code_unused;
wire [11:0] scroll_span = span_length * GLYPH + GAP;
// one full pass (text enters at the right edge, leaves at the left) takes 5 s = 300 frames:
// step = span / 300 pixel per frame = span * 873 / 1024 in 1/256 pixel
wire [21:0] scroll_step_wide = scroll_span * 10'd873;
wire [11:0] scroll_step = scroll_step_wide[21:10];
wire [19:0] scroll_limit = {scroll_span, 8'd0};
wire [19:0] scroll_acc_next = scroll_acc + {8'd0, scroll_step};
subtitle_text_rom u_span_rom (.msg(emerg_render ? 4'd4 : {2'b00, image_frame}),
                              .index(7'd0), .code(span_code_unused), .length(span_length));

// ---------------------------------------------------------------- text regions
reg [3:0]  text_msg;
reg [6:0]  text_index;
reg [3:0]  text_row;
reg [3:0]  text_col;
reg        text_region;
reg [23:0] text_color;
reg [1:0]  text_class;     // 0 subtitle, 1 emergency text, 2 panel text
reg        text_in_range;
wire [7:0]  text_code;
wire [6:0]  text_length;
wire [15:0] glyph_bits;
subtitle_text_rom u_text_rom (.msg(text_msg), .index(text_index),
                              .code(text_code), .length(text_length));
subtitle_font_rom u_font_rom (.clk(clk), .code(text_code), .row(text_row), .bits(glyph_bits));
reg        lk_valid;
reg [1:0]  lk_class;
reg [3:0]  lk_col;
reg [23:0] lk_color;
always @(posedge clk) begin
    lk_valid <= text_in_range;
    lk_class <= text_class;
    lk_col   <= text_col;
    lk_color <= text_color;
end
wire glyph_on = lk_valid && glyph_bits[15-lk_col];

integer stream_x, line_no, line_y, row_y, off_x;
reg [9:0] line_top;
always @* begin
    text_msg = 4'd0;
    text_index = 7'd0;
    text_row = 4'd0;
    text_col = 4'd0;
    text_region = 1'b0;
    text_color = 24'hFFFFFF;
    text_class = 2'd0;
    stream_x = 0;
    line_no = 0;
    line_y = 0;
    row_y = 0;
    off_x = 0;
    line_top = 10'd112;

    if (overlay_enable && pixel_y >= TEXT_Y && pixel_y < TEXT_Y+GLYPH) begin
        // scrolling subtitle, enters from the right edge
        stream_x = next_x + scroll_pos - GAP;
        text_msg = emerg_render ? 4'd4 : {2'b00, image_frame};
        text_region = stream_x >= 0;
        text_index = stream_x >> 5;
        text_col = (stream_x >> 1) & 15;
        text_row = (pixel_y - TEXT_Y) >> 1;
        if (emerg_render && text_index < 7) text_color = 24'hFFD32A;
    end else if (emerg_render && pixel_y >= 40 && pixel_y < 72 &&
                 next_x >= 224 && next_x < 416) begin
        text_msg = 4'd10;
        text_class = 2'd1;
        text_region = 1'b1;
        off_x = next_x - 224;
        text_index = off_x >> 5;
        text_col = (off_x >> 1) & 15;
        text_row = (pixel_y - 40) >> 1;
    end else if (emerg_render && pixel_y >= 112 && pixel_y < 348 &&
                 next_x >= 136 && next_x < 520) begin
        line_no = (pixel_y >= 316) ? 3 : (pixel_y >= 248) ? 2 : (pixel_y >= 180) ? 1 : 0;
        line_y = 112 + line_no * 68;
        row_y = pixel_y - line_y;
        text_msg = 4'd6 + line_no;
        text_class = 2'd1;
        text_region = row_y < GLYPH;
        off_x = next_x - 136;
        text_index = off_x >> 5;
        text_col = (off_x >> 1) & 15;
        text_row = row_y >> 1;
    end else if (panel_show) begin
        text_class = 2'd2;
        if (pixel_y >= PY0+12 && pixel_y < PY0+44 && next_x >= 256 && next_x < 384) begin
            text_msg = 4'd11; text_color = 24'hFFD32A;
            off_x = next_x - 256; text_region = 1'b1;
            text_index = off_x >> 5; text_col = (off_x >> 1) & 15; text_row = (pixel_y - (PY0+12)) >> 1;
        end else if (pixel_y >= 160 && pixel_y < 192 && next_x >= 124 && next_x < 188) begin
            text_msg = 4'd12; off_x = next_x - 124; text_region = 1'b1;
            text_index = off_x >> 5; text_col = (off_x >> 1) & 15; text_row = (pixel_y - 160) >> 1;
        end else if (pixel_y >= 220 && pixel_y < 252 && next_x >= 124 && next_x < 220) begin
            text_msg = 4'd13; off_x = next_x - 124; text_region = 1'b1;
            text_index = off_x >> 5; text_col = (off_x >> 1) & 15; text_row = (pixel_y - 220) >> 1;
        end else if (pixel_y >= 160 && pixel_y < 192 && next_x >= 412 && next_x < 444) begin
            text_msg = 4'd5; text_color = 24'hFFD32A; text_region = 1'b1;
            text_index = {4'd0, bright_frame};
            text_col = ((next_x - 412) >> 1) & 15; text_row = (pixel_y - 160) >> 1;
        end else if (pixel_y >= 220 && pixel_y < 252 && next_x >= 412 && next_x < 444) begin
            text_msg = 4'd5; text_color = 24'h30D8E8; text_region = 1'b1;
            text_index = {4'd0, contrast_frame};
            text_col = ((next_x - 412) >> 1) & 15; text_row = (pixel_y - 220) >> 1;
        end else if (pixel_y >= 290 && pixel_y < 322 && next_x >= 112 && next_x < 528) begin
            text_msg = 4'd14; text_color = 24'hD0D8E0; off_x = next_x - 112; text_region = 1'b1;
            text_index = off_x >> 5; text_col = (off_x >> 1) & 15; text_row = (pixel_y - 290) >> 1;
        end else if (pixel_y >= 330 && pixel_y < 362 && next_x >= 112 && next_x < 528) begin
            text_msg = 4'd15; text_color = 24'hD0D8E0; off_x = next_x - 112; text_region = 1'b1;
            text_index = off_x >> 5; text_col = (off_x >> 1) & 15; text_row = (pixel_y - 330) >> 1;
        end
    end
    text_in_range = text_region && text_index < text_length && stream_x < 4096;
end

// ---------------------------------------------------------------- picture composition
wire border_outer = pixel_x < 8 || pixel_x >= H_ACTIVE-8 || pixel_y < 8 || pixel_y >= V_ACTIVE-8;
wire border_any   = pixel_x < 16 || pixel_x >= H_ACTIVE-16 || pixel_y < 16 || pixel_y >= V_ACTIVE-16;
wire phase_a = frame_phase < 6'd30;
wire subtitle_band = pixel_y >= BAND_TOP && pixel_y < V_ACTIVE-8;
wire arrow_shaft = pixel_x >= 76 && pixel_x < 124 && pixel_y >= 232 && pixel_y < 264;
wire [9:0] head_dx = pixel_x - 10'd36;
wire [9:0] head_dy = (pixel_y >= 248) ? (pixel_y - 10'd248) : (10'd248 - pixel_y);
wire arrow_head = pixel_x >= 36 && pixel_x < 76 && head_dy <= head_dx && pixel_y >= 208 && pixel_y < 288;
wire arrow_pixel = arrow_shaft || arrow_head;

reg [23:0] e_rgb, n_rgb;
always @* begin
    e_rgb = 24'h4A0A12;
    if (pixel_y >= 104 && pixel_y < 392 && pixel_x >= 24 && pixel_x < H_ACTIVE-24)
        e_rgb = 24'h2E060C;
    if (pixel_y >= 24 && pixel_y < 88)
        e_rgb = 24'hB01820;
    if (arrow_pixel)
        e_rgb = 24'hFFD32A;
    if (border_any)
        e_rgb = (border_outer ^ phase_a) ? 24'hFFCC20 : 24'hD01018;
    if (subtitle_band)
        e_rgb = 24'h680C14;
    if (glyph_on && lk_class != 2'd2)
        e_rgb = lk_color;

    n_rgb = rgb_in;
    if (normal_enable && subtitle_band)
        n_rgb = 24'h102840;
    if (glyph_on && lk_class == 2'd0)
        n_rgb = lk_color;
end

// panel element: kind 0 none, 1 translucent fill, 2 opaque colour
wire in_panel = panel_show && pixel_x >= PX0 && pixel_x < PX0+PW && pixel_y >= PY0 && pixel_y < PY0+PH;
wire panel_border = pixel_x < PX0+3 || pixel_x >= PX0+PW-3 || pixel_y < PY0+3 || pixel_y >= PY0+PH-3;
wire panel_sep = pixel_x >= PX0+12 && pixel_x < PX0+PW-12 &&
                 ((pixel_y >= 147 && pixel_y < 149) || (pixel_y >= 279 && pixel_y < 281));
wire [9:0] bar_cx = pixel_x - BAR_X;
wire [3:0] bar_cell = bar_cx / 20;
wire [9:0] bar_cell_x = bar_cx - bar_cell * 20;
wire bar_x_ok = pixel_x >= BAR_X && pixel_x < BAR_X+160 && bar_cell_x < 16;
wire bar_a = bar_x_ok && pixel_y >= 164 && pixel_y < 188;
wire bar_b = bar_x_ok && pixel_y >= 224 && pixel_y < 248;
wire sel_bar_a = !sel_param_frame && pixel_x >= PX0+8 && pixel_x < PX0+14 && pixel_y >= 156 && pixel_y < 196;
wire sel_bar_b =  sel_param_frame && pixel_x >= PX0+8 && pixel_x < PX0+14 && pixel_y >= 216 && pixel_y < 256;
reg [1:0]  pk;
reg [23:0] pc_raw;
always @* begin
    pk = 2'd0;
    pc_raw = 24'd0;
    if (in_panel) begin
        pk = 2'd1;
        if (panel_sep) begin pk = 2'd2; pc_raw = 24'h8A7420; end
        if (bar_a) begin pk = 2'd2; pc_raw = (bar_cell <= bright_frame) ? 24'hFFD32A : 24'h3A4458; end
        if (bar_b) begin pk = 2'd2; pc_raw = (bar_cell <= contrast_frame) ? 24'h30D8E8 : 24'h3A4458; end
        if (sel_bar_a || sel_bar_b) begin pk = 2'd2; pc_raw = 24'hFFD32A; end
        if (panel_border) begin pk = 2'd2; pc_raw = 24'hFFD32A; end
        if (glyph_on && lk_class == 2'd2) begin pk = 2'd2; pc_raw = lk_color; end
    end
end

// ---------------------------------------------------------------- pipeline
reg [23:0] s1_e, s1_n, s1_pc;
reg [1:0]  s1_pk;
reg        s1_dis;
reg [3:0]  s1_a;
reg        s1_emg;
reg        s1_hs, s1_vs, s1_de, s1_on;
reg [23:0] s2_rgb, s2_pc;
reg [1:0]  s2_pk;
reg        s2_hs, s2_vs, s2_de;
reg [23:0] s3_rgb, s3_pc;
reg        s3_hs, s3_vs, s3_de;
reg [23:0] s4_rgb;
reg        s4_hs, s4_vs, s4_de;
assign hs_out = s4_hs;
assign vs_out = s4_vs;
assign de_out = s4_de;
assign rgb_out = s4_rgb;

wire [8:0] wa = s1_a * 9'd21;
wire [8:0] wb = (4'd12 - s1_a) * 9'd21;
wire [16:0] mix_r = s1_e[23:16] * wa + s1_n[23:16] * wb;
wire [16:0] mix_g = s1_e[15:8]  * wa + s1_n[15:8]  * wb;
wire [16:0] mix_b = s1_e[7:0]   * wa + s1_n[7:0]   * wb;

// translucent fill: 3/4 panel colour 0x0E1626 + 1/4 picture
function [7:0] fill_mix;
    input [7:0] base;
    input [7:0] tint;
    begin
        fill_mix = ((tint * 3) >> 2) + (base >> 2);
    end
endfunction

wire [4:0] f = panel_fade;
wire [4:0] nf = 5'd16 - panel_fade;
wire [12:0] lerp_r = s3_pc[23:16] * f + s3_rgb[23:16] * nf;
wire [12:0] lerp_g = s3_pc[15:8]  * f + s3_rgb[15:8]  * nf;
wire [12:0] lerp_b = s3_pc[7:0]   * f + s3_rgb[7:0]   * nf;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s1_e <= 0; s1_n <= 0; s1_pc <= 0; s1_pk <= 0; s1_dis <= 0; s1_a <= 0; s1_emg <= 0;
        s1_hs <= 0; s1_vs <= 0; s1_de <= 0; s1_on <= 0;
        s2_rgb <= 0; s2_pc <= 0; s2_pk <= 0; s2_hs <= 0; s2_vs <= 0; s2_de <= 0;
        s3_rgb <= 0; s3_pc <= 0; s3_hs <= 0; s3_vs <= 0; s3_de <= 0;
        s4_rgb <= 0; s4_hs <= 0; s4_vs <= 0; s4_de <= 0;
    end else begin
        s1_e <= e_rgb; s1_n <= n_rgb; s1_pc <= pc_raw; s1_pk <= pk;
        s1_dis <= (exit_cnt != 4'd0);
        s1_a <= exit_cnt;
        s1_emg <= emergency_frame;
        s1_hs <= hs_in; s1_vs <= vs_in; s1_de <= de_in;

        s2_hs <= s1_hs; s2_vs <= s1_vs; s2_de <= s1_de;
        s2_pc <= s1_pc; s2_pk <= s1_pk;
        if (!s1_de)           s2_rgb <= 24'd0;
        else if (s1_dis)      s2_rgb <= {mix_r[15:8], mix_g[15:8], mix_b[15:8]};
        else if (s1_emg)      s2_rgb <= s1_e;
        else                  s2_rgb <= s1_n;

        s3_hs <= s2_hs; s3_vs <= s2_vs; s3_de <= s2_de;
        s3_rgb <= s2_rgb;
        case (s2_pk)
            2'd1:    s3_pc <= {fill_mix(s2_rgb[23:16], 8'h0E), fill_mix(s2_rgb[15:8], 8'h16), fill_mix(s2_rgb[7:0], 8'h26)};
            2'd2:    s3_pc <= s2_pc;
            default: s3_pc <= s2_rgb;
        endcase

        s4_hs <= s3_hs; s4_vs <= s3_vs; s4_de <= s3_de;
        if (!s3_de)  s4_rgb <= 24'd0;
        else         s4_rgb <= {lerp_r[11:4], lerp_g[11:4], lerp_b[11:4]};
    end
end

// ---------------------------------------------------------------- frame state
always @(posedge clk or posedge rst) begin
    if (rst) begin
        emergency_sync <= 2'b00;
        valid_sync <= 2'b00;
        thumbnail_sync <= 2'b00;
        settings_sync <= 2'b00;
        image_sync_0 <= 2'b00;
        image_sync_1 <= 2'b00;
        emergency_frame <= 1'b0;
        display_valid_frame <= 1'b0;
        thumbnail_frame <= 1'b0;
        image_frame <= 2'b00;
        exit_cnt <= 4'd0;
        panel_on <= 1'b0;
        panel_hold <= 7'd0;
        panel_fade <= 5'd16;
        panel_div <= 3'd0;
        sel_param_frame <= 1'b0;
        bright_frame <= 3'd2;
        contrast_frame <= 3'd2;
        frame_phase <= 6'd0;
        emergency_previous <= 1'b0;
        vs_previous <= 1'b0;
        de_previous <= 1'b0;
        pixel_x <= 10'd0;
        pixel_y <= 10'd0;
        scroll_pos <= 12'd0;
        scroll_acc <= 20'd0;
    end else begin
        emergency_sync <= {emergency_sync[0], emergency_mode_async};
        valid_sync <= {valid_sync[0], display_valid_async};
        thumbnail_sync <= {thumbnail_sync[0], thumbnail_mode_async};
        settings_sync <= {settings_sync[0], settings_mode_async};
        image_sync_0 <= image_index_async;
        image_sync_1 <= image_sync_0;
        vs_previous <= vs_in;
        de_previous <= de_in;

        if (frame_start) begin
            display_valid_frame <= valid_sync[1];
            thumbnail_frame <= thumbnail_sync[1];
            image_frame <= image_sync_1;
            sel_param_frame <= selected_param_async;
            bright_frame <= brightness_level;
            contrast_frame <= contrast_level;
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            frame_phase <= (emergency_sync[1] && !emergency_previous) ? 6'd0 :
                           (frame_phase == 6'd59) ? 6'd0 : frame_phase + 1'b1;

            // emergency: hard cut in, 12-frame dissolve out
            if (emergency_sync[1]) begin
                emergency_frame <= 1'b1;
                exit_cnt <= 4'd0;
            end else if (emergency_frame) begin
                emergency_frame <= 1'b0;
                exit_cnt <= 4'd12;
            end else if (exit_cnt != 4'd0) begin
                exit_cnt <= exit_cnt - 1'b1;
            end
            emergency_previous <= emergency_sync[1];

            // quality panel: stays while SW4 is on, then fades out over 1.5 s
            if (emergency_sync[1]) begin
                panel_on <= 1'b0;
            end else if (settings_sync[1]) begin
                panel_on <= 1'b1;
                panel_hold <= 7'd0;
                panel_fade <= 5'd16;
                panel_div <= 3'd0;
            end else if (panel_on) begin
                panel_on <= 1'b0;
                panel_fade <= 5'd16;
            end

            if ((emergency_sync[1] != emergency_previous) ||
                (!emergency_sync[1] && image_sync_1 != image_frame)) begin
                scroll_pos <= 12'd0;
                scroll_acc <= 20'd0;
            end else if (scroll_acc_next >= scroll_limit) begin
                scroll_pos <= 12'd0;
                scroll_acc <= 20'd0;
            end else begin
                scroll_acc <= scroll_acc_next;
                scroll_pos <= scroll_acc_next[19:8];
            end
        end else begin
            if (!de_in)
                pixel_x <= 10'd0;
            else if (pixel_x < H_ACTIVE-1)
                pixel_x <= pixel_x + 1'b1;
            if (de_previous && !de_in && pixel_y < V_ACTIVE-1)
                pixel_y <= pixel_y + 1'b1;
        end
    end
end

endmodule
