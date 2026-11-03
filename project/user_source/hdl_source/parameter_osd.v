`timescale 1ns/1ps

module parameter_osd #(
    parameter integer HOLD_FRAMES = 120,
    parameter [23:0] PANEL_COLOR = 24'h101820,
    parameter [23:0] BORDER_COLOR = 24'h8090a0,
    parameter [23:0] SELECT_COLOR = 24'h40d090,
    parameter [23:0] FILL_COLOR = 24'hf0c840,
    parameter [23:0] EMPTY_COLOR = 24'h384048,
    parameter [23:0] TEXT_COLOR = 24'hf0f4f8
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       hs_in,
    input  wire       vs_in,
    input  wire       de_in,
    input  wire [23:0] rgb_in,
    input  wire       settings_mode_async,
    input  wire       selected_param_async,
    input  wire       cfg_toggle_async,
    input  wire [2:0] brightness_level,
    input  wire [2:0] contrast_level,
    output wire       hs_out,
    output wire       vs_out,
    output wire       de_out,
    output reg [23:0] rgb_out,
    output wire       osd_active
);

(* async_reg = "true" *) reg [1:0] settings_sync;
(* async_reg = "true" *) reg [1:0] selected_sync;
(* async_reg = "true" *) reg [1:0] cfg_toggle_sync;
reg cfg_toggle_seen;
reg [7:0] hold_count;
reg [9:0] pixel_x, pixel_y;
reg de_previous, vs_previous;
wire frame_start = vs_previous && !vs_in;
wire show_panel = settings_sync[1] || (hold_count != 0);

wire in_panel = pixel_x >= 10'd16 && pixel_x < 10'd224 &&
                pixel_y >= 10'd16 && pixel_y < 10'd82;
wire panel_border = in_panel &&
                    (pixel_x < 10'd18 || pixel_x >= 10'd222 ||
                     pixel_y < 10'd18 || pixel_y >= 10'd80);
wire brightness_row = pixel_y >= 10'd24 && pixel_y < 10'd44;
wire contrast_row = pixel_y >= 10'd52 && pixel_y < 10'd72;
wire selected_border = in_panel &&
                       ((!selected_sync[1] && brightness_row &&
                         (pixel_x < 10'd21 || pixel_x >= 10'd219 ||
                          pixel_y == 10'd24 || pixel_y == 10'd43)) ||
                        (selected_sync[1] && contrast_row &&
                         (pixel_x < 10'd21 || pixel_x >= 10'd219 ||
                          pixel_y == 10'd52 || pixel_y == 10'd71)));

function [4:0] glyph_row;
    input character_c;
    input [2:0] row;
    begin
        if (!character_c) begin
            case (row)
                3'd0: glyph_row = 5'b11110;
                3'd1: glyph_row = 5'b10001;
                3'd2: glyph_row = 5'b10001;
                3'd3: glyph_row = 5'b11110;
                3'd4: glyph_row = 5'b10001;
                3'd5: glyph_row = 5'b10001;
                default: glyph_row = 5'b11110;
            endcase
        end else begin
            case (row)
                3'd0: glyph_row = 5'b01111;
                3'd1: glyph_row = 5'b10000;
                3'd2: glyph_row = 5'b10000;
                3'd3: glyph_row = 5'b10000;
                3'd4: glyph_row = 5'b10000;
                3'd5: glyph_row = 5'b10000;
                default: glyph_row = 5'b01111;
            endcase
        end
    end
endfunction

wire label_b_area = pixel_x >= 10'd26 && pixel_x < 10'd36 &&
                    pixel_y >= 10'd27 && pixel_y < 10'd41;
wire label_c_area = pixel_x >= 10'd26 && pixel_x < 10'd36 &&
                    pixel_y >= 10'd55 && pixel_y < 10'd69;
wire [2:0] label_column = (label_b_area || label_c_area) ?
                          ((pixel_x - 10'd26) >> 1) : 3'd0;
wire [2:0] label_b_row = label_b_area ? ((pixel_y - 10'd27) >> 1) : 3'd0;
wire [2:0] label_c_row = label_c_area ? ((pixel_y - 10'd55) >> 1) : 3'd0;
wire [4:0] label_b_glyph = glyph_row(1'b0, label_b_row);
wire [4:0] label_c_glyph = glyph_row(1'b1, label_c_row);
wire label_b_pixel = label_b_area && label_b_glyph[4-label_column];
wire label_c_pixel = label_c_area && label_c_glyph[4-label_column];

wire bar_area = pixel_x >= 10'd52 && pixel_x < 10'd207 &&
                ((pixel_y >= 10'd28 && pixel_y < 10'd40) ||
                 (pixel_y >= 10'd56 && pixel_y < 10'd68));
wire [9:0] bar_local_x = pixel_x - 10'd52;
wire [2:0] bar_index = (bar_local_x < 10'd31)  ? 3'd0 :
                       (bar_local_x < 10'd62)  ? 3'd1 :
                       (bar_local_x < 10'd93)  ? 3'd2 :
                       (bar_local_x < 10'd124) ? 3'd3 : 3'd4;
wire bar_cell_x = (bar_local_x < 10'd23) ||
                  (bar_local_x >= 10'd31  && bar_local_x < 10'd54) ||
                  (bar_local_x >= 10'd62  && bar_local_x < 10'd85) ||
                  (bar_local_x >= 10'd93  && bar_local_x < 10'd116) ||
                  (bar_local_x >= 10'd124 && bar_local_x < 10'd147);
wire bar_cell = bar_area && bar_cell_x;
wire [2:0] row_level = brightness_row ? brightness_level : contrast_level;
wire bar_filled = bar_cell && (bar_index <= row_level);
wire default_marker = bar_area && (bar_local_x == 10'd86);

assign hs_out = hs_in;
assign vs_out = vs_in;
assign de_out = de_in;
assign osd_active = show_panel;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        settings_sync <= 2'b00;
        selected_sync <= 2'b00;
        cfg_toggle_sync <= 2'b00;
        cfg_toggle_seen <= 1'b0;
        hold_count <= 8'd0;
        pixel_x <= 10'd0;
        pixel_y <= 10'd0;
        de_previous <= 1'b0;
        vs_previous <= 1'b1;
    end else begin
        settings_sync <= {settings_sync[0], settings_mode_async};
        selected_sync <= {selected_sync[0], selected_param_async};
        cfg_toggle_sync <= {cfg_toggle_sync[0], cfg_toggle_async};
        de_previous <= de_in;
        vs_previous <= vs_in;

        if (cfg_toggle_sync[1] != cfg_toggle_seen) begin
            cfg_toggle_seen <= cfg_toggle_sync[1];
            hold_count <= HOLD_FRAMES;
        end else if (frame_start && !settings_sync[1] && hold_count != 0) begin
            hold_count <= hold_count - 1'b1;
        end

        if (frame_start) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
        end else begin
            if (!de_in)
                pixel_x <= 10'd0;
            else if (pixel_x < 10'd639)
                pixel_x <= pixel_x + 1'b1;
            if (de_previous && !de_in && pixel_y < 10'd479)
                pixel_y <= pixel_y + 1'b1;
        end
    end
end

always @* begin
    rgb_out = rgb_in;
    if (show_panel && de_in && in_panel) begin
        if (label_b_pixel || label_c_pixel || default_marker)
            rgb_out = TEXT_COLOR;
        else if (selected_border)
            rgb_out = SELECT_COLOR;
        else if (panel_border)
            rgb_out = BORDER_COLOR;
        else if (bar_filled)
            rgb_out = FILL_COLOR;
        else if (bar_cell)
            rgb_out = EMPTY_COLOR;
        else
            rgb_out = PANEL_COLOR;
    end
end

endmodule
