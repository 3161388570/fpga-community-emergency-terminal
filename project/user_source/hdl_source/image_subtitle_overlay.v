`timescale 1ns/1ps
module image_subtitle_overlay #(
    parameter integer DEBOUNCE_CYCLES = 500000
) (
    input clk,
    input rst,
    input sw1,
    input [1:0] image_index,
    input display_valid,
    input hs,
    input vs,
    input de,
    input [23:0] rgb,
    output reg hs_out,
    output reg vs_out,
    output reg de_out,
    output reg [23:0] rgb_out,
    output subtitle_active
);
localparam COUNT_WIDTH = DEBOUNCE_CYCLES > 1 ? $clog2(DEBOUNCE_CYCLES) : 1;
(* async_reg = "true" *) reg [1:0] switch_sync;
reg [COUNT_WIDTH-1:0] debounce_count;
reg switch_stable;
reg vs_previous;
reg de_previous;
reg [9:0] pixel_x;
reg [9:0] pixel_y;
reg frame_enable;
reg frame_valid;
reg [1:0] frame_image;
wire frame_start = vs_previous && !vs;
wire [9:0] local_x = pixel_x - 10'd286;
wire [9:0] local_y = pixel_y - 10'd430;
wire [5:0] glyph_column = local_x[6:1];
wire [4:0] glyph_line = local_y[5:1];
wire in_caption = pixel_x >= 286 && pixel_x < 354 && pixel_y >= 430 && pixel_y < 466;
function [33:0] label_row;
    input [1:0] image_id;
    input [4:0] row;
    begin
        case ({image_id, row})
            7'd1: label_row = {1'b0, 32'h08200000, 1'b0};
            7'd2: label_row = {1'b0, 32'h08207ffc, 1'b0};
            7'd3: label_row = {1'b0, 32'hfffe4924, 1'b0};
            7'd4: label_row = {1'b0, 32'h08204544, 1'b0};
            7'd5: label_row = {1'b0, 32'h00005ff4, 1'b0};
            7'd6: label_row = {1'b0, 32'h08884204, 1'b0};
            7'd7: label_row = {1'b0, 32'h08907ffc, 1'b0};
            7'd8: label_row = {1'b0, 32'h10a04824, 1'b0};
            7'd9: label_row = {1'b0, 32'h30c057d4, 1'b0};
            7'd10: label_row = {1'b0, 32'h5080644c, 1'b0};
            7'd11: label_row = {1'b0, 32'h918044c4, 1'b0};
            7'd12: label_row = {1'b0, 32'h12824424, 1'b0};
            7'd13: label_row = {1'b0, 32'h148243e4, 1'b0};
            7'd14: label_row = {1'b0, 32'h10824004, 1'b0};
            7'd15: label_row = {1'b0, 32'h107e7ffc, 1'b0};
            7'd16: label_row = {1'b0, 32'h10004004, 1'b0};
            7'd33: label_row = {1'b0, 32'h10000000, 1'b0};
            7'd34: label_row = {1'b0, 32'h70000000, 1'b0};
            7'd35: label_row = {1'b0, 32'h10000000, 1'b0};
            7'd36: label_row = {1'b0, 32'h10000000, 1'b0};
            7'd37: label_row = {1'b0, 32'h103c3e3c, 1'b0};
            7'd38: label_row = {1'b0, 32'h10424442, 1'b0};
            7'd39: label_row = {1'b0, 32'h10424442, 1'b0};
            7'd40: label_row = {1'b0, 32'h10423842, 1'b0};
            7'd41: label_row = {1'b0, 32'h10424042, 1'b0};
            7'd42: label_row = {1'b0, 32'h10423c42, 1'b0};
            7'd43: label_row = {1'b0, 32'h7c3c423c, 1'b0};
            7'd44: label_row = {1'b0, 32'h00004200, 1'b0};
            7'd45: label_row = {1'b0, 32'h00003c00, 1'b0};
            7'd46: label_row = {1'b0, 32'h00000000, 1'b0};
            7'd47: label_row = {1'b0, 32'h00000000, 1'b0};
            7'd48: label_row = {1'b0, 32'h00000000, 1'b0};
            7'd65: label_row = {1'b0, 32'h04400440, 1'b0};
            7'd66: label_row = {1'b0, 32'h04400440, 1'b0};
            7'd67: label_row = {1'b0, 32'hfffefffe, 1'b0};
            7'd68: label_row = {1'b0, 32'h04401440, 1'b0};
            7'd69: label_row = {1'b0, 32'h00001000, 1'b0};
            7'd70: label_row = {1'b0, 32'h1ff03ffc, 1'b0};
            7'd71: label_row = {1'b0, 32'h10104000, 1'b0};
            7'd72: label_row = {1'b0, 32'h1ff09ff0, 1'b0};
            7'd73: label_row = {1'b0, 32'h10101210, 1'b0};
            7'd74: label_row = {1'b0, 32'h1ff01110, 1'b0};
            7'd75: label_row = {1'b0, 32'h0100fffe, 1'b0};
            7'd76: label_row = {1'b0, 32'hfffe2210, 1'b0};
            7'd77: label_row = {1'b0, 32'h01002110, 1'b0};
            7'd78: label_row = {1'b0, 32'h01003ffc, 1'b0};
            7'd79: label_row = {1'b0, 32'h01000010, 1'b0};
            7'd80: label_row = {1'b0, 32'h01000060, 1'b0};
            7'd97: label_row = {1'b0, 32'h08200000, 1'b0};
            7'd98: label_row = {1'b0, 32'h08201ff0, 1'b0};
            7'd99: label_row = {1'b0, 32'hfffe1110, 1'b0};
            7'd100: label_row = {1'b0, 32'h08201110, 1'b0};
            7'd101: label_row = {1'b0, 32'h00001ff0, 1'b0};
            7'd102: label_row = {1'b0, 32'h7ffc1110, 1'b0};
            7'd103: label_row = {1'b0, 32'h01001110, 1'b0};
            7'd104: label_row = {1'b0, 32'h11101ff0, 1'b0};
            7'd105: label_row = {1'b0, 32'h09101110, 1'b0};
            7'd106: label_row = {1'b0, 32'h09200100, 1'b0};
            7'd107: label_row = {1'b0, 32'h0100fffe, 1'b0};
            7'd108: label_row = {1'b0, 32'hfffe0540, 1'b0};
            7'd109: label_row = {1'b0, 32'h01000920, 1'b0};
            7'd110: label_row = {1'b0, 32'h01003118, 1'b0};
            7'd111: label_row = {1'b0, 32'h0100c106, 1'b0};
            7'd112: label_row = {1'b0, 32'h01000100, 1'b0};
            default: label_row = 34'd0;
        endcase
    end
endfunction
wire [33:0] middle_row = label_row(frame_image, glyph_line);
wire [33:0] upper_row = label_row(frame_image, glyph_line - 1'b1);
wire [33:0] lower_row = label_row(frame_image, glyph_line + 1'b1);
wire [33:0] outline_row = middle_row | (middle_row << 1) | (middle_row >> 1) | upper_row | lower_row;
wire glyph_hit = middle_row[6'd33 - glyph_column];
wire outline_hit = outline_row[6'd33 - glyph_column];
assign subtitle_active = frame_enable && frame_valid;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        switch_sync <= 0;
        debounce_count <= 0;
        switch_stable <= 0;
    end else begin
        switch_sync <= {switch_sync[0], sw1};
        if (switch_sync[1] == switch_stable) debounce_count <= 0;
        else if (debounce_count == DEBOUNCE_CYCLES - 1) begin
            switch_stable <= switch_sync[1];
            debounce_count <= 0;
        end else debounce_count <= debounce_count + 1'b1;
    end
end
always @(posedge clk or posedge rst) begin
    if (rst) begin
        vs_previous <= 0;
        de_previous <= 0;
        pixel_x <= 0;
        pixel_y <= 0;
        frame_enable <= 0;
        frame_valid <= 0;
        frame_image <= 0;
        hs_out <= 0;
        vs_out <= 0;
        de_out <= 0;
        rgb_out <= 0;
    end else begin
        vs_previous <= vs;
        de_previous <= de;
        if (frame_start) begin
            pixel_x <= 0;
            pixel_y <= 0;
            frame_enable <= switch_stable;
            frame_valid <= display_valid;
            frame_image <= image_index;
        end else begin
            if (!de) pixel_x <= 0;
            else if (pixel_x < 640) pixel_x <= pixel_x + 1'b1;
            if (de_previous && !de && pixel_y < 480) pixel_y <= pixel_y + 1'b1;
        end
        hs_out <= hs;
        vs_out <= vs;
        de_out <= de;
        if (!de) rgb_out <= 0;
        else if (subtitle_active && in_caption && glyph_hit) rgb_out <= 24'hffffff;
        else if (subtitle_active && in_caption && outline_hit) rgb_out <= 0;
        else rgb_out <= rgb;
    end
end
endmodule
