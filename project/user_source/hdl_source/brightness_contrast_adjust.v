`timescale 1ns/1ps

module brightness_contrast_adjust (
    input  wire       clk,
    input  wire       rst,
    input  wire       hs_in,
    input  wire       vs_in,
    input  wire       de_in,
    input  wire [23:0] rgb_in,
    input  wire [2:0] brightness_level_async,
    input  wire [2:0] contrast_level_async,
    input  wire       cfg_toggle_async,
    output reg        hs_out,
    output reg        vs_out,
    output reg        de_out,
    output reg [23:0] rgb_out,
    output reg  [2:0] brightness_level_active,
    output reg  [2:0] contrast_level_active
);

(* async_reg = "true" *) reg [2:0] brightness_sync1, brightness_sync2;
(* async_reg = "true" *) reg [2:0] contrast_sync1, contrast_sync2;
(* async_reg = "true" *) reg [1:0] cfg_toggle_sync;
reg cfg_toggle_seen;
reg [2:0] brightness_pending;
reg [2:0] contrast_pending;
reg vs_previous;
wire frame_start = vs_previous && !vs_in;

function signed [8:0] brightness_offset;
    input [2:0] level;
    begin
        case (level)
            3'd0: brightness_offset = -9'sd60;
            3'd1: brightness_offset = -9'sd30;
            3'd3: brightness_offset =  9'sd30;
            3'd4: brightness_offset =  9'sd45;
            3'd5: brightness_offset =  9'sd60;
            3'd6: brightness_offset =  9'sd75;
            3'd7: brightness_offset =  9'sd90;
            default: brightness_offset = 9'sd0;
        endcase
    end
endfunction

// contrast gain in 1/64 steps: 0.6 0.8 1.0 1.15 1.3 1.45 1.6 1.8
function [6:0] contrast_gain;
    input [2:0] level;
    begin
        case (level)
            3'd0: contrast_gain = 7'd38;
            3'd1: contrast_gain = 7'd51;
            3'd3: contrast_gain = 7'd74;
            3'd4: contrast_gain = 7'd83;
            3'd5: contrast_gain = 7'd93;
            3'd6: contrast_gain = 7'd102;
            3'd7: contrast_gain = 7'd115;
            default: contrast_gain = 7'd64;
        endcase
    end
endfunction


function [7:0] saturate_u8;
    input signed [10:0] value;
    begin
        if (value < 0)
            saturate_u8 = 8'd0;
        else if (value > 11'sd255)
            saturate_u8 = 8'd255;
        else
            saturate_u8 = value[7:0];
    end
endfunction

// Gain and offset only change at a frame boundary, so they are looked up once per frame and
// kept in registers.  The pixel path is a 3-stage pipeline (centre, multiply, add+saturate)
// with no table decoding in it; hs/vs/de are delayed by the same 3 clocks.
reg [6:0]        gain_r;
reg signed [8:0] offset_r;
reg signed [8:0] c1_r, c1_g, c1_b;
reg signed [16:0] p2_r, p2_g, p2_b;
reg [1:0] hs_d, vs_d, de_d;
wire signed [10:0] adjusted_r = $signed(p2_r[16:6]) + 11'sd128 + $signed(offset_r);
wire signed [10:0] adjusted_g = $signed(p2_g[16:6]) + 11'sd128 + $signed(offset_r);
wire signed [10:0] adjusted_b = $signed(p2_b[16:6]) + 11'sd128 + $signed(offset_r);
wire [23:0] adjusted_rgb = {saturate_u8(adjusted_r),
                            saturate_u8(adjusted_g),
                            saturate_u8(adjusted_b)};

always @(posedge clk or posedge rst) begin
    if (rst) begin
        brightness_sync1 <= 3'd2;
        brightness_sync2 <= 3'd2;
        contrast_sync1 <= 3'd2;
        contrast_sync2 <= 3'd2;
        cfg_toggle_sync <= 2'b00;
        cfg_toggle_seen <= 1'b0;
        brightness_pending <= 3'd2;
        contrast_pending <= 3'd2;
        brightness_level_active <= 3'd2;
        contrast_level_active <= 3'd2;
        gain_r <= 7'd64;
        offset_r <= 9'sd0;
        vs_previous <= 1'b1;
        hs_d <= 2'b00;
        vs_d <= 2'b11;
        de_d <= 2'b00;
        hs_out <= 1'b0;
        vs_out <= 1'b1;
        de_out <= 1'b0;
        rgb_out <= 24'h000000;
    end else begin
        brightness_sync1 <= brightness_level_async;
        brightness_sync2 <= brightness_sync1;
        contrast_sync1 <= contrast_level_async;
        contrast_sync2 <= contrast_sync1;
        cfg_toggle_sync <= {cfg_toggle_sync[0], cfg_toggle_async};
        vs_previous <= vs_in;

        if (cfg_toggle_sync[1] != cfg_toggle_seen) begin
            cfg_toggle_seen <= cfg_toggle_sync[1];
            brightness_pending <= brightness_sync2;
            contrast_pending <= contrast_sync2;
        end
        if (frame_start) begin
            brightness_level_active <= brightness_pending;
            contrast_level_active <= contrast_pending;
            gain_r <= contrast_gain(contrast_pending);
            offset_r <= brightness_offset(brightness_pending);
        end

        // stage 1: centre
        c1_r <= $signed({1'b0, rgb_in[23:16]}) - 9'sd128;
        c1_g <= $signed({1'b0, rgb_in[15:8]})  - 9'sd128;
        c1_b <= $signed({1'b0, rgb_in[7:0]})   - 9'sd128;
        // stage 2: gain
        p2_r <= c1_r * $signed({1'b0, gain_r});
        p2_g <= c1_g * $signed({1'b0, gain_r});
        p2_b <= c1_b * $signed({1'b0, gain_r});
        hs_d <= {hs_d[0], hs_in};
        vs_d <= {vs_d[0], vs_in};
        de_d <= {de_d[0], de_in};
        // stage 3: offset + saturate
        hs_out <= hs_d[1];
        vs_out <= vs_d[1];
        de_out <= de_d[1];
        rgb_out <= de_d[1] ? adjusted_rgb : 24'h000000;
    end
end

endmodule
