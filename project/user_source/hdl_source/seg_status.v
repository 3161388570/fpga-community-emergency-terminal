`timescale 1ns/1ps

// Eight-digit status text, left to right (pos0 = leftmost).  Bit 7 is the decimal
// point and every segment is active low, like seg_decoder.
//   resident  n-01b2c2   overview  n-ALb2c2   quality  SE- b5c3 (dp on the item being edited)
//   emergency E-119---   (whole group blinks at 1 Hz)   TF not ready  n-------
module seg_status #(
    parameter integer CLK_FREQ_HZ = 50_000_000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       emergency,
    input  wire       settings,
    input  wire       selected_param,
    input  wire       overview,
    input  wire       ready,
    input  wire [1:0] page,
    input  wire [2:0] brightness,
    input  wire [2:0] contrast,
    output reg  [7:0] pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos7
);

localparam [7:0] BLANK = 8'hFF, DASH = 8'hBF;
localparam [7:0] CH_N = 8'hAB, CH_B = 8'h83, CH_C = 8'hA7, CH_E = 8'h86, CH_A = 8'h88,
                 CH_L = 8'hC7, CH_S = 8'h92;

function [7:0] digit;
    input [3:0] v;
    begin
        case (v)
            4'd0: digit = 8'hC0;  4'd1: digit = 8'hF9;  4'd2: digit = 8'hA4;  4'd3: digit = 8'hB0;
            4'd4: digit = 8'h99;  4'd5: digit = 8'h92;  4'd6: digit = 8'h82;  4'd7: digit = 8'hF8;
            4'd8: digit = 8'h80;  4'd9: digit = 8'h90;  default: digit = 8'hFF;
        endcase
    end
endfunction

(* async_reg = "true" *) reg [1:0] emg_s, set_s, sel_s, ov_s, rdy_s;
reg [1:0] page_s0, page_s1;
reg [2:0] b_s0, b_s1, c_s0, c_s1;
reg [31:0] blink_cnt;
reg        blink_on;
reg        emg_prev;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        emg_s <= 0; set_s <= 0; sel_s <= 0; ov_s <= 0; rdy_s <= 0;
        page_s0 <= 0; page_s1 <= 0; b_s0 <= 3'd2; b_s1 <= 3'd2; c_s0 <= 3'd2; c_s1 <= 3'd2;
        blink_cnt <= 0; blink_on <= 1'b1; emg_prev <= 1'b0;
        pos0 <= BLANK; pos1 <= BLANK; pos2 <= BLANK; pos3 <= BLANK;
        pos4 <= BLANK; pos5 <= BLANK; pos6 <= BLANK; pos7 <= BLANK;
    end else begin
        emg_s <= {emg_s[0], emergency};   set_s <= {set_s[0], settings};
        sel_s <= {sel_s[0], selected_param}; ov_s <= {ov_s[0], overview};
        rdy_s <= {rdy_s[0], ready};
        page_s0 <= page; page_s1 <= page_s0;
        b_s0 <= brightness; b_s1 <= b_s0; c_s0 <= contrast; c_s1 <= c_s0;
        emg_prev <= emg_s[1];
        if (emg_s[1] && !emg_prev) begin
            blink_cnt <= 0;
            blink_on <= 1'b1;
        end else if (blink_cnt == CLK_FREQ_HZ/2 - 1) begin
            blink_cnt <= 0;
            blink_on <= ~blink_on;
        end else
            blink_cnt <= blink_cnt + 1'b1;

        if (emg_s[1]) begin
            if (blink_on) begin
                pos0 <= CH_E; pos1 <= DASH; pos2 <= digit(4'd1); pos3 <= digit(4'd1);
                pos4 <= digit(4'd9); pos5 <= DASH; pos6 <= DASH; pos7 <= DASH;
            end else begin
                pos0 <= BLANK; pos1 <= BLANK; pos2 <= BLANK; pos3 <= BLANK;
                pos4 <= BLANK; pos5 <= BLANK; pos6 <= BLANK; pos7 <= BLANK;
            end
        end else if (set_s[1]) begin
            pos0 <= CH_S; pos1 <= CH_E; pos2 <= DASH; pos3 <= BLANK;
            pos4 <= CH_B; pos5 <= digit({1'b0, b_s1}) & (sel_s[1] ? 8'hFF : 8'h7F);
            pos6 <= CH_C; pos7 <= digit({1'b0, c_s1}) & (sel_s[1] ? 8'h7F : 8'hFF);
        end else if (!rdy_s[1]) begin
            pos0 <= CH_N; pos1 <= DASH; pos2 <= DASH; pos3 <= DASH;
            pos4 <= DASH; pos5 <= DASH; pos6 <= DASH; pos7 <= DASH;
        end else begin
            pos0 <= CH_N; pos1 <= DASH;
            if (ov_s[1]) begin
                pos2 <= CH_A; pos3 <= CH_L;
            end else begin
                pos2 <= digit(4'd0); pos3 <= digit({2'b00, page_s1} + 4'd1);
            end
            pos4 <= CH_B; pos5 <= digit({1'b0, b_s1});
            pos6 <= CH_C; pos7 <= digit({1'b0, c_s1});
        end
    end
end

endmodule
