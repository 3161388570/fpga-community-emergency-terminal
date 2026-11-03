`timescale 1ns/1ps

// Test-build only: replays a fixed timeline of switch and key actions so the board can
// be exercised while only the HDMI output is observed.  Seconds are counted from reset
// release on the 50 MHz clock.  Keys are active low like the board keys.
module test_sequencer #(
    parameter integer CLK_FREQ_HZ = 50_000_000
) (
    input  wire clk,
    input  wire rst_n,
    output reg  sw1,
    output reg  sw2,
    output reg  sw3,
    output reg  sw4,
    output reg  key1,
    output reg  key2,
    output reg  key3
);

reg [31:0] tick;
reg [15:0] sec;
reg [7:0]  tenth;       // 0.1 s steps inside the second
reg [31:0] sub;
wire [31:0] t10 = sec * 10 + tenth;      // time in 0.1 s

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sub <= 0; tenth <= 0; sec <= 0;
    end else begin
        if (sub == CLK_FREQ_HZ/10 - 1) begin
            sub <= 0;
            if (tenth == 9) begin tenth <= 0; sec <= sec + 1'b1; end
            else tenth <= tenth + 1'b1;
        end else sub <= sub + 1'b1;
    end
end

function in;
    input [31:0] t;
    input [31:0] from;
    input [31:0] to;     // [from, to) in 0.1 s
    begin
        in = (t >= from) && (t < to);
    end
endfunction

always @(*) begin
    sw1 = 1'b0; sw2 = 1'b0; sw3 = 1'b0; sw4 = 1'b0;
    key1 = 1'b1; key2 = 1'b1; key3 = 1'b1;
    // 30 s : KEY1 -> next page (fade)
    if (in(t10, 300, 303)) key1 = 1'b0;
    // 40 s : KEY1 again
    if (in(t10, 400, 403)) key1 = 1'b0;
    // 50 s : SW1 on = overview; 54 s KEY1 moves frame; 58 s KEY2 confirms; 62 s SW1 off
    if (in(t10, 500, 620)) sw1 = 1'b1;
    if (in(t10, 540, 543)) key1 = 1'b0;
    if (in(t10, 580, 583)) key2 = 1'b0;
    // 70 s : SW4 on = quality panel; KEY2 x3 raises brightness; SW1 picks contrast, KEY1 lowers it
    if (in(t10, 700, 900)) sw4 = 1'b1;
    if (in(t10, 720, 723)) key2 = 1'b0;
    if (in(t10, 740, 743)) key2 = 1'b0;
    if (in(t10, 760, 763)) key2 = 1'b0;
    if (in(t10, 780, 860)) sw1 = 1'b1;
    if (in(t10, 800, 803)) key1 = 1'b0;
    if (in(t10, 820, 823)) key1 = 1'b0;
    // 90 s : SW4 off -> panel fades out
    // 100 s : SW3 on = emergency for 12 s, then dissolve back
    if (in(t10, 1000, 1120)) sw3 = 1'b1;
    // 130 s : KEY2 long press (1.3 s) = previous page
    if (in(t10, 1300, 1313)) key2 = 1'b0;
    // 140 s : KEY2 short = auto-play on
    if (in(t10, 1400, 1403)) key2 = 1'b0;
end

endmodule
