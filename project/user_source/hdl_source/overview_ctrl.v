`timescale 1ns/1ps

// 2x2 overview state.  SW1 on (while resident) enters the overview with the gold
// frame on the current page; KEY1 moves the frame, KEY2 jumps to the selected page
// and leaves the overview, SW1 off leaves it without changing the page.  Emergency
// and the quality panel pre-empt it.
module overview_ctrl #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       sw1,
    input  wire       emergency_mode,
    input  wire       settings_mode,
    input  wire       transition_busy,
    input  wire [1:0] current_index,
    input  wire       move_pulse,
    input  wire       confirm_pulse,
    output reg        overview_mode,
    output reg  [1:0] selected_index,
    output reg        jump_req,
    output reg  [1:0] jump_index,
    output reg        wipe_toggle      // flips on every confirm: the full-screen wipe starts
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
localparam integer COUNT_WIDTH = (DEBOUNCE_CYCLES > 1) ? $clog2(DEBOUNCE_CYCLES) : 1;

(* async_reg = "true" *) reg [1:0] sw1_sync;
reg sw1_stable;
reg sw1_previous;
reg [COUNT_WIDTH-1:0] count;
wire sw1_rise = sw1_stable && !sw1_previous;
wire sw1_fall = !sw1_stable && sw1_previous;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sw1_sync <= 2'b00;
        sw1_stable <= 1'b0;
        sw1_previous <= 1'b0;
        count <= 0;
        overview_mode <= 1'b0;
        selected_index <= 2'd0;
        jump_req <= 1'b0;
        jump_index <= 2'd0;
        wipe_toggle <= 1'b0;
    end else begin
        sw1_sync <= {sw1_sync[0], sw1};
        sw1_previous <= sw1_stable;
        jump_req <= 1'b0;
        if (sw1_sync[1] == sw1_stable)
            count <= 0;
        else if (count == DEBOUNCE_CYCLES - 1) begin
            sw1_stable <= sw1_sync[1];
            count <= 0;
        end else
            count <= count + 1'b1;

        if (emergency_mode || settings_mode) begin
            overview_mode <= 1'b0;
        end else if (overview_mode) begin
            if (sw1_fall)
                overview_mode <= 1'b0;
            else if (move_pulse)
                selected_index <= selected_index + 1'b1;
            else if (confirm_pulse) begin
                overview_mode <= 1'b0;
                jump_req <= 1'b1;
                jump_index <= selected_index;
                wipe_toggle <= ~wipe_toggle;
            end
        end else if (sw1_rise && !transition_busy) begin
            overview_mode <= 1'b1;
            selected_index <= current_index;
        end
    end
end

endmodule
