`timescale 1ns/1ps

module theme_key_control #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_PRESS_MS = 800
) (
    input  wire clk,
    input  wire rst,
    input  wire key1_n,
    input  wire key2_n,
    input  wire settings_mode,
    input  wire emergency_mode,
    input  wire overview_mode,
    input  wire lock,
    output reg  ov_move_pulse,
    output reg  ov_confirm_pulse,
    output reg  next_pulse,
    output reg  auto_short_pulse,
    output reg  previous_pulse,
    output reg  decrease_pulse,
    output reg  increase_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
localparam integer DEBOUNCE_WIDTH = (DEBOUNCE_CYCLES > 1) ? $clog2(DEBOUNCE_CYCLES) : 1;
localparam integer LONG_PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
localparam integer LONG_PRESS_WIDTH = (LONG_PRESS_CYCLES > 1) ? $clog2(LONG_PRESS_CYCLES) : 1;

(* async_reg = "true" *) reg [1:0] key1_sync;
(* async_reg = "true" *) reg [1:0] key2_sync;
reg key1_stable;
reg key2_stable;
reg [DEBOUNCE_WIDTH-1:0] key1_count;
reg [DEBOUNCE_WIDTH-1:0] key2_count;
reg [LONG_PRESS_WIDTH-1:0] key2_hold_count;
reg key2_held;
reg key2_long_sent;
reg key2_normal_context;

wire key1_pressed = key1_stable && !key1_sync[1] &&
                    key1_count == DEBOUNCE_CYCLES - 1;
wire key2_pressed = key2_stable && !key2_sync[1] &&
                    key2_count == DEBOUNCE_CYCLES - 1;
wire key2_released = !key2_stable && key2_sync[1] &&
                     key2_count == DEBOUNCE_CYCLES - 1;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        key1_sync <= 2'b11;
        key2_sync <= 2'b11;
        key1_stable <= 1'b1;
        key2_stable <= 1'b1;
        key1_count <= 0;
        key2_count <= 0;
        key2_hold_count <= 0;
        key2_held <= 1'b0;
        key2_long_sent <= 1'b0;
        key2_normal_context <= 1'b0;
        next_pulse <= 1'b0;
        auto_short_pulse <= 1'b0;
        previous_pulse <= 1'b0;
        decrease_pulse <= 1'b0;
        increase_pulse <= 1'b0;
        ov_move_pulse <= 1'b0;
        ov_confirm_pulse <= 1'b0;
    end else begin
        key1_sync <= {key1_sync[0], key1_n};
        key2_sync <= {key2_sync[0], key2_n};
        next_pulse <= 1'b0;
        auto_short_pulse <= 1'b0;
        previous_pulse <= 1'b0;
        decrease_pulse <= 1'b0;
        increase_pulse <= 1'b0;
        ov_move_pulse <= 1'b0;
        ov_confirm_pulse <= 1'b0;

        if (key1_sync[1] == key1_stable)
            key1_count <= 0;
        else if (key1_count == DEBOUNCE_CYCLES - 1) begin
            key1_stable <= key1_sync[1];
            key1_count <= 0;
        end else
            key1_count <= key1_count + 1'b1;

        if (key2_sync[1] == key2_stable)
            key2_count <= 0;
        else if (key2_count == DEBOUNCE_CYCLES - 1) begin
            key2_stable <= key2_sync[1];
            key2_count <= 0;
        end else
            key2_count <= key2_count + 1'b1;

        if (key1_pressed && !emergency_mode && !lock) begin
            if (settings_mode)
                decrease_pulse <= 1'b1;
            else if (overview_mode)
                ov_move_pulse <= 1'b1;
            else
                next_pulse <= 1'b1;
        end

        if (key2_pressed && !lock) begin
            key2_held <= 1'b1;
            key2_long_sent <= 1'b0;
            key2_hold_count <= 0;
            key2_normal_context <= !settings_mode && !emergency_mode && !overview_mode;
            if (settings_mode && !emergency_mode)
                increase_pulse <= 1'b1;
            else if (overview_mode && !emergency_mode)
                ov_confirm_pulse <= 1'b1;
        end else if (key2_held && !key2_stable) begin
            // hold timer: paused while the keys are locked, restarted on unlock
            if (lock)
                key2_hold_count <= 0;
            else if (key2_hold_count < LONG_PRESS_CYCLES - 1)
                key2_hold_count <= key2_hold_count + 1'b1;
        end

        if (key2_released) begin
            if (key2_held && key2_normal_context && !emergency_mode && !lock) begin
                if (key2_hold_count >= LONG_PRESS_CYCLES - 1)
                    previous_pulse <= 1'b1;      // long press: fires on release, no short press
                else
                    auto_short_pulse <= 1'b1;
            end
            key2_held <= 1'b0;
            key2_long_sent <= 1'b0;
            key2_hold_count <= 0;
            key2_normal_context <= 1'b0;
        end
    end
end

endmodule
