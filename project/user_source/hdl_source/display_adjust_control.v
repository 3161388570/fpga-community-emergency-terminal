`timescale 1ns/1ps

module display_adjust_control #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer EXTERNAL_KEY_EVENTS = 0
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       settings_switch,
    input  wire       select_switch,
    input  wire       key_decrease_n,
    input  wire       key_increase_n,
    input  wire       decrease_event,
    input  wire       increase_event,
    output wire       settings_mode,
    output wire       selected_param,
    output reg  [2:0] brightness_level,
    output reg  [2:0] contrast_level,
    output reg        cfg_toggle,
    output reg        navigation_enabled
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
localparam integer COUNT_WIDTH = (DEBOUNCE_CYCLES > 1) ?
                                 $clog2(DEBOUNCE_CYCLES) : 1;

(* async_reg = "true" *) reg [1:0] mode_sync;
(* async_reg = "true" *) reg [1:0] select_sync;
(* async_reg = "true" *) reg [1:0] decrease_sync;
(* async_reg = "true" *) reg [1:0] increase_sync;
reg mode_stable;
reg select_stable;
reg decrease_stable;
reg increase_stable;
reg [COUNT_WIDTH-1:0] mode_count;
reg [COUNT_WIDTH-1:0] select_count;
reg [COUNT_WIDTH-1:0] decrease_count;
reg [COUNT_WIDTH-1:0] increase_count;
reg decrease_previous;
reg increase_previous;

wire legacy_decrease_press = decrease_previous && !decrease_stable;
wire legacy_increase_press = increase_previous && !increase_stable;
wire decrease_press = EXTERNAL_KEY_EVENTS ? decrease_event : legacy_decrease_press;
wire increase_press = EXTERNAL_KEY_EVENTS ? increase_event : legacy_increase_press;

assign settings_mode = mode_stable;
assign selected_param = select_stable;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        mode_sync <= 2'b00;
        select_sync <= 2'b00;
        decrease_sync <= 2'b11;
        increase_sync <= 2'b11;
        mode_stable <= 1'b0;
        select_stable <= 1'b0;
        decrease_stable <= 1'b1;
        increase_stable <= 1'b1;
        mode_count <= {COUNT_WIDTH{1'b0}};
        select_count <= {COUNT_WIDTH{1'b0}};
        decrease_count <= {COUNT_WIDTH{1'b0}};
        increase_count <= {COUNT_WIDTH{1'b0}};
        decrease_previous <= 1'b1;
        increase_previous <= 1'b1;
        brightness_level <= 3'd2;
        contrast_level <= 3'd2;
        cfg_toggle <= 1'b0;
        navigation_enabled <= 1'b0;
    end else begin
        mode_sync <= {mode_sync[0], settings_switch};
        select_sync <= {select_sync[0], select_switch};
        decrease_sync <= {decrease_sync[0], key_decrease_n};
        increase_sync <= {increase_sync[0], key_increase_n};

        if (mode_sync[1] == mode_stable) begin
            mode_count <= {COUNT_WIDTH{1'b0}};
        end else if (mode_count == DEBOUNCE_CYCLES - 1) begin
            mode_stable <= mode_sync[1];
            mode_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            mode_count <= mode_count + 1'b1;
        end

        if (select_sync[1] == select_stable) begin
            select_count <= {COUNT_WIDTH{1'b0}};
        end else if (select_count == DEBOUNCE_CYCLES - 1) begin
            select_stable <= select_sync[1];
            select_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            select_count <= select_count + 1'b1;
        end

        if (decrease_sync[1] == decrease_stable) begin
            decrease_count <= {COUNT_WIDTH{1'b0}};
        end else if (decrease_count == DEBOUNCE_CYCLES - 1) begin
            decrease_stable <= decrease_sync[1];
            decrease_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            decrease_count <= decrease_count + 1'b1;
        end

        if (increase_sync[1] == increase_stable) begin
            increase_count <= {COUNT_WIDTH{1'b0}};
        end else if (increase_count == DEBOUNCE_CYCLES - 1) begin
            increase_stable <= increase_sync[1];
            increase_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            increase_count <= increase_count + 1'b1;
        end

        decrease_previous <= decrease_stable;
        increase_previous <= increase_stable;

        if (mode_stable) begin
            navigation_enabled <= 1'b0;
            if (decrease_press) begin
                if (!select_stable && brightness_level != 0) begin
                    brightness_level <= brightness_level - 1'b1;
                    cfg_toggle <= ~cfg_toggle;
                end else if (select_stable && contrast_level != 0) begin
                    contrast_level <= contrast_level - 1'b1;
                    cfg_toggle <= ~cfg_toggle;
                end
            end else if (increase_press) begin
                if (!select_stable && brightness_level != 3'd7) begin
                    brightness_level <= brightness_level + 1'b1;
                    cfg_toggle <= ~cfg_toggle;
                end else if (select_stable && contrast_level != 3'd7) begin
                    contrast_level <= contrast_level + 1'b1;
                    cfg_toggle <= ~cfg_toggle;
                end
            end
        end else begin
            navigation_enabled <= decrease_stable && increase_stable;
        end
    end
end

endmodule
