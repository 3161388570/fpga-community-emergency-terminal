`timescale 1ns/1ps

module emergency_mode_control #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
) (
    input  wire clk,
    input  wire rst,
    input  wire emergency_switch,
    output wire emergency_mode
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
localparam integer COUNT_WIDTH = (DEBOUNCE_CYCLES > 1) ?
                                 $clog2(DEBOUNCE_CYCLES) : 1;

(* async_reg = "true" *) reg [1:0] switch_sync;
reg switch_stable;
reg [COUNT_WIDTH-1:0] debounce_count;

assign emergency_mode = switch_stable;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        switch_sync <= 2'b00;
        switch_stable <= 1'b0;
        debounce_count <= {COUNT_WIDTH{1'b0}};
    end else begin
        switch_sync <= {switch_sync[0], emergency_switch};
        if (switch_sync[1] == switch_stable) begin
            debounce_count <= {COUNT_WIDTH{1'b0}};
        end else if (debounce_count == DEBOUNCE_CYCLES - 1) begin
            switch_stable <= switch_sync[1];
            debounce_count <= {COUNT_WIDTH{1'b0}};
        end else begin
            debounce_count <= debounce_count + 1'b1;
        end
    end
end

endmodule
