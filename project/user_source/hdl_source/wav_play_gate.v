`timescale 1ns/1ps

module wav_play_gate (
    input  wire clk,
    input  wire rst,
    input  wire switch_in,
    input  wire playback_allowed,
    output wire wav_selected,
    output wire play_enable
);

(* async_reg = "true" *) reg [1:0] switch_sync;
reg [1:0] startup_count;
reg       switch_armed;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        switch_sync   <= 2'b00;
        startup_count <= 2'b00;
        switch_armed  <= 1'b0;
    end else begin
        switch_sync <= {switch_sync[0], switch_in};

        if (startup_count != 2'b11)
            startup_count <= startup_count + 1'b1;
        else if (!switch_sync[1])
            switch_armed <= 1'b1;
    end
end

assign wav_selected = switch_armed && switch_sync[1];
assign play_enable  = wav_selected && playback_allowed;

endmodule
