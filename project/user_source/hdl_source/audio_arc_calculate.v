module audio_arc_calculate #(
    parameter integer ACR_N = 6144
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        audio_valid,
    output reg         acr_valid,
    output reg [19:0]  acr_cts,
    output reg [19:0]  acr_n
);

localparam integer AUDIO_DIV = ACR_N >> 7;
localparam integer DIV_WIDTH = AUDIO_DIV > 1 ? $clog2(AUDIO_DIV) : 1;
reg [DIV_WIDTH-1:0] audio_div_cnt;
reg [19:0] acr_cts_cnt;
reg measurement_started;
wire audio_valid_div = audio_valid && (audio_div_cnt == AUDIO_DIV - 1);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        audio_div_cnt <= 0;
        acr_cts_cnt   <= 20'd0;
        acr_valid     <= 1'b0;
        acr_cts       <= 20'd0;
        acr_n         <= 20'd0;
        measurement_started <= 1'b0;
    end else begin
        acr_valid <= 1'b0;
        acr_cts   <= 20'd0;
        acr_n     <= 20'd0;
        if (audio_valid) begin
            if (audio_div_cnt == AUDIO_DIV - 1)
                audio_div_cnt <= 0;
            else
                audio_div_cnt <= audio_div_cnt + 1'b1;
        end
        if (audio_valid_div) begin
            measurement_started <= 1'b1;
            if (measurement_started) begin
                acr_valid <= 1'b1;
                acr_cts   <= acr_cts_cnt + 1'b1;
                acr_n     <= ACR_N;
            end
            acr_cts_cnt <= 20'd0;
        end else begin
            acr_cts_cnt <= acr_cts_cnt + 1'b1;
        end
    end
end

endmodule
