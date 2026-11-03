
module startup_pulse #(
    parameter [19:0] CNT_MAX = 20'd100000
)(
    input  wire I_clk,
    input  wire I_rst,
    output reg  O_pulse
);

reg [19:0] S_cnt;
reg        S_done;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_cnt   <= 20'd0;
        S_done  <= 1'b0;
        O_pulse <= 1'b0;
    end
    else begin
        O_pulse <= 1'b0;

        if (!S_done) begin
            if (S_cnt == CNT_MAX - 1'b1) begin
                S_cnt   <= S_cnt;
                S_done  <= 1'b1;
                O_pulse <= 1'b1;
            end
            else begin
                S_cnt <= S_cnt + 1'b1;
            end
        end
    end
end

endmodule

module boot_reset_sync (
    input wire clk,
    input wire async_rst,
    output wire rst
);
(* async_reg = "true" *) reg [1:0] release_sync = 2'b11;
always @(posedge clk or posedge async_rst) begin
    if (async_rst)
        release_sync <= 2'b11;
    else
        release_sync <= {release_sync[0], 1'b0};
end
assign rst = release_sync[1];
endmodule

module boot_reset_controller #(
    parameter STABLE_CYCLES = 1_000_000
)(
    input wire ref_clk,
    input wire sd_clk,
    input wire mem_clk,
    input wire video_clk,
    input wire rst_n,
    input wire sys_locked,
    input wire video_locked,
    input wire mem_ready,
    output wire rst_ref,
    output wire rst_sd,
    output wire rst_mem,
    output wire rst_video
);
localparam COUNT_BITS = (STABLE_CYCLES < 2) ? 1 : $clog2(STABLE_CYCLES);
wire source_reset = !rst_n || !sys_locked || !video_locked;
wire ref_reset;
wire sd_base_reset;
reg [COUNT_BITS-1:0] stable_count = 0;
reg startup_hold = 1'b1;
(* async_reg = "true" *) reg [1:0] mem_ready_sync;

boot_reset_sync ref_sync (.clk(ref_clk), .async_rst(source_reset), .rst(ref_reset));

always @(posedge ref_clk or posedge ref_reset) begin
    if (ref_reset) begin
        stable_count <= 0;
        startup_hold <= 1'b1;
    end else if (startup_hold) begin
        if (stable_count == STABLE_CYCLES - 1)
            startup_hold <= 1'b0;
        else
            stable_count <= stable_count + 1'b1;
    end
end

assign rst_ref = ref_reset || startup_hold;
boot_reset_sync mem_sync (.clk(mem_clk), .async_rst(rst_ref), .rst(rst_mem));
boot_reset_sync video_sync (.clk(video_clk), .async_rst(rst_ref), .rst(rst_video));
boot_reset_sync sd_sync (.clk(sd_clk), .async_rst(rst_ref), .rst(sd_base_reset));

always @(posedge sd_clk or posedge sd_base_reset) begin
    if (sd_base_reset)
        mem_ready_sync <= 2'b00;
    else
        mem_ready_sync <= {mem_ready_sync[0], mem_ready};
end

boot_reset_sync storage_sync (
    .clk(sd_clk),
    .async_rst(sd_base_reset || !mem_ready_sync[1]),
    .rst(rst_sd)
);
endmodule
