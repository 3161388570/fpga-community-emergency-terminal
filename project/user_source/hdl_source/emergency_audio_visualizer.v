`timescale 1ns/1ps
// STYLE: 1 time domain, 2 FFT spectrum. Both use the selected HDMI PCM.
// Combinational pixel overlay: no extra HS/VS/DE pipeline delay.
module emergency_audio_visualizer #(parameter integer STYLE=1)(
 input wire clk,rst,enable,vsync,de,
 input wire [23:0] rgb_in,
 input wire pcm_valid,
 input wire signed [15:0] pcm,
 output wire [23:0] rgb_out
);
generate if(STYLE==1) begin:g_wave
 emergency_audio_wave u_wave(.clk(clk),.rst(rst),.enable(enable),.vsync(vsync),.de(de),
 .rgb_in(rgb_in),.pcm_valid(pcm_valid),.pcm(pcm),.rgb_out(rgb_out));
end else if(STYLE==2) begin:g_spectrum
 // Clear stalled FFT captures on source dropout. Zero-valued valid PCM is
 // processed normally and naturally produces a zero spectrum.
 reg [20:0] idle;
 always @(posedge clk) begin
   if(rst || !enable || pcm_valid) idle<=0;
   else if(idle<21'd1250000) idle<=idle+1'b1;
 end
 wire clear_analysis=rst || !enable || idle==21'd1250000;
 promax_audio_waveform #(.WAVE_X(64),.WAVE_Y(356),.WAVE_WIDTH(512),.WAVE_HEIGHT(52)) u_fft_display(
 .clk(clk),.rst(clear_analysis),.vsync(vsync),.de(de),.rgb_in(rgb_in),
 .enable(enable),.pcm_tap_valid(pcm_valid),.pcm_tap_left(pcm),.rgb_out(rgb_out),.waveform_active());
end else begin:g_off
 assign rgb_out=rgb_in;
end endgenerate
endmodule
