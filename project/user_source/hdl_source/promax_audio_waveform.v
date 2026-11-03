`timescale 1ns/1ps

module promax_audio_waveform #(
    parameter integer WAVE_X = 48,
    parameter integer WAVE_Y = 350,
    parameter integer WAVE_WIDTH = 512,
    parameter integer WAVE_HEIGHT = 64,
    parameter integer BAND_COUNT = 64 // supported layouts: 16 or 64
) (
    input wire clk, rst, vsync, de,
    input wire [23:0] rgb_in,
    input wire enable, pcm_tap_valid,
    input wire signed [15:0] pcm_tap_left,
    output wire [23:0] rgb_out,
    output wire waveform_active
);

localparam integer BAR_SPACING = WAVE_WIDTH / BAND_COUNT;
localparam integer BAR_WIDTH = BAR_SPACING - ((BAND_COUNT == 64) ? 2 : 6);
localparam integer BAR_BOTTOM = WAVE_Y + WAVE_HEIGHT - 1;
reg [9:0] pixel_x,pixel_y;
reg de_q,vs_q;
wire frame_start=vs_q && !vsync;
(* async_reg = "true" *) reg [1:0] enable_sync;
reg frame_enable;
reg collecting,analysis_active;
reg [7:0] input_index,output_index,result_sequence,display_sequence;
// Two complete spectrum banks. Every bin group is written once per FFT;
// no resettable 64-way accumulators or parallel bank copying are needed.
reg [5:0] target_mem[0:127];
reg write_bank,ready_bank;
reg [5:0] accumulating_band,accumulating_peak;
// Display height and held peak share a small distributed memory.
reg [11:0] history[0:63];
reg history_valid,update_active,update_fresh,update_bank,update_initialized;
reg [5:0] update_index;
reg [1:0] decay_divider;
reg update_decay;
function [7:0] hann_coefficient;
    input [7:0] sample_index;
    reg [6:0] mirrored_index;
    begin
        mirrored_index = sample_index[7] ? 8'd255 - sample_index : sample_index[6:0];
        case (mirrored_index)
            7'd0: hann_coefficient = 8'd0;
            7'd1: hann_coefficient = 8'd0;
            7'd2: hann_coefficient = 8'd0;
            7'd3: hann_coefficient = 8'd0;
            7'd4: hann_coefficient = 8'd1;
            7'd5: hann_coefficient = 8'd1;
            7'd6: hann_coefficient = 8'd1;
            7'd7: hann_coefficient = 8'd2;
            7'd8: hann_coefficient = 8'd2;
            7'd9: hann_coefficient = 8'd3;
            7'd10: hann_coefficient = 8'd4;
            7'd11: hann_coefficient = 8'd5;
            7'd12: hann_coefficient = 8'd6;
            7'd13: hann_coefficient = 8'd6;
            7'd14: hann_coefficient = 8'd8;
            7'd15: hann_coefficient = 8'd9;
            7'd16: hann_coefficient = 8'd10;
            7'd17: hann_coefficient = 8'd11;
            7'd18: hann_coefficient = 8'd12;
            7'd19: hann_coefficient = 8'd14;
            7'd20: hann_coefficient = 8'd15;
            7'd21: hann_coefficient = 8'd17;
            7'd22: hann_coefficient = 8'd18;
            7'd23: hann_coefficient = 8'd20;
            7'd24: hann_coefficient = 8'd22;
            7'd25: hann_coefficient = 8'd23;
            7'd26: hann_coefficient = 8'd25;
            7'd27: hann_coefficient = 8'd27;
            7'd28: hann_coefficient = 8'd29;
            7'd29: hann_coefficient = 8'd31;
            7'd30: hann_coefficient = 8'd33;
            7'd31: hann_coefficient = 8'd35;
            7'd32: hann_coefficient = 8'd38;
            7'd33: hann_coefficient = 8'd40;
            7'd34: hann_coefficient = 8'd42;
            7'd35: hann_coefficient = 8'd45;
            7'd36: hann_coefficient = 8'd47;
            7'd37: hann_coefficient = 8'd49;
            7'd38: hann_coefficient = 8'd52;
            7'd39: hann_coefficient = 8'd54;
            7'd40: hann_coefficient = 8'd57;
            7'd41: hann_coefficient = 8'd60;
            7'd42: hann_coefficient = 8'd62;
            7'd43: hann_coefficient = 8'd65;
            7'd44: hann_coefficient = 8'd68;
            7'd45: hann_coefficient = 8'd71;
            7'd46: hann_coefficient = 8'd73;
            7'd47: hann_coefficient = 8'd76;
            7'd48: hann_coefficient = 8'd79;
            7'd49: hann_coefficient = 8'd82;
            7'd50: hann_coefficient = 8'd85;
            7'd51: hann_coefficient = 8'd88;
            7'd52: hann_coefficient = 8'd91;
            7'd53: hann_coefficient = 8'd94;
            7'd54: hann_coefficient = 8'd97;
            7'd55: hann_coefficient = 8'd100;
            7'd56: hann_coefficient = 8'd103;
            7'd57: hann_coefficient = 8'd106;
            7'd58: hann_coefficient = 8'd109;
            7'd59: hann_coefficient = 8'd113;
            7'd60: hann_coefficient = 8'd116;
            7'd61: hann_coefficient = 8'd119;
            7'd62: hann_coefficient = 8'd122;
            7'd63: hann_coefficient = 8'd125;
            7'd64: hann_coefficient = 8'd128;
            7'd65: hann_coefficient = 8'd131;
            7'd66: hann_coefficient = 8'd135;
            7'd67: hann_coefficient = 8'd138;
            7'd68: hann_coefficient = 8'd141;
            7'd69: hann_coefficient = 8'd144;
            7'd70: hann_coefficient = 8'd147;
            7'd71: hann_coefficient = 8'd150;
            7'd72: hann_coefficient = 8'd153;
            7'd73: hann_coefficient = 8'd156;
            7'd74: hann_coefficient = 8'd159;
            7'd75: hann_coefficient = 8'd162;
            7'd76: hann_coefficient = 8'd165;
            7'd77: hann_coefficient = 8'd168;
            7'd78: hann_coefficient = 8'd171;
            7'd79: hann_coefficient = 8'd174;
            7'd80: hann_coefficient = 8'd177;
            7'd81: hann_coefficient = 8'd180;
            7'd82: hann_coefficient = 8'd183;
            7'd83: hann_coefficient = 8'd186;
            7'd84: hann_coefficient = 8'd189;
            7'd85: hann_coefficient = 8'd191;
            7'd86: hann_coefficient = 8'd194;
            7'd87: hann_coefficient = 8'd197;
            7'd88: hann_coefficient = 8'd199;
            7'd89: hann_coefficient = 8'd202;
            7'd90: hann_coefficient = 8'd204;
            7'd91: hann_coefficient = 8'd207;
            7'd92: hann_coefficient = 8'd209;
            7'd93: hann_coefficient = 8'd212;
            7'd94: hann_coefficient = 8'd214;
            7'd95: hann_coefficient = 8'd216;
            7'd96: hann_coefficient = 8'd218;
            7'd97: hann_coefficient = 8'd221;
            7'd98: hann_coefficient = 8'd223;
            7'd99: hann_coefficient = 8'd225;
            7'd100: hann_coefficient = 8'd227;
            7'd101: hann_coefficient = 8'd229;
            7'd102: hann_coefficient = 8'd231;
            7'd103: hann_coefficient = 8'd232;
            7'd104: hann_coefficient = 8'd234;
            7'd105: hann_coefficient = 8'd236;
            7'd106: hann_coefficient = 8'd238;
            7'd107: hann_coefficient = 8'd239;
            7'd108: hann_coefficient = 8'd241;
            7'd109: hann_coefficient = 8'd242;
            7'd110: hann_coefficient = 8'd243;
            7'd111: hann_coefficient = 8'd245;
            7'd112: hann_coefficient = 8'd246;
            7'd113: hann_coefficient = 8'd247;
            7'd114: hann_coefficient = 8'd248;
            7'd115: hann_coefficient = 8'd249;
            7'd116: hann_coefficient = 8'd250;
            7'd117: hann_coefficient = 8'd251;
            7'd118: hann_coefficient = 8'd252;
            7'd119: hann_coefficient = 8'd252;
            7'd120: hann_coefficient = 8'd253;
            7'd121: hann_coefficient = 8'd253;
            7'd122: hann_coefficient = 8'd254;
            7'd123: hann_coefficient = 8'd254;
            7'd124: hann_coefficient = 8'd255;
            7'd125: hann_coefficient = 8'd255;
            7'd126: hann_coefficient = 8'd255;
            7'd127: hann_coefficient = 8'd255;
            default: hann_coefficient = 8'd0;
        endcase
    end
endfunction
function [5:0] frequency_band;
    input [7:0] bin_index;
    begin
        if (BAND_COUNT == 64) begin
            // 1..32: one bin per bar; 33..64: two; 65..127: four.
            // Finer low-frequency coverage without duplicating FFT bins.
            if (bin_index == 0) frequency_band = 0;
            else if (bin_index <= 32) frequency_band = bin_index - 1'b1;
            else if (bin_index <= 64) frequency_band = 32 + ((bin_index - 33) >> 1);
            else frequency_band = 48 + ((bin_index - 65) >> 2);
        end else if (bin_index <= 1) frequency_band = 4'd0;
        else if (bin_index == 2)  frequency_band = 4'd1;
        else if (bin_index == 3)  frequency_band = 4'd2;
        else if (bin_index <= 5)  frequency_band = 4'd3;
        else if (bin_index <= 7)  frequency_band = 4'd4;
        else if (bin_index <= 11) frequency_band = 4'd5;
        else if (bin_index <= 15) frequency_band = 4'd6;
        else if (bin_index <= 23) frequency_band = 4'd7;
        else if (bin_index <= 31) frequency_band = 4'd8;
        else if (bin_index <= 42) frequency_band = 4'd9;
        else if (bin_index <= 55) frequency_band = 4'd10;
        else if (bin_index <= 69) frequency_band = 4'd11;
        else if (bin_index <= 84) frequency_band = 4'd12;
        else if (bin_index <= 99) frequency_band = 4'd13;
        else if (bin_index <= 113) frequency_band = 4'd14;
        else                       frequency_band = 4'd15;
    end
endfunction
function [5:0] magnitude_level;
    input [24:0] magnitude;
    integer bit_index, exponent, scaled;
    reg [24:0] fraction;
    begin
        exponent = 0;
        for (bit_index = 0; bit_index < 25; bit_index = bit_index + 1)
            if (magnitude[bit_index])
                exponent = bit_index;
        fraction = magnitude >> (exponent > 1 ? exponent - 2 : 0);
        scaled = (exponent - 9) * 4 + fraction[1:0];
        if (magnitude < 25'd1024)
            magnitude_level = 6'd0;
        else if (scaled > 63)
            magnitude_level = 6'd63;
        else
            magnitude_level = scaled[5:0];
    end
endfunction
wire fft_done,fft_busy,fft_dout_en;
wire signed [23:0] fft_dout_re,fft_dout_im;
wire accept_sample=pcm_tap_valid && (collecting || (!analysis_active && !fft_busy && enable_sync[1]));
wire fft_start=accept_sample && !collecting;
wire signed [8:0] signed_window_gain={1'b0,hann_coefficient(input_index)};
wire signed [15:0] analysis_sample=accept_sample?pcm_tap_left:16'sd0;
wire signed [24:0] window_product=analysis_sample*signed_window_gain;
wire signed [15:0] fft_din_re=window_product >>> 8;
spectrum_fft_256 u_spectrum_fft_256(.clk(clk),.rst(rst),.mode(1'b0),
 .start(fft_start),.din_en(accept_sample),.din_re(fft_din_re),.din_im(16'd0),
 .done(fft_done),.busy(fft_busy),.dout_en(fft_dout_en),.dout_re(fft_dout_re),.dout_im(fft_dout_im));
wire [23:0] fft_abs_re=fft_dout_re[23]?(~fft_dout_re+1'b1):fft_dout_re;
wire [23:0] fft_abs_im=fft_dout_im[23]?(~fft_dout_im+1'b1):fft_dout_im;
wire [24:0] fft_magnitude={1'b0,fft_abs_re}+{1'b0,fft_abs_im};
wire [5:0] fft_level=magnitude_level(fft_magnitude);
wire [5:0] current_band=frequency_band(output_index);
wire [5:0] combined_peak=(fft_level>accumulating_peak)?fft_level:accumulating_peak;
always @(posedge clk) begin
 if(!rst && fft_dout_en && output_index>0 && output_index<128) begin
  if(current_band!=accumulating_band)
   target_mem[{write_bank,accumulating_band}]<=accumulating_peak;
  else if(output_index==127)
   target_mem[{write_bank,current_band}]<=combined_peak;
 end
end
always @(posedge clk or posedge rst) begin
 if(rst) begin
  collecting<=0;analysis_active<=0;input_index<=0;output_index<=0;
  result_sequence<=0;write_bank<=0;ready_bank<=0;
  accumulating_band<=0;accumulating_peak<=0;
 end else begin
  if(fft_start)analysis_active<=1;
  if(accept_sample) begin
   if(!collecting) begin collecting<=1;input_index<=1;end
   else if(input_index==255)begin collecting<=0;input_index<=0;end
   else input_index<=input_index+1'b1;
  end
  if(fft_dout_en)begin
   if(output_index==0)begin accumulating_band<=0;accumulating_peak<=0;end
   else if(output_index<128)begin
    accumulating_band<=current_band;
    accumulating_peak<=(current_band==accumulating_band)?combined_peak:fft_level;
   end
   if(output_index==127)begin
    ready_bank<=write_bank;write_bank<=~write_bank;result_sequence<=result_sequence+1'b1;
   end
   if(output_index==255)begin output_index<=0;analysis_active<=0;end
   else output_index<=output_index+1'b1;
  end
 end
end
wire [11:0] update_old=update_initialized?history[update_index]:12'd0;
wire [5:0] update_target=target_mem[{update_bank,update_index}];
wire [5:0] old_level=update_old[5:0],old_peak=update_old[11:6];
wire [5:0] next_level=(update_fresh && update_target>=old_level)?update_target:
                            ((old_level!=0)?old_level-1'b1:6'd0);
wire [5:0] next_peak=(update_fresh && update_target>=old_peak)?update_target:
                            ((update_decay && old_peak!=0)?old_peak-1'b1:old_peak);
always @(posedge clk)begin
 if(!rst && update_active)history[update_index]<={next_peak,next_level};
end
always @(posedge clk or posedge rst)begin
 if(rst)begin
  pixel_x<=0;pixel_y<=0;de_q<=0;vs_q<=1;enable_sync<=0;frame_enable<=0;
  display_sequence<=0;history_valid<=0;update_active<=0;update_index<=0;
  update_fresh<=0;update_bank<=0;update_initialized<=0;decay_divider<=0;update_decay<=0;
 end else begin
  de_q<=de;vs_q<=vsync;enable_sync<={enable_sync[0],enable};
  if(frame_start)begin
   pixel_x<=0;pixel_y<=0;frame_enable<=enable_sync[1];
   display_sequence<=result_sequence;
   update_active<=enable_sync[1];update_index<=0;
   update_bank<=ready_bank;update_fresh<=(result_sequence!=display_sequence);
   update_initialized<=history_valid;update_decay<=(decay_divider==0);
   decay_divider<=decay_divider+1'b1;
  end else begin
   if(!de)pixel_x<=0;else if(pixel_x<639)pixel_x<=pixel_x+1'b1;
   if(de_q && !de && pixel_y<479)pixel_y<=pixel_y+1'b1;
   if(update_active)begin
    if(update_index==BAND_COUNT-1)begin update_active<=0;history_valid<=1;end
    else update_index<=update_index+1'b1;
   end
  end
 end
end
wire [9:0] relative_x=pixel_x-WAVE_X;
wire [5:0] bar_index=relative_x/BAR_SPACING;
wire [9:0] bar_offset=relative_x%BAR_SPACING;
wire [11:0] shown=history_valid?history[bar_index]:12'd0;
wire [5:0] shown_level=shown[5:0],shown_peak=shown[11:6];
wire [15:0] level_scaled=shown_level*WAVE_HEIGHT;
wire [15:0] peak_scaled=shown_peak*WAVE_HEIGHT;
wire [9:0] bar_top=BAR_BOTTOM-(level_scaled>>6);
wire [9:0] peak_y=BAR_BOTTOM-(peak_scaled>>6);
wire in_region=frame_enable && de && pixel_x>=WAVE_X && pixel_x<WAVE_X+WAVE_WIDTH && pixel_y>=WAVE_Y && pixel_y<WAVE_Y+WAVE_HEIGHT;
wire bar_pixel=in_region && bar_offset<BAR_WIDTH && shown_level!=0 && pixel_y>=bar_top && pixel_y<BAR_BOTTOM;
wire peak_pixel=in_region && bar_offset<BAR_WIDTH && shown_peak!=0 && pixel_y>=peak_y && pixel_y<=peak_y+1;
wire baseline_pixel=in_region && pixel_y==BAR_BOTTOM;
wire [23:0] bar_color=(pixel_y<WAVE_Y+WAVE_HEIGHT/3)?24'hff5040:
                     (pixel_y<WAVE_Y+(WAVE_HEIGHT*2)/3)?24'hffd040:24'h20e080;
assign waveform_active=bar_pixel || peak_pixel || baseline_pixel;
assign rgb_out=peak_pixel?24'hf0f0f0:bar_pixel?bar_color:baseline_pixel?24'h406060:rgb_in;
endmodule
