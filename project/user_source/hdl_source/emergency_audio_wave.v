`timescale 1ns/1ps
// 128 signed points, 3 samples per point: 8 ms at 48 kHz.
// All inputs are synchronous to the pixel clock. Bank ownership changes only
// in vertical blanking; a completed capture is held until it is displayed.
module emergency_audio_wave (
    input wire clk, rst, enable, vsync, de,
    input wire [23:0] rgb_in,
    input wire pcm_valid,
    input wire signed [15:0] pcm,
    output wire [23:0] rgb_out
);
reg [9:0] x,y;
reg vs_q,de_q;
wire frame_start = vs_q && !vsync;
reg signed [7:0] bank0[0:127],bank1[0:127];
reg wr_bank,rd_bank,display_valid;
reg [7:0] count;
reg [1:0] decimate,idle_frames;
reg seen;
reg [15:0] capture_peak,display_peak;
wire [15:0] abs_pcm = pcm[15] ? (~pcm + 16'd1) : pcm;
always @(posedge clk) begin
    if(rst) begin
        x<=0; y<=0; vs_q<=1; de_q<=0;
    end else begin
        vs_q<=vsync; de_q<=de;
        if(frame_start) begin x<=0; y<=0; end
        else begin
            if(!de) x<=0; else if(x<639) x<=x+1'b1;
            if(de_q && !de && y<479) y<=y+1'b1;
        end
    end
    if(rst || !enable) begin
        wr_bank<=0; rd_bank<=1; display_valid<=0;
        count<=0; decimate<=0; capture_peak<=0; display_peak<=0;
        seen<=0; idle_frames<=0;
    end else begin
        if(frame_start) begin
            seen<=0;
            if(seen) idle_frames<=0;
            else if(idle_frames<3) idle_frames<=idle_frames+1'b1;
            if(count==128) begin
                rd_bank<=wr_bank; wr_bank<=~wr_bank;
                display_valid<=1; display_peak<=capture_peak;
                count<=0; decimate<=0; capture_peak<=0;
            end
            if(!seen && idle_frames>=2) begin
                display_valid<=0; display_peak<=0;
                count<=0; decimate<=0; capture_peak<=0;
            end
        end
        // A coincident frame/sample pulse is deliberately skipped.
        if(pcm_valid) seen<=1;
        if(pcm_valid && !frame_start && count<128) begin
            if(abs_pcm>capture_peak) capture_peak<=abs_pcm;
            if(decimate==0) begin
                if(wr_bank) bank1[count[6:0]]<=pcm[15:8];
                else bank0[count[6:0]]<=pcm[15:8];
                count<=count+1'b1; decimate<=2;
            end else decimate<=decimate-1'b1;
        end
    end
end
integer idx,prev_idx,a,b,ya,yb,lo,hi,peak_h;
reg wave,axis,peak,grid;
always @* begin
    idx=0; prev_idx=0; a=0; b=0; ya=382; yb=382; lo=382; hi=382;
    wave=0; axis=0; peak=0; grid=0;
    peak_h=(display_peak*50)>>15;
    if(peak_h>50) peak_h=50;
    if(enable && de && x>=64 && x<576 && y>=356 && y<408) begin
        idx=(x-64)>>2; prev_idx=(idx==0)?0:idx-1;
        axis=(y==382);
        grid=(((x-64)&63)==0) || y==356 || y==407;
        if(display_valid) begin
            a=rd_bank?$signed(bank1[idx]):$signed(bank0[idx]);
            b=rd_bank?$signed(bank1[prev_idx]):$signed(bank0[prev_idx]);
            ya=382-((a*24)>>>7); yb=382-((b*24)>>>7);
            lo=(ya<yb)?ya:yb; hi=(ya>yb)?ya:yb;
            // Connect adjacent samples at the first pixel of each 4 px cell.
            wave=(((x-64)&3)==0)?(y>=lo-1 && y<=hi+1):(y>=ya-1 && y<=ya+1);
        end
    end
    if(enable && de && x>=584 && x<596 && y>=357 && y<408)
        peak=(peak_h>0 && y>=408-peak_h);
end
assign rgb_out=wave?24'h40E8F8:peak?24'hFFD040:axis?24'h63838A:grid?24'h49343A:rgb_in;
endmodule
