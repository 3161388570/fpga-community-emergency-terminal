module rgb_video_adapter
#(
	parameter DATA_WIDTH = 24                       
)
(
	input                       video_clk,          
	input                       rst,
	output                      read_en,            
	input[DATA_WIDTH - 1:0]     read_data,          
	input                      hs,                 
	input                      vs,                 
	input                      de,                 

	output                      hs_r,                 
	output                      vs_r,                 
	output                      de_r,                 
	output[DATA_WIDTH - 1:0]    vout_data           

);
reg [20:0] hs_d;
reg [20:0] vs_d;
reg [20:0] de_d;
reg[DATA_WIDTH - 1:0]  vout_data_r;

assign read_en = de_d[18];
assign hs_r = hs_d[20];
assign vs_r = vs_d[20];
assign de_r = de_d[20];
assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(de_d[19])
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
always @(posedge video_clk or posedge rst)begin
	if(rst)begin
    	hs_d <= 21'b0;
        vs_d <= 21'b0;
        de_d <= 21'b0;
    end
	else begin
    	
    	hs_d <= {hs_d[19:0],hs};
        vs_d <= {vs_d[19:0],vs};
        de_d <= {de_d[19:0],de};
    end
end
endmodule

// 1:1 thumbnail stream adapter.
//
// The four-bank thumbnail reader now supplies one sample per display pixel
// (320 samples per tile segment, 640 words per active line) instead of
// every fourth sample plus a 2x2 pixel replicator.  The previous version
// held a 320-word line buffer and displayed each fetched word on two
// pixels and two lines, so the montage was 1/4-resolution content blown up
// to 1/2 scale: smooth gradients in the source photos became visible
// 1-3 level bands (the photos' 8-bit stair-steps, amplitude-multiplied by
// 4:1 point sampling) and thin lines were skipped entirely.
//
// With one word per pixel the stream is already in display order, so this
// module is the same pipeline as rgb_video_adapter: read at de_d[18],
// capture the FIFO output at de_d[19], present it at de_d[20].  No line
// buffer, no fetch/display line phase, and no replication remain.
// frame_start is kept in the port list so the top-level instantiation and
// the reader stay on the same per-frame restart pulse, but the 1:1 stream
// has no phase state that would need resetting.
module rgb_video_upscaler_2x2 (
    input wire video_clk,
    input wire rst,
    input wire frame_start,
    input wire stream_ready,
    output wire read_en,
    input wire [23:0] read_data,
    input wire hs,
    input wire vs,
    input wire de,
    output wire hs_r,
    output wire vs_r,
    output wire de_r,
    output wire [23:0] vout_data
);

reg [20:0] hs_d;
reg [20:0] vs_d;
reg [20:0] de_d;
reg [23:0] vout_data_r;

// stream_ready holds the stream until the reader has completed its
// per-frame prefill; without it an underrun would consume stale FIFO
// words and shift the rest of the frame.
assign read_en = de_d[18] && stream_ready;
assign hs_r = hs_d[20];
assign vs_r = vs_d[20];
assign de_r = de_d[20];
assign vout_data = vout_data_r;

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        hs_d <= 21'd0;
        vs_d <= 21'd0;
        de_d <= 21'd0;
        vout_data_r <= 24'd0;
    end else begin
        hs_d <= {hs_d[19:0], hs};
        vs_d <= {vs_d[19:0], vs};
        de_d <= {de_d[19:0], de};
        if (de_d[19])
            vout_data_r <= read_data;
        else
            vout_data_r <= 24'd0;
    end
end

endmodule

