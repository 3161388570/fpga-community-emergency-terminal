`timescale 1ns/1ps
module media_frame_reader
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter READ_DATA_BITS         = 32,
	parameter WRITE_DATA_BITS        = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter BURST_SIZE             = 128,
	parameter WRITE_V_FLIP           = 1,      
	parameter FRAME_WIDTH            = 640,
	parameter FRAME_HEIGHT           = 480,
    parameter TRANSITIONS           = 0,
    parameter READ_PRIORITY_ENABLE  = 1
)
(
input                            rst,
input                            mem_clk,
input							 Sdr_init_done,
input							 Sdr_init_ref_vld,
input							 Sdr_busy,
output							 App_rd_en,
output[ADDR_BITS - 1:0]			 App_rd_addr,
input							 Sdr_rd_en,
input[MEM_DATA_BITS - 1:0]		 Sdr_rd_dout,
input                            read_clk,
output                           read_req_ack,
output                           read_finish,
input[ADDR_BITS - 1:0]           read_addr_0,
input[ADDR_BITS - 1:0]           read_addr_1,
input[ADDR_BITS - 1:0]           read_addr_2,
input[ADDR_BITS - 1:0]           read_addr_3,
input[ADDR_BITS - 1:0]           read_len,
input                            read_en,
output[READ_DATA_BITS  - 1:0]    read_data,
output                           read_fifo_empty,
input frame_request,
input video_rst,
input [1:0] requested_bank,
input request_toggle,
input source_valid,
output ack_toggle,
output [1:0] active_bank,
output display_valid,
input loader_busy,
input transition_allow,
output reader_busy,
input rd_grant,
output [63:0] dbg_sum
);
wire [9:0] wrusedw;
wire read_fifo_aclr;
wire O_rd_busy;
wire read_priority;
wire rd_data_we;
diag_sum_counter u_diag_main_sum (
    .clk(mem_clk), .rst(rst), .clear(read_fifo_aclr), .we(rd_data_we),
    .data(Sdr_rd_dout[31:8]), .latch(read_finish),
    .sum_last(dbg_sum[63:32]), .chg(dbg_sum[31:16]), .frames(dbg_sum[15:0])
);
// The reader must still be allowed to start its own refill burst.  The
// priority signal is exported through reader_busy to stop the loader from
// starting a competing write burst at the same time.
wire App_wr_busy = loader_busy;
wire read_req;
wire [1:0] read_addr_index = active_bank;
assign reader_busy = O_rd_busy ||
                     (READ_PRIORITY_ENABLE != 0 && read_priority);
media_frame_commit #(.TRANSITIONS(TRANSITIONS)) commit (
.video_clk(read_clk), .rst(video_rst), .frame_request(frame_request),
.requested_bank(requested_bank), .request_toggle(request_toggle),
.source_valid(source_valid), .ack_toggle(ack_toggle), .active_bank(active_bank),
.display_valid(display_valid), .read_request(read_req), .transition_allow(transition_allow)
);
rfifo_32_32_512 #(
    .ADDR_WIDTH_W(10),
    .ADDR_WIDTH_R(10),
    .AL_FULL_NUM(1021),
    .AL_EMPTY_NUM(2)
) read_buf
	(
	.clkr                     	(read_clk                   ),          
	.clkw                     	(mem_clk                    ),          
	.rst                      	(read_fifo_aclr             ),          
	.we                     	(rd_data_we      			),          
	.re                     	(read_en                    ),          
	.di                      	(Sdr_rd_dout                ),          
	.empty_flag					(read_fifo_empty            ),          
	.full_flag					(                           ),          
	.wrusedw                	(wrusedw         	  	  		 ),          
	.rdusedw                	(                  			 ),          
	.dout						(read_data                  )
);

media_sdram_read_engine
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               ),
	.FIFO_DEPTH                 (1024                     ),
	.FIFO_MARGIN                (8                        ),
	.BURST_GAP_CYCLES           (12                       ),
	.MAX_INFLIGHT_BURSTS        (2                        ),
	.READ_LOW_WATERMARK         (704                      )
)
u_media_sdram_read_engine
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
    .Sdr_rd_en					(Sdr_rd_en),
	.App_wr_busy				(App_wr_busy),
    .O_rd_busy					(O_rd_busy),
	.read_priority				(read_priority),
	.rd_grant					(rd_grant),
	.rd_data_we					(rd_data_we),
	
    .App_rd_en					(App_rd_en),
    .App_rd_addr				(App_rd_addr),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),    
	.read_len                   (read_len                 ),
	.fifo_aclr                  (read_fifo_aclr           ),
	.wrusedw                	(wrusedw                   )
);

endmodule

module thumbnail_fourbank_reader #(
    parameter integer ADDR_BITS = 21,
    parameter integer FRAME_WIDTH = 640,
    parameter integer FRAME_HEIGHT = 480,
    parameter [ADDR_BITS-1:0] BUF0_ADDR = 21'd0,
    parameter [ADDR_BITS-1:0] BUF1_ADDR = 21'd307200,
    parameter [ADDR_BITS-1:0] BUF2_ADDR = 21'd614400,
    parameter [ADDR_BITS-1:0] BUF3_ADDR = 21'd921600
) (
    input wire rst,
    input wire enable,
    input wire frame_start,
    input wire mem_clk,
    output reg App_rd_en,
    output reg [ADDR_BITS-1:0] App_rd_addr,
    input wire Sdr_rd_en,
    input wire Sdr_init_ref_vld,
    input wire [31:0] Sdr_rd_dout,
    input wire read_clk,
    input wire read_en,
    output wire [31:0] read_data,
    output wire pixels_ready,
    output wire fifo_empty,
    output wire reader_idle,
    output wire [511:0] dbg_sum   // head-of-segment error histogram, see diag_head_hist
);

// Read-side fill control.
//  - The writer stops at FILL_LIMIT using the FIFO's write-clock level
//    counter (wrusedw).  rdusedw is generated in the 25 MHz domain and the
//    old code sampled it combinationally in the 125 MHz domain, so the
//    reader could overshoot the intended level.
//  - The FIFO is 1024 words deep and the display consumes 640 words per
//    active line, so PREFILL_LEVEL buffers more than one whole line
//    (640 + 64 early-start allowance).  With a 512-word FIFO the stream
//    could not hold a full line: a display line had to be supplied in real
//    time, and any shortfall (SDRAM refresh, an SD-card access, the bank
//    jump at the left/right segment boundary) ate into the same 448 words
//    the line was already drawing on.  Now a full line is always resident
//    and the reader only has to keep up on average.
localparam [9:0] FILL_LIMIT     = 10'd960;
localparam [9:0] PREFILL_LEVEL  = 10'd704;
localparam [5:0] INFLIGHT_LIMIT = 6'd32;
// Longer than one 256-word write burst plus controller latency.  A read
// accepted just before the restart may be held behind a write burst (the
// SDRAM IP gives writes priority); the window must outlast it or the late
// return would be appended to the fresh frame.
localparam [8:0] RESTART_CYCLES = 9'd383;

wire [9:0] rdusedw;
wire [9:0] wrusedw;
wire full_flag;
wire almost_full;
wire almost_empty;

// ---------------------------------------------------------------------
// Frame restart
// The video domain starts a new thumbnail frame at every VSYNC while the
// preview is active.  The pulse is converted to a toggle and synchronized,
// so it cannot be missed or sampled twice by the faster memory clock.
// Restarting the reader and flushing the FIFO every frame confines any
// lost/extra transaction to one frame instead of permanently shifting the
// 640-word line stream.
// ---------------------------------------------------------------------
reg frame_toggle_video;
always @(posedge read_clk or posedge rst) begin
    if (rst)
        frame_toggle_video <= 1'b0;
    else if (frame_start)
        frame_toggle_video <= ~frame_toggle_video;
end

(* async_reg = "true" *) reg [2:0] frame_sync_mem;
always @(posedge mem_clk or posedge rst) begin
    if (rst)
        frame_sync_mem <= 3'b000;
    else
        frame_sync_mem <= {frame_sync_mem[1:0], frame_toggle_video};
end

wire frame_restart = frame_sync_mem[2] ^ frame_sync_mem[1];

reg [8:0] restart_hold;
wire restart_active = (restart_hold != 9'd0);
wire reader_run = enable && !restart_active;

// Hold the FIFO in reset while the restart settles.  Reads that were
// already inside the SDRAM pipeline return during this window and are
// discarded, so they cannot be appended to the start of the new frame.
wire fifo_rst = rst || !enable || restart_active;

// ---------------------------------------------------------------------
// Guard reads at the tile/bank jumps
// The preview is the only stream in the design that leaves its SDRAM row
// every 320 words: the left tile of a display line ends at source column
// 638 of one buffer and the right tile starts at column 0 of the next, so
// the controller has to precharge and activate two different rows per
// display line.  The first words that come back after that switch still
// belong to the previously open row; written straight into the FIFO they
// appear as a short corrupt band at the start of every display line
// (measured in the photos as ~6 pixels at the left edge of the left
// column tiles, image dependent colour, stable position).
// After each 320-word segment the reader therefore issues GUARD_WORDS
// reads of the new segment's first address and drops the results.  The
// guards take the page miss, so the samples that are actually displayed
// are fetched from an already-open row.  Guards are only inserted between
// segments: at the first segment of a frame the FIFO is empty, and
// discarding words there would shift the whole frame by GUARD_WORDS
// samples.
localparam [5:0] GUARD_WORDS = 6'd32;

localparam [1:0] SEG_ISSUE = 2'd0,
                 SEG_DRAIN = 2'd1,
                 SEG_GUARD = 2'd2;

reg [1:0] seg_state;
reg [5:0] guard_left;         // guard reads still to issue
reg [5:0] guard_return_left;  // guard returns still to drop

// A return that really enters the FIFO, and the return itself (which also
// clears the in-flight counter whether or not it is kept).
wire return_valid = Sdr_rd_en && reader_run;
wire fifo_we      = return_valid && (guard_return_left == 6'd0);
// The trace needs the guard-phase start pulse (declared further down in the file as seg_state/outstanding).
wire guard_start_dbg;
diag_head_hist u_diag_thumb_sum (
    .clk(mem_clk), .rst(rst), .we(fifo_we), .data(Sdr_rd_dout[31:8]),
    .restart(frame_restart), .ref_vld(Sdr_init_ref_vld), .out(dbg_sum)
);

rfifo_32_32_512 #(
    .ADDR_WIDTH_W(10),
    .ADDR_WIDTH_R(10),
    .AL_FULL_NUM(1021),
    .AL_EMPTY_NUM(2)
) thumbnail_read_fifo (
    .clkw(mem_clk),
    .clkr(read_clk),
    .rst(fifo_rst),
    .we(fifo_we),
    .re(read_en),
    .di(Sdr_rd_dout),
    .dout(read_data),
    .valid(),
    .full_flag(full_flag),
    .empty_flag(fifo_empty),
    .afull(almost_full),
    .aempty(almost_empty),
    .wrusedw(wrusedw),
    .rdusedw(rdusedw)
);

// 1:1 half-resolution preview geometry.  Each tile is 320x240 and shows
// the whole 640x480 frame at 1/2 scale, so every display pixel is one
// fetched sample and every display line is one fetched source row:
//   - 320 samples per tile segment, source column = 2*sample (+2 stride)
//   - 240 lines per tile row,    source row    = 2*line
// The previous 4:1 point sampling (160 samples, +4 stride, every second
// line) was displayed through a 2x2 pixel replicator; dropping to 2:1
// halves the quantisation step of smooth gradients and keeps thin lines
// from being skipped, which is what made source banding visible in the
// montage.
reg [8:0] local_x;
reg [7:0] local_y;
reg tile_x;
reg tile_y;
reg x_last;   // local_x == 319, registered to shorten the address update path
reg y_last;   // local_y == 239
reg [ADDR_BITS-1:0] source_address_reg;
reg [5:0] outstanding;

// A guard read does not write the FIFO, so it is not limited by the fill
// level; only the in-flight cap applies.
wire can_issue = reader_run && (seg_state == SEG_ISSUE) &&
                 (outstanding < INFLIGHT_LIMIT) &&
                 (wrusedw <= FILL_LIMIT) &&
                 !Sdr_init_ref_vld;
wire can_guard = reader_run && (seg_state == SEG_GUARD) &&
                 (guard_left != 6'd0) &&
                 (outstanding < INFLIGHT_LIMIT) &&
                 !Sdr_init_ref_vld;

wire rd_issue = App_rd_en;
assign guard_start_dbg = reader_run && (seg_state == SEG_DRAIN) && (outstanding == 6'd0);
wire rd_return = return_valid;
wire [7:0] outstanding_next = {2'b00, outstanding} +
                              {7'd0, rd_issue} -
                              {7'd0, rd_return};

assign reader_idle = (outstanding == 6'd0);

// Prefill completion is evaluated in the memory clock domain where the
// write-side level is valid, then transferred to the video domain.
reg prefill_done_mem;
always @(posedge mem_clk or posedge rst) begin
    if (rst || !enable || restart_active)
        prefill_done_mem <= 1'b0;
    else if (wrusedw >= PREFILL_LEVEL)
        prefill_done_mem <= 1'b1;
end

(* async_reg = "true" *) reg [1:0] prefill_sync_video;
always @(posedge read_clk or posedge rst) begin
    if (rst)
        prefill_sync_video <= 2'b00;
    else
        prefill_sync_video <= {prefill_sync_video[0], prefill_done_mem};
end

assign pixels_ready = prefill_sync_video[1];

// Restart / drain window.  It is reloaded while the reader is disabled so
// that the first frame after thumb_owner rises also gets a clean start.
always @(posedge mem_clk or posedge rst) begin
    if (rst || !enable || frame_restart)
        restart_hold <= RESTART_CYCLES;
    else if (restart_active)
        restart_hold <= restart_hold - 1'b1;
end

// Exact in-flight count.  Both events are handled in the same clocked
// block, so a request and a return on the same edge are both applied.
// Saturation at zero only absorbs a return that belongs to the frame
// before the restart and arrives just after the flush window.
always @(posedge mem_clk or posedge rst) begin
    if (rst || !enable || frame_restart || restart_active)
        outstanding <= 6'd0;
    else if (outstanding_next[7])
        outstanding <= 6'd0;
    else if (outstanding_next > 8'd63)
        outstanding <= 6'd63;
    else
        outstanding <= outstanding_next[5:0];
end

always @(posedge mem_clk or posedge rst) begin
    if (rst || !enable || frame_restart) begin
        App_rd_en <= 1'b0;
        App_rd_addr <= {ADDR_BITS{1'b0}};
        local_x <= 9'd0;
        local_y <= 8'd0;
        x_last <= 1'b0;
        y_last <= 1'b0;
        tile_x <= 1'b0;
        tile_y <= 1'b0;
        source_address_reg <= {ADDR_BITS{1'b0}};
        seg_state <= SEG_ISSUE;
        guard_left <= 6'd0;
        guard_return_left <= 6'd0;
    end else if (restart_active) begin
        App_rd_en <= 1'b0;
    end else begin
        App_rd_en <= can_issue || can_guard;
        if (can_issue) begin
            App_rd_addr <= source_address_reg;
            if (!x_last) begin
                local_x <= local_x + 1'b1;
                x_last <= (local_x == 9'd318);
                source_address_reg <= source_address_reg + 21'd2;
            end else if (!tile_x) begin
                // Last sample (col 638) of a left tile -> same source row
                // of the right-tile bank: 307200 - 638.
                local_x <= 9'd0;
                x_last <= 1'b0;
                tile_x <= 1'b1;
                source_address_reg <= source_address_reg + 21'd306562;
            end else if (!y_last) begin
                // Last sample of a right tile -> left tile two source rows
                // below: -(307200 - 2*640 + 638).
                local_x <= 9'd0;
                x_last <= 1'b0;
                tile_x <= 1'b0;
                local_y <= local_y + 1'b1;
                y_last <= (local_y == 8'd238);
                source_address_reg <= source_address_reg - 21'd306558;
            end else if (!tile_y) begin
                // Last line of the top half -> BUF2 row 0: 614400 -
                // (307200 + 478*640 + 638).
                local_x <= 9'd0;
                x_last <= 1'b0;
                tile_x <= 1'b0;
                local_y <= 8'd0;
                y_last <= 1'b0;
                tile_y <= 1'b1;
                source_address_reg <= source_address_reg + 21'd642;
            end else begin
                local_x <= 9'd0;
                x_last <= 1'b0;
                tile_x <= 1'b0;
                local_y <= 8'd0;
                y_last <= 1'b0;
                tile_y <= 1'b0;
                source_address_reg <= {ADDR_BITS{1'b0}};
            end

            // The next word belongs to a different tile/bank.  Stop
            // issuing until every read of this 320-word segment has
            // returned; the address jump then starts a fresh transaction
            // instead of landing in the middle of a read stream.
            if (x_last)
                seg_state <= SEG_DRAIN;
        end else if (can_guard) begin
            // Throw-away read of the coming segment's first sample.  It
            // is the read that pays the precharge/activate, and its
            // result is dropped, so the samples that follow come from the
            // row that is already open.
            // Cycle through the first 8 sample addresses.  Repeating one address 32 times made the
            // SDRAM controller return stale words (the previous segment's data) for the 5th..8th
            // real reads after the guard, which showed as a red bar at the left edge of the left tiles.
            App_rd_addr <= source_address_reg + {16'd0, guard_left[2:0], 1'b0};
            guard_left <= guard_left - 1'b1;
            if (guard_left == 6'd1)
                seg_state <= SEG_ISSUE;
        end

        if (seg_state == SEG_DRAIN && outstanding == 6'd0) begin
            seg_state <= SEG_GUARD;
            guard_left <= GUARD_WORDS;
            guard_return_left <= GUARD_WORDS;
        end else if (return_valid && guard_return_left != 6'd0) begin
            guard_return_left <= guard_return_left - 1'b1;
        end
    end
end

endmodule

module media_frame_commit #(parameter TRANSITIONS = 0) (
    input video_clk,
    input rst,
    input frame_request,
    input [1:0] requested_bank,
    input request_toggle,
    input source_valid,
    input transition_allow,
    output reg ack_toggle,
    output reg [1:0] active_bank,
    output reg display_valid,
    output reg read_request
);
(* async_reg = "true" *) reg [2:0] request_sync;
(* async_reg = "true" *) reg [1:0] bank_sync_first;
reg [1:0] bank_sync_second;
(* async_reg = "true" *) reg [1:0] valid_sync;
reg frame_previous;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        request_sync <= 0;
        bank_sync_first <= 0;
        bank_sync_second <= 0;
        valid_sync <= 0;
        frame_previous <= 0;
        ack_toggle <= 0;
        active_bank <= 0;
        display_valid <= 0;
        read_request <= 0;
    end else begin
        request_sync <= {request_sync[1:0], request_toggle};
        bank_sync_first <= requested_bank;
        bank_sync_second <= bank_sync_first;
        valid_sync <= {valid_sync[0], source_valid};
        frame_previous <= frame_request;
        read_request <= frame_request;
        if (frame_request && !frame_previous) begin
            if (!TRANSITIONS || transition_allow || !display_valid)
                ack_toggle <= request_sync[2];
            if (!valid_sync[1]) display_valid <= 0;
            else if (request_sync[2] != ack_toggle &&
                     (!TRANSITIONS || transition_allow || !display_valid)) begin
                active_bank <= bank_sync_second;
                display_valid <= 1;
            end
        end
    end
end
endmodule

