`timescale 1ns/1ps

// Builds the 2x2 overview montage while an image streams in.
//   * every SAMPLE_X_STEP x SAMPLE_Y_STEP block of the frame is averaged (box filter),
//   * one finished output row (THUMB_WIDTH words) is kept in a ping-pong row buffer,
//   * the row is then written to SDRAM as one continuous burst, like the main frame writer
//     does (isolated single-word writes were not taken by the controller).
// The montage is DISPLAY_WIDTH words wide: bank 0 top-left, 1 top-right, 2 bottom-left,
// 3 bottom-right.  OUTPUT_X_SCALE/OUTPUT_Y_SCALE are not used (always 1:1).
module media_thumbnail_generator #(
    parameter integer ADDR_BITS = 21,
    parameter integer FRAME_WIDTH = 640,
    parameter integer FRAME_HEIGHT = 480,
    parameter integer SAMPLE_X_STEP = 2,
    parameter integer SAMPLE_Y_STEP = 2,
    parameter integer OUTPUT_X_SCALE = 1,
    parameter integer OUTPUT_Y_SCALE = 1,
    parameter integer INPUT_BOTTOM_UP = 1,
    parameter [ADDR_BITS-1:0] BASE_ADDRESS = 21'd1228800
) (
    input wire rst,
    input wire write_clk,
    input wire write_frame_start,
    input wire [1:0] image_bank,
    input wire write_en,
    input wire [31:0] write_data,
    input wire mem_clk,
    input wire go_ok,                 // port has been quiet long enough to start a burst
    output wire writer_busy,          // owns the SDRAM port (burst + tail)
    output wire writer_pending,
    output wire mem_write_en,
    output wire [ADDR_BITS-1:0] mem_write_addr,
    output wire [31:0] mem_write_data,
    output wire thumbnail_ready,
    output wire thumbnail_overflow,
    output wire [3:0] thumbnail_mask
);

localparam integer THUMB_WIDTH = FRAME_WIDTH / SAMPLE_X_STEP;
localparam integer THUMB_HEIGHT = FRAME_HEIGHT / SAMPLE_Y_STEP;
localparam integer DISPLAY_WIDTH = THUMB_WIDTH * 2;
localparam integer AVG_SHIFT = $clog2(SAMPLE_X_STEP * SAMPLE_Y_STEP);
localparam integer FIRST_ROW = INPUT_BOTTOM_UP ? (THUMB_HEIGHT - 1) : 0;
localparam [ADDR_BITS-1:0] BANK0_BASE = BASE_ADDRESS + (FIRST_ROW * DISPLAY_WIDTH);
localparam [ADDR_BITS-1:0] BANK1_BASE = BANK0_BASE + THUMB_WIDTH;
localparam [ADDR_BITS-1:0] BANK2_BASE = BANK0_BASE + (THUMB_HEIGHT * DISPLAY_WIDTH);
localparam [ADDR_BITS-1:0] BANK3_BASE = BANK2_BASE + THUMB_WIDTH;
localparam [ADDR_BITS-1:0] ROW_STEP = DISPLAY_WIDTH;

function [1:0] gray_next;
    input [1:0] g;
    begin
        case (g)
            2'd0: gray_next = 2'd1;
            2'd1: gray_next = 2'd3;
            2'd3: gray_next = 2'd2;
            default: gray_next = 2'd0;
        endcase
    end
endfunction

// ------------------------------------------------------------------ write clock side
reg [15:0] capture_x;
reg [15:0] capture_y;
reg [10:0] hs_c2, hs_c1, hs_c0;      // running row sums per colour byte
reg [10:0] blk_c2, blk_c1, blk_c0;
reg        blk_valid;
reg [8:0]  blk_x;
reg [1:0]  blk_row;
reg [12:0] rd_c2, rd_c1, rd_c0;
reg        rd_valid;
reg [8:0]  rd_x;
reg [1:0]  rd_row;
reg [10:0] rd_s2, rd_s1, rd_s0;
reg [38:0] acc [0:THUMB_WIDTH-1];
reg        emit;
reg [8:0]  emit_x;
reg [23:0] emit_pixel;
wire [10:0] tot_c2 = ((capture_x % SAMPLE_X_STEP) == 0 ? 11'd0 : hs_c2) + write_data[31:24];
wire [10:0] tot_c1 = ((capture_x % SAMPLE_X_STEP) == 0 ? 11'd0 : hs_c1) + write_data[23:16];
wire [10:0] tot_c0 = ((capture_x % SAMPLE_X_STEP) == 0 ? 11'd0 : hs_c0) + write_data[15:8];
wire [38:0] acc_rd = acc[blk_x];
wire [12:0] new_c2 = ((rd_row == 0) ? 13'd0 : rd_c2) + rd_s2;
wire [12:0] new_c1 = ((rd_row == 0) ? 13'd0 : rd_c1) + rd_s1;
wire [12:0] new_c0 = ((rd_row == 0) ? 13'd0 : rd_c0) + rd_s0;

always @(posedge write_clk or posedge rst) begin
    if (rst) begin
        capture_x <= 16'd0;
        capture_y <= 16'd0;
    end else if (write_frame_start) begin
        capture_x <= 16'd0;
        capture_y <= 16'd0;
    end else if (write_en) begin
        if (capture_x == FRAME_WIDTH - 1) begin
            capture_x <= 16'd0;
            if (capture_y == FRAME_HEIGHT - 1)
                capture_y <= 16'd0;
            else
                capture_y <= capture_y + 1'b1;
        end else begin
            capture_x <= capture_x + 1'b1;
        end
    end
end

always @(posedge write_clk) begin
    blk_valid <= 1'b0;
    emit <= 1'b0;
    if (write_en) begin
        hs_c2 <= tot_c2; hs_c1 <= tot_c1; hs_c0 <= tot_c0;
        if ((capture_x % SAMPLE_X_STEP) == SAMPLE_X_STEP - 1) begin
            blk_c2 <= tot_c2; blk_c1 <= tot_c1; blk_c0 <= tot_c0;
            blk_x <= capture_x / SAMPLE_X_STEP;
            blk_row <= capture_y % SAMPLE_Y_STEP;
            blk_valid <= 1'b1;
        end
    end
    // stage 2
    rd_valid <= blk_valid;
    rd_x <= blk_x;
    rd_row <= blk_row;
    rd_s2 <= blk_c2; rd_s1 <= blk_c1; rd_s0 <= blk_c0;
    {rd_c2, rd_c1, rd_c0} <= {acc_rd[38:26], acc_rd[25:13], acc_rd[12:0]};
    // stage 3
    if (rd_valid) begin
        acc[rd_x] <= {new_c2, new_c1, new_c0};
        if (rd_row == SAMPLE_Y_STEP - 1) begin
            emit <= 1'b1;
            emit_x <= rd_x;
            emit_pixel <= {new_c2[AVG_SHIFT+7:AVG_SHIFT], new_c1[AVG_SHIFT+7:AVG_SHIFT],
                           new_c0[AVG_SHIFT+7:AVG_SHIFT]};
        end
    end
end

// ping-pong row buffer, index {buffer, x}
reg [23:0] rowbuf [0:1023];
reg        wbuf;                          // buffer being filled
reg [1:0]  wseq;                          // gray-coded count of finished rows (mod 4)
reg [ADDR_BITS-1:0] frame_row_addr;       // address of the row being produced
reg [1:0]  frame_bank;
reg [7:0]  frame_row;
reg [ADDR_BITS-1:0] info_addr [0:1];
reg [1:0]  info_bank [0:1];
reg [7:0]  info_row [0:1];
reg        overflow_write;
reg [1:0]  rseq_wsync0, rseq_wsync1;      // burst-engine sequence seen from the write clock
reg        row_commit;                    // publish a finished row one clock after its info
wire [1:0] rseq_mem;

wire [ADDR_BITS-1:0] bank_base = (image_bank == 2'd0) ? BANK0_BASE :
                                 (image_bank == 2'd1) ? BANK1_BASE :
                                 (image_bank == 2'd2) ? BANK2_BASE : BANK3_BASE;
wire two_ahead = (gray_next(gray_next(rseq_wsync1)) == wseq);

always @(posedge write_clk) begin
    if (emit) rowbuf[{wbuf, emit_x}] <= emit_pixel;
    if (emit && emit_x == THUMB_WIDTH - 1) begin
        info_addr[wbuf] <= frame_row_addr;
        info_bank[wbuf] <= frame_bank;
        info_row[wbuf] <= frame_row;
    end
end

always @(posedge write_clk or posedge rst) begin
    if (rst) begin
        wbuf <= 1'b0;
        wseq <= 2'd0;
        frame_row_addr <= BANK0_BASE;
        frame_bank <= 2'd0;
        frame_row <= 8'd0;
        overflow_write <= 1'b0;
        rseq_wsync0 <= 2'd0;
        rseq_wsync1 <= 2'd0;
        row_commit <= 1'b0;
    end else begin
        rseq_wsync0 <= rseq_mem;
        rseq_wsync1 <= rseq_wsync0;
        row_commit <= 1'b0;
        if (write_frame_start) begin
            frame_row_addr <= bank_base;
            frame_bank <= image_bank;
            frame_row <= 8'd0;
        end
        if (emit) begin
            // first sample of a row: the buffer must not still wait for its burst
            if (emit_x == 9'd0 && two_ahead) overflow_write <= 1'b1;
            if (emit_x == THUMB_WIDTH - 1) begin
                row_commit <= 1'b1;
                frame_row <= frame_row + 1'b1;
                if (INPUT_BOTTOM_UP != 0)
                    frame_row_addr <= frame_row_addr - ROW_STEP;
                else
                    frame_row_addr <= frame_row_addr + ROW_STEP;
            end
        end
        if (row_commit) begin
            wseq <= gray_next(wseq);
            wbuf <= ~wbuf;
        end
    end
end

// ------------------------------------------------------------------ memory clock side
(* async_reg = "true" *) reg [1:0] wseq_m0, wseq_m1;
(* async_reg = "true" *) reg [1:0] ovf_m0, ovf_m1;
reg [1:0] rseq_r;
reg       rbuf;
localparam [1:0] S_IDLE = 2'd0, S_PRIME = 2'd1, S_WRITE = 2'd2, S_TAIL = 2'd3;
reg [1:0]  state;
reg [9:0]  rd_idx;          // row buffer read address
reg [9:0]  out_idx;
reg [5:0]  tail_cnt;
reg [ADDR_BITS-1:0] burst_addr;
reg        write_en_r;
reg [23:0] rb_q;
reg [3:0]  mask;
reg [7:0]  bank_rows0, bank_rows1, bank_rows2, bank_rows3;
reg [1:0]  cur_bank;
reg [7:0]  cur_row;
reg [7:0]  cur_rows_done;

assign rseq_mem = rseq_r;
wire pending = (wseq_m1 != rseq_r);
assign writer_pending = pending;
assign writer_busy = (state != S_IDLE);
assign mem_write_en = write_en_r;
assign mem_write_addr = burst_addr;
assign mem_write_data = {rb_q, 8'd0};
assign thumbnail_overflow = ovf_m1[0];
assign thumbnail_mask = mask;
assign thumbnail_ready = (mask == 4'b1111) && (state == S_IDLE) && !pending && !ovf_m1[0];

always @(posedge mem_clk) begin
    rb_q <= rowbuf[{rbuf, rd_idx[8:0]}];
end

always @(*) begin
    case (cur_bank)
        2'd0: cur_rows_done = bank_rows0;
        2'd1: cur_rows_done = bank_rows1;
        2'd2: cur_rows_done = bank_rows2;
        default: cur_rows_done = bank_rows3;
    endcase
end

always @(posedge mem_clk or posedge rst) begin
    if (rst) begin
        wseq_m0 <= 2'd0; wseq_m1 <= 2'd0;
        ovf_m0 <= 2'd0; ovf_m1 <= 2'd0;
        rseq_r <= 2'd0;
        rbuf <= 1'b0;
        state <= S_IDLE;
        rd_idx <= 10'd0;
        out_idx <= 10'd0;
        tail_cnt <= 6'd0;
        burst_addr <= {ADDR_BITS{1'b0}};
        write_en_r <= 1'b0;
        mask <= 4'd0;
        bank_rows0 <= 8'd0; bank_rows1 <= 8'd0; bank_rows2 <= 8'd0; bank_rows3 <= 8'd0;
        cur_bank <= 2'd0;
        cur_row <= 8'd0;
    end else begin
        wseq_m0 <= wseq; wseq_m1 <= wseq_m0;
        ovf_m0 <= {ovf_m0[0], overflow_write}; ovf_m1 <= {ovf_m1[0], ovf_m0[0]};
        write_en_r <= 1'b0;
        case (state)
            S_IDLE: begin
                if (pending && go_ok) begin
                    rd_idx <= 10'd0;
                    out_idx <= 10'd0;
                    burst_addr <= info_addr[rbuf];
                    cur_bank <= info_bank[rbuf];
                    cur_row <= info_row[rbuf];
                    state <= S_PRIME;
                end
            end
            S_PRIME: begin
                // rb_q follows rd_idx one clock later
                state <= S_WRITE;
                if (cur_row == 8'd0) begin
                    mask[cur_bank] <= 1'b0;
                    case (cur_bank)
                        2'd0: bank_rows0 <= 8'd0;
                        2'd1: bank_rows1 <= 8'd0;
                        2'd2: bank_rows2 <= 8'd0;
                        default: bank_rows3 <= 8'd0;
                    endcase
                end
            end
            S_WRITE: begin
                write_en_r <= 1'b1;
                if (out_idx != 10'd0) burst_addr <= burst_addr + 1'b1;
                out_idx <= out_idx + 1'b1;
                rd_idx <= rd_idx + 1'b1;
                if (out_idx == THUMB_WIDTH - 1) begin
                    state <= S_TAIL;
                    tail_cnt <= 6'd40;
                end
            end
            S_TAIL: begin
                if (tail_cnt != 6'd0) tail_cnt <= tail_cnt - 1'b1;
                else begin
                    rseq_r <= gray_next(rseq_r);
                    rbuf <= ~rbuf;
                    case (cur_bank)
                        2'd0: bank_rows0 <= bank_rows0 + 1'b1;
                        2'd1: bank_rows1 <= bank_rows1 + 1'b1;
                        2'd2: bank_rows2 <= bank_rows2 + 1'b1;
                        default: bank_rows3 <= bank_rows3 + 1'b1;
                    endcase
                    if (cur_rows_done == THUMB_HEIGHT - 1) mask[cur_bank] <= 1'b1;
                    state <= S_IDLE;
                end
            end
        endcase
    end
end

endmodule
