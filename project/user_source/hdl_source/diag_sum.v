`timescale 1ns/1ps
// Diagnostic only (not part of the product build): per-frame byte-sum checksum
// of a 24-bit pixel stream plus an on-screen hex read-out.
// sum_last = checksum of the last completed frame, chg = completed frames whose
// checksum differed from the previous frame, frames = completed frame count.
module diag_sum_counter (
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,
    input  wire        we,
    input  wire [23:0] data,
    input  wire        latch,
    output reg  [31:0] sum_last,
    output reg  [15:0] chg,
    output reg  [15:0] frames
);
reg [31:0] sum;
// Two input pipeline stages keep the SDRAM read-data timing path short; the
// control strobes are delayed by the same two cycles so they stay aligned.
reg        we_a, we_b, latch_a, latch_b, clear_a, clear_b;
reg [23:0] data_a;
reg [9:0]  bytes_b;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        we_a <= 1'b0; we_b <= 1'b0; latch_a <= 1'b0; latch_b <= 1'b0;
        clear_a <= 1'b0; clear_b <= 1'b0; data_a <= 24'd0; bytes_b <= 10'd0;
    end else begin
        we_a <= we; latch_a <= latch; clear_a <= clear; data_a <= data;
        we_b <= we_a; latch_b <= latch_a; clear_b <= clear_a;
        bytes_b <= {2'b00, data_a[23:16]} + {2'b00, data_a[15:8]} + {2'b00, data_a[7:0]};
    end
end
always @(posedge clk or posedge rst) begin
    if (rst) begin
        sum <= 32'd0;
        sum_last <= 32'd0;
        chg <= 16'd0;
        frames <= 16'd0;
    end else if (latch_b) begin
        sum_last <= sum;
        if (frames >= 16'd2 && sum != sum_last && chg != 16'hFFFF)
            chg <= chg + 1'b1;
        if (frames != 16'hFFFF)
            frames <= frames + 1'b1;
        sum <= we_b ? {22'd0, bytes_b} : 32'd0;
    end else if (clear_b) begin
        sum <= 32'd0;
    end else if (we_b) begin
        sum <= sum + {22'd0, bytes_b};
    end
end
endmodule

// Counts SDRAM command-bus hazards seen on the controller port (mem_clk domain).
//  hz1 = {wr&rd same clock, wr while a read return is outstanding,
//         rd within 16 clocks after a wr, wr within 16 clocks after a rd}
//  hz2 = {wr command count (32 bit), rd commands minus rd returns (16 bit),
//         commands presented while Sdr_busy (16 bit, saturating)}
module diag_hazard (
    input  wire        clk,
    input  wire        rst,
    input  wire        wr,
    input  wire        rd,
    input  wire        ret,
    input  wire        busy,
    output wire [63:0] hz1,
    output wire [63:0] hz2
);
reg [15:0] c_wr_rd, c_wr_out, c_rd_after_wr, c_wr_after_rd, c_busy, outstanding;
reg [31:0] c_wr;
reg [4:0]  since_wr, since_rd;
assign hz1 = {c_wr_rd, c_wr_out, c_rd_after_wr, c_wr_after_rd};
assign hz2 = {c_wr, outstanding, c_busy};
always @(posedge clk or posedge rst) begin
    if (rst) begin
        c_wr_rd <= 0; c_wr_out <= 0; c_rd_after_wr <= 0; c_wr_after_rd <= 0; c_busy <= 0;
        outstanding <= 0; c_wr <= 0; since_wr <= 5'd31; since_rd <= 5'd31;
    end else begin
        since_wr <= wr ? 5'd0 : (since_wr == 5'd31 ? 5'd31 : since_wr + 1'b1);
        since_rd <= rd ? 5'd0 : (since_rd == 5'd31 ? 5'd31 : since_rd + 1'b1);
        outstanding <= outstanding + rd - ret;
        if (wr) c_wr <= c_wr + 1'b1;
        if (wr && rd && c_wr_rd != 16'hFFFF) c_wr_rd <= c_wr_rd + 1'b1;
        if (wr && !rd && outstanding != 16'd0 && c_wr_out != 16'hFFFF) c_wr_out <= c_wr_out + 1'b1;
        if (rd && !wr && since_wr < 5'd16 && c_rd_after_wr != 16'hFFFF) c_rd_after_wr <= c_rd_after_wr + 1'b1;
        if (wr && !rd && since_rd < 5'd16 && c_wr_after_rd != 16'hFFFF) c_wr_after_rd <= c_wr_after_rd + 1'b1;
        if ((wr || rd) && busy && c_busy != 16'hFFFF) c_busy <= c_busy + 1'b1;
    end
end
endmodule

// Per-bank byte-sum of everything the main writer sends to the controller
// (addresses below the thumbnail region).  w01 = {bank0, bank1}, w23 = {bank2, bank3}.
module diag_wr_sums (
    input  wire        clk,
    input  wire        rst,
    input  wire        wr,
    input  wire [20:0] addr,
    input  wire [31:0] din,
    output wire [63:0] w01,
    output wire [63:0] w23
);
localparam [20:0] BANK = 21'd307200;
reg [31:0] s0, s1, s2, s3;
// stage 0: register the raw command (keeps the FIFO read-data path short)
reg        v0; reg [20:0] a0; reg [23:0] d0;
// stage 1: byte sum and bank decode
reg        v1; reg [9:0] b1; reg [3:0] sel1;
assign w01 = {s0, s1};
assign w23 = {s2, s3};
always @(posedge clk or posedge rst) begin
    if (rst) begin
        s0 <= 0; s1 <= 0; s2 <= 0; s3 <= 0;
        v0 <= 0; a0 <= 0; d0 <= 0; v1 <= 0; b1 <= 0; sel1 <= 0;
    end else begin
        v0 <= wr; a0 <= addr; d0 <= din[31:8];
        v1 <= v0 && (a0 < 21'd1228800);
        b1 <= {2'b00, d0[23:16]} + {2'b00, d0[15:8]} + {2'b00, d0[7:0]};
        sel1 <= {a0 >= 3*BANK, (a0 >= 2*BANK) && (a0 < 3*BANK),
                 (a0 >= BANK) && (a0 < 2*BANK), a0 < BANK};
        if (v1) begin
            if (sel1[0]) s0 <= s0 + b1;
            if (sel1[1]) s1 <= s1 + b1;
            if (sel1[2]) s2 <= s2 + b1;
            if (sel1[3]) s3 <= s3 + b1;
        end
    end
end
endmodule

// Continuous refresh statistics: {Sdr_init_ref_vld clocks (32), ref_vld rising edges (16), our refresh pulses seen (16)}
module diag_refstat (
    input  wire        clk,
    input  wire        rst,
    input  wire        ref_vld,
    input  wire        busy,
    input  wire        pulse,
    output wire [63:0] out
);
reg [31:0] vld_cycles;
reg [15:0] vld_rises, pulses;
reg        vq;
assign out = {vld_cycles, vld_rises, pulses};
always @(posedge clk or posedge rst) begin
    if (rst) begin vld_cycles <= 0; vld_rises <= 0; pulses <= 0; vq <= 0; end
    else begin
        vq <= ref_vld;
        if (ref_vld) vld_cycles <= vld_cycles + 1'b1;
        if (ref_vld && !vq && vld_rises != 16'hFFFF) vld_rises <= vld_rises + 1'b1;
        if (pulse && pulses != 16'hFFFF) pulses <= pulses + 1'b1;
    end
end
endmodule

// Thumbnail stream capture + per-tile change counters.
// The reader returns 320 words per segment: segment 0..479 = top half (even: tile 0, odd: tile 1),
// 480..959 = bottom half (even: tile 2, odd: tile 3).  Captured in the second frame after reset and then frozen:
//   H0..H3  the first 8 words of segment SEG_SEL (two 24-bit words per row: {8'h00,wA, 8'h00,wB})
//   T0,T1   the last 4 words of segment SEG_SEL-1
//   DEV     {dev0,dev1,dev2,dev3}: later frames whose per-tile byte-sum differs from the captured frame
module diag_tile_sums #(parameter [9:0] SEG_SEL = 10'd680) (
    input  wire         clk,
    input  wire         rst,
    input  wire         we,
    input  wire [23:0]  data,
    input  wire         restart,
    output wire [447:0] out
);
reg        we_a, we_b, rs_a, rs_b;
reg [23:0] d_a, d_b;
reg [9:0]  b_b;
reg [31:0] c0, c1, c2, c3, s0, s1, s2, s3;
reg [15:0] dev0, dev1, dev2, dev3;
reg [23:0] h0, h1, h2, h3, h4, h5, h6, h7, t0, t1, t2, t3;
reg [8:0]  wcnt;
reg [9:0]  seg;
reg [15:0] frames;
wire [1:0] tile = {(seg >= 10'd480), seg[0]};
assign out = {8'h00, h0, 8'h00, h1,  8'h00, h2, 8'h00, h3,  8'h00, h4, 8'h00, h5,  8'h00, h6, 8'h00, h7,
              8'h00, t0, 8'h00, t1,  8'h00, t2, 8'h00, t3,  dev0, dev1, dev2, dev3};
always @(posedge clk or posedge rst) begin
    if (rst) begin
        we_a <= 0; we_b <= 0; rs_a <= 0; rs_b <= 0; d_a <= 0; d_b <= 0; b_b <= 0;
        c0 <= 0; c1 <= 0; c2 <= 0; c3 <= 0; s0 <= 0; s1 <= 0; s2 <= 0; s3 <= 0;
        dev0 <= 0; dev1 <= 0; dev2 <= 0; dev3 <= 0;
        h0 <= 0; h1 <= 0; h2 <= 0; h3 <= 0; h4 <= 0; h5 <= 0; h6 <= 0; h7 <= 0;
        t0 <= 0; t1 <= 0; t2 <= 0; t3 <= 0;
        wcnt <= 0; seg <= 0; frames <= 0;
    end else begin
        we_a <= we; rs_a <= restart; d_a <= data;
        we_b <= we_a; rs_b <= rs_a; d_b <= d_a;
        b_b <= {2'b00, d_a[23:16]} + {2'b00, d_a[15:8]} + {2'b00, d_a[7:0]};
        if (rs_b) begin
            if (frames == 16'd1) begin
                s0 <= c0; s1 <= c1; s2 <= c2; s3 <= c3;
            end else if (frames >= 16'd2) begin
                if (c0 != s0 && dev0 != 16'hFFFF) dev0 <= dev0 + 1'b1;
                if (c1 != s1 && dev1 != 16'hFFFF) dev1 <= dev1 + 1'b1;
                if (c2 != s2 && dev2 != 16'hFFFF) dev2 <= dev2 + 1'b1;
                if (c3 != s3 && dev3 != 16'hFFFF) dev3 <= dev3 + 1'b1;
            end
            if (frames != 16'hFFFF) frames <= frames + 1'b1;
            c0 <= 0; c1 <= 0; c2 <= 0; c3 <= 0; wcnt <= 0; seg <= 0;
        end else if (we_b) begin
            case (tile)
                2'd0: c0 <= c0 + b_b;
                2'd1: c1 <= c1 + b_b;
                2'd2: c2 <= c2 + b_b;
                default: c3 <= c3 + b_b;
            endcase
            if (frames == 16'd1) begin
                if (seg == SEG_SEL) begin
                    case (wcnt[2:0])
                        3'd0: if (wcnt < 9'd8) h0 <= d_b;
                        3'd1: if (wcnt < 9'd8) h1 <= d_b;
                        3'd2: if (wcnt < 9'd8) h2 <= d_b;
                        3'd3: if (wcnt < 9'd8) h3 <= d_b;
                        3'd4: if (wcnt < 9'd8) h4 <= d_b;
                        3'd5: if (wcnt < 9'd8) h5 <= d_b;
                        3'd6: if (wcnt < 9'd8) h6 <= d_b;
                        default: if (wcnt < 9'd8) h7 <= d_b;
                    endcase
                end
                if (seg == SEG_SEL - 10'd1) begin
                    case (wcnt)
                        9'd316: t0 <= d_b;
                        9'd317: t1 <= d_b;
                        9'd318: t2 <= d_b;
                        9'd319: t3 <= d_b;
                        default: ;
                    endcase
                end
            end
            if (wcnt == 9'd319) begin wcnt <= 0; seg <= seg + 1'b1; end
            else wcnt <= wcnt + 1'b1;
        end
    end
end
endmodule

// Head-of-segment error histogram for the two bottom tiles (their left edge is white in the source images),
// plus a correlation with SDRAM refresh.  For each segment (row of a tile) the first 16 word times are examined:
//   err  = one of word positions 4..7 is not near-white (byte-sum < 720)
//   ref  = Sdr_init_ref_vld was high since the previous segment ended or during words 0..15
// Per frame: n = segments, A = segments with ref, B = segments with err, C = segments with both.
// out = { tile2 positions 0..31 (4 rows), tile3 positions 0..7 (1 row), tile2 {A,B,C,n}, tile3 {A,B,C,n}, {ref rises, 0,0,0} }
module diag_head_hist (
    input  wire         clk,
    input  wire         rst,
    input  wire         we,
    input  wire [23:0]  data,
    input  wire         restart,
    input  wire         ref_vld,
    output wire [511:0] out
);
reg        we_a, we_b, rs_a, rs_b;
reg [23:0] d_a;
reg [9:0]  b_b;
reg [8:0]  wcnt;
reg [9:0]  seg;
reg [255:0] cur2, last2;
reg [63:0]  cur3, last3;
reg [15:0] a2, b2, c2, n2, a3, b3, c3, n3, la2, lb2, lc2, ln2, la3, lb3, lc3, ln3;
reg [15:0] rr, lrr;
reg        ref_seen, seg_flag, seg_err, ref_q;
integer k;
wire in_bottom = (seg >= 10'd480);
wire is_t2 = in_bottom && !seg[0];
wire is_t3 = in_bottom && seg[0];
wire nonwhite = (b_b < 10'd720);
assign out = {last2, last3, la2, lb2, lc2, ln2, la3, lb3, lc3, ln3, lrr, 48'd0};
always @(posedge clk or posedge rst) begin
    if (rst) begin
        we_a <= 0; we_b <= 0; rs_a <= 0; rs_b <= 0; d_a <= 0; b_b <= 0; wcnt <= 0; seg <= 0;
        cur2 <= 0; cur3 <= 0; last2 <= 0; last3 <= 0;
        a2 <= 0; b2 <= 0; c2 <= 0; n2 <= 0; a3 <= 0; b3 <= 0; c3 <= 0; n3 <= 0;
        la2 <= 0; lb2 <= 0; lc2 <= 0; ln2 <= 0; la3 <= 0; lb3 <= 0; lc3 <= 0; ln3 <= 0;
        rr <= 0; lrr <= 0; ref_seen <= 0; seg_flag <= 0; seg_err <= 0; ref_q <= 0;
    end else begin
        we_a <= we; rs_a <= restart; d_a <= data; ref_q <= ref_vld;
        we_b <= we_a; rs_b <= rs_a;
        b_b <= {2'b00, d_a[23:16]} + {2'b00, d_a[15:8]} + {2'b00, d_a[7:0]};
        if (ref_vld) ref_seen <= 1'b1;
        if (ref_vld && !ref_q) rr <= rr + 1'b1;
        if (rs_b) begin
            last2 <= cur2; last3 <= cur3; cur2 <= 0; cur3 <= 0; wcnt <= 0; seg <= 0;
            la2 <= a2; lb2 <= b2; lc2 <= c2; ln2 <= n2; la3 <= a3; lb3 <= b3; lc3 <= c3; ln3 <= n3;
            a2 <= 0; b2 <= 0; c2 <= 0; n2 <= 0; a3 <= 0; b3 <= 0; c3 <= 0; n3 <= 0;
            lrr <= rr; rr <= 0; ref_seen <= 0; seg_flag <= 0; seg_err <= 0;
        end else if (we_b) begin
            if (wcnt == 9'd0) begin seg_flag <= ref_seen | ref_vld; seg_err <= 1'b0; ref_seen <= 1'b0; end
            else if (wcnt < 9'd16 && ref_vld) seg_flag <= 1'b1;
            if (wcnt >= 9'd4 && wcnt <= 9'd7 && nonwhite) seg_err <= 1'b1;
            if (wcnt == 9'd16 && in_bottom) begin
                if (is_t2) begin n2 <= n2 + 1'b1; if (seg_flag) a2 <= a2 + 1'b1; if (seg_err) b2 <= b2 + 1'b1; if (seg_flag && seg_err) c2 <= c2 + 1'b1; end
                else       begin n3 <= n3 + 1'b1; if (seg_flag) a3 <= a3 + 1'b1; if (seg_err) b3 <= b3 + 1'b1; if (seg_flag && seg_err) c3 <= c3 + 1'b1; end
            end
            if (wcnt < 9'd32 && nonwhite) begin
                for (k = 0; k < 32; k = k + 1) begin
                    if (is_t2 && wcnt == k && cur2[(31-k)*8 +: 8] != 8'hFF) cur2[(31-k)*8 +: 8] <= cur2[(31-k)*8 +: 8] + 1'b1;
                    if (is_t3 && wcnt == k && k < 8 && cur3[(7-k)*8 +: 8] != 8'hFF) cur3[(7-k)*8 +: 8] <= cur3[(7-k)*8 +: 8] + 1'b1;
                end
            end
            if (wcnt == 9'd319) begin wcnt <= 0; seg <= seg + 1'b1; end
            else wcnt <= wcnt + 1'b1;
        end
    end
end
endmodule

// Screen-side head error histogram (video clock).  For tile 2 (x 0..319, y 240..479) and tile 3 (x 320..639,
// y 240..479): for each of the first 32 pixels of every tile line, the number of lines of one frame in which the
// pixel is not near-white (byte-sum < 720).  Frozen after a few frames so the read-out is stable.
// out = { tile2 positions 0..31, tile3 positions 0..31 } (one byte each, position 0 first)
module diag_vid_hist (
    input  wire         clk,
    input  wire         rst,
    input  wire         vs,
    input  wire         de,
    input  wire [23:0]  rgb,
    output wire [511:0] out
);
reg vs_prev, de_prev;
reg [9:0] px, py;
reg [3:0] frame_cnt;
reg       frozen;
reg [255:0] cur2, cur3, last2, last3;
integer k;
wire [9:0] bsum = {2'b00, rgb[23:16]} + {2'b00, rgb[15:8]} + {2'b00, rgb[7:0]};
wire nonwhite = (bsum < 10'd720);
wire bottom = (py >= 10'd240);
assign out = {last2, last3};
always @(posedge clk or posedge rst) begin
    if (rst) begin
        vs_prev <= 0; de_prev <= 0; px <= 0; py <= 0; frame_cnt <= 0; frozen <= 0;
        cur2 <= 0; cur3 <= 0; last2 <= 0; last3 <= 0;
    end else begin
        vs_prev <= vs; de_prev <= de;
        if (vs_prev && !vs) begin
            px <= 0; py <= 0;
            if (!frozen) begin
                if (frame_cnt >= 4'd3) begin last2 <= cur2; last3 <= cur3; frozen <= 1'b1; end
                else frame_cnt <= frame_cnt + 1'b1;
            end
            cur2 <= 0; cur3 <= 0;
        end else begin
            if (!de) px <= 0; else if (px < 10'd639) px <= px + 1'b1;
            if (de_prev && !de && py < 10'd479) py <= py + 1'b1;
            if (de && nonwhite && bottom) begin
                for (k = 0; k < 32; k = k + 1) begin
                    if (px == k && cur2[(31-k)*8 +: 8] != 8'hFF) cur2[(31-k)*8 +: 8] <= cur2[(31-k)*8 +: 8] + 1'b1;
                    if (px == 10'd320 + k && cur3[(31-k)*8 +: 8] != 8'hFF) cur3[(31-k)*8 +: 8] <= cur3[(31-k)*8 +: 8] + 1'b1;
                end
            end
        end
    end
end
endmodule

// Address/data trace of the first real reads after the guard phase of one segment.
// guard_start counts the guard phases of the current frame; the one before segment SEG_N is recorded:
// the first 32 commands/returns are the guard reads, commands/returns #32..#39 are the real samples 0..7.
//   out = { addresses of commands #32..#39 (4 rows), data of returns #32..#39 (4 rows) }
module diag_ret_trace #(parameter [9:0] SEG_N = 10'd680) (
    input  wire         clk,
    input  wire         rst,
    input  wire         restart,
    input  wire         guard_start,
    input  wire         issue,
    input  wire [20:0]  addr,
    input  wire         ret,
    input  wire [23:0]  data,
    output wire [511:0] out
);
reg [9:0]  gcount;
reg [1:0]  frame_state;
reg        rec;
reg [7:0]  t;
reg [6:0]  iss_idx, ret_idx;
reg [20:0] a [0:7];
reg [23:0] d [0:7];
integer k;
assign out = {8'h00, 3'b000, a[0], 8'h00, 3'b000, a[1],  8'h00, 3'b000, a[2], 8'h00, 3'b000, a[3],
              8'h00, 3'b000, a[4], 8'h00, 3'b000, a[5],  8'h00, 3'b000, a[6], 8'h00, 3'b000, a[7],
              8'h00, d[0], 8'h00, d[1],  8'h00, d[2], 8'h00, d[3],
              8'h00, d[4], 8'h00, d[5],  8'h00, d[6], 8'h00, d[7]};
always @(posedge clk or posedge rst) begin
    if (rst) begin
        gcount <= 0; frame_state <= 0; rec <= 0; t <= 0; iss_idx <= 0; ret_idx <= 0;
        for (k = 0; k < 8; k = k + 1) begin a[k] <= 0; d[k] <= 0; end
    end else begin
        if (restart) begin
            gcount <= 0;
            if (frame_state == 0) frame_state <= 2'd1;
            else if (frame_state == 2'd1) frame_state <= 2'd2;
            rec <= 0;
        end else if (guard_start) begin
            gcount <= gcount + 1'b1;
            if (frame_state == 2'd1 && (gcount + 10'd1) == SEG_N) begin
                rec <= 1'b1; t <= 0; iss_idx <= 0; ret_idx <= 0;
            end
        end
        if (rec) begin
            if (issue) begin
                iss_idx <= iss_idx + 1'b1;
                if (iss_idx >= 7'd32 && iss_idx < 7'd40) a[iss_idx - 7'd32] <= addr;
            end
            if (ret) begin
                ret_idx <= ret_idx + 1'b1;
                if (ret_idx >= 7'd32 && ret_idx < 7'd40) d[ret_idx - 7'd32] <= data;
            end
            if (t == 8'd200) rec <= 1'b0; else t <= t + 1'b1;
        end
    end
end
endmodule

module diag_overlay (
    input  wire        clk,
    input  wire        rst,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    input  wire [575:0] rows,   // 9 rows of {32,16,16}: M1 W1 W2 B1..B6
    output reg         hs_out,
    output reg         vs_out,
    output reg         de_out,
    output reg  [23:0] rgb_out
);
// rows: "M1 SSSSSSSS CCCC NNNN", "M2 ...", "T1 ..." ; S = last frame byte-sum,
// C = frames whose sum changed, N = completed frames.  12x18 px cells.
localparam integer X0 = 296;
localparam integer Y0 = 8;
reg [575:0] s1;
reg vs_prev, de_prev;
reg [9:0] px, py;

function [4:0] glyph_row;
    input [5:0] ch;
    input [2:0] row;
    begin
        glyph_row = 5'b0;
        case (ch)
        6'd0: case(row) 0:glyph_row=5'b01110; 1:glyph_row=5'b10001; 2:glyph_row=5'b10011; 3:glyph_row=5'b10101; 4:glyph_row=5'b11001; 5:glyph_row=5'b10001; 6:glyph_row=5'b01110; default:glyph_row=5'b0; endcase
        6'd1: case(row) 0:glyph_row=5'b00100; 1:glyph_row=5'b01100; 2:glyph_row=5'b00100; 3:glyph_row=5'b00100; 4:glyph_row=5'b00100; 5:glyph_row=5'b00100; 6:glyph_row=5'b01110; default:glyph_row=5'b0; endcase
        6'd2: case(row) 0:glyph_row=5'b01110; 1:glyph_row=5'b10001; 2:glyph_row=5'b00001; 3:glyph_row=5'b00010; 4:glyph_row=5'b00100; 5:glyph_row=5'b01000; 6:glyph_row=5'b11111; default:glyph_row=5'b0; endcase
        6'd3: case(row) 0:glyph_row=5'b11110; 1:glyph_row=5'b00001; 2:glyph_row=5'b00001; 3:glyph_row=5'b01110; 4:glyph_row=5'b00001; 5:glyph_row=5'b00001; 6:glyph_row=5'b11110; default:glyph_row=5'b0; endcase
        6'd4: case(row) 0:glyph_row=5'b00010; 1:glyph_row=5'b00110; 2:glyph_row=5'b01010; 3:glyph_row=5'b10010; 4:glyph_row=5'b11111; 5:glyph_row=5'b00010; 6:glyph_row=5'b00010; default:glyph_row=5'b0; endcase
        6'd5: case(row) 0:glyph_row=5'b11111; 1:glyph_row=5'b10000; 2:glyph_row=5'b10000; 3:glyph_row=5'b11110; 4:glyph_row=5'b00001; 5:glyph_row=5'b00001; 6:glyph_row=5'b11110; default:glyph_row=5'b0; endcase
        6'd6: case(row) 0:glyph_row=5'b01110; 1:glyph_row=5'b10000; 2:glyph_row=5'b10000; 3:glyph_row=5'b11110; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b01110; default:glyph_row=5'b0; endcase
        6'd7: case(row) 0:glyph_row=5'b11111; 1:glyph_row=5'b00001; 2:glyph_row=5'b00010; 3:glyph_row=5'b00100; 4:glyph_row=5'b01000; 5:glyph_row=5'b01000; 6:glyph_row=5'b01000; default:glyph_row=5'b0; endcase
        6'd8: case(row) 0:glyph_row=5'b01110; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b01110; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b01110; default:glyph_row=5'b0; endcase
        6'd9: case(row) 0:glyph_row=5'b01110; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b01111; 4:glyph_row=5'b00001; 5:glyph_row=5'b00001; 6:glyph_row=5'b01110; default:glyph_row=5'b0; endcase
        6'd10: case(row) 0:glyph_row=5'b01110; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b11111; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b10001; default:glyph_row=5'b0; endcase
        6'd11: case(row) 0:glyph_row=5'b11110; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b11110; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b11110; default:glyph_row=5'b0; endcase
        6'd12: case(row) 0:glyph_row=5'b01111; 1:glyph_row=5'b10000; 2:glyph_row=5'b10000; 3:glyph_row=5'b10000; 4:glyph_row=5'b10000; 5:glyph_row=5'b10000; 6:glyph_row=5'b01111; default:glyph_row=5'b0; endcase
        6'd13: case(row) 0:glyph_row=5'b11110; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b10001; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b11110; default:glyph_row=5'b0; endcase
        6'd14: case(row) 0:glyph_row=5'b11111; 1:glyph_row=5'b10000; 2:glyph_row=5'b10000; 3:glyph_row=5'b11110; 4:glyph_row=5'b10000; 5:glyph_row=5'b10000; 6:glyph_row=5'b11111; default:glyph_row=5'b0; endcase
        6'd15: case(row) 0:glyph_row=5'b11111; 1:glyph_row=5'b10000; 2:glyph_row=5'b10000; 3:glyph_row=5'b11110; 4:glyph_row=5'b10000; 5:glyph_row=5'b10000; 6:glyph_row=5'b10000; default:glyph_row=5'b0; endcase
        6'd16: case(row) 0:glyph_row=5'b10001; 1:glyph_row=5'b11011; 2:glyph_row=5'b10101; 3:glyph_row=5'b10101; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b10001; default:glyph_row=5'b0; endcase
        6'd17: case(row) 0:glyph_row=5'b11111; 1:glyph_row=5'b00100; 2:glyph_row=5'b00100; 3:glyph_row=5'b00100; 4:glyph_row=5'b00100; 5:glyph_row=5'b00100; 6:glyph_row=5'b00100; default:glyph_row=5'b0; endcase
        6'd18: case(row) 0:glyph_row=5'b10001; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b11111; 4:glyph_row=5'b10001; 5:glyph_row=5'b10001; 6:glyph_row=5'b10001; default:glyph_row=5'b0; endcase
        6'd19: case(row) 0:glyph_row=5'b10001; 1:glyph_row=5'b10001; 2:glyph_row=5'b10001; 3:glyph_row=5'b10101; 4:glyph_row=5'b10101; 5:glyph_row=5'b11011; 6:glyph_row=5'b10001; default:glyph_row=5'b0; endcase
        default: glyph_row = 5'b0;
        endcase
    end
endfunction

integer rel_x, rel_y, lrow, cidx, col, gy;
reg [5:0]  ch;
reg [63:0] grp;
reg [3:0]  nib;
reg        pix;
reg [4:0]  gr;
reg        in_box;
reg [23:0] composed;
always @* begin
    rel_x = px - X0;
    rel_y = py - Y0;
    in_box = (px >= X0 - 4) && (px < X0 + 340) && (py >= Y0 - 4) && (py < Y0 + 148);
    // 16x16 px cells: no dividers
    lrow = rel_y >> 4;
    cidx = rel_x >> 4;
    col = (rel_x >> 1) & 7;
    gy = (rel_y >> 1) & 7;
    grp = s1[(8 - lrow) * 64 +: 64];
    ch = 6'd63;
    nib = 4'd0;
    if (cidx == 0) ch = (lrow >= 3) ? 6'd11 : (lrow >= 1) ? 6'd19 : 6'd16;
    else if (cidx == 1) ch = (lrow >= 3) ? (lrow - 2) : (lrow == 2) ? 6'd2 : 6'd1;
    else if (cidx >= 3 && cidx <= 10) begin nib = grp[63 - (cidx-3)*4 -: 4]; ch = {2'b00, nib}; end
    else if (cidx >= 12 && cidx <= 15) begin nib = grp[31 - (cidx-12)*4 -: 4]; ch = {2'b00, nib}; end
    else if (cidx >= 17 && cidx <= 20) begin nib = grp[15 - (cidx-17)*4 -: 4]; ch = {2'b00, nib}; end
    gr = glyph_row(ch, gy[2:0]);
    pix = (rel_x >= 0) && (rel_x < 336) && (rel_y >= 0) && (rel_y < 144) &&
          (gy < 7) && (col < 5) && gr[4-col];
    composed = pix ? 24'hFFFF40 : (in_box ? {2'b00, rgb_in[23:18], 2'b00, rgb_in[15:10], 2'b00, rgb_in[7:2]} : rgb_in);
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s1 <= 576'd0;
        vs_prev <= 1'b0; de_prev <= 1'b0; px <= 0; py <= 0;
        hs_out <= 0; vs_out <= 0; de_out <= 0; rgb_out <= 0;
    end else begin
        s1 <= rows;
        vs_prev <= vs_in; de_prev <= de_in;
        if (vs_prev && !vs_in) begin px <= 0; py <= 0; end
        else begin
            if (!de_in) px <= 0; else if (px < 639) px <= px + 1'b1;
            if (de_prev && !de_in && py < 479) py <= py + 1'b1;
        end
        hs_out <= hs_in; vs_out <= vs_in; de_out <= de_in;
        rgb_out <= de_in ? composed : 24'd0;
    end
end
endmodule
