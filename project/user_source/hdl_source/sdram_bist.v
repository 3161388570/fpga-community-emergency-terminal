`timescale 1ns/1ps
// SDRAM controller self-test.  Runs once after reset and owns the controller
// port (the normal clients are held in reset by the top level while `active` is
// high).  It measures how the controller really behaves with deterministic traffic:
//   r_busy  Sdr_busy:        {busy clocks (32), rising edges (16), longest run (16)} in a 2^20-clock idle window
//   r_ref   Sdr_init_ref_vld: same statistics
//   r_s0    writes at random spacing, read back after a long idle:
//           {bad words (32), bad bursts (16), bursts (16)}
//   r_s1    read right after a write, gap g = 0..7 clocks: bad bursts per gap (8 x 8 bit, g=0 first)
//   r_s2    write right after a read, gap g in {0,3,6,9,12,15,20,30}: bad bursts per gap (8 x 8 bit)
//   r_stat  {total clocks (32), sequencer state (16), done (16)}
// A burst is 256 consecutive words (half of them start in the middle of a row).
// Pattern word = {salt[10:0], address[20:0]} ^ 32'hA5C30F5A, so stale data never matches.
module sdram_bist (
    input  wire        clk,
    input  wire        rst,
    input  wire        Sdr_init_done,
    input  wire        Sdr_busy,
    input  wire        Sdr_ref_vld,
    input  wire        Sdr_rd_en,
    input  wire [31:0] Sdr_rd_dout,
    output reg         wr_en,
    output reg  [20:0] wr_addr,
    output reg  [31:0] wr_din,
    output reg         rd_en,
    output reg  [20:0] rd_addr,
    output reg         active,
    output reg         done,
    output wire [63:0] r_busy,
    output wire [63:0] r_ref,
    output wire [63:0] r_s0,
    output wire [63:0] r_s1,
    output wire [63:0] r_s2,
    output wire [63:0] r_stat
);
localparam [20:0] BASE0 = 21'h080000;      // test area: 8 regions of 1024 words
function [31:0] pat;
    input [20:0] a;
    input [10:0] s;
    begin pat = {s, a} ^ 32'hA5C30F5A; end
endfunction
function [20:0] region_base;
    input [2:0] r;
    input       ofs;
    begin region_base = BASE0 + {8'd0, r, 10'd0} + (ofs ? 21'd128 : 21'd0); end
endfunction

// ------------------------------------------------------------------ burst engine
localparam E_IDLE = 0, E_ISSUE = 1, E_DRAIN = 2, E_DONE = 3;
reg [1:0]  e_st;
reg        e_go, e_op, e_check;           // op: 0 write, 1 read
reg [20:0] e_base;
reg [10:0] e_salt;
reg [8:0]  e_cnt;
reg [7:0]  e_dcnt;
reg [8:0]  e_nret;
reg [8:0]  e_mism;
reg        e_done;
reg        e_bad;
reg [8:0]  e_badw;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        e_st <= E_IDLE; wr_en <= 0; rd_en <= 0; wr_addr <= 0; rd_addr <= 0; wr_din <= 0;
        e_cnt <= 0; e_dcnt <= 0; e_done <= 0; e_bad <= 0; e_badw <= 0; e_nret <= 0; e_mism <= 0;
    end else begin
        e_done <= 1'b0;
        case (e_st)
            E_IDLE: begin
                wr_en <= 0; rd_en <= 0;
                if (e_go) begin e_st <= E_ISSUE; e_cnt <= 0; e_nret <= 0; e_mism <= 0; end
            end
            E_ISSUE: begin
                if (!e_op) begin
                    wr_en <= 1; wr_addr <= e_base + e_cnt; wr_din <= pat(e_base + e_cnt, e_salt);
                end else begin
                    rd_en <= 1; rd_addr <= e_base + e_cnt;
                end
                e_cnt <= e_cnt + 1'b1;
                if (e_cnt == 9'd255) begin e_st <= E_DRAIN; e_dcnt <= 0; end
            end
            E_DRAIN: begin
                wr_en <= 0; rd_en <= 0;
                e_dcnt <= e_dcnt + 1'b1;
                if (e_dcnt == 8'd100) begin
                    e_badw <= e_mism + (9'd256 - e_nret);
                    e_bad  <= e_check && (e_mism != 0 || e_nret != 9'd256);
                    e_st <= E_DONE;
                end
            end
            E_DONE: begin e_done <= 1'b1; e_st <= E_IDLE; end
        endcase
        // return checker (reads come back in order)
        if (e_op && Sdr_rd_en && (e_st == E_ISSUE || e_st == E_DRAIN) && e_nret != 9'd256) begin
            if (Sdr_rd_dout != pat(e_base + e_nret, e_salt)) e_mism <= e_mism + 1'b1;
            e_nret <= e_nret + 1'b1;
        end
    end
end

// ------------------------------------------------------------------ idle-window statistics
reg [31:0] win_cnt, busy_cycles, ref_cycles;
reg [15:0] busy_rises, ref_rises, busy_len, ref_len, busy_max, ref_max;
reg        busy_q, ref_q;
reg        win_run;
wire       win_end = (win_cnt == 32'h000FFFFF);
always @(posedge clk or posedge rst) begin
    if (rst) begin
        win_cnt <= 0; busy_cycles <= 0; ref_cycles <= 0; busy_rises <= 0; ref_rises <= 0;
        busy_len <= 0; ref_len <= 0; busy_max <= 0; ref_max <= 0; busy_q <= 0; ref_q <= 0;
    end else if (win_run) begin
        win_cnt <= win_cnt + 1'b1;
        busy_q <= Sdr_busy; ref_q <= Sdr_ref_vld;
        if (Sdr_busy) begin
            busy_cycles <= busy_cycles + 1'b1;
            busy_len <= busy_len + 1'b1;
            if (!busy_q) busy_rises <= busy_rises + 1'b1;
            if (busy_len + 1'b1 > busy_max) busy_max <= busy_len + 1'b1;
        end else busy_len <= 0;
        if (Sdr_ref_vld) begin
            ref_cycles <= ref_cycles + 1'b1;
            ref_len <= ref_len + 1'b1;
            if (!ref_q) ref_rises <= ref_rises + 1'b1;
            if (ref_len + 1'b1 > ref_max) ref_max <= ref_len + 1'b1;
        end else ref_len <= 0;
    end
end
assign r_busy = {busy_cycles, busy_rises, busy_max};
assign r_ref  = {ref_cycles, ref_rises, ref_max};

// ------------------------------------------------------------------ scenario sequencer
// Every step: ph0 start the burst, ph1 wait for it to finish (record result), ph2 idle gap.
localparam [4:0]
    ST_WAIT = 0, ST_WIN = 1,
    S0_W = 2, S0_WAIT = 3, S0_R = 4,
    S1_W = 5, S1_R = 6,
    S2_R1 = 7, S2_W = 8, S2_R2 = 9,
    ST_DONE = 10;
reg [4:0]  st;
reg [1:0]  ph;
reg [31:0] total;
reg [15:0] gapc;
reg [2:0]  reg_i, rnd;
reg [3:0]  gi;
reg [3:0]  rep;
reg [10:0] salt;
reg [10:0] salts [0:7];
reg [15:0] lfsr;
reg [31:0] s0_badw;
reg [15:0] s0_badb, s0_n;
reg [7:0]  s1c [0:7];
reg [7:0]  s2c [0:7];
reg [10:0] last_salt;
reg        u1, u2;
reg [2:0]  uidx;
integer k;
assign r_s0 = {s0_badw, s0_badb, s0_n};
assign r_s1 = {s1c[0], s1c[1], s1c[2], s1c[3], s1c[4], s1c[5], s1c[6], s1c[7]};
assign r_s2 = {s2c[0], s2c[1], s2c[2], s2c[3], s2c[4], s2c[5], s2c[6], s2c[7]};
assign r_stat = {total, 11'd0, st, 15'd0, done};

function [5:0] g2;
    input [3:0] i;
    begin
        case (i)
            4'd0: g2 = 6'd0;  4'd1: g2 = 6'd3;  4'd2: g2 = 6'd6;  4'd3: g2 = 6'd9;
            4'd4: g2 = 6'd12; 4'd5: g2 = 6'd15; 4'd6: g2 = 6'd20; default: g2 = 6'd30;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st <= ST_WAIT; ph <= 0; active <= 1'b1; done <= 1'b0; win_run <= 0; total <= 0; gapc <= 0;
        reg_i <= 0; rnd <= 0; gi <= 0; rep <= 0; salt <= 11'd1; lfsr <= 16'hACE1;
        s0_badw <= 0; s0_badb <= 0; s0_n <= 0; e_go <= 0; e_op <= 0; e_check <= 0; e_base <= 0; e_salt <= 0;
        last_salt <= 0; u1 <= 0; u2 <= 0; uidx <= 0;
        for (k = 0; k < 8; k = k + 1) begin s1c[k] <= 0; s2c[k] <= 0; salts[k] <= 0; end
    end else begin
        e_go <= 1'b0;
        u1 <= 1'b0; u2 <= 1'b0;
        // registered, decoded counter update (keeps the enable path short)
        for (k = 0; k < 8; k = k + 1) begin
            if (u1 && uidx == k && s1c[k] != 8'hFF) s1c[k] <= s1c[k] + 1'b1;
            if (u2 && uidx == k && s2c[k] != 8'hFF) s2c[k] <= s2c[k] + 1'b1;
        end
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        if (st != ST_DONE && st != ST_WAIT) total <= total + 1'b1;
        case (st)
            ST_WAIT: if (Sdr_init_done) begin win_run <= 1'b1; st <= ST_WIN; end
            ST_WIN: if (win_end) begin win_run <= 1'b0; st <= S0_W; ph <= 0; reg_i <= 0; rnd <= 0; end

            // ---- S0: eight rounds of eight writes at random spacing, read back after a long idle
            S0_W: case (ph)
                0: begin e_op <= 0; e_check <= 0; e_go <= 1; e_base <= region_base(reg_i, rnd[0]);
                         e_salt <= salt; salts[reg_i] <= salt; salt <= salt + 1'b1; ph <= 1; end
                1: if (e_done) begin gapc <= {6'd0, lfsr[9:0]}; ph <= 2; end
                2: if (gapc != 0) gapc <= gapc - 1'b1;
                   else begin
                       ph <= 0;
                       if (reg_i == 3'd7) begin st <= S0_WAIT; gapc <= 16'd3000; end
                       else reg_i <= reg_i + 1'b1;
                   end
                default: ph <= 0;
            endcase
            S0_WAIT: if (gapc != 0) gapc <= gapc - 1'b1; else begin st <= S0_R; ph <= 0; reg_i <= 0; end
            S0_R: case (ph)
                0: begin e_op <= 1; e_check <= 1; e_go <= 1; e_base <= region_base(reg_i, rnd[0]);
                         e_salt <= salts[reg_i]; ph <= 1; end
                1: if (e_done) begin
                       s0_n <= s0_n + 1'b1;
                       if (e_bad) begin s0_badb <= s0_badb + 1'b1; s0_badw <= s0_badw + {23'd0, e_badw}; end
                       gapc <= 16'd300; ph <= 2;
                   end
                2: if (gapc != 0) gapc <= gapc - 1'b1;
                   else begin
                       ph <= 0;
                       if (reg_i != 3'd7) reg_i <= reg_i + 1'b1;
                       else if (rnd != 3'd7) begin rnd <= rnd + 1'b1; reg_i <= 0; st <= S0_W; end
                       else begin gi <= 0; rep <= 0; st <= S1_W; end
                   end
                default: ph <= 0;
            endcase

            // ---- S1: read right after a write, gap = gi clocks
            S1_W: case (ph)
                0: begin e_op <= 0; e_check <= 0; e_go <= 1; e_base <= region_base(rep[2:0], rep[0]);
                         e_salt <= salt; last_salt <= salt; salt <= salt + 1'b1; ph <= 1; end
                1: if (e_done) begin gapc <= {12'd0, gi}; ph <= 2; end
                2: if (gapc != 0) gapc <= gapc - 1'b1; else begin st <= S1_R; ph <= 0; end
                default: ph <= 0;
            endcase
            S1_R: case (ph)
                0: begin e_op <= 1; e_check <= 1; e_go <= 1; e_base <= region_base(rep[2:0], rep[0]);
                         e_salt <= last_salt; ph <= 1; end
                1: if (e_done) begin
                       u1 <= e_bad; uidx <= gi[2:0];
                       gapc <= 16'd300; ph <= 2;
                   end
                2: if (gapc != 0) gapc <= gapc - 1'b1;
                   else begin
                       ph <= 0;
                       if (rep != 4'd7) begin rep <= rep + 1'b1; st <= S1_W; end
                       else if (gi != 4'd7) begin gi <= gi + 1'b1; rep <= 0; st <= S1_W; end
                       else begin gi <= 0; rep <= 0; st <= S2_R1; end
                   end
                default: ph <= 0;
            endcase

            // ---- S2: write right after a read, gap = g2(gi) clocks
            S2_R1: case (ph)
                0: begin e_op <= 1; e_check <= 0; e_go <= 1; e_base <= region_base(rep[2:0] + 3'd1, 1'b0);
                         e_salt <= 0; ph <= 1; end
                1: if (e_done) begin gapc <= {10'd0, g2(gi)}; ph <= 2; end
                2: if (gapc != 0) gapc <= gapc - 1'b1; else begin st <= S2_W; ph <= 0; end
                default: ph <= 0;
            endcase
            S2_W: case (ph)
                0: begin e_op <= 0; e_check <= 0; e_go <= 1; e_base <= region_base(rep[2:0], rep[0]);
                         e_salt <= salt; last_salt <= salt; salt <= salt + 1'b1; ph <= 1; end
                1: if (e_done) begin gapc <= 16'd2000; ph <= 2; end
                2: if (gapc != 0) gapc <= gapc - 1'b1; else begin st <= S2_R2; ph <= 0; end
                default: ph <= 0;
            endcase
            S2_R2: case (ph)
                0: begin e_op <= 1; e_check <= 1; e_go <= 1; e_base <= region_base(rep[2:0], rep[0]);
                         e_salt <= last_salt; ph <= 1; end
                1: if (e_done) begin
                       u2 <= e_bad; uidx <= gi[2:0];
                       gapc <= 16'd300; ph <= 2;
                   end
                2: if (gapc != 0) gapc <= gapc - 1'b1;
                   else begin
                       ph <= 0;
                       if (rep != 4'd7) begin rep <= rep + 1'b1; st <= S2_R1; end
                       else if (gi != 4'd7) begin gi <= gi + 1'b1; rep <= 0; st <= S2_R1; end
                       else st <= ST_DONE;
                   end
                default: ph <= 0;
            endcase

            ST_DONE: begin active <= 1'b0; done <= 1'b1; end
            default: st <= ST_WAIT;
        endcase
    end
end
endmodule
