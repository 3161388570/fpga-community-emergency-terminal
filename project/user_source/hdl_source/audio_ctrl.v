`timescale 1ns/1ps

// Per-page WAV control (sd_card_clk domain).
//   page n   -> Nn.WAV, emergency -> N5.WAV (EM.WAV also accepted); every file loops.  A page's file restarts when the
//   page has finished fading in, after emergency enters/leaves, and when sound is unmuted.
//   Without the file, with SW2 off or with sound muted the built-in tones are used.
//   Auto-play period: (WAV bytes - 44) / 96000 s + 1 s of silence (+0.25 s for the fade-in),
//   or 4 s when no WAV is playing.
module audio_ctrl #(
    parameter integer CLK_FREQ_HZ = 100_000_000
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         wav_switch,       // SW2
    input  wire         sound_enabled,
    input  wire         emergency,
    input  wire [1:0]   current_index,
    input  wire         display_valid,
    input  wire         transition_busy,
    input  wire         scan_done,
    input  wire [4:0]   found_mask,
    input  wire [159:0] sizes,
    input  wire         stream_end,       // from the player, any clock domain
    output wire [2:0]   audio_select,
    output wire         wav_mode,         // WAV is the active source
    output wire         wav_allow,        // player may run (drops for a restart)
    output reg  [31:0]  auto_period_cycles
);

localparam [31:0] FALLBACK_CYCLES = CLK_FREQ_HZ * 6;     // fixed 6 s per page
localparam [31:0] TAIL_CYCLES     = CLK_FREQ_HZ + CLK_FREQ_HZ / 4;     // 1 s silence + fade-in

assign audio_select = emergency ? 3'd4 : {1'b0, current_index};
wire file_ok = found_mask[audio_select];
assign wav_mode = wav_switch && sound_enabled && scan_done && file_ok;

(* async_reg = "true" *) reg [1:0] end_sync;
reg        busy_previous, valid_previous, emergency_previous, sound_previous;
reg [1:0]  index_previous;
reg [15:0] hold_count;
reg [15:0] lockout_count;
wire restart = (busy_previous && !transition_busy) ||
               (display_valid && !valid_previous) ||
               (emergency != emergency_previous) ||
               (sound_enabled && !sound_previous) ||
               (index_previous != current_index && !transition_busy) ||
               (wav_mode && end_sync[1] && lockout_count == 16'd0);

assign wav_allow = (hold_count == 16'd0);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        end_sync <= 2'b00;
        busy_previous <= 1'b0;
        valid_previous <= 1'b0;
        emergency_previous <= 1'b0;
        sound_previous <= 1'b1;
        index_previous <= 2'd0;
        hold_count <= 16'hFFFF;
        lockout_count <= 16'hFFFF;
    end else begin
        end_sync <= {end_sync[0], stream_end};
        busy_previous <= transition_busy;
        valid_previous <= display_valid;
        emergency_previous <= emergency;
        sound_previous <= sound_enabled;
        index_previous <= current_index;
        if (restart) begin
            hold_count <= 16'd4096;
            lockout_count <= 16'hFFFF;
        end else begin
            if (hold_count != 16'd0) hold_count <= hold_count - 1'b1;
            if (lockout_count != 16'd0) lockout_count <= lockout_count - 1'b1;
        end
    end
end

// period = bytes * 1042 cycles (1041.67 exactly) + tail, pipelined
wire [31:0] page_size = sizes[current_index * 32 +: 32];
reg  [31:0] payload_bytes;
reg  [31:0] payload_cycles;
wire page_wav = wav_switch && sound_enabled && scan_done && found_mask[current_index];
always @(posedge clk or posedge rst) begin
    if (rst) begin
        payload_bytes <= 32'd0;
        payload_cycles <= 32'd0;
        auto_period_cycles <= FALLBACK_CYCLES;
    end else begin
        payload_bytes <= page_size - 32'd44;
        payload_cycles <= (payload_bytes << 10) + (payload_bytes << 4) + (payload_bytes << 1);
        auto_period_cycles <= FALLBACK_CYCLES;
    end
end

endmodule
