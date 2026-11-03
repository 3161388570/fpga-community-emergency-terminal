module hdmi_audio_tone #(
    parameter integer CLK_FREQ_HZ    = 25_000_000,
    parameter integer SAMPLE_RATE_HZ = 48_000,
    parameter integer TONE_FREQ_HZ   = 1_000,
    parameter signed [23:0] AMPLITUDE = 24'sd2_000_000,
    parameter integer MEDIA_LINK = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [1:0]  media_id,
    input  wire        media_valid,
    output reg         audio_valid,
    output reg [23:0]  audio_left,
    output reg [23:0]  audio_right
);

localparam integer SAMPLE_ACC_WIDTH = $clog2(CLK_FREQ_HZ + SAMPLE_RATE_HZ);
localparam [63:0] PHASE_STEP = (64'd4294967296 * TONE_FREQ_HZ + SAMPLE_RATE_HZ / 2) / SAMPLE_RATE_HZ;
reg [SAMPLE_ACC_WIDTH-1:0] sample_acc;
reg [31:0] phase_acc;
(* async_reg = "true" *) reg enable_meta;
(* async_reg = "true" *) reg enable_sync;
reg [6:0] gain;
(* async_reg = "true" *) reg [1:0] media_meta;
(* async_reg = "true" *) reg [1:0] media_sync;
(* async_reg = "true" *) reg [1:0] valid_sync;
reg [1:0] previous_media;
reg previous_media_valid;
localparam integer BLIP_SAMPLES = SAMPLE_RATE_HZ / 10;
localparam integer BLIP_WIDTH = $clog2(BLIP_SAMPLES + 1);
reg [BLIP_WIDTH-1:0] blip_remaining;
reg [2:0] active_tone;
wire [2:0] requested_tone = (MEDIA_LINK == 0) ? 3'd0 :
                            (blip_remaining != 0 ? 3'd4 : {1'b0, media_sync});
wire tone_changing = requested_tone != active_tone;
wire tone_enabled = enable_sync && ((MEDIA_LINK == 0) || valid_sync[1]);
wire [31:0] current_step = (MEDIA_LINK == 0) ? PHASE_STEP[31:0] : tone_step(active_tone);
wire [SAMPLE_ACC_WIDTH:0] sample_sum = {1'b0, sample_acc} + SAMPLE_RATE_HZ;
wire signed [11:0] sine_unit = sine_lookup(phase_acc);
wire signed [35:0] scaled_sine = sine_unit * AMPLITUDE;
wire signed [23:0] sine_value = scaled_sine >>> 10;
wire signed [31:0] faded_sample = sine_value * $signed({1'b0, gain});

function [31:0] tone_step;
    input [2:0] tone;
    begin
        case (tone)
            3'd0: tone_step = (64'd4294967296 * 523 + SAMPLE_RATE_HZ / 2) / SAMPLE_RATE_HZ;
            3'd1: tone_step = (64'd4294967296 * 659 + SAMPLE_RATE_HZ / 2) / SAMPLE_RATE_HZ;
            3'd2: tone_step = (64'd4294967296 * 784 + SAMPLE_RATE_HZ / 2) / SAMPLE_RATE_HZ;
            3'd3: tone_step = (64'd4294967296 * 1046 + SAMPLE_RATE_HZ / 2) / SAMPLE_RATE_HZ;
            default: tone_step = (64'd4294967296 * 1500 + SAMPLE_RATE_HZ / 2) / SAMPLE_RATE_HZ;
        endcase
    end
endfunction

function signed [11:0] sine_lookup;
    input [31:0] phase;
    reg [4:0] table_index;
    reg signed [11:0] magnitude;
    begin
        table_index = phase[30] ? 5'd16 - {1'b0, phase[29:26]} : {1'b0, phase[29:26]};
        case (table_index)
            0: magnitude = 0;
            1: magnitude = 100;
            2: magnitude = 200;
            3: magnitude = 297;
            4: magnitude = 392;
            5: magnitude = 483;
            6: magnitude = 569;
            7: magnitude = 650;
            8: magnitude = 724;
            9: magnitude = 792;
            10: magnitude = 851;
            11: magnitude = 903;
            12: magnitude = 946;
            13: magnitude = 980;
            14: magnitude = 1004;
            15: magnitude = 1019;
            16: magnitude = 1024;
            default: magnitude = 0;
        endcase
        sine_lookup = phase[31] ? -magnitude : magnitude;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        enable_meta <= 1'b0;
        enable_sync <= 1'b0;
        media_meta <= 2'd0;
        media_sync <= 2'd0;
        valid_sync <= 2'b00;
    end else begin
        enable_meta <= enable;
        enable_sync <= enable_meta;
        media_meta <= media_id;
        media_sync <= media_meta;
        valid_sync <= {valid_sync[0], media_valid};
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sample_acc   <= 0;
        phase_acc    <= 32'd0;
        gain         <= 7'd0;
        active_tone <= 3'd0;
        previous_media <= 2'd0;
        previous_media_valid <= 1'b0;
        blip_remaining <= 0;
        audio_valid  <= 1'b0;
        audio_left   <= 24'd0;
        audio_right  <= 24'd0;
    end else begin
        audio_valid <= 1'b0;
        if (sample_sum >= CLK_FREQ_HZ) begin
            sample_acc  <= sample_sum - CLK_FREQ_HZ;
            audio_valid <= 1'b1;
            previous_media <= media_sync;
            previous_media_valid <= valid_sync[1];
            if ((MEDIA_LINK != 0) && valid_sync[1] &&
                (!previous_media_valid || previous_media != media_sync))
                blip_remaining <= BLIP_SAMPLES;
            else if (blip_remaining != 0)
                blip_remaining <= blip_remaining - 1'b1;
            if (tone_changing && gain == 0) begin
                active_tone <= requested_tone;
                phase_acc <= 32'd0;
            end else if (tone_enabled || gain != 0)
                phase_acc <= phase_acc + current_step;
            else
                phase_acc <= 32'd0;
            if (tone_enabled && !tone_changing && gain < 64)
                gain <= gain + 1'b1;
            else if ((!tone_enabled || tone_changing) && gain != 0)
                gain <= gain - 1'b1;
            audio_left  <= faded_sample >>> 6;
            audio_right <= faded_sample >>> 6;
        end else begin
            sample_acc <= sample_sum;
        end
    end
end

endmodule
