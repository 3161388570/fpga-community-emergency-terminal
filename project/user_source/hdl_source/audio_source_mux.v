`timescale 1ns/1ps

module audio_source_mux (
    input  wire        wav_active,
    input  wire        wav_valid,
    input  wire [23:0] wav_left,
    input  wire [23:0] wav_right,
    input  wire        test_valid,
    input  wire [23:0] test_left,
    input  wire [23:0] test_right,
    input  wire        media_valid,
    input  wire [23:0] media_left,
    input  wire [23:0] media_right,
    input  wire [1:0]  source_mode,
    output reg         audio_valid,
    output reg  [23:0] audio_left,
    output reg  [23:0] audio_right,
    output reg  [1:0]  active_source
);

localparam [1:0] SOURCE_MEDIA = 2'd0;
localparam [1:0] SOURCE_TEST  = 2'd1;
localparam [1:0] SOURCE_WAV   = 2'd2;

always @* begin
    audio_valid   = 1'b0;
    audio_left    = 24'd0;
    audio_right   = 24'd0;
    active_source = SOURCE_MEDIA;

    if (wav_active) begin
        audio_valid   = wav_valid;
        audio_left    = wav_left;
        audio_right   = wav_right;
        active_source = SOURCE_WAV;
    end else if (source_mode == SOURCE_TEST && test_valid) begin
        audio_valid   = 1'b1;
        audio_left    = test_left;
        audio_right   = test_right;
        active_source = SOURCE_TEST;
    end else if (media_valid) begin
        audio_valid = 1'b1;
        audio_left  = media_left;
        audio_right = media_right;
    end
end

endmodule
