// Sound on/off key and the emergency buzzer.  Resident mode: the buzzer stays silent,
// KEY3 only gates the audio.  Emergency: 1 Hz intermittent beep (first half second of
// every second, in step with the flashing frame), and KEY3 gates audio and buzzer together.
module buzzer_ctrl #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BEEP_MS     = 100,
    parameter integer ERROR_MS    = 500
)(
    input  wire clk,
    input  wire rst,
    input  wire key_toggle,
    input  wire emergency,
    input  wire image_changed,
    input  wire load_error,
    output wire sound_enabled_out,
    output wire buzzer
);

localparam integer HALF_SECOND = CLK_FREQ_HZ / 2;

wire key_press;
reg  sound_enabled;
reg [31:0] phase_count;
reg        beep_phase;
reg        emergency_previous;
(* async_reg = "true" *) reg [1:0] emergency_sync;

assign sound_enabled_out = sound_enabled;
assign buzzer = !(emergency_sync[1] && sound_enabled && beep_phase);   // active low

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_buzzer (
    .clk         (clk),
    .rst         (rst),
    .button_in   (key_toggle),
    .press_pulse (key_press)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sound_enabled <= 1'b1;
        phase_count <= 32'd0;
        beep_phase <= 1'b1;
        emergency_previous <= 1'b0;
        emergency_sync <= 2'b00;
    end else begin
        emergency_sync <= {emergency_sync[0], emergency};
        emergency_previous <= emergency_sync[1];
        if (key_press)
            sound_enabled <= ~sound_enabled;
        if (emergency_sync[1] && !emergency_previous) begin
            phase_count <= 32'd0;
            beep_phase <= 1'b1;
        end else if (phase_count == HALF_SECOND - 1) begin
            phase_count <= 32'd0;
            beep_phase <= ~beep_phase;
        end else
            phase_count <= phase_count + 1'b1;
    end
end

endmodule
