`timescale 1ns/1ps

module display_mode_control (
    input  wire       clk,
    input  wire       rst,
    input  wire       vsync,
    input  wire       thumbnail_switch_async,
    input  wire       wipe_hold,
    input  wire       settings_mode_async,
    input  wire       emergency_mode_async,
    input  wire [3:0] cached_images_async,
    output reg        thumbnail_mode,
    output reg        thumbnail_frame_start
);

(* async_reg = "true" *) reg [1:0] thumbnail_switch_sync;
(* async_reg = "true" *) reg [1:0] settings_mode_sync;
(* async_reg = "true" *) reg [1:0] emergency_mode_sync;
(* async_reg = "true" *) reg [3:0] cached_images_sync_first;
(* async_reg = "true" *) reg [3:0] cached_images_sync_second;
reg thumbnail_switch_late;     // one stage later than the wipe hold, which must arrive first
reg vsync_previous;

wire frame_start = vsync_previous && !vsync;
wire thumbnail_request = (thumbnail_switch_late || wipe_hold) &&
                         (&cached_images_sync_second) &&
                         !settings_mode_sync[1] &&
                         !emergency_mode_sync[1];

always @(posedge clk or posedge rst) begin
    if (rst) begin
        thumbnail_switch_sync <= 2'b00;
        thumbnail_switch_late <= 1'b0;
        settings_mode_sync <= 2'b00;
        emergency_mode_sync <= 2'b00;
        cached_images_sync_first <= 4'b0000;
        cached_images_sync_second <= 4'b0000;
        vsync_previous <= 1'b1;
        thumbnail_mode <= 1'b0;
        thumbnail_frame_start <= 1'b0;
    end else begin
        thumbnail_switch_sync <= {thumbnail_switch_sync[0], thumbnail_switch_async};
        thumbnail_switch_late <= thumbnail_switch_sync[1];
        settings_mode_sync <= {settings_mode_sync[0], settings_mode_async};
        emergency_mode_sync <= {emergency_mode_sync[0], emergency_mode_async};
        cached_images_sync_first <= cached_images_async;
        cached_images_sync_second <= cached_images_sync_first;
        vsync_previous <= vsync;
        thumbnail_frame_start <= frame_start && thumbnail_request;
        if (frame_start)
            thumbnail_mode <= thumbnail_request;
    end
end

endmodule
