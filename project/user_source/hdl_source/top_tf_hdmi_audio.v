
module top(
    input                       clk,
    input                       sw1,
    input                       sw2,
    input                       sw3,
    input                       sw4,
    input                       rst_n,          // 实体 KEY4：系统复位
    input                       key1,           // 实体 KEY1：下一张/调节减
    input                       key2,           // 实体 KEY2：轮播/上一张/调节加
    input                       key3,           // 实体 KEY3：声音开关

    output                      buzzer,

	output [7:0]                seg_sel,
	output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter TEST_SEQ = 0;   // test builds: replay a timed script of switch/key actions
wire t_sw1, t_sw2, t_sw3, t_sw4, t_key1, t_key2, t_key3;
generate if (TEST_SEQ) begin : g_seq
    test_sequencer u_test_sequencer (.clk(clk), .rst_n(rst_n),
        .sw1(t_sw1), .sw2(t_sw2), .sw3(t_sw3), .sw4(t_sw4),
        .key1(t_key1), .key2(t_key2), .key3(t_key3));
end else begin : g_noseq
    assign t_sw1 = 1'b0; assign t_sw2 = 1'b0; assign t_sw3 = 1'b0; assign t_sw4 = 1'b0;
    assign t_key1 = 1'b1; assign t_key2 = 1'b1; assign t_key3 = 1'b1;
end endgenerate
wire i_sw1  = TEST_SEQ ? t_sw1  : sw1;
wire i_sw2  = TEST_SEQ ? (t_sw2 | sw2) : sw2;
wire i_sw3  = TEST_SEQ ? t_sw3  : sw3;
wire i_sw4  = TEST_SEQ ? t_sw4  : sw4;
wire i_key1 = TEST_SEQ ? t_key1 : key1;
wire i_key2 = TEST_SEQ ? t_key2 : key2;
wire i_key3 = TEST_SEQ ? t_key3 : key3;

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 21'd307200;   // 640*480
parameter BUF0_ADDR     = 21'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
parameter BUF2_ADDR     = FRAME_PIXELS * 2;
parameter BUF3_ADDR     = FRAME_PIXELS * 3;
parameter THUMBNAIL_ADDR = FRAME_PIXELS * 4;
parameter [1:0] AUDIO_SOURCE_MODE = 2'd0;
parameter AUTO_THUMBNAIL = 1'b0;   // diagnostic builds: force the four-up preview without touching SW1
parameter THUMBNAIL_ENABLE = 1;   // 2x2 box-filtered montage is written to THUMBNAIL_ADDR while images load and shown by the main reader
parameter THUMBNAIL_SAMPLE_X_STEP = 2;
parameter THUMBNAIL_SAMPLE_Y_STEP = 2;
parameter THUMBNAIL_OUTPUT_X_SCALE = 1;
parameter THUMBNAIL_OUTPUT_Y_SCALE = 1;
parameter AUDIO_VISUALIZER_ENABLE = 1;
parameter integer AUDIO_VISUALIZER_STYLE = 2; // 1 waveform, 2 FFT spectrum

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;
wire        image_changed;
wire        load_error;
wire [2:0]  image_count;
wire [1:0]  current_index;
wire        auto_play_active;
wire        sound_enabled;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        media_audio_valid;
wire [23:0] media_audio_left;
wire [23:0] media_audio_right;
wire        test_audio_valid;
wire [23:0] test_audio_left;
wire [23:0] test_audio_right;
wire [1:0]  active_audio_source;
wire        audio_file_found;
wire [31:0] audio_first_cluster;
wire [31:0] audio_file_size;
wire [31:0] audio_fat_start_sector;
wire [31:0] audio_data_start_sector;
wire [7:0]  audio_sectors_per_cluster;
wire        audio_sd_req;
wire [31:0] audio_sd_lba;
wire        audio_sd_grant;
wire        audio_sd_busy;
wire [7:0]  audio_sd_data;
wire        audio_sd_data_valid;
wire [8:0]  audio_sd_byte_index;
wire        audio_sd_end;
wire signed [15:0] wav_sample;
wire        wav_sample_valid;
wire [31:0] wav_sample_counter;
wire        wav_stream_active;
wire        wav_stream_end;
wire        wav_buffer_low;
wire        media_scan_done;
wire        sys_pll_lock;
wire        video_pll_lock;
wire        rst_all;
wire        rst_sd;
wire        rst_mem;
wire        rst_video;
wire        wav_selected;
wire        wav_play_enable;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;
wire [6:0]  seg_data_1;
wire [6:0]  seg_data_2;
wire [6:0]  seg_data_3;
wire [6:0]  seg_data_4;
wire osd_active;
(* async_reg = "true" *) reg [1:0] osd_status_sync;
always @(posedge clk or posedge rst_all) begin
    if (rst_all) osd_status_sync <= 0;
    else osd_status_sync <= {osd_status_sync[0], osd_active};
end

wire settings_mode;
wire selected_param;
wire [2:0] brightness_level;
wire [2:0] contrast_level;
wire cfg_toggle;
wire navigation_enabled;
wire emergency_mode;
wire key_next_event;
wire key_auto_short_event;
wire key_previous_event;
wire key_decrease_event;
wire key_increase_event;
wire key_ov_move_event;
wire key_ov_confirm_event;
wire overview_mode;
wire wipe_toggle;
wire wipe_hold;
wire [1:0] ov_sel;
wire jump_req;
wire [1:0] jump_index;
wire transition_busy;
(* async_reg = "true" *) reg [1:0] transition_busy_sd;
(* async_reg = "true" *) reg [1:0] ov_sel_v0;
reg [1:0] ov_sel_v1;
wire [31:0] auto_period_cycles;
wire [2:0] audio_select;
wire [4:0] audio_found_mask;
wire [159:0] audio_sizes;
wire wav_mode;
wire wav_allow;
wire [7:0] seg_p0, seg_p1, seg_p2, seg_p3, seg_p4, seg_p5, seg_p6, seg_p7;
always @(posedge sd_card_clk or posedge rst_sd) begin
    if (rst_sd) transition_busy_sd <= 2'b00;
    else transition_busy_sd <= {transition_busy_sd[0], transition_busy};
end
always @(posedge video_clk or posedge rst_video) begin
    if (rst_video) begin ov_sel_v0 <= 2'b00; ov_sel_v1 <= 2'b00; end
    else begin ov_sel_v0 <= ov_sel; ov_sel_v1 <= ov_sel_v0; end
end

wire        video_read_req;
wire        video_read_req_ack;
wire        main_read_en;
wire        thumbnail_read_en;
wire [31:0] main_video_read_data;
wire        main_fifo_empty;
wire [31:0] thumbnail_fifo_data;
wire [23:0] main_vout_data;
wire [23:0] thumbnail_vout_data;
wire        main_hs_r;
wire        main_vs_r;
wire        main_de_r;
wire        thumbnail_hs_r;
wire        thumbnail_vs_r;
wire        thumbnail_de_r;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;
wire [3:0] cached_images;
wire thumbnail_ready_mem;
wire thumbnail_overflow;
wire [3:0] thumbnail_mask_mem;
(* async_reg = "true" *) reg [1:0] thumbnail_mode_mem_sync;
(* async_reg = "true" *) reg [1:0] thumb_owner_video_sync;
wire thumbnail_mode;
wire thumbnail_frame_start;
wire display_request_toggle;
wire display_ack_toggle;
wire [1:0] reader_bank;
wire reader_display_valid;
wire reader_frame_req;

wire main_App_rd_en;
wire [ADDR_BITS-1:0] main_App_rd_addr;
wire thumb_App_rd_en;
wire [ADDR_BITS-1:0] thumb_App_rd_addr;
wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;
// overview = the pre-built 2x2 montage frame, read like any other frame
wire [ADDR_BITS-1:0] video_read_addr_0 = thumbnail_mode ? THUMBNAIL_ADDR : BUF0_ADDR;
wire [ADDR_BITS-1:0] video_read_addr_1 = thumbnail_mode ? THUMBNAIL_ADDR : BUF1_ADDR;
wire [ADDR_BITS-1:0] video_read_addr_2 = thumbnail_mode ? THUMBNAIL_ADDR : BUF2_ADDR;
wire [ADDR_BITS-1:0] video_read_addr_3 = thumbnail_mode ? THUMBNAIL_ADDR : BUF3_ADDR;

wire thumb_mode_mem = thumbnail_mode_mem_sync[1];
wire thumb_pixels_ready;
wire thumb_reader_idle;
reg thumb_owner;
reg [10:0] thumb_switch_delay;
reg [8:0] main_drain_delay;
reg [8:0] thumb_exit_delay;
wire thumb_owner_video = thumb_owner_video_sync[1];
wire main_Sdr_rd_en = Sdr_rd_en && !thumb_owner;

assign App_rd_en = thumb_owner ? thumb_App_rd_en : main_App_rd_en;
assign App_rd_addr = thumb_owner ? thumb_App_rd_addr : main_App_rd_addr;

// HDMI 1.4b 相关
wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;
reg         video_source_started;
wire        video_source_rst = rst_video || !video_source_started;

always @(posedge video_clk or posedge rst_video) begin
    if (rst_video)
        video_source_started <= 1'b0;
    else if (edid_trig)
        video_source_started <= 1'b1;
end

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

boot_reset_controller boot_reset_m0 (
    .ref_clk(clk),
    .sd_clk(sd_card_clk),
    .mem_clk(ext_mem_clk),
    .video_clk(video_clk),
    .rst_n(rst_n),
    .sys_locked(sys_pll_lock),
    .video_locked(video_pll_lock),
    .mem_ready(Sdr_init_done),
    .rst_ref(rst_all),
    .rst_sd(rst_sd),
    .rst_mem(rst_mem),
    .rst_video(rst_video)
);

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .reset      (~rst_n),
    .extlock    (sys_pll_lock),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .reset      (~rst_n),
    .extlock    (video_pll_lock),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk)
);

display_mode_control u_display_mode_control (
    .clk                   (video_clk),
    .rst                   (rst_video),
    .vsync                 (vs_0),
    .thumbnail_switch_async(overview_mode | AUTO_THUMBNAIL),
    .wipe_hold             (wipe_hold),
    .settings_mode_async   (settings_mode),
    .emergency_mode_async  (emergency_mode),
    .cached_images_async   (cached_images),
    .thumbnail_mode        (thumbnail_mode),
    .thumbnail_frame_start (thumbnail_frame_start)
);

always @(posedge ext_mem_clk or posedge rst_mem) begin
    if (rst_mem)
        thumbnail_mode_mem_sync <= 2'b00;
    else
        thumbnail_mode_mem_sync <= {thumbnail_mode_mem_sync[0], thumbnail_mode};
end

always @(posedge ext_mem_clk or posedge rst_mem) begin
    if (rst_mem) begin
        thumb_owner <= 1'b0;
        thumb_switch_delay <= 11'd0;
        main_drain_delay <= 9'd0;
        thumb_exit_delay <= 9'd0;
    end else if (thumb_mode_mem) begin
        thumb_exit_delay <= 9'd0;
        if (!thumb_owner) begin
            if (thumb_switch_delay != 11'h7ff) begin
                thumb_switch_delay <= thumb_switch_delay + 1'b1;
            end else if (main_App_rd_en) begin
                main_drain_delay <= 9'd0;
            end else if (main_drain_delay != 9'd383) begin
                main_drain_delay <= main_drain_delay + 1'b1;
            end else begin
                thumb_owner <= 1'b0;     // montage comes from the main reader; no separate thumbnail reader
            end
        end
    end else begin
        thumb_switch_delay <= 11'd0;
        main_drain_delay <= 9'd0;
        if (thumb_owner) begin
            if (thumb_exit_delay != 9'd383)
                thumb_exit_delay <= thumb_exit_delay + 1'b1;
            else
                thumb_owner <= 1'b0;
        end else begin
            thumb_exit_delay <= 9'd0;
        end
    end
end

always @(posedge video_clk or posedge rst_video) begin
    if (rst_video)
        thumb_owner_video_sync <= 2'b00;
    else
        thumb_owner_video_sync <= {thumb_owner_video_sync[0], thumb_owner};
end

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_mem) begin
    if (rst_mem)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
wire media_sd_init_done;
wire media_bmp_ready;
wire media_decode_error;
wire [2:0] media_decode_error_code;
wire [3:0] media_state_code_i;
wire media_scan_found_valid;
wire [31:0] media_scan_found_sector;
wire media_sd_sec_read;
wire media_sd_sec_read_data_valid;
wire media_sd_sec_read_end;
wire media_scan_start_pulse;
wire media_load_start_pulse;
wire [31:0] media_load_sector;
wire media_load_abort;
wire media_transport_reset;
wire media_write_acknowledged;
media_player_control #(
    .CLK_FREQ_HZ       (100_000_000),
    .AUTO_PLAY_MS      (3000),
    .LONG_PRESS_MS     (1000),
    .FOUR_BANK_CACHE   (1),
    .EXTERNAL_KEY_EVENTS(1)
) u_media_player_control(
    .clk               (sd_card_clk),
    .rst               (rst_sd),
    .key_next          (1'b1),
    .key_auto          (1'b1),
    .key_next_event    (key_next_event),
    .key_auto_short_event(key_auto_short_event),
    .key_auto_long_event(key_previous_event),
    .pause             (emergency_mode),
    .auto_period_cycles(auto_period_cycles),
    .jump_req          (jump_req),
    .jump_index        (jump_index),
    .state_code        (state_code),
    .display_valid     (display_valid),
    .image_changed     (image_changed),
    .load_error        (load_error),
    .image_count       (image_count),
    .current_index     (current_index),
    .auto_play_active  (auto_play_active),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),
    .cached_images     (cached_images),
    .display_request_toggle(display_request_toggle),
    .display_ack_toggle(display_ack_toggle),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
.sd_init_done(media_sd_init_done),
.bmp_ready(media_bmp_ready),
.decode_error(media_decode_error),
.decode_error_code(media_decode_error_code),
.state_code_i(media_state_code_i),
.scan_done(media_scan_done),
.scan_found_valid(media_scan_found_valid),
.scan_found_sector(media_scan_found_sector),
.sd_sec_read(media_sd_sec_read),
.sd_sec_read_data_valid(media_sd_sec_read_data_valid),
.sd_sec_read_end(media_sd_sec_read_end),
.scan_start_pulse(media_scan_start_pulse),
.load_start_pulse(media_load_start_pulse),
.load_sector(media_load_sector),
.load_abort(media_load_abort),
.transport_reset(media_transport_reset),
.write_acknowledged(media_write_acknowledged)
);
media_bmp_source u_media_bmp_source (
.clk(sd_card_clk), .rst(rst_sd),
.bmp_width(16'd640), .bmp_height(16'd480),
.sd_init_done(media_sd_init_done),
.bmp_ready(media_bmp_ready),
.decode_error(media_decode_error),
.decode_error_code(media_decode_error_code),
.state_code_i(media_state_code_i),
.scan_done(media_scan_done),
.scan_found_valid(media_scan_found_valid),
.scan_found_sector(media_scan_found_sector),
 .audio_select(audio_select), .audio_found_mask(audio_found_mask), .audio_sizes(audio_sizes),
 .audio_sd_req(audio_sd_req),
 .audio_sd_addr(audio_sd_lba),
 .audio_sd_grant(audio_sd_grant),
 .audio_sd_busy(audio_sd_busy),
 .audio_sd_data(audio_sd_data),
 .audio_sd_data_valid(audio_sd_data_valid),
 .audio_sd_byte_index(audio_sd_byte_index),
 .audio_sd_end(audio_sd_end),
 .audio_file_found(audio_file_found),
 .audio_first_cluster(audio_first_cluster),
 .audio_file_size(audio_file_size),
 .audio_fat_start_sector(audio_fat_start_sector),
 .audio_data_start_sector(audio_data_start_sector),
 .audio_sectors_per_cluster(audio_sectors_per_cluster),
.sd_sec_read(media_sd_sec_read),
.sd_sec_read_data_valid(media_sd_sec_read_data_valid),
.sd_sec_read_end(media_sd_sec_read_end),
.scan_start_pulse(media_scan_start_pulse),
.load_start_pulse(media_load_start_pulse),
.load_sector(media_load_sector),
.load_abort(media_load_abort),
.transport_reset(media_transport_reset),
.write_acknowledged(media_write_acknowledged),
.write_req(sd_card_write_req), .write_en(sd_card_write_en), .write_data(sd_card_write_data),
.SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso)
);

buzzer_ctrl #(
    .CLK_FREQ_HZ (100_000_000),
    .BEEP_MS     (100),
    .ERROR_MS    (500)
) buzzer_ctrl_m0 (
    .clk           (sd_card_clk),
    .rst           (rst_sd),
    .key_toggle    (i_key3),
    .emergency     (emergency_mode),
    .image_changed (image_changed),
    .load_error    (load_error),
    .sound_enabled_out(sound_enabled),
    .buzzer        (buzzer)
);

wire audio_reset = rst_video;
(* async_reg = "true" *) reg [1:0] wav_selected_video_sync;

always @(posedge video_clk or posedge rst_video) begin
    if (rst_video)
        wav_selected_video_sync <= 2'b00;
    else
        wav_selected_video_sync <= {wav_selected_video_sync[0], wav_selected};
end

assign wav_selected = wav_mode;
assign wav_play_enable = wav_mode && wav_allow;

audio_ctrl #(.CLK_FREQ_HZ(100_000_000)) u_audio_ctrl (
    .clk                (sd_card_clk),
    .rst                (rst_sd),
    .wav_switch         (i_sw2),
    .sound_enabled      (sound_enabled),
    .emergency          (emergency_mode),
    .current_index      (current_index),
    .display_valid      (display_valid),
    .transition_busy    (transition_busy_sd[1]),
    .scan_done          (media_scan_done),
    .found_mask         (audio_found_mask),
    .sizes              (audio_sizes),
    .stream_end         (wav_stream_end),
    .audio_select       (audio_select),
    .wav_mode           (wav_mode),
    .wav_allow          (wav_allow),
    .auto_period_cycles (auto_period_cycles)
);

wav_stream_player wav_stream_player_m0 (
    .wr_clk              (sd_card_clk),
    .wr_rst_n            (~rst_sd),
    .wr_abort            (media_transport_reset && !rst_sd),
    .play_enable         (wav_play_enable),
    .first_cluster       (audio_first_cluster),
    .file_size_bytes     (audio_file_size),
    .fat_start_sector    (audio_fat_start_sector),
    .data_start_sector   (audio_data_start_sector),
    .sectors_per_cluster (audio_sectors_per_cluster),
    .sd_req              (audio_sd_req),
    .sd_lba              (audio_sd_lba),
    .sd_grant            (audio_sd_grant),
    .sd_busy             (audio_sd_busy),
    .sd_valid            (audio_sd_data_valid),
    .sd_byte             (audio_sd_data),
    .sd_byte_index       (audio_sd_byte_index),
    .sd_done             (audio_sd_end),
    .rd_clk              (video_clk),
    .rd_rst_n            (~audio_reset),
    .sample_tick         (media_audio_valid),
    .sample_out          (wav_sample),
    .sample_valid        (wav_sample_valid),
    .sample_ready        (1'b1),
    .sample_counter      (wav_sample_counter),
    .stream_active       (wav_stream_active),
    .stream_end          (wav_stream_end),
    .buffer_low          (wav_buffer_low)
);

hdmi_audio_tone #(
    .CLK_FREQ_HZ    (25_000_000),
    .SAMPLE_RATE_HZ (48_000),
    .AMPLITUDE      (24'sd2_000_000),
    .MEDIA_LINK     (1)
) hdmi_media_tone_m0 (
    .clk         (video_clk),
    .rst         (audio_reset),
    .enable      (sound_enabled),
    .media_id    (current_index),
    .media_valid (display_valid),
    .audio_valid (media_audio_valid),
    .audio_left  (media_audio_left),
    .audio_right (media_audio_right)
);

hdmi_audio_tone #(
    .CLK_FREQ_HZ    (25_000_000),
    .SAMPLE_RATE_HZ (48_000),
    .TONE_FREQ_HZ   (1000),
    .AMPLITUDE      (24'sd2_000_000),
    .MEDIA_LINK     (0)
) hdmi_test_tone_m0 (
    .clk         (video_clk),
    .rst         (audio_reset),
    .enable      (sound_enabled),
    .media_id    (2'd0),
    .media_valid (1'b1),
    .audio_valid (test_audio_valid),
    .audio_left  (test_audio_left),
    .audio_right (test_audio_right)
);

audio_source_mux audio_source_mux_m0 (
    .wav_active   (wav_selected_video_sync[1]),
    .wav_valid    (wav_sample_valid),
    .wav_left     ({wav_sample, 8'b0}),
    .wav_right    ({wav_sample, 8'b0}),
    .test_valid   (test_audio_valid),
    .test_left    (test_audio_left),
    .test_right   (test_audio_right),
    .media_valid  (media_audio_valid),
    .media_left   (media_audio_left),
    .media_right  (media_audio_right),
    .source_mode  (AUDIO_SOURCE_MODE),
    .audio_valid  (audio_valid),
    .audio_left   (audio_left_data),
    .audio_right  (audio_right_data),
    .active_source(active_audio_source)
);

audio_arc_calculate #(
    .ACR_N (6144)
) audio_arc_calculate_m0 (
    .clk         (video_clk),
    .rst         (audio_reset),
    .audio_valid (audio_valid),
    .acr_valid   (acr_valid),
    .acr_cts     (acr_cts),
    .acr_n       (acr_n)
);

display_adjust_control #(
    .CLK_FREQ_HZ   (100_000_000),
    .DEBOUNCE_MS   (20),
    .EXTERNAL_KEY_EVENTS(1)
) u_display_adjust_control (
    .clk               (sd_card_clk),
    .rst               (rst_sd),
    .settings_switch   (i_sw4 && !emergency_mode),
    .select_switch     (i_sw1),
    .key_decrease_n    (1'b1),
    .key_increase_n    (1'b1),
    .decrease_event    (key_decrease_event),
    .increase_event    (key_increase_event),
    .settings_mode     (settings_mode),
    .selected_param    (selected_param),
    .brightness_level  (brightness_level),
    .contrast_level    (contrast_level),
    .cfg_toggle        (cfg_toggle),
    .navigation_enabled(navigation_enabled)
);

theme_key_control #(
    .CLK_FREQ_HZ  (100_000_000),
    .DEBOUNCE_MS  (20),
    .LONG_PRESS_MS(1000)
) u_theme_key_control (
    .clk             (sd_card_clk),
    .rst             (rst_sd),
    .key1_n          (i_key1),
    .key2_n          (i_key2),
    .settings_mode   (settings_mode),
    .emergency_mode  (emergency_mode),
    .overview_mode   (overview_mode),
    .lock            (transition_busy_sd[1]),
    .ov_move_pulse   (key_ov_move_event),
    .ov_confirm_pulse(key_ov_confirm_event),
    .next_pulse      (key_next_event),
    .auto_short_pulse(key_auto_short_event),
    .previous_pulse  (key_previous_event),
    .decrease_pulse  (key_decrease_event),
    .increase_pulse  (key_increase_event)
);

overview_ctrl #(
    .CLK_FREQ_HZ (100_000_000),
    .DEBOUNCE_MS (20)
) u_overview_ctrl (
    .clk             (sd_card_clk),
    .rst             (rst_sd),
    .sw1              (i_sw1),
    .emergency_mode  (emergency_mode),
    .settings_mode   (settings_mode),
    .transition_busy (transition_busy_sd[1]),
    .current_index   (current_index),
    .move_pulse      (key_ov_move_event),
    .confirm_pulse   (key_ov_confirm_event),
    .overview_mode   (overview_mode),
    .selected_index  (ov_sel),
    .jump_req        (jump_req),
    .jump_index      (jump_index),
    .wipe_toggle     (wipe_toggle)
);

emergency_mode_control #(
    .CLK_FREQ_HZ (100_000_000),
    .DEBOUNCE_MS (20)
) u_emergency_mode_control (
    .clk             (sd_card_clk),
    .rst             (rst_sd),
    .emergency_switch(i_sw3),
    .emergency_mode  (emergency_mode)
);

seg_status #(.CLK_FREQ_HZ(50_000_000)) seg_status_m0 (
    .clk            (clk),
    .rst_n          (rst_n),
    .emergency      (emergency_mode),
    .settings       (settings_mode),
    .selected_param (selected_param),
    .overview       (overview_mode),
    .ready          (display_valid),
    .page           (current_index),
    .brightness     (brightness_level),
    .contrast       (contrast_level),
    .pos0(seg_p0), .pos1(seg_p1), .pos2(seg_p2), .pos3(seg_p3),
    .pos4(seg_p4), .pos5(seg_p5), .pos6(seg_p6), .pos7(seg_p7)
);

// seg_data_0 is the rightmost digit; pos0 is the leftmost
seg_scan #(.BOOT_TEST_CYCLES(75000000)) seg_scan_m0(
    .clk               (clk),
    // The display scanner only depends on the raw 50 MHz clock.
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        (seg_p7),
    .seg_data_1        (seg_p6),
    .seg_data_2        (seg_p5),
    .seg_data_3        (seg_p4),
    .seg_data_4        (seg_p3),
    .seg_data_5        (seg_p2),
    .seg_data_6        (seg_p1),
    .seg_data_7        (seg_p0)
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_ctrl video_timing_ctrl_m0(
    .video_clk         (video_clk),
    .rst               (video_source_rst),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

rgb_video_adapter rgb_video_adapter_m0(
    .video_clk         (video_clk),
    .rst               (rst_video),
    .read_en           (main_read_en),
    .read_data         (main_fifo_empty ? 24'h000000 : main_video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (main_hs_r),
    .vs_r              (main_vs_r),
    .de_r              (main_de_r),
    .vout_data         (main_vout_data)
);

rgb_video_upscaler_2x2 rgb_video_thumbnail_upscaler_m0(
    .video_clk         (video_clk),
    .rst               (rst_video),
    .frame_start       (thumbnail_frame_start),
    .stream_ready      (thumb_pixels_ready),
    .read_en           (thumbnail_read_en),
    .read_data         (thumbnail_fifo_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (thumbnail_hs_r),
    .vs_r              (thumbnail_vs_r),
    .de_r              (thumbnail_de_r),
    .vout_data         (thumbnail_vout_data)
);

assign hs = main_hs_r;
assign vs = main_vs_r;
assign de = main_de_r;
assign vout_data_raw = main_vout_data;

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data = reader_display_valid ? vout_data_raw : 24'h2050A0;

wire media_loader_busy, media_reader_busy;
wire transition_allow;
wire transition_hs, transition_vs, transition_de;
wire [23:0] transition_rgb;
wire [23:0] thumbnail_rgb;
wire thumbnail_border_active;
wire adjust_hs, adjust_vs, adjust_de;
wire [23:0] adjust_rgb;
wire [2:0] brightness_level_active;
wire [2:0] contrast_level_active;
wire subtitle_hs, subtitle_vs, subtitle_de;
wire [23:0] subtitle_rgb;
wire emergency_active;
wire osd_hs, osd_vs, osd_de;
wire [23:0] osd_rgb;
wire [63:0] dbg_m1, dbg_m2, dbg_hz, dbg_hz2, dbg_w01, dbg_w23;
wire [511:0] dbg_t1;
wire [63:0] bist_r_busy, bist_r_ref, bist_r_s0, bist_r_s1, bist_r_s2, bist_r_stat;
wire bist_active, bist_done;
wire bist_wr_en, bist_rd_en;
wire [20:0] bist_wr_addr, bist_rd_addr;
wire [31:0] bist_wr_din;
wire rst_cli = rst_mem || bist_active;
wire dg_hs, dg_vs, dg_de;
wire [23:0] dg_rgb;
brightness_contrast_adjust u_brightness_contrast_adjust (
    .clk                    (video_clk),
    .rst                    (rst_video),
    .hs_in                  (hs),
    .vs_in                  (vs),
    .de_in                  (de),
    .rgb_in                 (vout_data),
    .brightness_level_async (brightness_level),
    .contrast_level_async   (contrast_level),
    .cfg_toggle_async       (cfg_toggle),
    .hs_out                 (adjust_hs),
    .vs_out                 (adjust_vs),
    .de_out                 (adjust_de),
    .rgb_out                (adjust_rgb),
    .brightness_level_active(brightness_level_active),
    .contrast_level_active  (contrast_level_active)
);
video_frame_transition u_video_frame_transition (
    .clk(video_clk), .rst(rst_video), .frame_request(video_read_req),
    .request_toggle(display_request_toggle), .ack_toggle(display_ack_toggle),
    .display_valid(reader_display_valid), .auto_play(auto_play_active),
    .wipe_toggle(wipe_toggle), .wipe_tile(ov_sel_v1), .thumbnail_mode(thumbnail_mode), .wipe_hold(wipe_hold),
    .commit_allow(transition_allow), .busy(transition_busy), .hs(adjust_hs), .vs(adjust_vs), .de(adjust_de), .rgb(adjust_rgb),
    .hs_out(transition_hs), .vs_out(transition_vs), .de_out(transition_de),
    .rgb_out(transition_rgb)
);
thumbnail_overlay u_thumbnail_overlay (
    .clk          (video_clk),
    .rst          (rst_video),
    .mode         (thumbnail_mode),
    .focus_index  (ov_sel_v1),
    .vsync        (transition_vs),
    .de           (transition_de),
    .rgb_in       (transition_rgb),
    .test_pattern (1'b0),
    .diag         ({thumbnail_ready_mem, thumbnail_overflow, thumbnail_mask_mem}),
    .rgb_out      (thumbnail_rgb),
    .border_active(thumbnail_border_active)
);
ui_overlay u_ui_overlay (
    .clk                 (video_clk),
    .rst                 (rst_video),
    .emergency_mode_async(emergency_mode),
    .thumbnail_mode_async(thumbnail_mode),
    .image_index_async   (current_index),
    .display_valid_async (reader_display_valid),
    .settings_mode_async (settings_mode),
    .selected_param_async(selected_param),
    .brightness_level    (brightness_level_active),
    .contrast_level      (contrast_level_active),
    .hs_in               (transition_hs),
    .vs_in               (transition_vs),
    .de_in               (transition_de),
    .rgb_in              (thumbnail_rgb),
    .hs_out              (osd_hs),
    .vs_out              (osd_vs),
    .de_out              (osd_de),
    .rgb_out             (osd_rgb),
    .emergency_active    (emergency_active),
    .osd_active          (osd_active)
);
wire [23:0] visualizer_rgb;
emergency_audio_visualizer #(.STYLE(AUDIO_VISUALIZER_STYLE)) u_emergency_audio_visualizer (
 .clk(video_clk), .rst(rst_video),
 .enable(emergency_active && AUDIO_VISUALIZER_ENABLE),
 .vsync(osd_vs), .de(osd_de), .rgb_in(osd_rgb),
 .pcm_valid(audio_valid && !audio_reset), .pcm(audio_left_data[23:8]),
 .rgb_out(visualizer_rgb)
);
// ---- diagnostic: checksum of the main pixel stream leaving the main FIFO adapter ----
reg main_vs_r_d;
always @(posedge video_clk) main_vs_r_d <= main_vs_r;
/* diagnostics removed from the product build: u_diag_main_video_sum */
/* diagnostics removed from the product build: u_diag_hazard */
/* diagnostics removed from the product build: u_diag_wr_sums */
/* BIST finished its job (controller is clean in isolation); instance kept for reference
sdram_bist u_sdram_bist (
    .clk(ext_mem_clk), .rst(rst_mem), .Sdr_init_done(Sdr_init_done), .Sdr_busy(Sdr_busy),
    .Sdr_ref_vld(Sdr_init_ref_vld), .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
    .wr_en(bist_wr_en), .wr_addr(bist_wr_addr), .wr_din(bist_wr_din),
    .rd_en(bist_rd_en), .rd_addr(bist_rd_addr), .active(bist_active), .done(bist_done),
    .r_busy(bist_r_busy), .r_ref(bist_r_ref), .r_s0(bist_r_s0), .r_s1(bist_r_s1), .r_s2(bist_r_s2),
    .r_stat(bist_r_stat)
);
*/
assign bist_active = 1'b0;
assign bist_wr_en = 1'b0; assign bist_rd_en = 1'b0;
assign bist_wr_addr = 21'd0; assign bist_rd_addr = 21'd0; assign bist_wr_din = 32'd0;
assign bist_r_busy = 64'd0; assign bist_r_ref = 64'd0; assign bist_r_s0 = 64'd0;
assign bist_r_s1 = 64'd0; assign bist_r_s2 = 64'd0; assign bist_r_stat = 64'd0;
wire [511:0] vid_hist;
/* diagnostics removed from the product build: u_diag_vid_hist */
/* diagnostics removed from the product build: u_diag_overlay */
media_frame_reader #(
    .TRANSITIONS(1),
    .READ_PRIORITY_ENABLE(0)
) u_media_frame_reader (
    .mem_clk           (ext_mem_clk),
    .rst               (rst_cli),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),


    .App_rd_en         (main_App_rd_en),
    .App_rd_addr       (main_App_rd_addr),
    .Sdr_rd_en         (main_Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (video_read_addr_0),
    .read_addr_1       (video_read_addr_1),
    .read_addr_2       (video_read_addr_2),
    .read_addr_3       (video_read_addr_3),
    .read_len          (FRAME_PIXELS),
    .read_en           (main_read_en),
    .read_data         (main_video_read_data),
    .read_fifo_empty   (main_fifo_empty),
.frame_request(video_read_req), .video_rst(rst_video),
.requested_bank(disp_buf_idx), .request_toggle(display_request_toggle),
.source_valid(|cached_images), .ack_toggle(display_ack_toggle),
.active_bank(reader_bank), .display_valid(reader_display_valid),
.loader_busy(media_loader_busy), .reader_busy(media_reader_busy),
.transition_allow(transition_allow),
.rd_grant(!thumb_owner),
.dbg_sum(dbg_m1)
);

thumbnail_fourbank_reader #(
    .ADDR_BITS(ADDR_BITS),
    .FRAME_WIDTH(640),
    .FRAME_HEIGHT(480),
    .BUF0_ADDR(BUF0_ADDR),
    .BUF1_ADDR(BUF1_ADDR),
    .BUF2_ADDR(BUF2_ADDR),
    .BUF3_ADDR(BUF3_ADDR)
) u_thumbnail_fourbank_reader (
    .rst               (rst_cli),
    .enable            (thumb_owner && thumb_mode_mem),
    .frame_start       (thumbnail_frame_start),
    .mem_clk           (ext_mem_clk),
    .App_rd_en         (thumb_App_rd_en),
    .App_rd_addr       (thumb_App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_rd_dout       (Sdr_rd_dout),
    .read_clk          (video_clk),
    .read_en           (thumbnail_read_en && thumbnail_mode),
    .read_data         (thumbnail_fifo_data),
    .pixels_ready      (thumb_pixels_ready),
    .fifo_empty        (),
    .reader_idle       (thumb_reader_idle),
    .dbg_sum           (dbg_t1)
);
media_frame_loader #(
    .WRITE_V_FLIP(1), .FRAME_WIDTH(640), .FRAME_HEIGHT(480),
    .THUMBNAIL_ENABLE(THUMBNAIL_ENABLE),
    .THUMBNAIL_SAMPLE_X_STEP(THUMBNAIL_SAMPLE_X_STEP),
    .THUMBNAIL_SAMPLE_Y_STEP(THUMBNAIL_SAMPLE_Y_STEP),
    .THUMBNAIL_OUTPUT_X_SCALE(THUMBNAIL_OUTPUT_X_SCALE),
    .THUMBNAIL_OUTPUT_Y_SCALE(THUMBNAIL_OUTPUT_Y_SCALE),
    .THUMBNAIL_BASE_ADDRESS(THUMBNAIL_ADDR)
) u_media_frame_loader (
    .mem_clk           (ext_mem_clk),
    .rst               (rst_cli),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),


    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_frame_start (media_load_start_pulse),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (BUF2_ADDR),
    .write_addr_3      (BUF3_ADDR),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .reader_busy       (media_reader_busy),
    .loader_busy       (media_loader_busy),
    .thumbnail_ready   (thumbnail_ready_mem),
    .thumbnail_overflow(thumbnail_overflow),
    .thumbnail_mask(thumbnail_mask_mem)
);

// The controller only refreshes when it is idle and its own refresh request is
// issued; request a refresh every 31.25 us (2048 per 64 ms) ourselves as well.
reg [11:0] ref_pulse_cnt;
reg        ref_pulse;
always @(posedge ext_mem_clk or posedge rst_mem) begin
    if (rst_mem) begin
        ref_pulse_cnt <= 12'd0;
        ref_pulse <= 1'b0;
    end else begin
        ref_pulse <= (ref_pulse_cnt == 12'd3904);
        ref_pulse_cnt <= (ref_pulse_cnt == 12'd3904) ? 12'd0 : ref_pulse_cnt + 1'b1;
    end
end
wire [63:0] dbg_ref;
/* diagnostics removed from the product build: u_diag_refstat */
sdram U3(
    .Clk               (ext_mem_clk),
    .App_ref_req       (ref_pulse),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_mem),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (bist_active ? bist_wr_en : App_wr_en),
    .App_wr_addr       (bist_active ? bist_wr_addr : App_wr_addr),
    .App_wr_dm         (bist_active ? 4'b0000 : App_wr_dm),
    .App_wr_din        (bist_active ? bist_wr_din : App_wr_din),
    .App_rd_en         (bist_active ? bist_rd_en : App_rd_en),
    .App_rd_addr       (bist_active ? bist_rd_addr : App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (video_source_rst),
    .I_vs          (osd_vs),
    .I_de          (osd_de),
    .I_rgb         (visualizer_rgb),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_video),
    .O_pulse (edid_trig)
);

// ===================== HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_video),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
