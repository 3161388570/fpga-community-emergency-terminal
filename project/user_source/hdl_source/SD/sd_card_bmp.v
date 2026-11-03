
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    parameter integer LOAD_TIMEOUT_CYCLES = CLK_FREQ_HZ * 5,
    parameter integer SECTOR_TIMEOUT_CYCLES = CLK_FREQ_HZ / 2,
    parameter integer AUTO_PLAY_MS = 1000,
    parameter integer LONG_PRESS_MS = 1000,
    parameter integer USE_FAT32 = 1,
    parameter integer FOUR_BANK_CACHE = 0
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output      [3:0]           state_code,
    output reg                  image_changed,
    output reg                  load_error,
    output      [2:0]           image_count,
    output      [1:0]           current_index,
    output                      auto_play_active,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,
    output reg [3:0]            cached_images,
    output reg                  display_request_toggle,
    input                       display_ack_toggle,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_stable;
reg key_auto_previous;
reg key_auto_held;
reg key_auto_long_sent;
localparam integer LONG_PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
localparam integer LONG_COUNT_WIDTH = (LONG_PRESS_CYCLES > 1) ? $clog2(LONG_PRESS_CYCLES) : 1;
reg [LONG_COUNT_WIDTH-1:0] key_auto_count;
wire key_auto_short = key_auto_stable && !key_auto_previous && key_auto_held && !key_auto_long_sent;
wire key_auto_long = !key_auto_stable && key_auto_held && !key_auto_long_sent &&
                     (key_auto_count >= LONG_PRESS_CYCLES - 1);

localparam [31:0] AUTO_PERIOD_CYCLES = (CLK_FREQ_HZ / 1000) * AUTO_PLAY_MS;
localparam integer AUTO_COUNT_WIDTH = (AUTO_PERIOD_CYCLES > 1) ? $clog2(AUTO_PERIOD_CYCLES) : 1;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire [3:0]       state_code_i;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire             decode_error;
wire [2:0]       decode_error_code;
reg [2:0]        last_error;
reg [3:0]        bad_images;
reg              retry_pending;
reg              retry_used;
wire [2:0] usable_count;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [AUTO_COUNT_WIDTH-1:0] auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              prev_req_pending;
reg              load_busy;
reg [31:0]       load_timeout_cnt;

reg              load_abort;
reg              source_busy_seen;
reg              frame_request_seen;
(* async_reg = "true" *) reg [1:0] write_ack_sync;
reg [31:0]       sector_idle_cnt;
reg              sector_reset;
wire             transport_reset = rst || load_abort || sector_reset;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
reg              write_done_seen;
reg background_load;
reg commit_pending;
reg [1:0] commit_index;
(* async_reg = "true" *) reg [1:0] display_ack_sync;
wire [3:0] preload_excluded = bad_images | cached_images;
wire [1:0] preload_index = select_image(2'd3, img_found_count, preload_excluded, 1'b0);
wire preload_available = preload_index < img_found_count && !preload_excluded[preload_index];

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_current;
wire manual_next = next_req_pending || key_next_press;
wire manual_prev = prev_req_pending || key_auto_long;
wire [1:0] requested_index = retry_pending ? load_idx : manual_next ? next_from_current :
                                  manual_prev ? select_image(img_idx, img_found_count, bad_images, 1'b1) : next_from_current;
wire [1:0] first_index = retry_pending ? load_idx : select_image(2'd3, img_found_count, bad_images, 1'b0);
wire switch_requested = retry_pending || ((usable_count > 1) &&
                        (manual_next || manual_prev || (auto_play_en && auto_tick && !key_auto_short)));

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = (auto_cnt >= (AUTO_PERIOD_CYCLES - 32'd1));
assign next_from_current = select_image(img_idx, img_found_count, bad_images, 1'b0);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign state_code = (last_error != 0) ? {1'b1, last_error} : state_code_i;
assign image_count = img_found_count;
assign current_index = img_idx;
assign auto_play_active = auto_play_en;
assign usable_count = {2'd0, (img_found_count > 0 && !bad_images[0])} +
                      {2'd0, (img_found_count > 1 && !bad_images[1])} +
                      {2'd0, (img_found_count > 2 && !bad_images[2])} +
                      {2'd0, (img_found_count > 3 && !bad_images[3])};

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(),
    .button_stable_out(key_auto_stable)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        key_auto_previous <= 1'b1;
        key_auto_held <= 1'b0;
        key_auto_long_sent <= 1'b0;
        key_auto_count <= 0;
    end else begin
        key_auto_previous <= key_auto_stable;
        if (key_auto_stable) begin
            key_auto_held <= 1'b0;
            key_auto_long_sent <= 1'b0;
            key_auto_count <= 0;
        end else begin
            key_auto_held <= 1'b1;
            if (key_auto_long)
                key_auto_long_sent <= 1'b1;
            else if (!key_auto_long_sent && key_auto_count < LONG_PRESS_CYCLES - 1)
                key_auto_count <= key_auto_count + 1'b1;
        end
    end
end

function [1:0] select_image;
    input [1:0] cur;
    input [2:0] count;
    input [3:0] excluded;
    input backwards;
    integer distance;
    reg [1:0] candidate;
    reg found;
    begin
        select_image = cur;
        found = 1'b0;
        for (distance = 1; distance <= 4; distance = distance + 1) begin
            candidate = backwards ? cur - distance : cur + distance;
            if (!found && candidate < count && !excluded[candidate]) begin
                select_image = candidate;
                found = 1'b1;
            end
        end
    end
endfunction

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        write_ack_sync <= 2'b00;
        display_ack_sync <= 0;
        sector_idle_cnt <= 32'd0;
        sector_reset <= 1'b0;
    end else begin
        write_ack_sync <= {write_ack_sync[0], write_req_ack};
        display_ack_sync <= {display_ack_sync[0], display_ack_toggle};
        sector_reset <= 1'b0;
        if (!sd_init_done || !sd_sec_read || sd_sec_read_data_valid || sd_sec_read_end || load_abort) begin
            sector_idle_cnt <= 32'd0;
        end else if (sector_idle_cnt >= SECTOR_TIMEOUT_CYCLES - 1) begin
            sector_idle_cnt <= 32'd0;
            sector_reset <= 1'b1;
        end else begin
            sector_idle_cnt <= sector_idle_cnt + 32'd1;
        end
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        prev_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        load_abort            <= 1'b0;
        source_done_seen      <= 1'b0;
        source_busy_seen      <= 1'b0;
        frame_request_seen    <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
        image_changed         <= 1'b0;
        load_error            <= 1'b0;
        bad_images            <= 4'd0;
        retry_pending         <= 1'b0;
        retry_used            <= 1'b0;
        last_error            <= 3'd0;
        cached_images         <= 0;
        display_request_toggle <= 0;
        background_load       <= 0;
        commit_pending        <= 0;
        commit_index          <= 0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        image_changed    <= 1'b0;
        load_error       <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            prev_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            load_abort            <= 1'b0;
            source_done_seen      <= 1'b0;
            source_busy_seen      <= 1'b0;
            frame_request_seen    <= 1'b0;
            write_done_seen       <= 1'b0;
            display_valid         <= 1'b0;
            bad_images            <= 4'd0;
            retry_pending         <= 1'b0;
            retry_used            <= 1'b0;
            cached_images         <= 0;
            commit_pending        <= 0;
            background_load       <= 0;
        end else begin
            if (FOUR_BANK_CACHE && commit_pending && display_ack_sync[1] == display_request_toggle) begin
                commit_pending <= 0;
                img_idx <= commit_index;
                display_valid <= 1;
                first_image_committed <= 1;
                image_changed <= 1;
            end
            // 扫描阶段缓存前 4 张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住 bmp_read 已经把源图送完 FIFO，但还不能切显示，得等整帧写完
            if (load_busy && !bmp_ready)
                source_busy_seen <= 1'b1;
            if (load_busy && write_req && write_ack_sync[1])
                frame_request_seen <= 1'b1;
            if (load_busy && source_busy_seen && bmp_ready)
                source_done_seen <= 1'b1;

            // 记住写帧完成脉冲，避免“先写完后源结束”导致脉冲丢失而卡死
            if (load_busy && frame_request_seen && write_finish_pulse)
                write_done_seen <= 1'b1;

            // 只有“源图送完 + 整帧写完”都满足，才提交新图并切换显示缓冲区
            if (load_busy && decode_error) begin
                load_busy <= 1'b0;
                load_timeout_cnt <= 0;
                source_done_seen <= 1'b0;
                write_done_seen <= 1'b0;
                source_busy_seen <= 1'b0;
                frame_request_seen <= 1'b0;
                load_error <= 1'b1;
                last_error <= decode_error_code;
                if (!retry_used) begin
                    retry_used <= 1'b1;
                    retry_pending <= 1'b1;
                end else begin
                    bad_images[load_idx] <= 1'b1;
                    retry_pending <= 1'b0;
                    retry_used <= 1'b0;
                end
            end else if (load_busy && source_done_seen && write_done_seen) begin
                load_busy             <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                if (FOUR_BANK_CACHE) begin
                    cached_images[load_idx] <= 1;
                    if (!background_load || !display_valid) begin
                        disp_buf_idx <= pending_buf_idx;
                        commit_index <= load_idx;
                        display_request_toggle <= ~display_request_toggle;
                        commit_pending <= 1;
                    end
                end else begin
                    disp_buf_idx          <= pending_buf_idx;
                    img_idx               <= load_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                    image_changed         <= 1'b1;
                end
                retry_used            <= 1'b0;
                last_error            <= 3'd0;
            end else if (load_busy) begin
                if (load_timeout_cnt >= (LOAD_TIMEOUT_CYCLES - 32'd1)) begin
                    // 超时（例如读到坏图或截断的文件导致死锁），强制释放 busy 以恢复按键响应
                    load_busy             <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_abort            <= 1'b1;
                    load_error            <= 1'b1;
                    last_error            <= 3'd4;
                end else begin
                    load_timeout_cnt <= load_timeout_cnt + 32'd1;
                end
            end else begin
                load_timeout_cnt <= 32'd0;
            end

            // 上电后自动发起一次“扫描前 4 张 BMP”
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                prev_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                bad_images            <= 4'd0;
                retry_pending         <= 1'b0;
                retry_used            <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                cached_images         <= 0;
                commit_pending        <= 0;
            end else begin
                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                if (key_next_press && scan_done && (usable_count > 3'd1))
                    next_req_pending <= 1'b1;
                if (key_auto_long && scan_done && (usable_count > 3'd1))
                    prev_req_pending <= 1'b1;

                // 自动播放开/关
                if (key_auto_short && scan_done && (usable_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                // 自动播放 1s 计数：只有当前没有写图任务时才计时
                if (scan_done && auto_play_en && !key_auto_short && !manual_next && !manual_prev && display_valid && !load_busy && !commit_pending && first_image_committed && (usable_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 首图自动加载到 buffer0
                if (FOUR_BANK_CACHE) begin
                    if (scan_done && bmp_ready && !load_busy && !commit_pending && !load_start_pulse &&
                        display_ack_sync[1] == display_request_toggle) begin
                        if (display_valid && switch_requested && !retry_pending && cached_images[requested_index]) begin
                            disp_buf_idx <= requested_index;
                            commit_index <= requested_index;
                            display_request_toggle <= ~display_request_toggle;
                            commit_pending <= 1;
                            if (manual_next) next_req_pending <= 0;
                            else if (manual_prev) prev_req_pending <= 0;
                            auto_cnt <= 0;
                        end else if ((!first_image_committed && usable_count != 0) ||
                                     (display_valid && switch_requested) || preload_available) begin
                            load_idx <= !first_image_committed ? first_index : switch_requested ? requested_index : preload_index;
                            load_sector <= sector_lut(!first_image_committed ? first_index : switch_requested ? requested_index : preload_index);
                            pending_buf_idx <= !first_image_committed ? first_index : switch_requested ? requested_index : preload_index;
                            write_buf_idx <= !first_image_committed ? first_index : switch_requested ? requested_index : preload_index;
                            background_load <= first_image_committed && (!switch_requested || (retry_pending && background_load));
                            load_start_pulse <= 1;
                            load_busy <= 1;
                            source_done_seen <= 0;
                            source_busy_seen <= 0;
                            frame_request_seen <= 0;
                            write_done_seen <= 0;
                            retry_pending <= 0;
                            if (!retry_pending && switch_requested) begin
                                if (manual_next) next_req_pending <= 0;
                                else if (manual_prev) prev_req_pending <= 0;
                            end
                            auto_cnt <= 0;
                        end
                    end
                end else if (scan_done && !first_image_committed && bmp_ready && !load_busy && (usable_count != 3'd0)) begin
                    load_idx         <= first_index;
                    load_sector      <= sector_lut(first_index);
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    source_busy_seen <= 1'b0;
                    frame_request_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;
                    retry_pending    <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动下一张优先：写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy &&
                         (retry_pending || ((usable_count > 3'd1) &&
                         (manual_next || manual_prev || (auto_play_en && auto_tick && !key_auto_short))))) begin
                    load_idx         <= requested_index;
                    load_sector      <= sector_lut(requested_index);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    source_busy_seen <= 1'b0;
                    frame_request_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    retry_pending <= 1'b0;
                    if (!retry_pending) begin
                        if (manual_next)
                            next_req_pending <= 1'b0;
                        else if (manual_prev)
                            prev_req_pending <= 1'b0;
                    end
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

compatible_bmp_reader #(.USE_FAT32(USE_FAT32)) bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),
    .decode_error           (decode_error),
    .error_code             (decode_error_code),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code_i),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_ack_sync[1]),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

sd_card_top #(.SPI_HIGH_SPEED_DIV(2)) sd_card_top_m0(
    .clk                    (clk),
    .rst                    (transport_reset),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'h00),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module compatible_bmp_reader #(
    parameter integer USE_FAT32 = 1
)(
    input                       clk,
    input                       rst,
    output                      ready,
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output                      scan_done,
    output                      scan_found_valid,
    output [31:0]               scan_found_sector,
    output [2:0]                scan_found_total,
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,
    input                       sd_init_done,
    output [3:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output                      write_req,
    input                       write_req_ack,
    output                      sd_sec_read,
    output [31:0]               sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,
    output                      bmp_data_wr_en,
    output [23:0]               bmp_data,
    output                      decode_error,
    output [2:0]                error_code
);

reg fallback_active;
reg fallback_scan_start;
reg fat_scan_in_progress;

wire fat_ready;
wire fat_scan_done;
wire fat_scan_found_valid;
wire [31:0] fat_scan_found_sector;
wire [2:0] fat_scan_found_total;
wire [3:0] fat_state_code;
wire fat_write_req;
wire fat_sd_sec_read;
wire [31:0] fat_sd_sec_read_addr;
wire fat_bmp_data_wr_en;
wire [23:0] fat_bmp_data;
wire fat_filesystem_valid;
wire fat_decode_error;
wire [2:0] fat_error_code;

wire raw_ready;
wire raw_scan_done;
wire raw_scan_found_valid;
wire [31:0] raw_scan_found_sector;
wire [2:0] raw_scan_found_total;
wire [3:0] raw_state_code;
wire raw_write_req;
wire raw_sd_sec_read;
wire [31:0] raw_sd_sec_read_addr;
wire raw_bmp_data_wr_en;
wire [23:0] raw_bmp_data;
wire raw_decode_error;
wire [2:0] raw_error_code;

assign ready             = fallback_active ? raw_ready             : fat_ready;
assign scan_done         = fallback_active ? raw_scan_done         : (fat_scan_done && !fat_scan_in_progress);
assign scan_found_valid  = fallback_active ? raw_scan_found_valid  : fat_scan_found_valid;
assign scan_found_sector = fallback_active ? raw_scan_found_sector : fat_scan_found_sector;
assign scan_found_total  = fallback_active ? raw_scan_found_total  : fat_scan_found_total;
assign state_code        = fallback_active ? (USE_FAT32 ? {1'b1, raw_state_code[2:0]} : raw_state_code) : fat_state_code;
assign write_req         = fallback_active ? raw_write_req         : fat_write_req;
assign sd_sec_read       = fallback_active ? raw_sd_sec_read       : fat_sd_sec_read;
assign sd_sec_read_addr  = fallback_active ? raw_sd_sec_read_addr  : fat_sd_sec_read_addr;
assign bmp_data_wr_en    = fallback_active ? raw_bmp_data_wr_en    : fat_bmp_data_wr_en;
assign bmp_data          = fallback_active ? raw_bmp_data          : fat_bmp_data;
assign decode_error      = fallback_active ? raw_decode_error : fat_decode_error;
assign error_code        = fallback_active ? raw_error_code : fat_error_code;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fallback_active     <= !USE_FAT32;
        fallback_scan_start <= 1'b0;
        fat_scan_in_progress <= 1'b0;
    end else if (!sd_init_done) begin
        fallback_active     <= !USE_FAT32;
        fallback_scan_start <= 1'b0;
        fat_scan_in_progress <= 1'b0;
    end else begin
        fallback_scan_start <= 1'b0;

        if (scan_start) begin
            fallback_active      <= !USE_FAT32;
            fat_scan_in_progress <= USE_FAT32 != 0;
        end else if (fat_scan_in_progress && fat_scan_done) begin
            fat_scan_in_progress <= 1'b0;
            if (!fat_filesystem_valid) begin
                fallback_active     <= 1'b1;
                fallback_scan_start <= 1'b1;
            end
        end
    end
end

fat32_bmp_reader u_fat32_reader(
    .clk(clk), .rst(rst), .ready(fat_ready),
    .scan_start(scan_start && USE_FAT32 != 0), .scan_start_sector(scan_start_sector),
    .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
    .scan_done(fat_scan_done), .scan_found_valid(fat_scan_found_valid),
    .scan_found_sector(fat_scan_found_sector), .scan_found_total(fat_scan_found_total),
    .load_start(load_start && !fallback_active), .load_abort(load_abort),
    .load_sector(load_sector), .sd_init_done(sd_init_done), .state_code(fat_state_code),
    .bmp_width(bmp_width), .bmp_height(bmp_height),
    .write_req(fat_write_req), .write_req_ack(write_req_ack && !fallback_active),
    .sd_sec_read(fat_sd_sec_read), .sd_sec_read_addr(fat_sd_sec_read_addr),
    .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
    .sd_sec_read_end(sd_sec_read_end), .bmp_data_wr_en(fat_bmp_data_wr_en),
    .bmp_data(fat_bmp_data), .filesystem_valid(fat_filesystem_valid),
    .decode_error(fat_decode_error), .error_code(fat_error_code)
);

bmp_read u_raw_reader(
    .clk(clk), .rst(rst), .ready(raw_ready),
    .scan_start(fallback_scan_start || (scan_start && USE_FAT32 == 0)), .scan_start_sector(scan_start_sector),
    .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
    .scan_done(raw_scan_done), .scan_found_valid(raw_scan_found_valid),
    .scan_found_sector(raw_scan_found_sector), .scan_found_total(raw_scan_found_total),
    .load_start(load_start && fallback_active), .load_abort(load_abort),
    .load_sector(load_sector), .sd_init_done(sd_init_done), .state_code(raw_state_code),
    .bmp_width(bmp_width), .bmp_height(bmp_height),
    .write_req(raw_write_req), .write_req_ack(write_req_ack && fallback_active),
    .sd_sec_read(raw_sd_sec_read), .sd_sec_read_addr(raw_sd_sec_read_addr),
    .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
    .sd_sec_read_end(sd_sec_read_end), .bmp_data_wr_en(raw_bmp_data_wr_en),
    .bmp_data(raw_bmp_data), .decode_error(raw_decode_error), .error_code(raw_error_code)
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse,
    output wire button_stable_out
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

(* async_reg = "true" *) reg button_sync0;
(* async_reg = "true" *) reg button_sync1;
reg button_stable;
localparam integer COUNT_WIDTH = (DEBOUNCE_CYCLES > 1) ? $clog2(DEBOUNCE_CYCLES) : 1;
reg [COUNT_WIDTH-1:0] cnt;
assign button_stable_out = button_stable;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
