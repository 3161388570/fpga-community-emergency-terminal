
module media_player_control #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter integer LOAD_TIMEOUT_CYCLES = CLK_FREQ_HZ * 5,
    parameter integer SECTOR_TIMEOUT_CYCLES = CLK_FREQ_HZ / 2,
    parameter integer AUTO_PLAY_MS = 1000,
    parameter integer LONG_PRESS_MS = 1000,
    parameter integer FOUR_BANK_CACHE = 0,
    parameter integer EXTERNAL_KEY_EVENTS = 0
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       key_next_event,
    input                       key_auto_short_event,
    input                       key_auto_long_event,
    input                       pause,
    input       [31:0]          auto_period_cycles,
    input                       jump_req,
    input       [1:0]           jump_index,
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

    input                       write_req,
    input                       write_req_ack,
    input sd_init_done,
    input bmp_ready,
    input decode_error,
    input [2:0] decode_error_code,
    input [3:0] state_code_i,
    input scan_done,
    input scan_found_valid,
    input [31:0] scan_found_sector,
    input sd_sec_read,
    input sd_sec_read_data_valid,
    input sd_sec_read_end,
    output reg scan_start_pulse,
    output reg load_start_pulse,
    output reg [31:0] load_sector,
    output reg load_abort,
    output transport_reset,
    output write_acknowledged
);
wire legacy_key_next_press;
wire key_auto_stable;
reg key_auto_previous;
reg key_auto_held;
reg key_auto_long_sent;
reg key_auto_long_pulse;
localparam integer LONG_PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
localparam integer LONG_COUNT_WIDTH = (LONG_PRESS_CYCLES > 1) ? $clog2(LONG_PRESS_CYCLES) : 1;
reg [LONG_COUNT_WIDTH-1:0] key_auto_count;
wire legacy_key_auto_short = key_auto_stable && !key_auto_previous && key_auto_held && !key_auto_long_sent;
wire key_auto_long_detect = !key_auto_stable && key_auto_held && !key_auto_long_sent &&
                            (key_auto_count >= LONG_PRESS_CYCLES - 1);
wire key_next_press = EXTERNAL_KEY_EVENTS ? key_next_event : legacy_key_next_press;
wire key_auto_short = EXTERNAL_KEY_EVENTS ? key_auto_short_event : legacy_key_auto_short;
wire key_auto_long = EXTERNAL_KEY_EVENTS ? key_auto_long_event : key_auto_long_pulse;

localparam [31:0] AUTO_PERIOD_CYCLES = (CLK_FREQ_HZ / 1000) * AUTO_PLAY_MS;
localparam integer AUTO_COUNT_WIDTH = (AUTO_PERIOD_CYCLES > 1) ? $clog2(AUTO_PERIOD_CYCLES) : 1;

reg [2:0]        last_error;
reg [3:0]        bad_images;
reg              retry_pending;
reg              retry_used;
wire [2:0] usable_count;

reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0] auto_cnt;
reg              jump_pending;
reg [1:0]        jump_idx;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              
reg [1:0]        load_idx;             
reg [1:0]        pending_buf_idx;      
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              prev_req_pending;
reg              load_busy;
reg [31:0]       load_timeout_cnt;

reg              source_busy_seen;
reg              frame_request_seen;
(* async_reg = "true" *) reg [1:0] write_ack_sync;
reg [31:0]       sector_idle_cnt;
reg              sector_reset;
assign transport_reset = rst || load_abort || sector_reset;
assign write_acknowledged = write_ack_sync[1];


reg              source_done_seen;
reg              write_done_seen;
reg background_load;
reg commit_pending;
reg [1:0] commit_index;
reg pause_previous;
reg [1:0] pause_saved_index;
reg restore_pending;
(* async_reg = "true" *) reg [1:0] display_ack_sync;
wire [3:0] preload_excluded = bad_images | cached_images;
wire [1:0] preload_index = select_image(2'd3, img_found_count, preload_excluded, 1'b0);
wire preload_available = preload_index < img_found_count && !preload_excluded[preload_index];


reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_current;
wire manual_next = next_req_pending || key_next_press;
wire manual_prev = prev_req_pending || key_auto_long;
wire [1:0] requested_index = jump_pending ? jump_idx : restore_pending ? pause_saved_index : retry_pending ? load_idx : manual_next ? next_from_current :
                                  manual_prev ? select_image(img_idx, img_found_count, bad_images, 1'b1) : next_from_current;
wire [1:0] first_index = retry_pending ? load_idx : select_image(2'd3, img_found_count, bad_images, 1'b0);
wire switch_requested = !pause && (jump_pending || restore_pending || retry_pending || ((usable_count > 1) &&
                        (manual_next || manual_prev || (auto_play_en && auto_tick && !key_auto_short))));

reg auto_tick_r;
reg [31:0] auto_limit;
always @(posedge clk) begin
    auto_limit <= auto_period_cycles - 32'd2;      // registered compare, one cycle early
    auto_tick_r <= (auto_cnt >= auto_limit);
end
assign auto_tick  = auto_tick_r;
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

media_key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(legacy_key_next_press)
);

media_key_press_debounce #(
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
        key_auto_long_pulse <= 1'b0;
        key_auto_count <= 0;
    end else begin
        key_auto_long_pulse <= 1'b0;
        key_auto_previous <= key_auto_stable;
        if (key_auto_stable) begin
            key_auto_held <= 1'b0;
            key_auto_long_sent <= 1'b0;
            key_auto_count <= 0;
        end else begin
            key_auto_held <= 1'b1;
            if (key_auto_long_detect) begin
                key_auto_long_sent <= 1'b1;
                key_auto_long_pulse <= 1'b1;
            end else if (!key_auto_long_sent && key_auto_count < LONG_PRESS_CYCLES - 1)
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
            next_buf_lut = 2'd0;                 
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
        jump_pending          <= 1'b0;
        jump_idx              <= 2'd0;
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
        pause_previous        <= 1'b0;
        pause_saved_index     <= 2'd0;
        restore_pending       <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        image_changed    <= 1'b0;
        load_error       <= 1'b0;
        pause_previous   <= pause;

        if (pause && !pause_previous) begin
            pause_saved_index <= img_idx;
            next_req_pending <= 1'b0;
            prev_req_pending <= 1'b0;
        end else if (!pause && pause_previous && cached_images[pause_saved_index] && img_idx != pause_saved_index) begin
            restore_pending <= 1'b1;
        end

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
            jump_pending          <= 1'b0;
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
            restore_pending       <= 1'b0;
        end else begin
            if (FOUR_BANK_CACHE && commit_pending && display_ack_sync[1] == display_request_toggle) begin
                commit_pending <= 0;
                img_idx <= commit_index;
                display_valid <= 1;
                first_image_committed <= 1;
                image_changed <= 1;
            end
            
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

            
            if (load_busy && !bmp_ready)
                source_busy_seen <= 1'b1;
            if (load_busy && write_req && write_ack_sync[1])
                frame_request_seen <= 1'b1;
            if (load_busy && source_busy_seen && bmp_ready)
                source_done_seen <= 1'b1;

            
            if (load_busy && frame_request_seen && write_finish_pulse)
                write_done_seen <= 1'b1;

            
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
                
                if (!pause && jump_req && scan_done && (usable_count > 3'd1) && jump_index != img_idx) begin
                    jump_pending <= 1'b1;
                    jump_idx <= jump_index;
                end
                if (!pause && key_next_press && scan_done && (usable_count > 3'd1))
                    next_req_pending <= 1'b1;
                if (!pause && key_auto_long && scan_done && (usable_count > 3'd1))
                    prev_req_pending <= 1'b1;

                
                if (!pause && key_auto_short && scan_done && (usable_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                    if (!auto_play_en)
                        next_req_pending <= 1'b1;     // starting: go to the next page at once
                end

                
                if (pause) begin
                    auto_cnt <= auto_cnt;
                end else if (scan_done && auto_play_en && !key_auto_short && !manual_next && !manual_prev && display_valid && !load_busy && !commit_pending && first_image_committed && (usable_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                
                if (FOUR_BANK_CACHE) begin
                    if (!pause && scan_done && bmp_ready && !load_busy && !commit_pending && !load_start_pulse &&
                        display_ack_sync[1] == display_request_toggle) begin
                        if (display_valid && switch_requested && !retry_pending && cached_images[requested_index]) begin
                            disp_buf_idx <= requested_index;
                            commit_index <= requested_index;
                            display_request_toggle <= ~display_request_toggle;
                            commit_pending <= 1;
                            if (manual_next) next_req_pending <= 0;
                            else if (manual_prev) prev_req_pending <= 0;
                            if (restore_pending) restore_pending <= 0;
                            if (jump_pending) jump_pending <= 0;
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
                end else if (!pause && scan_done && !first_image_committed && bmp_ready && !load_busy && (usable_count != 3'd0)) begin
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
                
                else if (!pause && scan_done && bmp_ready && display_valid && !load_busy &&
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

endmodule
module media_key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    
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

