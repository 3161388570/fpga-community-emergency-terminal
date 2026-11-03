module fat32_bmp_reader #(
    parameter integer MAX_ROOT_CLUSTERS = 4096
)(
    input                       clk,
    input                       rst,
    output                      ready,

    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,
    input  [2:0]                audio_select,        // 0..3 = N1..N4.WAV, 4 = N5.WAV (or EM.WAV)
    output reg                  audio_file_found,
    output reg  [31:0]          audio_first_cluster,
    output reg  [31:0]          audio_file_size,
    output wire [4:0]           audio_found_mask,
    output wire [159:0]         audio_sizes,
    output wire [31:0]          audio_fat_start_sector,
    output wire [31:0]          audio_data_start_sector,
    output wire [7:0]           audio_sectors_per_cluster,

    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,
    output reg                 filesystem_valid,
    output reg                 decode_error,
    output reg [2:0]           error_code
);

localparam [4:0] ST_IDLE       = 5'd0;
localparam [4:0] ST_BOOT       = 5'd1;
localparam [4:0] ST_BPB        = 5'd2;
localparam [4:0] ST_ROOT       = 5'd3;
localparam [4:0] ST_FAT_ROOT   = 5'd4;
localparam [4:0] ST_LOAD_HDR   = 5'd5;
localparam [4:0] ST_LOAD_WAIT  = 5'd6;
localparam [4:0] ST_LOAD_DATA  = 5'd7;
localparam [4:0] ST_FAT_LOAD   = 5'd8;
localparam [4:0] ST_ADDR_ROOT  = 5'd9;
localparam [4:0] ST_ADDR_HDR   = 5'd10;
localparam [4:0] ST_ADDR_DATA  = 5'd11;
localparam [4:0] ST_RELEASE    = 5'd12;

reg [4:0] state;
reg [9:0] byte_index;

reg [31:0] partition_lba;
reg [15:0] bytes_per_sector;
reg [7:0]  sectors_per_cluster;
reg [15:0] reserved_sectors;
reg [7:0]  fat_count;
reg [31:0] fat_size;
reg [31:0] root_cluster;
reg [31:0] volume_start;
reg [31:0] fat_start_sector;
reg [31:0] data_start_sector;
reg [31:0] total_sectors;
reg [15:0] root_entries;
reg [15:0] fat16_size;
reg [15:0] fs_version;
reg [15:0] ext_flags;
reg [15:0] boot_signature;
reg [7:0] partition_type;
reg [31:0] partition_candidate;
reg [31:0] max_cluster;
reg [31:0] visited_clusters;
reg [31:0] chain_anchor;
reg [31:0] chain_power;
reg [31:0] chain_length;
reg [31:0] entry_size;
reg [31:0] catalog_cluster [0:3];
reg [31:0] catalog_size [0:3];
reg [31:0] load_size;
reg [31:0] load_cluster_latched;
reg [31:0] dib_size;
reg [15:0] planes;
reg [31:0] row_byte_pos;
reg [31:0] pixel_count;
integer catalog_index;
integer load_index;
reg [32:0] fat_area;
reg [32:0] data_offset_sectors;
reg [31:0] data_sectors;
reg [31:0] data_clusters;
reg [31:0] cluster_limit;
reg [31:0] active_fat_offset;
reg [31:0] load_cluster_quotient;
reg [31:0] load_cluster_limit;
reg bpb_basic_ok;
reg bpb_area_ok;
reg bpb_cluster_ok;
reg bpb_root_ok;
reg bpb_capacity_ok;
reg bpb_address_ok;
reg bpb_geometry_ok;
wire [31:0] row_data_bytes = {16'd0, bmp_width} * 32'd3;
wire [31:0] row_stride = (row_data_bytes + 3) & 32'hfffffffc;
wire [47:0] payload_end;
wire [31:0] expected_pixels = {16'd0, bmp_width} * {16'd0, bmp_height};
wire next_cluster_valid;

reg [31:0] current_cluster;
reg [31:0] cluster_offset;
reg [7:0]  sector_in_cluster;
reg [8:0]  fat_byte_offset;
reg [31:0] fat_entry;

reg [7:0] entry_first;
reg [87:0] entry_sfn;
reg [1:0]  scan_slot;
reg [7:0] entry_ext0;
reg [7:0] entry_ext1;
reg [7:0] entry_ext2;
reg [7:0] entry_attr;
reg [15:0] entry_cluster_hi;
reg [15:0] entry_cluster_lo;
reg        directory_end;
reg        scan_stop_requested;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

wire bpb_valid;
wire header_match;
wire directory_entry_is_bmp;
wire bmp_data_valid;
wire [31:0] entry_cluster;
wire [31:0] fat_lookup_sector;
wire [31:0] fat_next_cluster;

assign ready = (state == ST_IDLE);

// N1.WAV .. N4.WAV and EM.WAV in the root directory (8.3 names, extension WAV)
reg [4:0]   wav_found;
reg         wav_check;
reg [159:0] wav_clusters;
reg [159:0] wav_sizes;
wire wav_name_tail = entry_sfn[87:16] == 72'h564157_202020202020;
wire wav_n_ok = wav_name_tail && entry_sfn[7:0] == 8'h4e && entry_sfn[15:11] == 5'b00110 &&
                entry_sfn[10:8] >= 3'd1 && entry_sfn[10:8] <= 3'd5;   // '1'..'5' = 0x31..0x35 (N5 = emergency)
wire wav_em_ok = wav_name_tail && entry_sfn[15:0] == 16'h4d45;       // "EM"
wire [2:0] wav_entry_idx = wav_em_ok ? 3'd4 : (entry_sfn[10:8] - 3'd1);
wire wav_entry_ok = (wav_n_ok || wav_em_ok) && (entry_attr & 8'h18) == 0 &&
                    entry_cluster >= 2 && entry_cluster <= max_cluster && entry_size > 32'd44;
assign audio_found_mask = wav_found;
assign audio_sizes = wav_sizes;
// registered selection: the player only samples these while idle
always @(posedge clk) begin
    audio_file_found    <= wav_found[audio_select];
    audio_first_cluster <= wav_clusters[audio_select*32 +: 32];
    audio_file_size     <= wav_sizes[audio_select*32 +: 32];
end
assign audio_fat_start_sector = fat_start_sector;
assign audio_data_start_sector = data_start_sector;
assign audio_sectors_per_cluster = sectors_per_cluster;
assign payload_end = {16'd0, pixel_offset} + ({16'd0, row_stride} * bmp_height);
assign next_cluster_valid = (fat_next_cluster >= 2) && (fat_next_cluster <= max_cluster) && (fat_next_cluster < 32'h0ffffff0);
assign bpb_valid = bpb_geometry_ok && (boot_signature == 16'haa55);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fat_area <= 0;
        data_offset_sectors <= 0;
        data_sectors <= 0;
        data_clusters <= 0;
        cluster_limit <= 0;
        active_fat_offset <= 0;
        load_cluster_quotient <= 0;
        load_cluster_limit <= 0;
        bpb_basic_ok <= 0;
        bpb_area_ok <= 0;
        bpb_cluster_ok <= 0;
        bpb_root_ok <= 0;
        bpb_capacity_ok <= 0;
        bpb_address_ok <= 0;
        bpb_geometry_ok <= 0;
    end else begin
        if (state == ST_LOAD_HDR || state == ST_LOAD_WAIT || state == ST_RELEASE ||
            state == ST_LOAD_DATA || state == ST_FAT_LOAD) begin
            load_cluster_quotient <= load_size >> (9 + cluster_shift(sectors_per_cluster));
            load_cluster_limit <= load_cluster_quotient + 1;
        end
        if ((state == ST_BOOT || state == ST_BPB) && byte_index >= 48) begin
        fat_area <= {1'b0, fat_size} << (fat_count == 2);
        data_offset_sectors <= {17'd0, reserved_sectors} + fat_area;
        data_sectors <= total_sectors - data_offset_sectors[31:0];
        data_clusters <= data_sectors >> cluster_shift(sectors_per_cluster);
        cluster_limit <= data_clusters + 1;
        active_fat_offset <= reserved_sectors + ((ext_flags[7] && ext_flags[0]) ? fat_size : 0);
        bpb_basic_ok <= (bytes_per_sector == 16'd512) &&
                   (sectors_per_cluster != 8'd0) &&
                   ((sectors_per_cluster & (sectors_per_cluster - 1'b1)) == 0) &&
                   (reserved_sectors != 16'd0) &&
                   ((fat_count == 1) || (fat_count == 2)) &&
                   (fat_size != 32'd0) &&
                   (root_entries == 0) && (fat16_size == 0) && (fs_version == 0) &&
                   (!ext_flags[7] || ({4'd0, ext_flags[3:0]} < fat_count));
        bpb_area_ok <= data_offset_sectors < {1'b0, total_sectors};
        bpb_cluster_ok <= data_clusters != 0 && data_clusters < 32'h0fffffee;
        bpb_root_ok <= root_cluster >= 2 && root_cluster <= cluster_limit;
        bpb_capacity_ok <= ({8'd0, fat_size} << 7) >= ({8'd0, cluster_limit} + 1);
        bpb_address_ok <= ({1'b0, sd_sec_read_addr} + total_sectors) <= 33'h100000000;
        bpb_geometry_ok <= bpb_basic_ok && bpb_area_ok && bpb_cluster_ok && bpb_root_ok && bpb_capacity_ok && bpb_address_ok;
        end
    end
end
reg header_offset_valid;
reg header_payload_valid;
reg header_file_valid;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_offset_valid <= 0;
        header_payload_valid <= 0;
        header_file_valid <= 0;
    end else begin
        header_offset_valid <= ({1'b0, pixel_offset} >= ({1'b0, dib_size} + 14));
        header_payload_valid <= (payload_end <= {16'd0, file_len});
        header_file_valid <= (file_len <= load_size);
    end
end
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width == {16'd0, bmp_width}) && (bmp_width != 0) &&
                      (height == {16'd0, bmp_height}) && (bmp_height != 0) &&
                      (dib_size >= 40) && (planes == 1) &&
                      header_offset_valid && header_payload_valid && header_file_valid &&
                      (bit_count == 16'd24) &&
                      (compression == 32'd0);
assign directory_entry_is_bmp = (entry_first != 8'h00) &&
                                (entry_first != 8'hE5) &&
                                (entry_attr != 8'h0F) &&
                                ((entry_attr & 8'h18) == 8'h00) &&
                                ((entry_ext0 == "B") || (entry_ext0 == "b")) &&
                                ((entry_ext1 == "M") || (entry_ext1 == "m")) &&
                                ((entry_ext2 == "P") || (entry_ext2 == "p")) &&
                                (entry_cluster >= 32'd2) && (entry_cluster <= max_cluster) &&
                                ({sd_sec_read_data, entry_size[23:0]} >= 54);
assign entry_cluster = {4'd0, entry_cluster_hi[11:0], entry_cluster_lo};
assign fat_lookup_sector = fat_start_sector + (current_cluster >> 7);
assign fat_next_cluster = fat_entry & 32'h0FFF_FFFF;
assign bmp_data_valid = sd_sec_read_data_valid &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt < payload_end) && (row_byte_pos < row_data_bytes);

function [3:0] cluster_shift;
    input [7:0] sectors;
    begin
        case (sectors)
            1: cluster_shift = 0;
            2: cluster_shift = 1;
            4: cluster_shift = 2;
            8: cluster_shift = 3;
            16: cluster_shift = 4;
            32: cluster_shift = 5;
            64: cluster_shift = 6;
            128: cluster_shift = 7;
            default: cluster_shift = 0;
        endcase
    end
endfunction

function [31:0] cluster_sector_offset;
    input [31:0] cluster;
    begin
        case (sectors_per_cluster)
            8'd1:   cluster_sector_offset = cluster - 32'd2;
            8'd2:   cluster_sector_offset = (cluster - 32'd2) << 1;
            8'd4:   cluster_sector_offset = (cluster - 32'd2) << 2;
            8'd8:   cluster_sector_offset = (cluster - 32'd2) << 3;
            8'd16:  cluster_sector_offset = (cluster - 32'd2) << 4;
            8'd32:  cluster_sector_offset = (cluster - 32'd2) << 5;
            8'd64:  cluster_sector_offset = (cluster - 32'd2) << 6;
            8'd128: cluster_sector_offset = (cluster - 32'd2) << 7;
            default: cluster_sector_offset = 32'd0;
        endcase
    end
endfunction

function [31:0] fat_area_sectors;
    input [31:0] one_fat_size;
    input [7:0]  number_of_fats;
    begin
        case (number_of_fats)
            8'd1: fat_area_sectors = one_fat_size;
            8'd2: fat_area_sectors = one_fat_size << 1;
            8'd3: fat_area_sectors = one_fat_size + (one_fat_size << 1);
            8'd4: fat_area_sectors = one_fat_size << 2;
            default: fat_area_sectors = one_fat_size;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        byte_index <= 10'd0;
    end else if (sd_sec_read_data_valid) begin
        byte_index <= byte_index + 10'd1;
    end else if (sd_sec_read_end || !sd_sec_read) begin
        byte_index <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst || ((state == ST_IDLE) && scan_start)) begin
        partition_lba      <= 32'd0;
        bytes_per_sector   <= 16'd0;
        sectors_per_cluster <= 8'd0;
        reserved_sectors   <= 16'd0;
        fat_count          <= 8'd0;
        fat_size           <= 32'd0;
        root_cluster       <= 32'd0;
        total_sectors <= 0;
        root_entries <= 0;
        fat16_size <= 0;
        fs_version <= 0;
        ext_flags <= 0;
        boot_signature <= 0;
        partition_type <= 0;
        partition_candidate <= 0;
    end else if ((state == ST_BOOT) && sd_sec_read_end && !bpb_valid &&
                 (partition_lba != 32'd0)) begin
        bytes_per_sector    <= 16'd0;
        sectors_per_cluster <= 8'd0;
        reserved_sectors    <= 16'd0;
        fat_count           <= 8'd0;
        fat_size            <= 32'd0;
        root_cluster        <= 32'd0;
        boot_signature <= 0;
    end else if (((state == ST_BOOT) || (state == ST_BPB)) && sd_sec_read_data_valid) begin
        case (byte_index)
            10'd11: bytes_per_sector[7:0]    <= sd_sec_read_data;
            10'd12: bytes_per_sector[15:8]   <= sd_sec_read_data;
            10'd13: sectors_per_cluster      <= sd_sec_read_data;
            10'd14: reserved_sectors[7:0]    <= sd_sec_read_data;
            10'd15: reserved_sectors[15:8]   <= sd_sec_read_data;
            10'd16: fat_count                <= sd_sec_read_data;
            10'd17: root_entries[7:0] <= sd_sec_read_data;
            10'd18: root_entries[15:8] <= sd_sec_read_data;
            10'd22: fat16_size[7:0] <= sd_sec_read_data;
            10'd23: fat16_size[15:8] <= sd_sec_read_data;
            10'd32: total_sectors[7:0] <= sd_sec_read_data;
            10'd33: total_sectors[15:8] <= sd_sec_read_data;
            10'd34: total_sectors[23:16] <= sd_sec_read_data;
            10'd35: total_sectors[31:24] <= sd_sec_read_data;
            10'd36: fat_size[7:0]            <= sd_sec_read_data;
            10'd37: fat_size[15:8]           <= sd_sec_read_data;
            10'd38: fat_size[23:16]          <= sd_sec_read_data;
            10'd39: fat_size[31:24]          <= sd_sec_read_data;
            10'd40: ext_flags[7:0] <= sd_sec_read_data;
            10'd41: ext_flags[15:8] <= sd_sec_read_data;
            10'd42: fs_version[7:0] <= sd_sec_read_data;
            10'd43: fs_version[15:8] <= sd_sec_read_data;
            10'd44: root_cluster[7:0]        <= sd_sec_read_data;
            10'd45: root_cluster[15:8]       <= sd_sec_read_data;
            10'd46: root_cluster[23:16]      <= sd_sec_read_data;
            10'd47: root_cluster[31:24]      <= sd_sec_read_data;
            10'd510: boot_signature[7:0] <= sd_sec_read_data;
            10'd511: boot_signature[15:8] <= sd_sec_read_data;
            default: ;
        endcase
        if (state == ST_BOOT && byte_index >= 446 && byte_index < 510) begin
            case ((byte_index - 446) & 15)
                4: partition_type <= sd_sec_read_data;
                8: partition_candidate[7:0] <= sd_sec_read_data;
                9: partition_candidate[15:8] <= sd_sec_read_data;
                10: partition_candidate[23:16] <= sd_sec_read_data;
                11: partition_candidate[31:24] <= sd_sec_read_data;
                15: if (partition_lba == 0 && (partition_type == 8'h0b || partition_type == 8'h0c) && partition_candidate != 0)
                    partition_lba <= partition_candidate;
                default: ;
            endcase
        end
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        entry_first      <= 8'd0;
        entry_sfn        <= 88'd0;
        entry_ext0       <= 8'd0;
        entry_ext1       <= 8'd0;
        entry_ext2       <= 8'd0;
        entry_attr       <= 8'd0;
        entry_cluster_hi <= 16'd0;
        entry_cluster_lo <= 16'd0;
        entry_size <= 0;
    end else if ((state == ST_ROOT) && sd_sec_read_data_valid) begin
        case (byte_index[4:0])
            5'd0: begin
                entry_first      <= sd_sec_read_data;
                entry_sfn[7:0]   <= sd_sec_read_data;
                entry_ext0       <= 8'd0;
                entry_ext1       <= 8'd0;
                entry_ext2       <= 8'd0;
                entry_attr       <= 8'd0;
                entry_cluster_hi <= 16'd0;
                entry_cluster_lo <= 16'd0;
            end
            5'd1:  entry_sfn[15:8]  <= sd_sec_read_data;
            5'd2:  entry_sfn[23:16] <= sd_sec_read_data;
            5'd3:  entry_sfn[31:24] <= sd_sec_read_data;
            5'd4:  entry_sfn[39:32] <= sd_sec_read_data;
            5'd5:  entry_sfn[47:40] <= sd_sec_read_data;
            5'd6:  entry_sfn[55:48] <= sd_sec_read_data;
            5'd7:  entry_sfn[63:56] <= sd_sec_read_data;
            5'd8: begin entry_ext0 <= sd_sec_read_data; entry_sfn[71:64] <= sd_sec_read_data; end
            5'd9: begin entry_ext1 <= sd_sec_read_data; entry_sfn[79:72] <= sd_sec_read_data; end
            5'd10: begin entry_ext2 <= sd_sec_read_data; entry_sfn[87:80] <= sd_sec_read_data; end
            5'd11: entry_attr             <= sd_sec_read_data;
            5'd20: entry_cluster_hi[7:0]  <= sd_sec_read_data;
            5'd21: entry_cluster_hi[15:8] <= sd_sec_read_data;
            5'd26: entry_cluster_lo[7:0]  <= sd_sec_read_data;
            5'd27: entry_cluster_lo[15:8] <= sd_sec_read_data;
            5'd28: entry_size[7:0] <= sd_sec_read_data;
            5'd29: entry_size[15:8] <= sd_sec_read_data;
            5'd30: entry_size[23:16] <= sd_sec_read_data;
            5'd31: entry_size[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
        dib_size <= 0;
        planes <= 0;
    end else if ((state == ST_LOAD_HDR) && sd_sec_read_data_valid) begin
        case (byte_index)
            10'd0:  header_0             <= sd_sec_read_data;
            10'd1:  header_1             <= sd_sec_read_data;
            10'd2:  file_len[7:0]        <= sd_sec_read_data;
            10'd3:  file_len[15:8]       <= sd_sec_read_data;
            10'd4:  file_len[23:16]      <= sd_sec_read_data;
            10'd5:  file_len[31:24]      <= sd_sec_read_data;
            10'd10: pixel_offset[7:0]    <= sd_sec_read_data;
            10'd11: pixel_offset[15:8]   <= sd_sec_read_data;
            10'd12: pixel_offset[23:16]  <= sd_sec_read_data;
            10'd13: pixel_offset[31:24]  <= sd_sec_read_data;
            10'd14: dib_size[7:0] <= sd_sec_read_data;
            10'd15: dib_size[15:8] <= sd_sec_read_data;
            10'd16: dib_size[23:16] <= sd_sec_read_data;
            10'd17: dib_size[31:24] <= sd_sec_read_data;
            10'd18: width[7:0]           <= sd_sec_read_data;
            10'd19: width[15:8]          <= sd_sec_read_data;
            10'd20: width[23:16]         <= sd_sec_read_data;
            10'd21: width[31:24]         <= sd_sec_read_data;
            10'd22: height[7:0]          <= sd_sec_read_data;
            10'd23: height[15:8]         <= sd_sec_read_data;
            10'd24: height[23:16]        <= sd_sec_read_data;
            10'd25: height[31:24]        <= sd_sec_read_data;
            10'd26: planes[7:0] <= sd_sec_read_data;
            10'd27: planes[15:8] <= sd_sec_read_data;
            10'd28: bit_count[7:0]       <= sd_sec_read_data;
            10'd29: bit_count[15:8]      <= sd_sec_read_data;
            10'd30: compression[7:0]     <= sd_sec_read_data;
            10'd31: compression[15:8]    <= sd_sec_read_data;
            10'd32: compression[23:16]   <= sd_sec_read_data;
            10'd33: compression[31:24]   <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fat_entry <= 32'd0;
    end else if (((state == ST_ROOT) || (state == ST_LOAD_DATA)) && sd_sec_read_end &&
                 ((sector_in_cluster + 8'd1) >= sectors_per_cluster)) begin
        fat_entry <= 32'd0;
    end else if (((state == ST_FAT_ROOT) || (state == ST_FAT_LOAD)) && sd_sec_read_data_valid) begin
        if (byte_index == fat_byte_offset)
            fat_entry[7:0] <= sd_sec_read_data;
        else if (byte_index == fat_byte_offset + 9'd1)
            fat_entry[15:8] <= sd_sec_read_data;
        else if (byte_index == fat_byte_offset + 9'd2)
            fat_entry[23:16] <= sd_sec_read_data;
        else if (byte_index == fat_byte_offset + 9'd3)
            fat_entry[31:24] <= sd_sec_read_data;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt  <= 32'd0;
        bmp_byte_idx <= 2'd0;
        row_byte_pos <= 0;
        pixel_count <= 0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
        if (sd_sec_read_data_valid && bmp_len_cnt >= pixel_offset && bmp_len_cnt < payload_end)
            row_byte_pos <= (row_byte_pos == row_stride - 1) ? 0 : row_byte_pos + 1;
        if (bmp_data_valid && bmp_byte_idx == 2)
            pixel_count <= pixel_count + 1;
    end else if ((state != ST_FAT_LOAD) && (state != ST_LOAD_WAIT) &&
                 (state != ST_ADDR_DATA) && (state != ST_RELEASE)) begin
        bmp_len_cnt  <= 32'd0;
        bmp_byte_idx <= 2'd0;
        row_byte_pos <= 0;
        pixel_count <= 0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else begin
        bmp_data_wr_en <= 1'b0;
        if ((state == ST_LOAD_DATA) && bmp_data_valid && !load_abort && sd_init_done) begin
            case (bmp_byte_idx)
                2'd0: bmp_data[7:0]  <= sd_sec_read_data;
                2'd1: bmp_data[15:8] <= sd_sec_read_data;
                2'd2: begin
                    bmp_data[23:16] <= sd_sec_read_data;
                    bmp_data_wr_en  <= 1'b1;
                end
                default: ;
            endcase
        end
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (catalog_index = 0; catalog_index < 4; catalog_index = catalog_index + 1) begin
            catalog_cluster[catalog_index] <= 0;
            catalog_size[catalog_index] <= 0;
        end
    end else if (scan_found_valid && scan_found_total >= 1 && scan_found_total <= 4) begin
        catalog_cluster[scan_slot] <= scan_found_sector;
        catalog_size[scan_slot] <= entry_size;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                <= ST_IDLE;
        state_code           <= 4'd0;
        sd_sec_read          <= 1'b0;
        sd_sec_read_addr     <= 32'd0;
        write_req            <= 1'b0;
        scan_done            <= 1'b0;
        scan_found_valid     <= 1'b0;
        scan_found_sector    <= 32'd0;
        scan_found_total     <= 3'd0;
        wav_check            <= 1'b0;
        wav_found            <= 5'd0;
        wav_clusters         <= 160'd0;
        wav_sizes            <= 160'd0;
        volume_start         <= 32'd0;
        fat_start_sector     <= 32'd0;
        data_start_sector    <= 32'd0;
        current_cluster      <= 32'd0;
        cluster_offset       <= 32'd0;
        sector_in_cluster    <= 8'd0;
        fat_byte_offset      <= 9'd0;
        directory_end        <= 1'b0;
        scan_stop_requested  <= 1'b0;
        filesystem_valid <= 0;
        decode_error <= 0;
        error_code <= 0;
        max_cluster <= 0;
        visited_clusters <= 0;
        chain_anchor <= 0;
        chain_power <= 1;
        chain_length <= 0;
        load_size <= 0;
        load_cluster_latched <= 0;
    end else if (!sd_init_done) begin
        state               <= ST_IDLE;
        state_code          <= 4'd0;
        sd_sec_read         <= 1'b0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_total    <= 3'd0;
        wav_found           <= 5'd0;
        directory_end       <= 1'b0;
        scan_stop_requested <= 1'b0;
        filesystem_valid <= 0;
        decode_error <= 0;
    end else if (load_abort) begin
        state       <= ST_IDLE;
        state_code  <= 4'd1;
        sd_sec_read <= 1'b0;
        write_req   <= 1'b0;
        decode_error <= 0;
    end else begin
        scan_found_valid <= 1'b0;
        decode_error <= 1'b0;
        // WAV directory entry is evaluated one cycle after the entry's last byte arrived
        wav_check <= 1'b0;
        if (wav_check && wav_entry_ok) begin
            wav_found[wav_entry_idx] <= 1'b1;
            wav_clusters[wav_entry_idx*32 +: 32] <= entry_cluster;
            wav_sizes[wav_entry_idx*32 +: 32] <= entry_size;
        end

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
                if (scan_start) begin
                    scan_done            <= 1'b0;
                    scan_found_total     <= 3'd0;
                    wav_found            <= 5'd0;
                    directory_end        <= 1'b0;
                    scan_stop_requested  <= 1'b0;
                    filesystem_valid <= 0;
                    error_code <= 0;
                    visited_clusters <= 1;
                    chain_power <= 1;
                    chain_length <= 0;
                    sd_sec_read_addr     <= scan_start_sector;
                    state                <= ST_BOOT;
                end else if (load_start && scan_done && filesystem_valid) begin
                    load_size <= 0;
                    for (load_index = 0; load_index < 4; load_index = load_index + 1)
                        if (load_index < scan_found_total && catalog_cluster[load_index] == load_sector)
                            load_size <= catalog_size[load_index];
                    load_cluster_latched <= load_sector;
                    chain_anchor <= load_sector;
                    chain_power <= 1;
                    chain_length <= 0;
                    visited_clusters <= 1;
                    error_code <= 0;
                    current_cluster  <= load_sector;
                    cluster_offset   <= cluster_sector_offset(load_sector);
                    sector_in_cluster <= 8'd0;
                    state            <= ST_ADDR_HDR;
                end
            end

            ST_BOOT: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bpb_valid && byte_index == 512) begin
                        filesystem_valid <= 1;
                        max_cluster <= cluster_limit;
                        chain_anchor <= root_cluster;
                        volume_start      <= scan_start_sector;
                        fat_start_sector  <= scan_start_sector + active_fat_offset;
                        data_start_sector <= scan_start_sector + data_offset_sectors[31:0];
                        current_cluster   <= root_cluster;
                        cluster_offset    <= cluster_sector_offset(root_cluster);
                        sector_in_cluster <= 8'd0;
                        state             <= ST_ADDR_ROOT;
                    end else if (partition_lba != 32'd0 && boot_signature == 16'haa55 && byte_index == 512) begin
                        sd_sec_read_addr    <= partition_lba;
                        state               <= ST_BPB;
                    end else begin
                        scan_done <= 1'b1;
                        error_code <= 1;
                        state     <= ST_IDLE;
                    end
                end
            end

            ST_BPB: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bpb_valid && byte_index == 512) begin
                        filesystem_valid <= 1;
                        max_cluster <= cluster_limit;
                        chain_anchor <= root_cluster;
                        volume_start      <= partition_lba;
                        fat_start_sector  <= partition_lba + active_fat_offset;
                        data_start_sector <= partition_lba + data_offset_sectors[31:0];
                        current_cluster   <= root_cluster;
                        cluster_offset    <= cluster_sector_offset(root_cluster);
                        sector_in_cluster <= 8'd0;
                        state             <= ST_ADDR_ROOT;
                    end else begin
                        scan_done <= 1'b1;
                        error_code <= 1;
                        state     <= ST_IDLE;
                    end
                end
            end

            ST_ROOT: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_data_valid && (byte_index[4:0] == 5'd31)) begin
                    if (entry_first == 8'h00)
                        directory_end <= 1'b1;
                    else if (!directory_end)
                        wav_check <= 1'b1;
                    if (directory_entry_is_bmp && !scan_stop_requested && !directory_end && scan_found_total < 4) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= entry_cluster;
                        // files named 1.BMP..4.BMP go to their own slot, others in directory order
                        scan_slot <= (entry_sfn[7:0] >= 8'h31 && entry_sfn[7:0] <= 8'h34 && entry_sfn[15:8] == 8'h20)
                                     ? (entry_sfn[1:0] - 2'd1) : scan_found_total[1:0];
                        scan_found_total  <= scan_found_total + 3'd1;
                        if ((scan_found_total + 3'd1) >= scan_target_count)
                            scan_stop_requested <= 1'b1;
                    end
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (directory_end || (scan_stop_requested && (&wav_found)) ||
                        (byte_index != 512)) begin
                        scan_done <= 1'b1;
                        if (byte_index != 512) error_code <= 2;
                        state     <= ST_IDLE;
                    end else if ((sector_in_cluster + 8'd1) < sectors_per_cluster) begin
                        sector_in_cluster <= sector_in_cluster + 8'd1;
                        sd_sec_read_addr  <= sd_sec_read_addr + 32'd1;
                    end else begin
                        fat_byte_offset   <= (current_cluster[6:0] << 2);
                        sd_sec_read_addr  <= fat_lookup_sector;
                        state             <= ST_FAT_ROOT;
                    end
                end
            end

            ST_FAT_ROOT: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (fat_next_cluster >= 32'h0FFF_FFF8 || !next_cluster_valid ||
                        fat_next_cluster == chain_anchor || visited_clusters >= MAX_ROOT_CLUSTERS || visited_clusters >= max_cluster || byte_index != 512) begin
                        scan_done <= 1'b1;
                        if (fat_next_cluster < 32'h0ffffff8 || byte_index != 512) error_code <= 3;
                        state     <= ST_IDLE;
                    end else begin
                        visited_clusters <= visited_clusters + 1;
                        if (chain_length + 1 == chain_power) begin
                            chain_anchor <= fat_next_cluster;
                            chain_power <= chain_power << 1;
                            chain_length <= 0;
                        end else chain_length <= chain_length + 1;
                        current_cluster   <= fat_next_cluster;
                        cluster_offset    <= cluster_sector_offset(fat_next_cluster);
                        sector_in_cluster <= 8'd0;
                        state             <= ST_ADDR_ROOT;
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd3;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match && byte_index == 512 && load_size >= 54) begin
                        write_req <= 1'b1;
                        state     <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                        decode_error <= 1;
                        error_code <= (byte_index != 512) ? 2 : 1;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req            <= 1'b0;
                    current_cluster       <= load_cluster_latched;
                    cluster_offset        <= cluster_sector_offset(load_cluster_latched);
                    sector_in_cluster     <= 8'd0;
                    state                 <= ST_RELEASE;
                end
            end

            ST_RELEASE: begin
                if (!write_req_ack) state <= ST_ADDR_DATA;
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (byte_index != 512) begin
                        state <= ST_IDLE;
                        decode_error <= 1;
                        error_code <= 2;
                    end else if (bmp_len_cnt >= payload_end) begin
                        state <= ST_IDLE;
                        if (pixel_count != expected_pixels || bmp_byte_idx != 0) begin
                            decode_error <= 1;
                            error_code <= 3;
                        end
                    end else if ((sector_in_cluster + 8'd1) < sectors_per_cluster) begin
                        sector_in_cluster <= sector_in_cluster + 8'd1;
                        sd_sec_read_addr  <= sd_sec_read_addr + 32'd1;
                    end else begin
                        fat_byte_offset  <= (current_cluster[6:0] << 2);
                        sd_sec_read_addr <= fat_lookup_sector;
                        state            <= ST_FAT_LOAD;
                    end
                end
            end

            ST_FAT_LOAD: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (!next_cluster_valid || byte_index != 512 || visited_clusters >= max_cluster ||
                        fat_next_cluster == chain_anchor ||
                        visited_clusters >= load_cluster_limit) begin
                        state <= ST_IDLE;
                        decode_error <= 1;
                        error_code <= 3;
                    end else begin
                        visited_clusters <= visited_clusters + 1;
                        if (chain_length + 1 == chain_power) begin
                            chain_anchor <= fat_next_cluster;
                            chain_power <= chain_power << 1;
                            chain_length <= 0;
                        end else chain_length <= chain_length + 1;
                        current_cluster   <= fat_next_cluster;
                        cluster_offset    <= cluster_sector_offset(fat_next_cluster);
                        sector_in_cluster <= 8'd0;
                        state             <= ST_ADDR_DATA;
                    end
                end
            end

            ST_ADDR_ROOT: begin
                sd_sec_read_addr <= data_start_sector + cluster_offset;
                state            <= ST_ROOT;
            end

            ST_ADDR_HDR: begin
                sd_sec_read_addr <= data_start_sector + cluster_offset;
                state            <= ST_LOAD_HDR;
            end

            ST_ADDR_DATA: begin
                sd_sec_read_addr <= data_start_sector + cluster_offset;
                state            <= ST_LOAD_DATA;
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
