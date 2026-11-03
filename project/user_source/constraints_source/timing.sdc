
# ============================================================
# lab_ex4_tf_optimized.sdc
# 用于：TF 卡 BMP 轮播 + SDRAM 双缓冲 + HDMI 视频输出
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径按当前工程实例名编写：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#
#    若重新生成 PLL 后实例路径变化，应同步更新这些路径。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]


# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    像素域和 5x 串化域通过专用串化器连接，不按普通寄存器路径分析。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# SDRAM SIP 使用同频 0/180 度时钟直接连接硬核 DQ PAD。该半周期接口的
# setup/hold 已由 SDRAM 控制器和硬核模型定义，不能按普通 fabric 路径重复收敛。
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {sys_pll_m0/pll_inst.clkc[2]}]
set_false_path -from [get_clocks {sys_pll_m0/pll_inst.clkc[2]}] -to [get_clocks {ext_mem_clk}]



# ------------------------------------------------------------
# 6. 各功能时钟域通过异步 FIFO、握手同步器或异步复位连接。
#    禁止 STA 把不相关 PLL 相位之间的 CDC 路径当作同步路径收敛。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {clk}] \
    -group [get_clocks {sd_card_clk}] \
    -group [get_clocks {ext_mem_clk}] \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------
