# 社区应急信息终端

安路 EG4S20BG256 的正式版 FPGA 工程。使用 Tang Dynasty 6.2.168116，建议在纯英文路径打开 project/td_project/lab_ex4_tf_optimized.al。

project/user_source 包含 HDL、IP 和约束。build_release.tcl 为原构建脚本，使用时请先核对其英文路径映射。工程中原有 al_ip/PLL.ipc 条目标为 AutoExcluded，实际构建使用 sys_pll 和 video_pll。仓库保存源码，重新构建需重新检查时序。
