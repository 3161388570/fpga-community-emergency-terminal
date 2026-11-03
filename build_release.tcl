cd project/td_project
import_device eagle_s20.db -package EG4S20BG256
open_project lab_ex4_tf_optimized.al -single_run
load_run_param -run phy_1
commit_param -step design
elaborate -top top
read_adc ../user_source/constraints_source/pin.adc
read_sdc ../user_source/constraints_source/timing.sdc
read_sdc -ip rfifo_32_32_512 ../user_source/hdl_source/ip/afifo_32_16_256.tcl
read_sdc -ip wfifo_32_32_512 ../user_source/hdl_source/ip/afifo_16_32_256.tcl
file mkdir release_seed6_holdfix
cd release_seed6_holdfix
export_db elaborate.db
commit_param -step rtl
optimize_rtl
commit_param -step gate
optimize_gate -packarea gate.area
legalize_phy_inst
export_db gate.db
commit_param -step place
set_param place seed 6
place
commit_param -step route
set_param route fix_hold on
route
update_timing -mode final
report_timing_status -file phy.ts
report_timing_summary -file pr.timing
report_area -io_info -file phy.area
export_db pr.db
commit_param -step bitgen
bitgen -bit spectrum64_release.bit -bin spectrum64_release.bin
puts SPECTRUM64_RELEASE_BUILD_COMPLETE
exit
