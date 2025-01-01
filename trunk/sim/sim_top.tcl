    alias clc ".main clear"

clc
exec vlib work
vmap work work

set TB                 "ARM_new_Testbench"
set hdl_path           "../src/hdl"
set inc_path           "../src/inc"

# set run_time           "1 us"
set run_time          "-all"

#============================ Add verilog files  ===============================
# Add all required HDL files
vlog    +acc -incr -source  +define+SIM     $hdl_path/*.v

# Include any header files from inc directory if needed
vlog    +acc -incr -source  +incdir+$inc_path +define+SIM   ./tb/$TB.v

onerror {break}

#================================ simulation ====================================
vsim    -voptargs=+acc -debugDB $TB

#======================= adding signals to wave window ==========================
# add wave -hex -group    {TB}                sim:/$TB/*
#add wave -hex -group    {top}               sim:/$TB/uut/*  
# add wave -hex -group -r {all}               sim:/$TB/*
#add wave -hex -group    {regfile}               sim:/$TB/ID_Stage_Inst/rf/*  
# add wave -position insertpoint sim:/ARM_new_Testbench/Hazard_Detection_Unit_Inst/*
do wave.do
# add wave -position insertpoint  \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/clk \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/rst \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/alu_res \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/Val_Rm \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/mem_w_en \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/mem_r_en \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/res_data \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/dataAdr \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/generatedAddr \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/mem_data \
# sim:/ARM_new_Testbench/Mem_Stage_Inst/M1/i \
# sim:/ARM_new_Testbench/EXE_Reg_Inst/pc_in \
# sim:/ARM_new_Testbench/EXE_Stage_Inst/ALU_res \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/clk \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/rst \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/src1 \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/src2 \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/Dest_wb \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/Result_WB \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/writeBackEn \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/Val_Rn \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/Val_Rm \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/register_array \
# sim:/ARM_new_Testbench/ID_Stage_Inst/rf/i
# do wave.do
#=========================== Configure wave signals =============================
configure wave -signalnamewidth 2

#====================================== run =====================================
run $run_time