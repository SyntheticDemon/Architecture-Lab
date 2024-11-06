alias clc ".main clear"

clc
exec vlib work
vmap work work

set TB                  "ID_Stage_Testbench"
set hdl_path           "../src/hdl"
set inc_path           "../src/inc"

set run_time           "1 us"
#set run_time          "-all"

#============================ Add verilog files  ===============================
# Add all required HDL files
vlog    +acc -incr -source  +define+SIM     $hdl_path/branch_mux.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/ID_Stage_Reg.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/ID_Stage.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/IF_Stage.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/IF_Stage_Reg.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/instruction_memory.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/pc_incrementer.v
vlog    +acc -incr -source  +define+SIM     $hdl_path/program_counter.v

# Include any header files from inc directory if needed
vlog    +acc -incr -source  +incdir+$inc_path +define+SIM   ./tb/$TB.v

onerror {break}

#================================ simulation ====================================
vsim    -voptargs=+acc -debugDB $TB

#======================= adding signals to wave window ==========================
add wave -hex -group    {TB}                sim:/$TB/*
#add wave -hex -group    {top}               sim:/$TB/uut/*  
add wave -hex -group -r {all}               sim:/$TB/*

#=========================== Configure wave signals =============================
configure wave -signalnamewidth 2

#====================================== run =====================================
run $run_time