`timescale 1ns/1ns

`define WORD_WIDTH 32
`define REG_FILE_DEPTH 4
`define REG_FILE_SIZE 16
`define MEMORY_DATA_LEN 8
`define MEMORY_SIZE 2048
`define SIGNED_IMM_WIDTH 24
`define SHIFTER_OPERAND_WIDTH 12

`define CLOCK_PERIOD 20

module ARM_new_Testbench;
    parameter clock_period = `CLOCK_PERIOD;

    reg clk;
    reg rst;
  //reg enableForwarding;

    wire [`WORD_WIDTH-1:0] IF_stage_pc_out;
  wire [`WORD_WIDTH-1:0] IF_stage_instruction_out;
  wire [`WORD_WIDTH-1:0] branch_address;
  wire EXE_stage_B_out;
  wire hazard_detected;

  IF_Stage  IF_Stage_Inst (
   .clk(clk),
   .rst(rst),
   .freeze(hazard_detected),
   .Branch_taken(EXE_stage_B_out),
   .BranchAddr(branch_address),
   .PC(IF_stage_pc_out),
   .Instruction(IF_stage_instruction_out)
  );

  wire [`WORD_WIDTH-1:0] IF_reg_pc_out;
  wire [`WORD_WIDTH-1:0] IF_reg_instruction_out;

  IF_Stage_Reg  IF_Reg_Inst (
   .clk(clk),
   .rst(rst),
   .freeze(hazard_detected),
   .flush(EXE_stage_B_out),
   .PC_in(IF_stage_pc_out),
   .Instruction_in(IF_stage_instruction_out),
   .PC(IF_reg_pc_out),
   .Instruction(IF_reg_instruction_out)
  );

  //wire [`WORD_WIDTH-1:0]              ID_stage_pc_out;
  //wire [`WORD_WIDTH-1:0]              ID_stage_instruction_out;
    wire [`REG_FILE_DEPTH-1:0]              ID_stage_reg_file_dst;
  wire [`REG_FILE_DEPTH-1:0]                ID_stage_reg_file_src1, ID_stage_reg_file_src2;
    wire [`WORD_WIDTH-1:0]                      ID_stage_val_Rn, ID_stage_val_Rm;
    wire [`SIGNED_IMM_WIDTH-1:0]            ID_stage_signed_immediate;
    wire [`SHIFTER_OPERAND_WIDTH-1:0] ID_stage_shifter_operand;
    wire [3:0]                                              ID_stage_EX_command_out;
    wire [3:0]              status;
    wire ID_stage_mem_read_out, ID_stage_mem_write_out,
        ID_stage_WB_en_out,
        ID_stage_Imm_out,
        ID_stage_B_out,
        ID_stage_SR_update_out;

  wire [`REG_FILE_DEPTH-1:0] WB_Stage_dst_out;
  wire [`WORD_WIDTH-1:0] WB_Value;
  wire WB_Stage_WB_en_out;
  wire has_src2;

  ID_Stage ID_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .hazard(hazard_detected),
    //.PC_IN(IF_reg_pc_out),
    .Instruction(IF_reg_instruction_out),
    .Dest_wb(WB_Stage_dst_out),
      .Result_WB(WB_Value),
      .writeBackEn(WB_Stage_WB_en_out),
    .SR(status),
    //.PC(ID_stage_pc_out),
    //.instruction(ID_stage_instruction_out),
      .Dest(ID_stage_reg_file_dst),
      .Val_Rn(ID_stage_val_Rn), .Val_Rm(ID_stage_val_Rm),
      .Signed_imm_24(ID_stage_signed_immediate),
      .Shift_operand(ID_stage_shifter_operand),
      .EXE_CMD(ID_stage_EX_command_out),
      .MEM_R_EN(ID_stage_mem_read_out), .MEM_W_EN(ID_stage_mem_write_out),
        .WB_EN(ID_stage_WB_en_out),
        .imm(ID_stage_Imm_out),
        .B(ID_stage_B_out),
        .S(ID_stage_SR_update_out),
        //.has_src2(has_src2),
        .Two_src(has_src1),
        .src1(ID_stage_reg_file_src1),
    .src2(ID_stage_reg_file_src2)
  );

  wire [`WORD_WIDTH-1:0] ID_reg_pc_out;
  //wire [`WORD_WIDTH-1:0] ID_reg_instruction_out;
  wire [`REG_FILE_DEPTH-1:0] ID_reg_reg_file_dst_out;
  wire [`WORD_WIDTH-1:0] ID_reg_val_Rn_out, ID_reg_val_Rm_out;
  wire [`SIGNED_IMM_WIDTH-1:0] ID_reg_signed_immediate_out;
  wire [`SHIFTER_OPERAND_WIDTH-1:0] ID_reg_shifter_operand_out;
  wire [3:0] ID_reg_SR_out, ID_reg_EX_command_out;
  wire ID_reg_mem_read_out, ID_reg_mem_write_out,
    ID_reg_WB_en_out,
    ID_reg_Imm_out,
    ID_reg_B_out,
    ID_reg_SR_update_out;
  //wire [`REG_FILE_DEPTH-1:0] ID_reg_reg_file_src1, ID_reg_reg_file_src2;

  ID_Stage_Reg ID_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .flush(EXE_stage_B_out),
    .PC_IN(IF_reg_pc_out),
    //.instruction_in(ID_stage_instruction_out),
    .Dest_IN(ID_stage_reg_file_dst),
      .Val_Rn_IN(ID_stage_val_Rn), .Val_Rm_IN(ID_stage_val_Rm),
      .Signed_imm_24_IN(ID_stage_signed_immediate),
      .Shift_operand_IN(ID_stage_shifter_operand),
    .EXE_CMD_IN(ID_stage_EX_command_out),
      .MEM_R_EN_IN(ID_stage_mem_read_out), .MEM_W_EN_IN(ID_stage_mem_write_out),
        .WB_EN_IN(ID_stage_WB_en_out),
        .imm_IN(ID_stage_Imm_out),
        .B_in(ID_stage_B_out),
        .S_in(ID_stage_SR_update_out),
    //.reg_file_src1_in(ID_stage_reg_file_src1),
    //.reg_file_src2_in(ID_stage_reg_file_src2),
    .PC(ID_reg_pc_out),
    //.instruction(ID_reg_instruction_out),
    .Dest(ID_reg_reg_file_dst_out),
      .Val_Rn(ID_reg_val_Rn_out), .Val_Rm(ID_reg_val_Rm_out),
      .Signed_imm_24(ID_reg_signed_immediate_out),
      .Shift_operand(ID_reg_shifter_operand_out),
    .EXE_CMD(ID_reg_EX_command_out),
      .MEM_R_EN(ID_reg_mem_read_out), .MEM_W_EN(ID_reg_mem_write_out),
        .WB_EN(ID_reg_WB_en_out),
        .imm(ID_reg_Imm_out),
        .B_out(ID_reg_B_out),
    .S_out(ID_reg_SR_update_out),
    .SR_in(status),
    .SR_out(ID_reg_SR_out)//,
    //.reg_file_src1_out(ID_reg_reg_file_src1),
    //.reg_file_src2_out(ID_reg_reg_file_src2)
  );

  wire [`WORD_WIDTH-1:0] EXE_stage_pc_out;
  wire [`WORD_WIDTH-1:0] EXE_stage_instruction_out;
  wire [`REG_FILE_DEPTH-1:0] EXE_stage_reg_file_dst_out;
  wire [`WORD_WIDTH-1:0] EXE_stage_val_Rm_out;
  wire [3:0] EXE_stage_SR_out;
  wire [`WORD_WIDTH-1:0] ALU_res;
  wire EXE_stage_mem_read_out, EXE_stage_mem_write_out,
    EXE_stage_WB_en_out;

  //wire [1:0] EXE_sel_src1, EXE_sel_src2;
  wire [`WORD_WIDTH-1:0] Mem_Stage_ALU_res_out;

  EXE_Stage EXE_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .pc_in(ID_reg_pc_out),
    //.instruction_in(ID_reg_instruction_out),
    //.MEM_stage_val(Mem_Stage_ALU_res_out),
    //.WB_stage_val(WB_Value),
    .signed_immediate(ID_reg_signed_immediate_out),
    .EX_command(ID_reg_EX_command_out),
    .SR_in(ID_reg_SR_out),
    .shifter_operand(ID_reg_shifter_operand_out),
    .dst_in(ID_reg_reg_file_dst_out),
    .mem_read_in(ID_reg_mem_read_out), .mem_write_in(ID_reg_mem_write_out),
    .imm(ID_reg_Imm_out),
    .WB_en_in(ID_reg_WB_en_out),
    .B_in(ID_reg_B_out),
    .val_Rn_in(ID_reg_val_Rn_out), .val_Rm_in(ID_reg_val_Rm_out),

    // .sel_src1(EXE_sel_src1),
    // .sel_src2(EXE_sel_src2),

    .dst_out(EXE_stage_reg_file_dst_out),
    .SR_out(EXE_stage_SR_out),
    .ALU_res(ALU_res),
    .val_Rm_out(EXE_stage_val_Rm_out),
    .branch_address(branch_address),
    .mem_read_out(EXE_stage_mem_read_out), .mem_write_out(EXE_stage_mem_write_out),
    .WB_en_out(EXE_stage_WB_en_out),
    .B_out(EXE_stage_B_out),
    .pc(EXE_stage_pc_out)//,
    //.instruction(EXE_stage_instruction_out)
  );

  wire [`WORD_WIDTH-1:0] EXE_reg_pc_out;
  wire [`WORD_WIDTH-1:0] EXE_reg_instruction_out;
  wire [`REG_FILE_DEPTH-1:0] EXE_reg_dst_out;
  wire [`WORD_WIDTH-1:0] EXE_reg_ALU_res_out;
  wire [`WORD_WIDTH-1:0] EXE_reg_val_Rm_out;
  wire EXE_reg_mem_read_out, EXE_reg_mem_write_out, EXE_reg_WB_en_out;

  EXE_Stage_Reg EXE_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .pc_in(EXE_stage_pc_out),
    //.instruction_in(EXE_stage_instruction_out),
    .dst_in(EXE_stage_reg_file_dst_out),
    .mem_read_in(EXE_stage_mem_read_out), .mem_write_in(EXE_stage_mem_write_out),
    .WB_en_in(EXE_stage_WB_en_out),
    .val_Rm_in(EXE_stage_val_Rm_out),
    .ALU_res_in(ALU_res),
    .dst_out(EXE_reg_dst_out),
    .ALU_res_out(EXE_reg_ALU_res_out),
    .val_Rm_out(EXE_reg_val_Rm_out),
    .mem_read_out(EXE_reg_mem_read_out), .mem_write_out(EXE_reg_mem_write_out),
    .WB_en_out(EXE_reg_WB_en_out),
    .pc(EXE_reg_pc_out)//,
    //.instruction(EXE_reg_instruction_out)
  );

  wire [`REG_FILE_DEPTH-1:0] Mem_Stage_dst_out;
  wire [`WORD_WIDTH-1:0] Mem_Stage_mem_out;
  wire Mem_Stage_read_out, Mem_Stage_WB_en_out;

  Mem_Stage Mem_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .dst(EXE_reg_dst_out),
    .ALU_res(EXE_reg_ALU_res_out),
    .val_Rm(EXE_reg_val_Rm_out),
    .mem_read(EXE_reg_mem_read_out),
    .mem_write(EXE_reg_mem_write_out),
    .WB_en(EXE_reg_WB_en_out),

    .dst_out(Mem_Stage_dst_out),
    .ALU_res_out(Mem_Stage_ALU_res_out),
    .mem_out(Mem_Stage_mem_out),
    .mem_read_out(Mem_Stage_read_out),
    .WB_en_out(Mem_Stage_WB_en_out)
  );

  wire [`REG_FILE_DEPTH-1:0] Mem_Reg_dst_out;
  wire [`WORD_WIDTH-1:0] Mem_Reg_ALU_res_out;
  wire [`WORD_WIDTH-1:0] Mem_Reg_mem_out;
  wire Mem_Reg_read_out, Mem_Reg_WB_en_out;

  MEM_Reg Mem_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .dst(Mem_Stage_dst_out),
    .ALU_res(Mem_Stage_ALU_res_out),
    .mem(Mem_Stage_mem_out),
    .mem_read(Mem_Stage_read_out),
    .WB_en(Mem_Stage_WB_en_out),

    .dst_out(Mem_Reg_dst_out),
    .ALU_res_out(Mem_Reg_ALU_res_out),
    .mem_out(Mem_Reg_mem_out),
    .mem_read_out(Mem_Reg_read_out),
    .WB_en_out(Mem_Reg_WB_en_out)
  );

  WB_Stage WB_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .dst(Mem_Reg_dst_out),
    .ALU_res(Mem_Reg_ALU_res_out),
    .mem(Mem_Reg_mem_out),
    .mem_read(Mem_Reg_read_out),
    .WB_en(Mem_Reg_WB_en_out),
    .WB_Dest(WB_Stage_dst_out),
    .WB_en_out(WB_Stage_WB_en_out),
    .WB_Value(WB_Value)
  );

  Status_Reg Status_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .load(ID_reg_SR_update_out),
    .status_in(EXE_stage_SR_out),
    .status(status)
  );

  wire EXE_WB_en = ID_reg_WB_en_out;
  wire MEM_WB_en = EXE_reg_WB_en_out;
  wire[`REG_FILE_DEPTH-1:0] EXE_dest = ID_reg_reg_file_dst_out;
  wire[`REG_FILE_DEPTH-1:0] MEM_dest = EXE_reg_dst_out;

  HazardUnit Hazard_Detection_Unit_Inst(
    //.enableForwarding(enableForwarding),
    .rn(ID_stage_reg_file_src1),
    .rdm(ID_stage_reg_file_src2),
    .destEx(EXE_dest),
    .destMem(MEM_dest),
    .wbEnEx(EXE_WB_en),
    .wbEnMem(MEM_WB_en),
    //.EXE_memread_en(EXE_stage_mem_read_out),
    //.has_src1(has_src1),
    .twoSrc(has_src2),
    .hazard(hazard_detected)
  );

  // Forwarding_Unit Forwarding_Unit_Inst(
  //   .enable(enableForwarding),
  //   .src1(ID_reg_reg_file_src1),
  //   .src2(ID_reg_reg_file_src2),
  //   .MEM_dest(MEM_dest),
  //   .WB_dest(WB_Stage_dst_out),
  //   .MEM_WB_en(MEM_WB_en),
  //   .WB_WB_en(WB_Stage_WB_en_out),
  //   .sel_src1(EXE_sel_src1),
  //   .sel_src2(EXE_sel_src2)
  // );


  initial begin
    clk = 0;
    forever clk = #clock_period ~clk;
  end

  initial begin
    //enableForwarding = 0;
    rst = 1;
    # (clock_period / 2);
    rst = 0;
    # (1000*clock_period);
    $stop;
  end

endmodule
