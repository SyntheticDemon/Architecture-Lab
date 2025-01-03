`timescale 1ns / 1ps

module ARM_Testbench;

    // Inputs for IF_Stage and IF_Stage_Reg
    reg clk;
    reg rst;
    // Outputs from IF_Stage and IF_Stage_Reg
    wire [31:0] PC;
    wire [31:0] PC_Reg_ID;
    wire [31:0] PC_Reg_IF;
    wire [31:0] Instruction;
    wire [31:0] Instruction_Reg;

    // Wires for ID_Stage inputs and outputs
    wire [31:0] Result_WB;         // Result from the Write-Back stage
    wire writeBackEn;              // Write-back enable signal
    wire [3:0] Dest_wb;            // Destination register address in Write-Back
    wire hazard;                   // Hazard detection signal
    wire WB_EN;                   // Write-back enable output
    wire WB_EN_reg;                   // Write-back enable output
    wire MEM_R_EN;                // Memory read enable output
    wire MEM_R_EN_reg;                // Memory read enable output
    wire MEM_W_EN;                // Memory write enable output
    wire MEM_W_EN_reg;                // Memory write enable output
    wire [3:0] EXE_CMD;           // Execution command output
    wire [3:0] EXE_CMD_reg;           // Execution command output
    wire [31:0] Val_Rn;           // Value of Rn output
    wire [31:0] Val_Rm;           // Value of Rm output
    wire [31:0] Val_Rn_reg;           // Value of Rn output
    wire [31:0] Val_Rm_reg;           // Value of Rm output
    wire imm;                     // Immediate value output
    wire imm_ID;
    wire [3:0] SR_out;
    wire [11:0] Shift_operand;    // Shift operand output
    wire [23:0] Signed_imm_24;    // Signed immediate value output
    wire [23:0] Signed_imm_24_reg;    // Signed immediate value output
    wire [11:0] Shift_operand_reg;    // Shift operand output
    wire [3:0] Dest;              // Destination register output
    wire [3:0] Dest_reg;              // Destination register output
    //wire [3:0] Dest_exe_Reg;
    wire [3:0] src1, src2;        // Source register addresses
    wire Two_src;                 // Two source operand indicator
    wire B;
    wire S;
    wire B_reg;
    wire S_reg;
    wire [31:0] EXE_branch_address_output;
    // Instantiate the IF_Stage module
    IF_Stage if_stage_inst (
        .clk(clk),
        .rst(rst),
        .freeze(hazard),
        .Branch_taken(B_reg),
        .BranchAddr(EXE_branch_address_output),
        .PC(PC),
        .Instruction(Instruction)
    );

    // Instantiate the IF_Stage_Reg module
    IF_Stage_Reg if_stage_reg_inst (
        .clk(clk),
        .rst(rst),
        .freeze(hazard),
        .flush(B_reg),
        .PC_in(PC),
        .Instruction_in(Instruction),
        .PC(PC_Reg_IF),
        .Instruction(Instruction_Reg)
    );
    // Instantiate the ID_Stage module
    ID_Stage id_stage_inst (
        .clk(clk),
        .rst(rst),
        .Instruction(Instruction_Reg), // Instruction from IF_Stage_Reg
        .Result_WB(Result_WB),         // Result from WB stage
        .writeBackEn(writeBackEn),     // Write-back enable
        .Dest_wb(Dest_wb),             // Destination register address
        .hazard(hazard),               // Hazard detection signal
        .SR(SR_out),                       // Status Register

        .WB_EN(WB_EN),                 // Write-back enable output
        .MEM_R_EN(MEM_R_EN),           // Memory read enable output
        .MEM_W_EN(MEM_W_EN),           // Memory write enable output
        .EXE_CMD(EXE_CMD),             // Execution command output
        .Val_Rn(Val_Rn),               // Value of Rn output
        .Val_Rm(Val_Rm),               // Value of Rm output
        .imm(imm),                     // Immediate value output
        .Shift_operand(Shift_operand), // Shift operand output
        .Signed_imm_24(Signed_imm_24), // Signed immediate value output
        .Dest(Dest),                   // Destination register output
        .src1(src1),                   // Source 1 register
        .src2(src2),                   // Source 2 register
        .B(B),
        .S(S),
        .Two_src(Two_src)              // Two source operand indicator
    );

    wire [3:0] ID_SR_out;
    // Instantiate the ID_Stage_Reg module
    ID_Stage_Reg id_stage_reg_inst (
        .clk(clk),
        .rst(rst),
        .flush(B_reg),
        .WB_EN_IN(WB_EN),      // Pass in writeBackEn
        .MEM_R_EN_IN(MEM_R_EN),      // Pass in MEM_R_EN if needed
        .MEM_W_EN_IN(MEM_W_EN),      // Pass in MEM_W_EN if needed
        .B_in(B),         // Pass in branch signal if needed
        .S_in(S),         // Pass in branch signal if needed
        .EXE_CMD_IN(EXE_CMD),        // Pass in execution command if needed
        .PC_IN(PC_Reg_IF),           // Pass in the PC from IF_Stage_Reg
        .Val_Rn_IN(Val_Rn),          // Pass in Val_Rn if needed
        .Val_Rm_IN(Val_Rm),          // Pass in Val_Rm if needed
        .imm_IN(imm),                // Pass in immediate value if needed
        .Shift_operand_IN(Shift_operand), // Pass in shift operand if needed
        .Signed_imm_24_IN(Signed_imm_24), // Pass in signed immediate if needed
        .Dest_IN(Dest),              // Pass in destination register if needed
        .WB_EN(WB_EN_reg),               // Output write back enable
        .MEM_R_EN(MEM_R_EN_reg),         // Output memory read enable
        .MEM_W_EN(MEM_W_EN_reg),         // Output memory write enable
        .EXE_CMD(EXE_CMD_reg),           // Output execution command
        .PC(PC_Reg_ID),
        .Val_Rn(Val_Rn_reg),             // Output value of Rn
        .Val_Rm(Val_Rm_reg),             // Output value of Rm
        .imm(imm_ID),
        .SR_in(SR_out),
        
        .B_out(B_reg),
        .S_out(S_reg),
        .SR_out(ID_SR_out),
        .Shift_operand(Shift_operand_reg), // Output shift operand
        .Signed_imm_24(Signed_imm_24_reg), // Output signed immediate
        .Dest(Dest_reg)                  // Output destination register
    );

    wire [31:0] EXE_stage_pc_out;
    wire [31:0] EXE_stage_instruction_out;
    wire [3:0] Dest_exe_reg;
    wire [31:0] EXE_stage_val_Rm_out;
    wire [3:0] EXE_stage_SR_out;
    wire [31:0] ALU_res;
    wire EXE_stage_mem_read_out, EXE_stage_mem_write_out,EXE_stage_WB_en_out,EXE_stage_B_out;

    wire [1:0] EXE_sel_src1, EXE_sel_src2;
    wire [31:0] Mem_Stage_ALU_res_out;
    wire [3:0] SR_final;
    // EXE Stage instantiation
    EXE_Stage exe_stage_inst (
        .clk(clk),
        .rst(rst),
        .pc_in(PC_Reg_ID),
        .instruction_in(Instruction_Reg),
        .MEM_stage_val(Mem_Stage_ALU_res_out),
        //.WB_stage_val(Result_WB),
        .signed_immediate(Signed_imm_24_reg),
        .EX_command(EXE_CMD_reg),
        .SR_in(ID_SR_out),
        .shifter_operand(Shift_operand_reg),
        .dst_in(Dest_reg),
        .mem_read_in(MEM_R_EN_reg),
        .mem_write_in(MEM_W_EN_reg),
        .imm(imm_ID),
        .WB_en_in(WB_EN_reg),
        .B_in(B_reg),
        .val_Rn_in(Val_Rn_reg),
        .val_Rm_in(Val_Rm_reg),
        // .sel_src1(EXE_sel_src1),
        // .sel_src2(EXE_sel_src2),
        .dst_out(Dest_exe_reg),
        .SR_out(SR_final),
        .ALU_res(ALU_res),
        .val_Rm_out(EXE_stage_val_Rm_out),
        .branch_address(EXE_branch_address_output),
        .mem_read_out(EXE_stage_mem_read_out),
        .mem_write_out(EXE_stage_mem_write_out),
        .WB_en_out(EXE_stage_WB_en_out),
        .B_out(EXE_stage_B_out),
        .pc(EXE_stage_pc_out),
        .instruction(EXE_stage_instruction_out)
    );
    wire [31:0] EXE_reg_pc_out;
    wire [31:0] EXE_reg_instruction_out;
    wire [3:0] EXE_reg_dst_out;
    wire [31:0] EXE_reg_ALU_res_out;
    wire [31:0] EXE_reg_val_Rm_out;
    wire EXE_reg_mem_read_out, EXE_reg_mem_write_out, EXE_reg_WB_en_out;
    Status_Reg status_register
    (
                .clk(clk),
                .rst(rst),
                .load(S_reg),
                .status_in(SR_final),
                .status(SR_out)
    );
    // EXE Stage Register instantiation
    EXE_Stage_Reg exe_stage_reg_inst (
        .clk(clk),
        .rst(rst),
        .pc_in(EXE_stage_pc_out),
        .instruction_in(EXE_stage_instruction_out),
        .dst_in(Dest_exe_reg),
        .mem_read_in(EXE_stage_mem_read_out),
        .mem_write_in(EXE_stage_mem_write_out),
        .WB_en_in(EXE_stage_WB_en_out),
        .val_Rm_in(EXE_stage_val_Rm_out),
        .ALU_res_in(ALU_res),
        .dst_out(EXE_reg_dst_out),
        .ALU_res_out(EXE_reg_ALU_res_out),
        .val_Rm_out(EXE_reg_val_Rm_out),
        .mem_read_out(EXE_reg_mem_read_out),
        .mem_write_out(EXE_reg_mem_write_out),
        .WB_en_out(EXE_reg_WB_en_out),
        .pc(EXE_reg_pc_out),
        .instruction(EXE_reg_instruction_out)
    );


  wire [3:0] Mem_Stage_dst_out;
  wire [31:0] Mem_Stage_mem_out;
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

  wire [3:0] Mem_Reg_dst_out;
  wire [31:0] Mem_Reg_ALU_res_out;
  wire [31:0] Mem_Reg_mem_out;
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

    .WB_Dest(Dest_wb),
    .WB_en_out(writeBackEn),
    .WB_Value(Result_WB)
  );

  wire EXE_WB_en = WB_EN_reg;
  wire MEM_WB_en = EXE_reg_WB_en_out;
  wire[3:0] EXE_dest = Dest_reg;
  wire[3:0] MEM_dest = EXE_reg_dst_out;

  HazardUnit HazardUnit_inst(
    .rn(src1),
    .rdm(src2),
    .twoSrc(Two_src),
    .destEx(EXE_dest),
    .destMem(MEM_dest),
    .wbEnEx(EXE_WB_en),
    .wbEnMem(MEM_WB_en),
    //.memREn(),
    
    .hazard(hazard)
);

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Test sequence
    initial begin
        // Initialize inputs
        rst = 0;
        //Branch_taken = 0;
        //flush = 0;
        //Result_WB = 32'h0;
        //writeBackEn = 0;
        //Dest_wb = 4'b0000;
        //hazard = 0;
        
        // Apply reset
        rst = 1; #10;
        rst = 0; #10;

        repeat (1000) begin
            #10; // Wait for a few clock cycles

            // Display the current PC and lagged values
            $display("Current PC_Reg = %h, Lagged PC (1 cycle) = %h, Lagged PC (2 cycles) = %h, Instruction = %h , Instruction_reg = %h, ALU Res out = %h, Mem out = %h, Write back value = %h",
                     PC, PC_Reg_IF, PC_Reg_ID,Instruction,Instruction_Reg,EXE_reg_ALU_res_out, Mem_Reg_mem_out, Result_WB); // Print current and lagged PC
        end

        // End the simulation
        $stop;
    end

endmodule
