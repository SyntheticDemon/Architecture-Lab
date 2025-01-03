`timescale 1ns / 1ps

module ID_Stage_Testbench;

    // Inputs for IF_Stage and IF_Stage_Reg
    reg clk;
    reg rst;
    reg freeze;
    reg flush;
    reg Branch_taken;
    // TODO give branch taken
    // Outputs from IF_Stage and IF_Stage_Reg
    wire [31:0] PC;
    wire [31:0] PC_Reg_ID;
    wire [31:0] PC_Reg_IF;
    wire [31:0] Instruction;
    wire [31:0] Instruction_Reg;

    // Wires for ID_Stage inputs and outputs
    reg [31:0] Result_WB;         // Result from the Write-Back stage
    reg writeBackEn;              // Write-back enable signal
    reg [3:0] Dest_wb;            // Destination register address in Write-Back
    reg hazard;                   // Hazard detection signal
    reg [3:0] SR;                 // Status Register

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
    wire [11:0] Shift_operand;    // Shift operand output
    wire [23:0] Signed_imm_24;    // Signed immediate value output
    wire [23:0] Signed_imm_24_reg;    // Signed immediate value output
    wire [11:0] Shift_operand_reg;    // Shift operand output
    wire [3:0] Dest;              // Destination register output
    wire [3:0] Dest_reg;              // Destination register output
    wire [3:0] src1, src2;        // Source register addresses
    wire Two_src;                 // Two source operand indicator
    wire B;
    wire S;
    wire B_reg;
    wire S_reg;
    // Instantiate the IF_Stage module
    IF_Stage if_stage_inst (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .Branch_taken(Branch_taken),
        .PC(PC),
        .Instruction(Instruction)
    );

    // Instantiate the IF_Stage_Reg module
    IF_Stage_Reg if_stage_reg_inst (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .flush(flush),
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
        .SR(SR),                       // Status Register
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

    // Instantiate the ID_Stage_Reg module
    ID_Stage_Reg id_stage_reg_inst (
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .WB_EN_IN(writeBackEn),      // Pass in writeBackEn
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
        .B_out(B_reg),
        .S_out(S_reg),
        .Shift_operand(Shift_operand_reg), // Output shift operand
        .Signed_imm_24(Signed_imm_24_reg), // Output signed immediate
        .Dest(Dest_reg)                  // Output destination register
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
        freeze = 0;
        Branch_taken = 0;
        flush = 0;
        Result_WB = 32'h0;
        writeBackEn = 0;
        Dest_wb = 4'b0000;
        hazard = 0;
        SR = 4'b0000;
        
        // Apply reset
        rst = 1; #10;
        rst = 0; #10;

        repeat (1000) begin
            #10; // Wait for a few clock cycles

            // Display the current PC and lagged values
            $display("Current PC_Reg = %h, Lagged PC (1 cycle) = %h, Lagged PC (2 cycles) = %h, Instruction = %h , Instruction_reg = %h",
                     PC, PC_Reg_IF, PC_Reg_ID,Instruction,Instruction_Reg); // Print current and lagged PC
        end

        // End the simulation
        $stop;
    end

endmodule
