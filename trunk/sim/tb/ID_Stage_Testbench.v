`timescale 1ns / 1ps

module ID_Stage_Testbench;

    // Inputs for IF_Stage and IF_Stage_Reg
 reg clk;
    reg rst;
    reg freeze;
    reg flush;
    reg Branch_taken;
    reg [31:0] BranchAddr;
    reg [31:0] PC_in;
    reg [31:0] Instruction_in;

    // Outputs from IF_Stage and IF_Stage_Reg
    wire [31:0] PC;
    wire [31:0] Instruction;
    wire [31:0] PC_Reg;
    wire [31:0] Instruction_Reg;

    // Wires for ID_Stage inputs and outputs
    reg [31:0] Result_WB;         // Result from the Write-Back stage
    reg writeBackEn;              // Write-back enable signal
    reg [3:0] Dest_wb;            // Destination register address in Write-Back
    reg hazard;                   // Hazard detection signal
    reg [3:0] SR;                 // Status Register

    wire WB_EN;                   // Write-back enable output
    wire MEM_R_EN;                // Memory read enable output
    wire MEM_W_EN;                // Memory write enable output
    wire [3:0] EXE_CMD;           // Execution command output
    wire [31:0] Val_Rn;           // Value of Rn output
    wire [31:0] Val_Rm;           // Value of Rm output
    wire [31:0] imm;              // Immediate value output
    wire [11:0] Shift_operand;    // Shift operand output
    wire [23:0] Signed_imm_24;    // Signed immediate value output
    wire [3:0] Dest;              // Destination register output
    wire [3:0] src1, src2;        // Source register addresses
    wire Two_src;                 // Two source operand indicator
    // Registers to store the lagged PC values
    reg [31:0] PC_lag1; // Register for the previous PC value
    reg [31:0] PC_lag2; // Register for the PC value before the previous one

    // Instantiate the IF_Stage module
    IF_Stage if_stage_inst (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .Branch_taken(Branch_taken),
        .BranchAddr(BranchAddr),
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
        .PC(PC_Reg),
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
        .B_IN(Branch_taken),          // Pass in branch signal if needed
        .EXE_CMD_IN(EXE_CMD),        // Pass in execution command if needed
        .PC_IN(PC_Reg),              // Pass in the PC from IF_Stage_Reg
        .Val_Rn_IN(Val_Rn),          // Pass in Val_Rn if needed
        .Val_Rm_IN(Val_Rm),          // Pass in Val_Rm if needed
        .imm_IN(imm),                // Pass in immediate value if needed
        .Shift_operand_IN(Shift_operand), // Pass in shift operand if needed
        .Signed_imm_24_IN(Signed_imm_24), // Pass in signed immediate if needed
        .Dest_IN(Dest),              // Pass in destination register if needed
        .WB_EN(WB_EN),               // Output write back enable
        .MEM_R_EN(MEM_R_EN),         // Output memory read enable
        .MEM_W_EN(MEM_W_EN),         // Output memory write enable
        .B(B),                        // Output branch signal
        .EXE_CMD(EXE_CMD),           // Output execution command
        .Val_Rn(Val_Rn),             // Output value of Rn
        .Val_Rm(Val_Rm),             // Output value of Rm
        .imm(imm),                   // Output immediate value
        .Shift_operand(Shift_operand), // Output shift operand
        .Signed_imm_24(Signed_imm_24), // Output signed immediate
        .Dest(Dest)                  // Output destination register
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

        // Apply reset
        rst = 1; #10;
        rst = 0; #10;
        $display("After Reset: PC_Reg = %h, Instruction_Reg = %h", PC_Reg, Instruction_Reg);
        
        // Initialize lagged PC registers
        PC_lag1 = 32'b0;
        PC_lag2 = 32'b0;

        // Test 1: Incrementally increase the address
        $display("\n--- Test 1: Incrementally Increasing the Address ---");
        BranchAddr = 32'h00000000; // Start address
        $display("PC = %h, Instruction = %h, PC_Reg = %h, Instruction_Reg = %h", PC, Instruction, PC_Reg, Instruction_Reg);
        
        repeat (7) begin
            #10; // Wait for a few clock cycles

            // Update lagged PC values
            PC_lag2 <= PC_lag1; // Shift PC_lag1 to PC_lag2
            PC_lag1 <= PC_Reg;  // Update PC_lag1 to current PC_Reg

            // Display the current PC and lagged values
            $display("Current PC_Reg = %h, Lagged PC (1 cycle) = %h, Lagged PC (2 cycles) = %h",
                     PC_Reg, PC_lag1, PC_lag2); // Print current and lagged PC
        end

        // End the simulation
        $finish;
    end

endmodule
