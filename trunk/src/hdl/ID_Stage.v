module ID_Stage (
    input clk,               // Clock signal
    input rst,               // Reset signal
    // from IF Reg
    input [31:0] Instruction, // Instruction from IF stage
    // from WB stage
    input [31:0] Result_WB,   // Result from WB stage
    input writeBackEn,        // Write enable signal
    input [3:0] Dest_wb,      // Destination register
    // from hazard detect module
    input hazard,             // Hazard detection signal
    // from Status Register
    input [3:0] SR,           // Status register
    // to next stage
    output WB_EN,             // Write back enable
    output MEM_R_EN,          // Memory read enable
    output MEM_W_EN,          // Memory write enable
    output [3:0] EXE_CMD,     // Execution command
    output [31:0] Val_Rn,     // Value of Rn
    output [31:0] Val_Rm,     // Value of Rm
    output imm,               // Immediate value
    output [11:0] Shift_operand, // Shift operand
    output signed [23:0] Signed_imm_24, // Signed immediate value
    output [3:0] Dest,        // Destination register
    // to hazard detect module
    output [3:0] src1,        // Source 1
    output [3:0] src2,        // Source 2
    output B,
    output S,
    output Two_src            // Indicator for two source operands
);

    wire [3:0] cond_code;          // Condition code from instruction
    wire [1:0] mode;               // Mode from instruction
    wire [3:0] opcode;             // Opcode from instruction
    wire cond_result;              // Condition check result
    wire ctrl_wbEn, ctrl_memRead, ctrl_memWrite, ctrl_branch, ctrl_s;
    wire [3:0] reg_src2;           // Source register 2
    wire [3:0] ctrl_aluCmd;        // ALU command from control unit
    wire cond_final;               // Final condition after hazard check

    // Assign instruction fields to named wires
    assign cond_code = Instruction[31:28];
    assign mode = Instruction[27:26];
    assign opcode = Instruction[24:21];
    assign imm = Instruction[25];
    assign Shift_operand = Instruction[11:0];
    assign Signed_imm_24 = Instruction[23:0];
    assign Dest = Instruction[15:12];
    assign src1 = Instruction[19:16];

    // Two source operands indicator
    assign Two_src = ~imm | MEM_W_EN;

    ConditionCheck condition_check(
        .condition(cond_code),
        .status(SR),
        .result(cond_result)
    );

    ControlUnit control_unit(
        .mode(mode),
        .opcode(opcode),
        .sIn(Instruction[20]),
        .aluCmd(ctrl_aluCmd),
        .memRead(ctrl_memRead),
        .memWrite(ctrl_memWrite),
        .wbEn(ctrl_wbEn),
        .branch(ctrl_branch),
        .sOut(ctrl_s)
    );

    RegisterFile rf(
        .clk(clk),
        .rst(rst),
        .src1(Instruction[19:16]),
        .src2(reg_src2),
        .Dest_wb(Dest_wb),
        .Result_WB(Result_WB),
        .writeBackEn(writeBackEn),
        .Val_Rn(Val_Rn),
        .Val_Rm(Val_Rm)
    );

    // Mux to select source register 2
    Mux2to1 #(.DATA_WIDTH(4)) mux_regSrc2(
        .in0(Instruction[3:0]),
        .in1(Instruction[15:12]),
        .sel(MEM_W_EN),
        .out(reg_src2)
    );

    // Assign outputs based on control signals and condition check
    assign cond_final = ~cond_result | hazard;
    assign src2 = reg_src2;

    // Mux to control signals based on final condition
    Mux2to1 #(.DATA_WIDTH(9)) mux_ctrlSignals(
        .in0({ctrl_aluCmd, ctrl_memRead, ctrl_memWrite, ctrl_wbEn, ctrl_branch, ctrl_s}),
        .in1(9'd0),
        .sel(cond_final),
        .out({EXE_CMD, MEM_R_EN, MEM_W_EN, WB_EN, B, S})
    );

endmodule
