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
    input [3:0] SR,          // Status register
    // to next stage
    output reg WB_EN,        // Write back enable
    output reg MEM_R_EN,     // Memory read enable
    output reg MEM_W_EN,     // Memory write enable
    output reg [3:0] EXE_CMD, // Execution command
    output reg [31:0] Val_Rn, // Value of Rn
    output reg [31:0] Val_Rm, // Value of Rm
    output reg [31:0] imm,    // Immediate value
    output reg [11:0] Shift_operand, // Shift operand
    output reg [23:0] Signed_imm_24, // Signed immediate value
    output reg [3:0] Dest,    // Destination register
    // to hazard detect module
    output reg [3:0] src1,    // Source 1
    output reg [3:0] src2,    // Source 2
    output reg Two_src        // Indicator for two source operands
);
    // Empty module - no internal logic
endmodule
