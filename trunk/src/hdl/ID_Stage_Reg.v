module ID_Stage_Reg (
    input clk,               // Clock signal
    input rst,               // Reset signal
    input flush,             // Flush signal
    // inputs from previous stage
    input WB_EN_IN,         // Write back enable input
    input MEM_R_EN_IN,      // Memory read enable input
    input MEM_W_EN_IN,      // Memory write enable input
    input B_IN,             // Branch input
    input [3:0] EXE_CMD_IN, // Execution command input
    input [31:0] PC_IN,     // Program counter input
    input [31:0] Val_Rn_IN, // Value of Rn input
    input [31:0] Val_Rm_IN, // Value of Rm input
    input imm_IN,    // Immediate input
    input signed [11:0] Shift_operand_IN, // Shift operand input
    input [23:0] Signed_imm_24_IN, // Signed immediate value input
    input [3:0] Dest_IN,    // Destination register input
    // outputs
    output reg WB_EN,       // Write back enable
    output reg MEM_R_EN,    // Memory read enable
    output reg MEM_W_EN,    // Memory write enable
    output reg B,           // Branch signal
    output reg [3:0] EXE_CMD, // Execution command
    output reg [31:0] PC,   // Program counter
    output reg [31:0] Val_Rn, // Value of Rn
    output reg [31:0] Val_Rm, // Value of Rm
    output reg [11:0] Shift_operand, // Shift operand
    output reg [23:0] Signed_imm_24, // Signed immediate value
    output reg [3:0] Dest   // Destination register
);
   always @(posedge clk or posedge rst) begin
        if (rst) begin
            PC <= 32'b0;       // Reset PC to 0
        end else if (flush) begin
            PC <= 32'b0;       // Reset PC to 0 on flush
        end else begin
            PC <= PC_IN;       // Pass the incoming PC value
        end
    end
endmodule
