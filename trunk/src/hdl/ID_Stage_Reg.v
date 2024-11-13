module ID_Stage_Reg (
    input clk,               // Clock signal
    input rst,               // Reset signal
    input flush,             // Flush signal
    // Inputs from previous stage
    input WB_EN_IN,          // Write back enable input
    input MEM_R_EN_IN,       // Memory read enable input
    input MEM_W_EN_IN,       // Memory write enable input
    input B_in,              // Branch input
    input S_in,              // Branch input
    input [3:0] EXE_CMD_IN,  // Execution command input
    input [31:0] PC_IN,      // Program counter input
    input [31:0] Val_Rn_IN,  // Value of Rn input
    input [31:0] Val_Rm_IN,  // Value of Rm input
    input imm_IN,            // Immediate input
    input signed [11:0] Shift_operand_IN, // Shift operand input
    input [23:0] Signed_imm_24_IN,        // Signed immediate value input
    input [3:0] Dest_IN,     // Destination register input
    // Outputs
    output reg WB_EN,        // Write back enable
    output reg MEM_R_EN,     // Memory read enable
    output reg MEM_W_EN,     // Memory write enable
    output reg [3:0] EXE_CMD, // Execution command
    output reg [31:0] PC,    // Program counter
    output reg [31:0] Val_Rn, // Value of Rn
    output reg [31:0] Val_Rm, // Value of Rm
    output reg imm,          // Immediate signal
    output reg [11:0] Shift_operand, // Shift operand
    output reg [23:0] Signed_imm_24, // Signed immediate value
    output reg [3:0] Dest,    // Destination register
    output reg B_out,
    output reg S_out
);

    // Sequential logic for the registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            WB_EN <= 1'b0;
            MEM_R_EN <= 1'b0;
            MEM_W_EN <= 1'b0;
            B_out <= 1'b0;
            S_out <= 1'b0;
            EXE_CMD <= 4'b0;
            PC <= 32'b0;
            Val_Rn <= 32'b0;
            Val_Rm <= 32'b0;
            imm <= 1'b0;
            Shift_operand <= 12'b0;
            Signed_imm_24 <= 24'b0;
            Dest <= 4'b0;
        end else if (flush) begin
            WB_EN <= 1'b0;
            MEM_R_EN <= 1'b0;
            MEM_W_EN <= 1'b0;
            B_out <= 1'b0;
            S_out <= 1'b0;
            EXE_CMD <= 4'b0;
            PC <= 32'b0;
            Val_Rn <= 32'b0;
            Val_Rm <= 32'b0;
            imm <= 1'b0;
            Shift_operand <= 12'b0;
            Signed_imm_24 <= 24'b0;
            Dest <= 4'b0;
        end else begin
            WB_EN <= WB_EN_IN;
            MEM_R_EN <= MEM_R_EN_IN;
            MEM_W_EN <= MEM_W_EN_IN;
            B_out <= B_in;
            S_out <= S_in;
            EXE_CMD <= EXE_CMD_IN;
            PC <= PC_IN;
            Val_Rn <= Val_Rn_IN;
            Val_Rm <= Val_Rm_IN;
            imm <= imm_IN;
            Shift_operand <= Shift_operand_IN;
            Signed_imm_24 <= Signed_imm_24_IN;
            Dest <= Dest_IN;
        end
    end

endmodule
