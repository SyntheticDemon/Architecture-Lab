module IF_Stage_Reg (
    input clk,                     // Clock signal
    input rst,                     // Reset signal
    input freeze,                  // Freeze signal
    input flush,                   // Flush signal
    input [31:0] PC_in,            // Input PC
    input [31:0] Instruction_in,   // Input Instruction
    output reg [31:0] PC,          // Output PC
    output reg [31:0] Instruction  // Output Instruction
);

    // Sequential logic to update PC and Instruction
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset outputs
            PC <= 32'b0;
            Instruction <= 32'b0;
        end else if (flush) begin
            // Flush outputs
            PC <= 32'b0;
            Instruction <= 32'b0;
        end else if (!freeze) begin
            // Update outputs if not frozen
            PC <= PC_in;
            Instruction <= Instruction_in;
        end
        // If freeze is active, retain current values
    end

endmodule
