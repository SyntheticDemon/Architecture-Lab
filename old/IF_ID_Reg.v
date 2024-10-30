module IF_ID_Reg(
    input clk, rst,
    input freeze,               // Pipeline stall signal
    input flush,                // Pipeline flush signal
    input [31:0] pc_in,        // PC from IF stage
    input [31:0] inst_in,      // Instruction from IF stage
    output reg [31:0] pc_out,  // PC to ID stage
    output reg [31:0] inst_out // Instruction to ID stage
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_out <= 32'b0;
            inst_out <= 32'b0;
        end
        else if (flush) begin
            pc_out <= 32'b0;
            inst_out <= 32'b0;
        end
        else if (~freeze) begin
            pc_out <= pc_in;
            inst_out <= inst_in;
        end
    end
endmodule