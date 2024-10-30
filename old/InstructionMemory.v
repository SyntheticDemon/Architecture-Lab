// Instruction Memory module
module InstructionMemory(
    input [31:0] pc,
    output reg [31:0] inst
);
    reg [31:0] memory [0:255];  // 256 words of 32-bit memory
    
    initial begin
        // Example program - you can modify these instructions
        memory[0] = 32'h E3A00001;  // MOV R0, #1
        memory[1] = 32'h E3A01002;  // MOV R1, #2
        memory[2] = 32'h E0802001;  // ADD R2, R0, R1
        memory[3] = 32'h E0A03002;  // ADC R3, R0, R2
        memory[4] = 32'h E2834001;  // ADD R4, R3, #1
    end
    
    always @(*) begin
        inst = memory[pc[9:2]];  // Word-aligned access
    end
endmodule