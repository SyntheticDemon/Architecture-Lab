module instruction_memory (
    input [31:0] address,        // 32-bit address for selecting one of the 6 instructions
    output reg [31:0] instruction // 32-bit instruction output
);

    // Define the instructions based on the given binary values
    always @(*) begin
        case (address[2:0]) // Use the lower 3 bits of the address
            3'b000: instruction = 32'b000000_00001_00010_00000_00000000000;
            3'b001: instruction = 32'b000000_00011_00010_00000_00000000000;
            3'b010: instruction = 32'b000000_00011_00100_00000_00000000000;
            3'b011: instruction = 32'b000000_00101_00110_00000_00000000000;
            3'b100: instruction = 32'b000000_00111_00110_00010_00000000000;
            3'b101: instruction = 32'b000000_01001_01010_00011_00000000000;
            3'b110: instruction = 32'b000000_01011_01100_00000_00000000000;
            3'b111: instruction = 32'b000000_01101_01110_00000_00000000000;
            default: instruction = 32'b000000_00000_00000_00000_00000000000; // Default instruction (NOP)
        endcase
    end

endmodule
