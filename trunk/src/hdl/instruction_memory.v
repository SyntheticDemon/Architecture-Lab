// module instruction_memory (
//     input [31:0] address,        // 32-bit address for selecting one of the 6 instructions
//     output reg [31:0] instruction // 32-bit instruction output
// );

//     // Define the instructions based on the given binary values
//     always @(*) begin
//         case (address[2:0]) // Use the lower 3 bits of the address
//             3'b000: instruction = 32'b000000_00001_00010_00000_00000000000;
//             3'b001: instruction = 32'b000000_00011_00010_00000_00000000000;
//             3'b010: instruction = 32'b000000_00011_00100_00000_00000000000;
//             3'b011: instruction = 32'b000000_00101_00110_00000_00000000000;
//             3'b100: instruction = 32'b000000_00111_00110_00010_00000000000;
//             3'b101: instruction = 32'b000000_01001_01010_00011_00000000000;
//             3'b110: instruction = 32'b000000_01011_01100_00000_00000000000;
//             3'b111: instruction = 32'b000000_01101_01110_00000_00000000000;
//             default: instruction = 32'b000000_00000_00000_00000_00000000000; // Default instruction (NOP)
//         endcase
//     end

// endmodule


module instruction_memory (
    input [31:0] address,        // 32-bit address for selecting an instruction
    output reg [31:0] instruction // 32-bit instruction output
);

    // Define the instructions based on the provided binary values
    always @(*) begin
        case (address[5:0]) // Use the lower 6 bits of the address for indexing up to 64 instructions
            6'd0 : instruction = 32'b1110_00_1_1101_0_0000_0000_000000010100; // MOV    R0, #20
            6'd1 : instruction = 32'b1110_00_1_1101_0_0000_0001_101000000001; // MOV    R1, #4096
            6'd2 : instruction = 32'b1110_00_1_1101_0_0000_0010_000100000011; // MOV    R2, #0xC0000000
            6'd3 : instruction = 32'b1110_00_0_0100_1_0010_0011_000000000010; // ADDS   R3, R2, R2
            6'd4 : instruction = 32'b1110_00_0_0101_0_0000_0100_000000000000; // ADC    R4, R0, R0
            6'd5 : instruction = 32'b1110_00_0_0010_0_0100_0101_000100000100; // SUB    R5, R4, R4, LSL #2
            6'd6 : instruction = 32'b1110_00_0_0110_0_0000_0110_000010100000; // SBC    R6, R0, R0, LSR #1
            6'd7 : instruction = 32'b1110_00_0_1100_0_0101_0111_000101000010; // ORR    R7, R5, R2, ASR #2
            6'd8 : instruction = 32'b1110_00_0_0000_0_0111_1000_000000000011; // AND    R8, R7, R3
            6'd9 : instruction = 32'b1110_00_0_1111_0_0000_1001_000000000110; // MVN    R9, R6
            6'd10: instruction = 32'b1110_00_0_0001_0_0100_1010_000000000101; // EOR    R10, R4, R5
            6'd11: instruction = 32'b1110_00_0_1010_1_1000_0000_000000000110; // CMP    R8, R6
            6'd12: instruction = 32'b0001_00_0_0100_0_0001_0001_000000000001; // ADDNE  R1, R1, R1
            6'd13: instruction = 32'b1110_00_0_1000_1_1001_0000_000000001000; // TST    R9, R8
            6'd14: instruction = 32'b0000_00_0_0100_0_0010_0010_000000000010; // ADDEQ  R2, R2, R2
            6'd15: instruction = 32'b1110_00_1_1101_0_0000_0000_101100000001; // MOV    R0, #1024
            6'd16: instruction = 32'b1110_01_0_0100_0_0000_0001_000000000000; // STR    R1, [R0], #0
            6'd17: instruction = 32'b1110_01_0_0100_1_0000_1011_000000000000; // LDR    R11, [R0], #0
            6'd18: instruction = 32'b1110_01_0_0100_0_0000_0010_000000000100; // STR    R2, [R0], #4
            6'd19: instruction = 32'b1110_01_0_0100_0_0000_0011_000000001000; // STR    R3, [R0], #8
            6'd20: instruction = 32'b1110_01_0_0100_0_0000_0100_000000001101; // STR    R4, [R0], #13
            6'd21: instruction = 32'b1110_01_0_0100_0_0000_0101_000000010000; // STR    R5, [R0], #16
            6'd22: instruction = 32'b1110_01_0_0100_0_0000_0110_000000010100; // STR    R6, [R0], #20
            6'd23: instruction = 32'b1110_01_0_0100_1_0000_1010_000000000100; // LDR    R10, [R0], #4
            6'd24: instruction = 32'b1110_01_0_0100_0_0000_0111_000000011000; // STR    R7, [R0], #24
            6'd25: instruction = 32'b1110_00_1_1101_0_0000_0001_000000000100; // MOV    R1, #4
            6'd26: instruction = 32'b1110_00_1_1101_0_0000_0010_000000000000; // MOV    R2, #0
            6'd27: instruction = 32'b1110_00_1_1101_0_0000_0011_000000000000; // MOV    R3, #0
            6'd28: instruction = 32'b1110_00_0_0100_0_0000_0100_000100000011; // ADD    R4, R0, R3, LSL #2
            6'd29: instruction = 32'b1110_01_0_0100_1_0100_0101_000000000000; // LDR    R5, [R4], #0
            6'd30: instruction = 32'b1110_01_0_0100_1_0100_0110_000000000100; // LDR    R6, [R4], #4
            6'd31: instruction = 32'b1110_00_0_1010_1_0101_0000_000000000110; // CMP    R5, R6
            6'd32: instruction = 32'b1100_01_0_0100_0_0100_0110_000000000000; // STRGT  R6, [R4], #0
            6'd33: instruction = 32'b1100_01_0_0100_0_0100_0101_000000000100; // STRGT  R5, [R4], #4
            6'd34: instruction = 32'b1110_00_1_0100_0_0011_0011_000000000001; // ADD    R3, R3, #1
            6'd35: instruction = 32'b1110_00_1_1010_1_0011_0000_000000000011; // CMP    R3, #3
            6'd36: instruction = 32'b1011_10_1_0_111111111111111111110111;    // BLT    #-9
            6'd37: instruction = 32'b1110_00_1_0100_0_0010_0010_000000000001; // ADD    R2, R2, #1
            6'd38: instruction = 32'b1110_00_0_1010_1_0010_0000_000000000001; // CMP    R2, R1
            6'd39: instruction = 32'b1011_10_1_0_111111111111111111110011;    // BLT    #-13
            6'd40: instruction = 32'b1110_01_0_0100_1_0000_0001_000000000000; // LDR    R1, [R0], #0
            6'd41: instruction = 32'b1110_01_0_0100_1_0000_0010_000000000100; // LDR    R2, [R0], #4
            6'd42: instruction = 32'b1110_01_0_0100_1_0000_0011_000000001000; // LDR    R3, [R0], #8
            6'd43: instruction = 32'b1110_01_0_0100_1_0000_0100_000000001100; // LDR    R4, [R0], #12
            6'd44: instruction = 32'b1110_01_0_0100_1_0000_0101_000000010000; // LDR    R5, [R0], #16
            6'd45: instruction = 32'b1110_01_0_0100_1_0000_0110_000000010100; // LDR    R6, [R0], #20
            6'd46: instruction = 32'b1110_10_1_0_111111111111111111111111;    // B      #-1
            default: instruction = 32'b000000_00000_00000_00000_00000000000; // Default (NOP)
        endcase
    end

endmodule