
`timescale 1ns / 1ps

module instruction_memory_tb;

    // Inputs
    reg [31:0] address;

    // Outputs
    wire [31:0] instruction;

    // Instantiate the instruction_memory module
    instruction_memory uut (
        .address(address),
        .instruction(instruction)
    );

    // Test sequence
    initial begin
        // Display header
        $display("Time\tAddress\t\t\tInstruction");
        $display("------------------------------------------------");

        // Test each address from 0 to 5
        address = 32'b00000000000000000000000000000000; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        address = 32'b00000000000000000000000000000001; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        address = 32'b00000000000000000000000000000010; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        address = 32'b00000000000000000000000000000011; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        address = 32'b00000000000000000000000000000100; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        address = 32'b00000000000000000000000000000101; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        // Test an address outside the defined range
        address = 32'b00000000000000000000000000000110; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        // Test a larger address to check if only the lower 3 bits matter
        address = 32'b00000000000000000000000000001000; #10;
        $display("%0dns\t%h\t%h", $time, address, instruction);

        // End the simulation
        $finish;
    end

endmodule
