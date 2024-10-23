`timescale 1ns / 1ps

module IF_Stage_Testbench;

    // Inputs for IF_Stage and IF_Stage_Reg
    reg clk;
    reg rst;
    reg freeze;
    reg flush;
    reg Branch_taken;
    reg [31:0] BranchAddr;
    reg [31:0] PC_in;
    reg [31:0] Instruction_in;

    // Outputs
    wire [31:0] PC;
    wire [31:0] Instruction;
    wire [31:0] PC_Reg;
    wire [31:0] Instruction_Reg;

    // Instantiate the IF_Stage module
    IF_Stage if_stage_inst (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .Branch_taken(Branch_taken),
        .BranchAddr(BranchAddr),
        .PC(PC),
        .Instruction(Instruction)
    );

    // Instantiate the IF_Stage_Reg module
    IF_Stage_Reg if_stage_reg_inst (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .flush(flush),
        .PC_in(PC),
        .Instruction_in(Instruction),
        .PC(PC_Reg),
        .Instruction(Instruction_Reg)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Test sequence
    initial begin
        // Initialize inputs
        rst = 0;
        freeze = 0;
        flush = 0;
        Branch_taken = 0;
        BranchAddr = 32'h00000000;
        PC_in = 32'h00000000;
        Instruction_in = 32'h00000000;

        // Apply reset
        rst = 1; #10;
        rst = 0; #10;

        // Test 1: Normal operation (no branch, no freeze)
        BranchAddr = 32'h00000010; // Arbitrary branch address
        #10; // Wait for a few clock cycles
        $display("Normal Operation: PC = %h, Instruction = %h", PC_Reg, Instruction_Reg);

        // Test 2: Freeze active
        freeze = 1;
        #20; // Wait while freeze is active
        freeze = 0;
        #10;
        $display("After Freeze: PC = %h, Instruction = %h", PC_Reg, Instruction_Reg);

        // Test 3: Branch taken
        Branch_taken = 1;
        BranchAddr = 32'h00000020;
        #10;
        Branch_taken = 0;
        #10;
        $display("After Branch Taken: PC = %h, Instruction = %h", PC_Reg, Instruction_Reg);

        // Test 4: Flush active
        flush = 1;
        #10;
        flush = 0;
        #10;
        $display("After Flush: PC = %h, Instruction = %h", PC_Reg, Instruction_Reg);

        // Test 5: Reset again
        rst = 1; #10;
        rst = 0; #10;
        $display("After Reset: PC = %h, Instruction = %h", PC_Reg, Instruction_Reg);

        // End the simulation
        $finish;
    end

endmodule
