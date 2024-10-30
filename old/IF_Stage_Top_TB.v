`timescale 1ns/1ps

module IF_Stage_Top_TB();
    // Test signals
    reg clk, rst;
    reg branchTaken, freeze, flush;
    reg [31:0] branchAddr;
    wire [31:0] if_id_pc, if_id_instruction;
    
    // Instantiate the top module
    IF_Stage_Top dut(
        .clk(clk),
        .rst(rst),
        .branchTaken(branchTaken),
        .freeze(freeze),
        .flush(flush),
        .branchAddr(branchAddr),
        .if_id_pc(if_id_pc),
        .if_id_instruction(if_id_instruction)
    );
    
    // Clock generation
    always begin
        #5 clk = ~clk;
    end
    
    // Test stimulus
    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        branchTaken = 0;
        freeze = 0;
        flush = 0;
        branchAddr = 32'h0;
        
        // Reset sequence
        #20 rst = 0;
        
        // Test normal instruction fetch
        #20;  // Let it fetch a few instructions
        
        // Test freeze
        freeze = 1;
        #20;
        freeze = 0;
        
        // Test branch
        #20;
        branchTaken = 1;
        branchAddr = 32'h00000008;  // Branch to instruction at offset 8
        #10;
        branchTaken = 0;
        
        // Test flush
        #20;
        flush = 1;
        #10;
        flush = 0;
        
        // Continue normal execution
        #50;
        
        // End simulation
        #20 $finish;
    end
    
    // Monitor changes
    initial begin
        $monitor("Time=%0t rst=%b freeze=%b flush=%b branch=%b pc=%h inst=%h",
                 $time, rst, freeze, flush, branchTaken, if_id_pc, if_id_instruction);
    end
endmodule