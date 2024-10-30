module IF_Stage(
    input clk, rst,
    input branchTaken,          // Control signal for branch
    input freeze,               // Pipeline stall signal
    input [31:0] branchAddr,    // Target address for branch
    output [31:0] pc,           // Program counter output
    output [31:0] instruction   // Fetched instruction
);
    // Internal signals
    wire [31:0] pc_reg_in;      // Input to PC register
    wire [31:0] pc_reg_out;     // Output from PC register
    wire [31:0] pc_plus_4;      // PC + 4 for sequential execution
    
    // PC Register
    Register #(32) pc_reg(
        .clk(clk),
        .rst(rst),
        .in(pc_reg_in),
        .ld(~freeze),           // Load new PC when not frozen
        .clr(1'b0),             // Clear signal not used
        .out(pc_reg_out)
    );
    
    // PC + 4 Adder
    Adder #(32) pc_adder(
        .a(pc_reg_out),
        .b(32'd4),
        .out(pc_plus_4)
    );
    
    // Multiplexer for branch
    Mux2To1 #(32) pc_mux(
        .a0(pc_plus_4),
        .a1(branchAddr),
        .sel(branchTaken),
        .out(pc_reg_in)
    );
    
    // Instruction Memory
    InstructionMemory inst_mem(
        .pc(pc_reg_out),
        .inst(instruction)
    );
    
    // Output current PC+4
    assign pc = pc_plus_4;
endmodule