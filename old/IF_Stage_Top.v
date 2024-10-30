module IF_Stage_Top(
    input clk, rst,
    input branchTaken,
    input freeze,
    input flush,
    input [31:0] branchAddr,
    output [31:0] if_id_pc,
    output [31:0] if_id_instruction
);
    // Internal signals
    wire [31:0] if_pc;
    wire [31:0] if_instruction;
    
    // Instantiate IF stage
    IF_Stage if_stage(
        .clk(clk),
        .rst(rst),
        .branchTaken(branchTaken),
        .freeze(freeze),
        .branchAddr(branchAddr),
        .pc(if_pc),
        .instruction(if_instruction)
    );
    
    // Instantiate IF/ID register
    IF_ID_Reg if_id_reg(
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .flush(flush),
        .pc_in(if_pc),
        .inst_in(if_instruction),
        .pc_out(if_id_pc),
        .inst_out(if_id_instruction)
    );
endmodule