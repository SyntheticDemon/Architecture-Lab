module Mem_Stage
(
    input clk, rst,
    input [3:0] dst,
    input [31:0] ALU_res,
    input [31:0] val_Rm,
    input mem_read, mem_write, WB_en,

    output [3:0] dst_out,
    output [31:0] ALU_res_out,
    output [31:0] mem_out,
    output mem_read_out, WB_en_out
);

    assign dst_out = dst;
    assign mem_read_out = mem_read;
    assign WB_en_out = WB_en;
    assign ALU_res_out = ALU_res;

    Memory M1(
    .clk(clk),
    .rst(rst),
    .alu_res(ALU_res),
    .Val_Rm(val_Rm),
    .mem_w_en(mem_write),
    .mem_r_en(mem_read),
    .res_data(mem_out)
    );

endmodule