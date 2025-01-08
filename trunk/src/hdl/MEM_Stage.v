module Mux2To1 #(
    parameter N = 32
)(
    input [N-1:0] a0, a1,
    input sel,
    output [N-1:0] out
);
    assign out = sel ? a1 : a0;
endmodule

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
    output mem_read_out, WB_en_out,
// c
    output freeze,
    inout [15:0] SRAM_DQ,
    output [17:0] SRAM_ADDR,
    output SRAM_UB_N,
    output SRAM_LB_N,
    output SRAM_WE_N,
    output SRAM_CE_N,
    output SRAM_OE_N
);

    assign dst_out = dst;
    assign mem_read_out = mem_read;
    // assign WB_en_out = WB_en;
    assign ALU_res_out = ALU_res;


    wire ready;
    assign freeze = ~ready;

    // wire [31:0] read_data;

    SRAMCTRL sram_controller(
        .clk(clk), .rst(rst),
        .wrEn(mem_write), .rdEn(mem_read),
        .address(ALU_res),
        .writeData(val_Rm),
        .readData(mem_out),
        .ready(ready),
        .SRAM_DQ(SRAM_DQ),
        .SRAM_ADDR(SRAM_ADDR),
        .SRAM_UB_N(SRAM_UB_N),
        .SRAM_LB_N(SRAM_LB_N),
        .SRAM_WE_N(SRAM_WE_N),
        .SRAM_CE_N(SRAM_CE_N),
        .SRAM_OE_N(SRAM_OE_N)
    );


    // Memory M1(
    // .clk(clk),
    // .rst(rst),
    // .alu_res(ALU_res),
    // .Val_Rm(val_Rm),
    // .mem_w_en(mem_write),
    // .mem_r_en(mem_read),
    // .res_data(mem_out)
    // );


  Mux2To1 #(1) write_back_enabled_mux(
        .a0(WB_en),
        .a1(1'b0),
        .sel(freeze),
        .out(WB_en_out)
    );
    
endmodule