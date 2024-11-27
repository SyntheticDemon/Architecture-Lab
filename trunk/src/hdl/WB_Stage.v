module WB_Stage
(
    input                    clk,
    input                    rst,
    input [3:0] dst,
    input [31:0] ALU_res,
    input [31:0] mem,
    input mem_read, WB_en,

    output [3:0] WB_Dest,
    output WB_en_out,
    output [31:0] WB_Value
);

assign WB_Dest = dst;
assign WB_en_out = WB_en;

Mux2to1 #(.DATA_WIDTH(32)) MUX_2_to_1_Reg_File (
		.in0(ALU_res), .in1(mem),
		.sel(mem_read),
		.out(WB_Value)
	);

endmodule



