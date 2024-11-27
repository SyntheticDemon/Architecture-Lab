module Memory
(
    input                    clk,
    input                    rst,
    input  [31:0] alu_res,
    input  [31:0] Val_Rm,
    input                    mem_w_en,
    input                    mem_r_en,
    output [31:0] res_data
);

    wire [31:0] generatedAddr = {alu_res[31:2], 2'b00} - 32'd1024;

    reg [7:0] mem_data [0:2047];

	integer i;

	always @(posedge clk, posedge rst)
	begin
		if (rst)
			begin
				for (i=0; i < 2048; i = i+1)
					mem_data [i] <= i;
			end
		else if (mem_w_en) begin	
                mem_data[generatedAddr] <= Val_Rm[7:0];
                mem_data[{generatedAddr[31:1], 1'b1}] <= Val_Rm[15:8];
                mem_data[{generatedAddr[31:2], 2'b10}] <= Val_Rm[23:16];
                mem_data[{generatedAddr[31:2], 2'b11}] <= Val_Rm[31:24];
		end
	end
    assign res_data = mem_r_en ? {mem_data[{generatedAddr[31:2], 2'b11}], mem_data[{generatedAddr[31:2], 2'b10}], 
                                    mem_data[{generatedAddr[31:1], 1'b1}], mem_data[{generatedAddr}]}: 32'b0;

endmodule