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

    wire [31:0] dataAdr;
    assign dataAdr = alu_res - 32'd1024;
    assign generatedAddr = {2'b00, alu_res[31:2]}; // Align address to the word boundary

    reg [31:0] mem_data [0:2047];

	integer i;

	always @(posedge clk, posedge rst)
	begin
        if (rst)
            for (i = 0; i < 2048; i = i + 1) begin
                mem_data[i] <= 32'd0;
            end
        else if (mem_w_en) begin	
                mem_data[generatedAddr] <= Val_Rm;
		end
	end
    assign res_data = mem_r_en ? mem_data[generatedAddr] : 32'b0;

endmodule