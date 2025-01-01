module Memory
(
    input                    clk,
    input                    rst,
    input  [31:0] alu_res,
    input  [31:0] Val_Rm,
    input                    mem_w_en,
    input                    mem_r_en,
    output reg [31:0] res_data
);

    wire [31:0] dataAdr;
    assign dataAdr = alu_res - 32'd1024;
    wire [31:0] generatedAddr;
    assign generatedAddr = {2'b00, dataAdr[31:2]}; // Align address to the word boundary

    reg [31:0] mem_data [0:63];

	integer i;

	always @(posedge clk, posedge rst)
	begin
        if (rst)
            for (i = 0; i < 64; i = i + 1) begin
                mem_data[i] <= 32'd0;
            end
        else if (mem_w_en) begin	
                mem_data[generatedAddr] <= Val_Rm;
		end
	end


    always @(mem_r_en, generatedAddr) begin
        if (mem_r_en)
            res_data = mem_data[generatedAddr];
    end

endmodule