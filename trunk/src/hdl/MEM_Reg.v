module MEM_Reg
(
    input                       clk,
    input                       rst,
    input [3:0] dst,
    input [31:0] ALU_res,
    input [31:0] mem,
    input mem_read, WB_en,

    output reg [3:0] dst_out,
    output reg [31:0] ALU_res_out,
    output reg [31:0] mem_out,
    output reg mem_read_out, WB_en_out
);

always @(posedge clk, posedge rst) begin
    if(rst) begin
        dst_out <= 0;
        ALU_res_out <= 0;
        mem_out <= 0;
        mem_read_out <= 0;
        WB_en_out <= 0;
    end
    else begin
        dst_out <= dst;
        ALU_res_out <= ALU_res;
        mem_out <= mem;
        mem_read_out <= mem_read;
        WB_en_out <= WB_en;
    end
end

endmodule
