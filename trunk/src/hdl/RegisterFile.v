module RegisterFile (
    input clk, rst,
    input [3:0] src1, src2, Dest_wb,
    input [31:0] Result_WB,
    input writeBackEn,
    output [31:0] Val_Rn,Val_Rm 
);

    reg [31:0] register_array [15:0];
    assign Val_Rn = register_array[src1];
    assign Val_Rm = register_array[src2];
    integer i;
    always @(negedge clk) begin
        if (rst) begin
            for (i = 0; i < 16; i = i + 1) begin
                register_array[i] <= 32'b0;
            end
        end else if (writeBackEn) begin
            register_array[Dest_wb] <= Result_WB;
        end
    end

endmodule