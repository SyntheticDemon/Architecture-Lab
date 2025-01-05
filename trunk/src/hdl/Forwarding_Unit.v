module Forwarding_Unit (
    input enable,
    input  [3:0] src1,
    input  [3:0] src2,
    input  [3:0] MEM_dest,
    input  [3:0] WB_dest,
    input        MEM_WB_en,
    input        WB_WB_en,
    output reg [1:0] sel_src1,
    output reg [1:0] sel_src2
);

  always @(*) begin
    sel_src1 = 2'b00;
    sel_src2 = 2'b00;
    if(enable) begin
        if ((src1 == MEM_dest) && (MEM_WB_en == 1'b1))
            sel_src1 = 2'b01;
        else if ((src1 == WB_dest) && (WB_WB_en == 1'b1))
            sel_src1 =  2'b10;
        if ((src2 == MEM_dest) && (MEM_WB_en == 1'b1))
            sel_src2 = 2'b01;
        else if ((src2 == WB_dest) && (WB_WB_en == 1'b1))
            sel_src2 =  2'b10;
    end
  end

endmodule