
module Mux2to1 #(
    parameter DATA_WIDTH = 32  // Bit-width of each input
)(
    input [DATA_WIDTH-1:0] in0,  // First input
    input [DATA_WIDTH-1:0] in1,  // Second input
    input sel,                   // Select signal (1 bit)
    output reg [DATA_WIDTH-1:0] out // MUX output
);

    always @(*) begin
        out = (sel == 1'b0) ? in0 : in1;
    end

endmodule
