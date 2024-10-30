module Mux2To1 #(
    parameter WIDTH = 32
)(
    input [WIDTH-1:0] a0, a1,
    input sel,
    output [WIDTH-1:0] out
);
    assign out = sel ? a1 : a0;
endmodule