module Adder #(
    parameter WIDTH = 32
)(
    input [WIDTH-1:0] a, b,
    output [WIDTH-1:0] out
);
    assign out = a + b;
endmodule