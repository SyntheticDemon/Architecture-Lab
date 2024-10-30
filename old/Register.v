module Register #(
    parameter WIDTH = 32
)(
    input clk, rst,
    input [WIDTH-1:0] in,
    input ld,                   // Load enable
    input clr,                  // Clear signal
    output reg [WIDTH-1:0] out
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            out <= {WIDTH{1'b0}};
        else if (clr)
            out <= {WIDTH{1'b0}};
        else if (ld)
            out <= in;
    end
endmodule