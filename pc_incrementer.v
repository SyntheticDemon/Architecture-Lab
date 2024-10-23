<<<<<<< HEAD
module pc_incrementer (
    input [31:0] pc_in,
    output [31:0] pc_out
);

    assign pc_out = pc_in + 32'd4;

endmodule
=======
module pc_incrementer (
    input [31:0] pc_in,
    output [31:0] pc_out
);

    assign pc_out = pc_in + 32'd1; // TODO make the adder 4 4 4 4 next time

endmodule
>>>>>>> origin/main
