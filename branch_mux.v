module branch_mux (
    input branch_taken,          // Signal indicating if a branch is taken
    input [31:0] branch_address, // Address to branch to if branch_taken is high
    input [31:0] next_pc,        // Next sequential PC address
    output reg [31:0] pc_out     // Selected PC address output
);

    // Multiplexer logic
    always @(*) begin
        if (branch_taken) begin
            pc_out = branch_address; // If branch is taken, select branch_address
        end else begin
            pc_out = next_pc;        // Otherwise, select next_pc
        end
    end

endmodule
