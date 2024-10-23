module PC (
    input clk,                // Clock signal
    input reset,              // Reset signal
    input freeze,             // Freeze signal
    input [31:0] next_address, // Input address for the next value of the PC
    output reg [31:0] pc       // Current value of the PC
);

    // On every clock edge
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'b0; // Reset the PC to 0
        end else if (!freeze) begin
            pc <= next_address; // Update the PC with the new address if not frozen
        end
        // If freeze is active, retain the current value of pc
    end

endmodule
