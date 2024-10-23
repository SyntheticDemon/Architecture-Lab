module IF_Stage (
    input clk,                     // Clock signal
    input rst,                     // Reset signal
    input freeze,                  // Freeze signal
    input Branch_taken,            // Branch taken signal
    input [31:0] BranchAddr,       // Branch address input
    output [31:0] PC,              // Program counter output
    output [31:0] Instruction      // Fetched instruction output
);

    // Internal signals
    wire [31:0] next_address;       // Next sequential address (PC + 4)
    wire [31:0] selected_address;   // Address selected by branch_mux

    // Instantiate the program counter
    program_counter pc_module (
        .clk(clk),
        .reset(rst),
        .freeze(freeze),
        .next_address(selected_address), // Use the selected address from the branch_mux
        .pc(PC)
    );

    // Instantiate the PC incrementer to add 4 to the current PC
    pc_incrementer pc_inc (
        .pc_in(PC),
        .pc_out(next_address)
    );

    // Instantiate the branch multiplexer
    branch_mux branch_selector (
        .branch_taken(Branch_taken),
        .branch_address(BranchAddr),
        .next_pc(next_address),
        .pc_out(selected_address)
    );

    // Instantiate the instruction memory
    instruction_memory im (
        .address(PC),
        .instruction(Instruction)
    );

endmodule
