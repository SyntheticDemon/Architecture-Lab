module ID_Stage (
    input clk,               // Clock signal
    input rst,               // Reset signal
    // from IF Reg
    input [31:0] Instruction, // Instruction from IF stage
    // from WB stage
    input [31:0] Result_WB,   // Result from WB stage
    input writeBackEn,        // Write enable signal
    input [3:0] Dest_wb,      // Destination register
    // from hazard detect module
    input hazard,             // Hazard detection signal
    // from Status Register
    input [3:0] SR,          // Status register
    // to next stage
    output WB_EN,        // Write back enable
    output MEM_R_EN,     // Memory read enable
    output MEM_W_EN,     // Memory write enable
    output [3:0] EXE_CMD, // Execution command
    output [31:0] Val_Rn, // Value of Rn
    output [31:0] Val_Rm, // Value of Rm
    output [31:0] imm,    // Immediate value
    output [11:0] Shift_operand, // Shift operand
    output [23:0] Signed_imm_24, // Signed immediate value
    output [3:0] Dest,    // Destination register
    // to hazard detect module
    output [3:0] src1,    // Source 1
    output [3:0] src2,    // Source 2
    output Two_src        // Indicator for two source operands
);


    wire [3:0] _src1;
    wire [3:0] _src2;
    wire [3:0] condition;
    assign condition = Instruction[31:28];
    wire cond_result;
    ConditionCheck condition_check(.result(cond_result),.condition(condition),.status(SR));
    Mux2to1 #(.DATA_WIDTH(4)) write_enabled_selector(
        .in0(
            Instruction[15:12]

        ),
        .in1(
            Instruction[3:0]
        ),
        .out(_src2),
        .sel(MEM_W_EN)
    );
    
    RegisterFile register_file(
     .clk(clk), .rst(rst),
    .src1(), .src2(_src2), .Dest_wb(Dest_wb),
     .Result_WB(Result_WB),
    .writeBackEn(writeBackEn),
    .Val_Rn(Val_Rn), .Val_Rm(Val_Rm)
);


endmodule
