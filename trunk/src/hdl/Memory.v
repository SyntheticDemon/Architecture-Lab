module SRAM(
    input clk, rst,
    input SRAM_WE_N,
    input [17:0] SRAM_ADDR,
    inout [15:0] SRAM_DQ
);
    reg [15:0] memory [0:511];
    assign SRAM_DQ = (SRAM_WE_N == 1'b1) ? memory[SRAM_ADDR] : 16'dz;

    always @(posedge clk) begin
        if (SRAM_WE_N == 1'b0) begin
            memory[SRAM_ADDR] = SRAM_DQ;
        end
    end
endmodule


module SRAMCTRL(
    input clk, rst,
    input wrEn, rdEn,
    input [31:0] address,
    input [31:0] writeData,
    output reg [31:0] readData,
    output reg ready,            // to freeze other stages

    inout [15:0] SRAM_DQ,        // SRAM Data bus 16 bits
    output reg [17:0] SRAM_ADDR, // SRAM Address bus 18 bits
    output SRAM_UB_N,            // SRAM High-byte data mask
    output SRAM_LB_N,            // SRAM Low-byte data mask
    output reg SRAM_WE_N,        // SRAM Write enable
    output SRAM_CE_N,            // SRAM Chip enable
    output SRAM_OE_N             // SRAM Output enable
);
    assign {SRAM_UB_N, SRAM_LB_N, SRAM_CE_N, SRAM_OE_N} = 4'b0000;

    wire [31:0] memAddr;
    assign memAddr = address - 32'd1024;

    wire [17:0] sramLowAddr, sramHighAddr, sramUpLowAddess;
    assign sramLowAddr = {memAddr[18:3], 2'd0};
    assign sramHighAddr = sramLowAddr + 18'd1;
    assign sramUpLowAddess = sramLowAddr + 18'd2;

    wire [17:0] sramLowAddrWrite, sramHighAddrWrite;
    assign sramLowAddrWrite = {memAddr[18:2], 1'b0};
    assign sramHighAddrWrite = sramLowAddrWrite + 18'd1;

    reg [15:0] dq;
    assign SRAM_DQ = wrEn ? dq : 16'bz;

    localparam Idle = 3'd0, DataLow = 3'd1, DataHigh = 3'd2,FirstIdle = 3'd3, SecondIdle = 3'd4, Done = 3'd5;
    reg [2:0] ps, ns;

    always @(ps or wrEn or rdEn) begin
        case (ps)
            Idle: ns = (wrEn == 1'b1 || rdEn == 1'b1) ? DataLow : Idle;
            DataLow: ns = DataHigh;
            DataHigh: ns = Done;
            // FirstIdle: ns = SecondIdle;
            // SecondIdle: ns = Done;
            Done: ns = Idle;
        endcase
    end

    always @(*) begin
        SRAM_ADDR = 18'b0;
        SRAM_WE_N = 1'b1;
        ready = 1'b0;

        case (ps)
            Idle: ready = ~(wrEn | rdEn);
            DataLow: begin
                SRAM_WE_N = ~wrEn;
                if (rdEn) begin
                    SRAM_ADDR = sramLowAddr;
                    readData[15:0] <= SRAM_DQ;
                end
                else if (wrEn) begin
                    SRAM_ADDR = sramLowAddrWrite;
                    dq = writeData[15:0];
                end
            end
            DataHigh: begin
                SRAM_WE_N = ~wrEn;
                if (rdEn) begin
                    SRAM_ADDR = sramHighAddr;
                    readData[31:16] <= SRAM_DQ;
                end
                else if (wrEn) begin
                    SRAM_ADDR = sramHighAddrWrite;
                    dq = writeData[31:16];
                end
            end
            FirstIdle: begin
              
            end
            SecondIdle: begin
              
            end
            Done: ready = 1'b1;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) ps <= Idle;
        else ps <= ns;
    end
endmodule

// module Memory
// (
//     input                    clk,
//     input                    rst,
//     input  [31:0] alu_res,
//     input  [31:0] Val_Rm,
//     input                    mem_w_en,
//     input                    mem_r_en,
//     output reg [31:0] res_data
// );

//     wire [31:0] dataAdr;
//     wire [31:0] generatedAddr;
    
//     assign dataAdr = alu_res - 32'd1024;
//     assign generatedAddr = {2'b00, dataAdr[31:2]}; // Align address to the word boundary

//     reg [31:0] mem_data [0:63];

//   integer i;

//   always @(posedge clk, posedge rst)
//   begin
//         if (rst)
//             for (i = 0; i < 64; i = i + 1) begin
//                 mem_data[i] <= 32'd0;
//             end
//         else if (mem_w_en) begin  
//                 mem_data[generatedAddr] <= Val_Rm;
//     end
//   end


//     always @(mem_r_en, generatedAddr) begin
//         if (mem_r_en)
//             res_data = mem_data[generatedAddr];
//         else 
//             res_data = 32'b0;
//     end

// endmodule