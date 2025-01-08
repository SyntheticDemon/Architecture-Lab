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
    input wr_en,
    input rd_en,
    input [31:0] address,
    input [31:0] writeData,
    output reg [31:0] readData,
    output sram_freeze,            // to freeze other stages

    inout [15:0] SRAM_DQ,        // SRAM Data bus 16 bits
    output reg [17:0] SRAM_ADDR, // SRAM Address bus 18 bits
    output SRAM_UB_N,            // SRAM High-byte data mask
    output SRAM_LB_N,            // SRAM Low-byte data mask
    output reg SRAM_WE_N,        // SRAM Write enable
    output SRAM_CE_N,            // SRAM Chip enable
    output SRAM_OE_N             // SRAM Output enable
);
    assign {SRAM_UB_N, SRAM_LB_N, SRAM_CE_N, SRAM_OE_N} = 4'b0000;

reg [3:0] ps, ns;
parameter IDLE = 4'd0, WriteDataLow = 4'd1, WriteDataHigh = 4'd2, W3 = 4'd3, W4 = 4'd4, W5 =4'd5,
   ReadDataLow = 4'd6, ReadDataHigh = 4'd7, R3 = 4'd8, R4 = 4'd9, R5 =4'd10;

reg ready;
wire [31:0] addressSub1024;
assign addressSub1024 = address - 1024;


assign sram_freeze = (ps == IDLE & (rd_en || wr_en)) ? 1'b1 : ~ready;

assign SRAM_DQ = ps == WriteDataLow ? writeData[15:0] :
                 ps == WriteDataHigh ? writeData[31:16] : 16'bz;



always @(*) begin
    ready = 1'b0;
    SRAM_WE_N = 1'b1;

    case (ps)
        IDLE: begin
            ready = 1;
        end 
        WriteDataLow: begin
            SRAM_WE_N = 0;
            SRAM_ADDR = addressSub1024[17:0]>>1;
        end
        WriteDataHigh: begin
            SRAM_WE_N = 0;
            SRAM_ADDR = (addressSub1024[17:0]>>1) + 1;
        end
        W3: begin
        end
        W4: begin
        end
        W5: begin
            ready = 1;
        end
        ReadDataLow: begin
            SRAM_ADDR = addressSub1024[17:0]>>1;
            readData[15:0] = SRAM_DQ;
        end
        ReadDataHigh: begin
            SRAM_ADDR = (addressSub1024[17:0]>>1) + 1;
            readData[31:16] = SRAM_DQ;
        end
        R3: begin

        end
        R4: begin

        end
        R5: begin
            ready = 1;
        end
        default: begin
        end
    endcase
end

always @(*) begin
    ns = 4'd0;
    case (ps)
        IDLE: begin
            ns = wr_en ? WriteDataLow : rd_en ? ReadDataLow : IDLE;
        end 
        WriteDataLow: begin
            ns = WriteDataHigh;
        end
        WriteDataHigh: begin
            ns = W3;
        end
        W3: begin
            ns = W4;
        end
        W4: begin
            ns = W5;
        end
        W5: begin
            ns = IDLE;
        end
        ReadDataLow: begin
            ns = ReadDataHigh;
        end
        ReadDataHigh: begin
            ns = R3;
        end
        R3: begin
            ns = R4;
        end
        R4: begin
            ns = R5;
        end
        R5: begin
            ns = IDLE;
        end
        default: 
            ns = IDLE;
    endcase

end


always @(posedge clk, posedge rst) begin
    if (rst)
        ps <= 4'd0;
    else
        ps <= ns;
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