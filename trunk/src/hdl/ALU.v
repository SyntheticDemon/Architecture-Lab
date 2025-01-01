`define EX_MOV 4'b0001
`define EX_MVN 4'b1001
`define EX_ADD 4'b0010
`define EX_ADC 4'b0011
`define EX_SUB 4'b0100
`define EX_SBC 4'b0101
`define EX_AND 4'b0110
`define EX_ORR 4'b0111
`define EX_EOR 4'b1000
`define EX_CMP 4'b1100
`define EX_TST 4'b1110
`define EX_LDR 4'b1010
`define EX_STR 4'b1010

module ALU
(
    input       [31:0]   val1, val2,
    input                           carry,
    input       [3:0]               EX_command,
    output  [31:0]   res,
    output  [3:0]               SR
);

    reg V1, C1;
    wire N1, Z1;
    reg [32:0] temp_res;

    assign res = temp_res[31:0];
    assign SR = {Z1, C1, N1, V1};

    assign N1 = res[31];
    assign Z1 = |res ? 0:1;

    always @(*) begin
        V1 = 1'b0;
        C1 = 1'b0;
        case (EX_command)
            `EX_MOV: begin
                temp_res = val2;
            end
            `EX_MVN: begin
                temp_res = ~val2;
            end
            `EX_ADD: begin
                temp_res = val1 + val2;
                C1 = temp_res[32];
                V1 = (val1[31] ~^ val2[31]) & (temp_res[31] ^ val1[31]);
            end
            `EX_ADC: begin
                temp_res = val1 + val2 + carry;
                C1 = temp_res[32];
                V1 = (val1[31] ~^ val2[31]) & (temp_res[31] ^ val1[31]);
            end
            `EX_SUB: begin
                temp_res = {val1[31], val1} - {val2[31], val2};
                C1 = temp_res[32];
                V1 = (val1[31] ^ val2[31]) & (temp_res[31] ^ val1[31]);

            end
            `EX_SBC: begin
                temp_res = val1 - val2 - {31'b0,~carry}; 
                C1 = temp_res[32];
                V1 = (val1[31] ^ val2[31]) & (temp_res[31] ^ val1[31]);
            end
            `EX_AND: begin
                temp_res = val1 & val2;
            end
            `EX_ORR: begin
                temp_res = val1 | val2;
            end
            `EX_EOR: begin
                temp_res = val1 ^ val2;
            end
            `EX_CMP: begin
                temp_res = {val1[31], val1} - {val2[31], val2};
                C1 = temp_res[32];
                V1 = (val1[31] ^ val2[31]) & (temp_res[31] ^ val1[31]);
            end
            `EX_TST: begin
                temp_res = val1 & val2;
            end
            `EX_LDR: begin
                temp_res = val1 + val2;
            end
            `EX_STR: begin
                temp_res = val1 - val2;
            end
            default:
                temp_res = 32'b0;

        endcase
    end

endmodule