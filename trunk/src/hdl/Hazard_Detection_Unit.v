module HazardUnit(
    input [3:0] rn, rdm,
    input twoSrc,
    input [3:0] destEx, destMem,
    input wbEnEx, wbEnMem, //memREn,
    //input forwardEn,
    output reg hazard
);
   always @(*) begin
        hazard = 1'b0;
        
        // Check EX Stage
        if (wbEnEx) begin
            if ((rn == destEx) || (twoSrc && (rdm == destEx))) begin
                hazard = 1'b1;
            end
        end
        
        // Check MEM Stage
        if (wbEnMem) begin
            if ((rn == destMem) || (twoSrc && (rdm == destMem))) begin
                hazard = 1'b1;
            end
        end
    end
endmodule
