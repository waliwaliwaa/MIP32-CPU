`timescale 1ns/1ps
module WBMCU (
    input [31:0] Inst,
    output reg RegDst,
    output reg RegWr,
    output reg MemtoReg
);
    reg [5:0] OP_Code; 
    initial begin
        OP_Code = 0;
        RegWr = 0;
        MemtoReg = 0;
        RegDst = 0;
    end
    always @(*) begin
        OP_Code = Inst[31:26];
        RegWr = 0;
        MemtoReg = 0;
        RegDst = 0;
        case(OP_Code)
            6'b000000: begin
                RegDst = 1;
                RegWr = 1;
            end
            6'b100011: begin
                RegWr = 1;
                MemtoReg = 1;
            end
        endcase
    end
    
endmodule