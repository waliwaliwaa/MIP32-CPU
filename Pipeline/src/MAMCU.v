`timescale 1ns/1ps
module MAMCU (
    input [31:0] Inst,
    output reg MemRd,
    output reg MemWr,
    output reg Jump,
    output reg Branch
);
    reg [5:0] OP_Code;
    initial begin
        OP_Code = 0;
        MemRd = 0;
        MemWr = 0;
        Jump = 0;
        Branch = 0;
    end
    always @(*) begin
        OP_Code = Inst[31:26];
        MemRd = 0;
        MemWr = 0;
        Jump = 0;
        Branch = 0;
        case(OP_Code)
            6'b100011: begin
                MemRd = 1;
            end
            6'b101011: begin
                MemWr = 1;
            end 
            6'b000100: begin
                Branch = 1;
            end
            6'b000010: begin
                Jump = 1;
            end
        endcase
    end
    
endmodule