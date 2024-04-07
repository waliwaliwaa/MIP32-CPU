`timescale 1ns/1ps
module MCU (
    input [5:0] OP_Code,
    output reg [1:0] ALUOp,
    output reg ALUSrc,
    output reg RegDst,
    output reg RegWr,
    output reg MemtoReg,
    output reg MemRd,
    output reg MemWr,
    output reg Jump,
    output reg Branch
);
    always @(*) begin
        ALUOp = 2'b00;
        ALUSrc = 0;
        RegDst = 0;
        RegWr = 0;
        MemtoReg = 0;
        MemRd = 0;
        MemWr = 0;
        Jump = 0;
        Branch = 0;
        case(OP_Code)
            6'b000000: begin
                RegDst = 1;
                RegWr = 1;
                ALUOp = 2'b10;
            end
            6'b100011: begin
                RegWr = 1;
                ALUSrc = 1;
                MemRd = 1;
                MemtoReg = 1;
            end
            6'b101011: begin
                ALUSrc = 1;
                MemWr = 1;
            end 
            6'b000100: begin
                Branch = 1;
                ALUOp = 2'b01;
            end
            6'b000010: begin
                Jump = 1;
            end
        endcase
    end
    
endmodule