`timescale 1ns/1ps
module EXMCU (
    input [31:0] Inst,
    output reg [1:0] ALUOp,
    output reg ALUSrc
);
    reg [5:0] OP_Code;
    initial begin
        OP_Code = 0;
        ALUOp = 0;
        ALUSrc = 0;
    end
    always @(*) begin
        OP_Code = Inst[31:26];
        ALUOp = 2'b00;
        ALUSrc = 0;
        case(OP_Code)
            6'b000000: begin
                ALUOp = 2'b10;
            end
            6'b100011: begin
                ALUSrc = 1;
            end
            6'b101011: begin
                ALUSrc = 1;
            end 
            6'b000100: begin
                ALUOp = 2'b01;
            end
        endcase
    end
    
endmodule