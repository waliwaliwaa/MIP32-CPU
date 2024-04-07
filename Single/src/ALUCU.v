`timescale 1ns/1ps
module ALUCU (
    input [1:0] ALUOp,
    input [5:0] funct,
    output reg [2:0] ALUCtrl
);
    always @(*) begin
        ALUCtrl = 3'b000;
        case(ALUOp)
            2'b00:
                ALUCtrl = 3'b100;
            2'b01:
                ALUCtrl = 3'b110;
            default:
                case(funct)
                    6'b100000:
                        ALUCtrl = 3'b100;
                    6'b100001:
                        ALUCtrl = 3'b101;
                    6'b100010:
                        ALUCtrl = 3'b110;
                    6'b100100:
                        ALUCtrl = 3'b000;
                    6'b100101:
                        ALUCtrl = 3'b001;
                    6'b100111:
                        ALUCtrl = 3'b011;
                    6'b101010:
                        ALUCtrl = 3'b010;
                endcase
        endcase
    end
endmodule