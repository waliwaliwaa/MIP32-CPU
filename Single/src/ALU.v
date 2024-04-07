`timescale 1ns/1ps
module ALU (
    input [31:0] a,
    input [31:0] b,
    output reg [31:0] out,
    input [2:0] mod,
    output reg zero,
    output reg overflow
);
    always @(*) begin
        out = 0;
        case(mod)
            3'b000: 
                out = a & b;
            3'b001: 
                out = a | b;
            3'b100: begin
                out = a + b;
                overflow = (a[31] == b[31] && out[31] != a[31]) ? 1 : 0;
            end
            3'b101:
                out = a + b;
            3'b110: begin
                out = a - b;
                overflow = (a[31] != b[31] && out[31] == b[31]) ? 1 : 0;
            end
            3'b011:
                out = ~ (a | b);
            3'b010:
                out = (a < b) ? 1 : 0;
        endcase
        zero = (out == 0) ? 1 : 0;
    end
endmodule