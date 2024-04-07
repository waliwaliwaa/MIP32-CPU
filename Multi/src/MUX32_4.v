`timescale 1ns/1ps
module MUX32_4 (
    input [1:0] Ctrl,
    input [31:0] i0,
    input [31:0] i1,
    input [31:0] i2,
    input [31:0] i3,
    output [31:0] out
);
    always @* begin
        case (Ctrl)
            2'b00: 
                out = i0;
            2'b01: 
                out = i1;
            2'b10: 
                out = i2;
            default: 
                out = i3;
        endcase
    end

endmodule