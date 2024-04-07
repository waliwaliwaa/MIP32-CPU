`timescale 1ns/1ps
module MUX32_2 (
    input Ctrl,
    input [31:0] i0,
    input [31:0] i1,
    output [31:0] out
);
    always @* begin
        if (Ctrl == 0)
            out = i0;
        else
            out = i1;
    end

endmodule