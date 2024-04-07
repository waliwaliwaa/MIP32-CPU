`timescale 1ns/1ps
module MUX5 (
    input Ctrl,
    input [4:0] i0,
    input [4:0] i1,
    output [4:0] out
);
    always @* begin
        if (Ctrl == 0)
            out = i0;
        else
            out = i1;
    end

endmodule
