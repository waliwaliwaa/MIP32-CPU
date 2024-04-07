`timescale 1ns/1ps
module SigExt (
    input [15:0] in,
    output [31:0] out
);
    assign out = {{16{1'b0}}, {in}};
endmodule