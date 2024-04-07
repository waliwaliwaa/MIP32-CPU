`timescale 1ns/1ps
module TmpR (
    input clk,
    input [31:0] Data,
    output reg[31:0] Output
);
    always @(posedge clk ) begin
        Output <= Data;
    end
    
endmodule