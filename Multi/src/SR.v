`timescale 1ns/1ps
module SR (
    input clk,
    input [3:0] NS,
    output reg [3:0] S
);
    initial begin
        S = 0;
        #10;
    end
    always @(posedge clk ) begin
        S <= NS;
    end
    
endmodule