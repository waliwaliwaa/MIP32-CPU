`timescale 1ns/1ps
module PC (
    input clk,
    input reset,
    input [31:0] addrin,
    output reg [31:0] addrout
);
    initial begin
        addrout = 0;
    end
    always @(posedge clk) begin
        if(reset == 1) 
            addrout = 0;
        else
            addrout = addrin;
    end
endmodule