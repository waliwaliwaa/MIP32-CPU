`timescale 1ns/1ps
module PC (
    input clk,
    input reset,
    input W,
    input [31:0] addrin,
    output reg [31:0] addrout
);
    reg [31:0] TmpAddr;
    initial begin
        addrout = 0;
        TmpAddr = 4;
    end
    always @(posedge clk) begin
        if(reset == 0)
            addrout <= TmpAddr;
    end
    // neg fetch
    always @(negedge clk ) begin
        if(W == 1)
            TmpAddr <= addrin;
    end
endmodule