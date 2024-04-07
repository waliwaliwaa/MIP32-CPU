`timescale 1ns/1ps
module IM (
    input [31:0] Addr,
    output reg [31:0] Inst
);
    reg [31:0] mem [0:31];
    initial begin
        $readmemb("D:/CPU/IM.txt",mem);
    end
    always @(*) begin
        Inst = mem[Addr];
    end
endmodule