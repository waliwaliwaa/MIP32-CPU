`timescale 1ns/1ps
module IM (
    input [31:0] Addr,
    output [5:0] OP_Code,
    output [4:0] Rs,
    output [4:0] Rt,
    output [4:0] Rd,
    output [4:0] Shamt,
    output [5:0] Func,
    output [15:0] Imm16,
    output [25:0] Addr26
);
    reg [31:0] mem [0:31];
    reg [31:0] Inst;
    initial begin
        $readmemb("D:/CPU/IM.txt",mem);
    end
    always @(*) begin
        Inst = mem[Addr];
    end
    assign OP_Code = Inst[31:26];
    assign Rs = Inst[25:21];
    assign Rt = Inst[20:16];
    assign Rd = Inst[15:11];
    assign Shamt = Inst[10:6];
    assign Func = Inst[5:0];
    assign Imm16 = Inst[15:0];
    assign Addr26 = Inst[25:0];
endmodule