`timescale 1ns/1ps
module IR (
    input clk,
    input W_Enable,
    input [31:0] Data,
    output reg [5:0] OP_Code,
    output reg [4:0] Rs,
    output reg [4:0] Rt,
    output reg [4:0] Rd,
    output reg [4:0] Shamt,
    output reg [5:0] Func,
    output reg [15:0] Imm16,
    output reg [25:0] Addr26
);
    reg [31:0] Inst;
    always @(*) begin
        if (W_Enable == 1)
            Inst <= Data;
    end
    always @(posedge clk) begin
        OP_Code <= Inst[31:26];
        Rs <= Inst[25:21];
        Rt <= Inst[20:16];
        Rd <= Inst[15:11];
        Shamt <= Inst[10:6];
        Func <= Inst[5:0];
        Imm16 <= Inst[15:0];
        Addr26 <= Inst[25:0];
    end
endmodule