`timescale 1ns/1ps
module MCU (
    input [5:0] OP_Code,
    input [3:0] S,
    output PCWr,
    output PCWrCond,
    output IorD,
    output MemRd,
    output MemWr,
    output IRWr,
    output MemtoReg,
    output [1:0] PCSrc,
    output [1:0] ALUOp,
    output [1:0] ALUSrcB,
    output ALUSrcA,
    output RegWr,
    output RegDst,
    output [3:0] NS
);
    wire [16:0] Status;
    assign Status[16] = !S;
    assign Status[15] = !S[3:1] & S[0];
    assign Status[14] = !S[3:2] & S[1] & !S[0];
    assign Status[13] = !S[3:2] & !(~S[1:0]);
    assign Status[12] = !S[3] & !(~S[2]) & !S[1:0];
    assign Status[11] = !S[3] & S[2] & !S[1] & S[0];
    assign Status[10] = !S[3] & !(~S[2:1]) & !S[0];
    assign Status[9] = !S[3] & !(~S[2:0]);
    assign Status[8] = S[3] & !S[2:0];
    assign Status[7] = S[3] & !S[2:1] & S[0];
    assign Status[6] = !OP_Code[5:2] & OP_Code[1] &!OP_Code[0] & !S[3:1] & S[0];
    assign Status[5] = !OP_Code[5:3] & OP_Code[2] & !OP_Code[1:0] & !S[3:1] & S[0];
    assign Status[4] = !OP_Code & !S[3:1] & S[0];
    assign Status[3] = OP_Code[5] & !OP_Code[4:2] & !(~OP_Code[1:0]) & !S[3:2] & S[1] & !S[0];
    assign Status[2] = OP_Code[5] & !OP_Code[4] & OP_Code[3] & !OP_Code[2] & !(~OP_Code[1:0]) & !S[3:1] & S[0];
    assign Status[1] = OP_Code[5] & !OP_Code[4:2] & !(~OP_Code[1:0]) & !S[3:1] & S[0];
    assign Status[0] = OP_Code[5] & !OP_Code[4] & OP_Code[3] & !OP_Code[2] & !(~OP_Code[1:0]) & !S[3:2] & S[1] & !S[0];

    assign PCWr = Status[16] | Status[7];
    assign PCWrCond = Status[8];
    assign IorD = Status[13] | Status[11];
    assign MemRd = Status[16] | Status[13];
    assign MemWr = Status[11];
    assign IRWr = Status[16];
    assign MemtoReg = Status[12];
    assign PCSrc[1] = Status[7];
    assign PCSrc[0] = Status[8];
    assign ALUOp[1] = Status[10];
    assign ALUOp[0] = Status[8];
    assign ALUSrcB[1] = Status[15] | Status[14];
    assign ALUSrcB[0] = Status[16] | Status[15];
    assign ALUSrcA = Status[14] | Status[10] | Status[8];
    assign RegWr = Status[12] | Status[9];
    assign RegDst = Status[9];
    assign NS[3] = Status[6] | Status[5];
    assign NS[2] = Status[13] | Status[10] | Status[4] | Status[0];
    assign NS[1] = Status[10] | Status[4] | Status[3] | Status[2] | Status[1] ;
    assign NS[0] = Status[16] | Status[10] | Status[6] | Status[3] | Status[0];
    
endmodule