`timescale 1ns/1ps
module Multi (
    input clk,
    input reset
);// page321
    wire [31:0] Inst, PC_in, PC_out;
    wire [31:0] RF_R_data1, RF_R_data2, RF_W_data;
    wire [31:0] DM_Addr, DM_R_data, DM_W_data;
    wire [31:0] SHL32_in, SHL32_out;
    wire [31:0] MUX32_21_i0, MUX32_21_i1, MUX32_21_out;
    wire [31:0] MUX32_22_i0, MUX32_22_i1, MUX32_22_out;
    wire [31:0] MUX32_23_i0, MUX32_23_i1, MUX32_23_out;
    wire [31:0] MUX32_41_i0, MUX32_41_i1, MUX32_41_i2, MUX32_41_i3, MUX32_41_out;
    wire [31:0] MUX32_42_i0, MUX32_42_i1, MUX32_42_i2, MUX32_42_i3, MUX32_42_out;
    wire [31:0] Sig32;
    wire [31:0] A_in, A_out, B_in, B_out, ALUOut_in, ALUOut_out, MDR_in, MDR_out;
    wire [31:0] ALU_a, ALU_b, ALU_c;
    wire [27:0] SHL26_out;
    wire [25:0] Addr26, SHL26_in;
    wire [15:0] Imm16, Sig16;
    wire [5:0] OP_Code, Func, ALUCU_funct;
    wire [4:0] RF_R_Reg1, RF_R_Reg2, RF_W_Reg;
    wire [4:0] Rs, Rt, Rd, Shamt;
    wire [4:0] MUX5_i0, MUX5_i1, MUX5_out;
    wire [3:0] S, NS;
    wire [2:0] ALUCtrl, ALU_mod;
    wire [1:0] ALUOp, PCSrc, ALUSrcB;
    wire [1:0] MUX32_41_c, MUX32_42_c;
    wire PCWr, PCWrCond, IorD, MemRd, MemWr, IRWr, MemtoReg, ALUSrcA, RegWr, RegDst;
    wire RF_W, DM_R, DM_W, ALU_z, ALU_o; 
    wire MUX32_21_ctrl, MUX32_22_ctrl, MUX32_23_ctrl, MUX5_ctrl, PC_W;

    assign ALU_a = MUX32_21_out;
    assign ALU_b = MUX32_42_out;
    assign ALU_mod = ALUCtrl;
    assign ALUCU_funct = Func;
    assign DM_Addr = MUX32_22_out;
    assign DM_W_data = B_out;
    assign DM_R  = MemRd;
    assign DM_W = MemWr;
    assign Inst = DM_R_data;
    assign MUX5_ctrl = RegDst;
    assign MUX5_i0 = Rt;
    assign MUX5_i1 = Rd;
    assign MUX32_21_ctrl = ALUSrcA;
    assign MUX32_21_i0 = PC_out;
    assign MUX32_21_i1 = A_out;
    assign MUX32_22_ctrl = IorD;
    assign MUX32_22_i0 = PC_out;
    assign MUX32_22_i1 = ALUOut_out;
    assign MUX32_23_ctrl = MemtoReg;
    assign MUX32_23_i0 = ALUOut_out;
    assign MUX32_23_i1 = MDR_out;
    assign MUX32_41_c = PCSrc;
    assign MUX32_41_i0 = ALU_c;
    assign MUX32_41_i1 = ALUOut_out;
    assign MUX32_41_i2 = {{PC_out[31:28]}, {SHL26_out}};
    assign MUX32_42_c = ALUSrcB;
    assign MUX32_42_i0 = B_out;
    assign MUX32_42_i1 = 4;
    assign MUX32_42_i2 = Sig32;
    assign MUX32_42_i3 = SHL32_out;
    assign PC_in = MUX32_41_out;
    assign PC_W = PCWr | (PCWrCond & ALU_z);
    assign RF_W = RegWr;
    assign RF_W_data = MUX32_23_out;
    assign RF_W_Reg = MUX5_out;
    assign RF_R_Reg1 = Rs;
    assign RF_R_Reg2 = Rt;
    assign SHL26_in = Addr26;
    assign SHL32_in = Sig32;
    assign Sig16 = Imm16;
    assign A_in = RF_R_data1;
    assign B_in = RF_R_data2;
    assign ALUOut_in = ALU_c;
    assign MDR_in = DM_R_data;

    ALU alu(ALU_a, ALU_b, ALU_c, ALU_mod, ALU_z, ALU_o);
    ALUCU alucu(ALUOp, ALUCU_funct, ALUCtrl);
    DM dm(.R_Enable(DM_R), .W_Enable(DM_W), .Addr(DM_Addr), .R_data(DM_R_data), .W_data(DM_W_data), .IorD(IorD));
    IR ir(.clk(clk), .W_Enable(IRWr), .Data(Inst), .OP_Code(OP_Code), .Rs(Rs), .Rt(Rt), .Rd(Rd), .Shamt(Shamt), .Func(Func), .Imm16(Imm16), .Addr26(Addr26));
    MCU mcu(.OP_Code(OP_Code), .S(S), .NS(NS), .PCSrc(PCSrc), .PCWr(PCWr), .PCWrCond(PCWrCond), .IorD(IorD), .IRWr(IRWr), .ALUOp(ALUOp), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .RegDst(RegDst), .RegWr(RegWr), .MemtoReg(MemtoReg), .MemRd(MemRd), .MemWr(MemWr));
    MUX5 mux5(MUX5_ctrl, MUX5_i0, MUX5_i1, MUX5_out);
    MUX32_2 mux32_21(MUX32_21_ctrl, MUX32_21_i0, MUX32_21_i1, MUX32_21_out);
    MUX32_2 mux32_22(MUX32_22_ctrl, MUX32_22_i0, MUX32_22_i1, MUX32_22_out);
    MUX32_2 mux32_23(MUX32_23_ctrl, MUX32_23_i0, MUX32_23_i1, MUX32_23_out);
    MUX32_4 mux32_41(MUX32_41_c, MUX32_41_i0, MUX32_41_i1, MUX32_41_i2, MUX32_41_i3, MUX32_41_out);
    MUX32_4 mux32_42(MUX32_42_c, MUX32_42_i0, MUX32_42_i1, MUX32_42_i2, MUX32_42_i3, MUX32_42_out);
    PC pc(clk, reset, PC_W, PC_in, PC_out);
    RF rf(.clk(clk), .W_Enable(RF_W), .W_Reg(RF_W_Reg), .W_data(RF_W_data), .R_Reg1(RF_R_Reg1), .R_Reg2(RF_R_Reg2), .R_data1(RF_R_data1), .R_data2(RF_R_data2));
    SHL2_26 shl2_26(SHL26_in, SHL26_out);
    SHL2_32 shl2_32(SHL32_in, SHL32_out); 
    SigExt sigext(Sig16, Sig32);
    SR sr(.clk(clk), .NS(NS), .S(S));
    TmpR regA(clk, A_in, A_out), regB(clk, B_in, B_out), ALUOut(clk, ALUOut_in, ALUOut_out);
    MDR mdr(clk, MDR_in, MDR_out);

endmodule