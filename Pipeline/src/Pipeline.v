`timescale 1ns/1ps
module Pipeline (
    input clk,
    input reset
);
    wire [31:0] Inst_Addr, Inst, PC_in, PC_out, FIIDout, IDEXout, EXMAout, MAWBout, NPC1, NPC2, NPC3;
    wire [31:0] RF_R_data1, RF_R_data2, RF_W_data;
    wire [31:0] DM_Addr, DM_R_data, DM_W_data;
    wire [31:0] Add1_a, Add1_b, Add1_c, Add2_a, Add2_b, Add2_c;
    wire [31:0] SHL32_in, SHL32_out;
    wire [31:0] MUX32_2_1_i0, MUX32_2_1_i1, MUX32_2_1_out;
    wire [31:0] MUX32_2_2_i0, MUX32_2_2_i1, MUX32_2_2_out;
    wire [31:0] MUX32_2_3_i0, MUX32_2_3_i1, MUX32_2_3_out;
    wire [31:0] MUX32_4_1_i0, MUX32_4_1_i1, MUX32_4_1_i2, MUX32_4_1_i3, MUX32_4_1_out;
    wire [31:0] Sig32;
    wire [31:0] ALU_a, ALU_b, ALU_c;
    wire [31:0] EXMA_Rtout, IDEX_Rtout;
    wire [27:0] SHL26_out;
    wire [25:0] Addr26, SHL26_in;
    wire [15:0] Imm16, Sig16;
    wire [5:0] OP_Code, Func, ALUCU_funct;
    wire [4:0] RF_R_Reg1, RF_R_Reg2, RF_W_Reg;
    wire [4:0] Rs, Rt, Rd, Shamt;
    wire [4:0] MUX5_i0, MUX5_i1, MUX5_out;
    wire [2:0] ALUCtrl, ALU_mod;
    wire [1:0]  MUX32_4_1_c, ALUOp;
    wire ALUSrc, RegDst, RegWr, MemtoReg, MemRd, MemWr, Jump, Branch, PCSrc;
    wire RF_W, DM_R, DM_W, ALU_z, ALU_o, ZF;
    wire MUX32_2_1_ctrl, MUX32_2_2_ctrl, MUX32_2_3_ctrl, MUX5_ctrl;

    assign Add1_a = 4;
    assign Add1_b = PC_out;
    assign Add2_b = SHL32_out;
    assign ALU_b = MUX32_2_2_out;
    assign ALU_mod = ALUCtrl;
    assign ALUCU_funct = Func;
    assign Func = IDEXout[5:0];
    assign DM_R  = MemRd;
    assign DM_W = MemWr;
    assign DM_W_data = EXMA_Rtout;
    assign Inst_Addr = PC_out;
    assign MUX5_ctrl = RegDst;
    assign MUX5_i0 = MAWBout[20:16];
    assign MUX5_i1 = MAWBout[15:11]; 
    assign MUX32_2_1_ctrl = MemtoReg;
    assign MUX32_2_2_i0 = IDEX_Rtout;
    assign MUX32_2_2_ctrl = ALUSrc;
    assign PCSrc = (ZF & Branch) | Jump;
    assign MUX32_2_3_ctrl = PCSrc;
    assign MUX32_2_3_i0 = Add1_c;
    assign MUX32_2_3_i1 = MUX32_4_1_out;
    assign MUX32_4_1_c = {ZF & Branch , Jump};
    assign PC_in = MUX32_2_3_out;
    assign RF_W = RegWr;
    assign RF_W_data = MUX32_2_1_out;
    assign RF_W_Reg = MUX5_out;
    assign RF_R_Reg1 = FIIDout[25:21];
    assign RF_R_Reg2 = FIIDout[20:16];
    assign SHL26_in = IDEXout[25:0];
    assign SHL32_in = MUX32_2_2_i1;
    assign Sig16 = FIIDout[15:0];
    assign NPC2 = Add2_c;
    assign NPC3 = {{Add2_a[31:28]}, {SHL26_out}};

    ADD32 add1(Add1_a, Add1_b, Add1_c), add2(Add2_a, Add2_b, Add2_c);
    ALU alu(ALU_a, ALU_b, ALU_c, ALU_mod, ALU_z, ALU_o);
    ALUCU alucu(ALUOp, ALUCU_funct, ALUCtrl);
    DM dm(.R_Enable(DM_R), .W_Enable(DM_W), .Addr(DM_Addr), .R_data(DM_R_data), .W_data(DM_W_data));
    IM im(Inst_Addr, Inst);
    EXMCU exmcu(.Inst(IDEXout), .ALUOp(ALUOp),  .ALUSrc(ALUSrc));
    MAMCU mamcu(.Inst(EXMAout), .MemRd(MemRd), .MemWr(MemWr), .Jump(Jump), .Branch(Branch));
    WBMCU wbmcu(.Inst(MAWBout), .RegWr(RegWr), .MemtoReg(MemtoReg), .RegDst(RegDst));
    MUX5 mux5(MUX5_ctrl, MUX5_i0, MUX5_i1, MUX5_out);
    MUX32_2 mux32_2_1(MUX32_2_1_ctrl, MUX32_2_1_i0, MUX32_2_1_i1, MUX32_2_1_out);
    MUX32_2 mux32_2_2(MUX32_2_2_ctrl, MUX32_2_2_i0, MUX32_2_2_i1, MUX32_2_2_out);
    MUX32_2 mux32_2_3(MUX32_2_3_ctrl, MUX32_2_3_i0, MUX32_2_3_i1, MUX32_2_3_out);
    MUX32_4 mux32_4_1(MUX32_4_1_c, MUX32_4_1_i0, MUX32_4_1_i1, MUX32_4_1_i2, MUX32_4_1_i3, MUX32_4_1_out);
    PC pc(clk, reset, PC_in, PC_out);
    RF rf(.clk(clk), .W_Enable(RF_W), .W_Reg(RF_W_Reg), .W_data(RF_W_data), .R_Reg1(RF_R_Reg1), .R_Reg2(RF_R_Reg2), .R_data1(RF_R_data1), .R_data2(RF_R_data2));
    SHL2_26 shl2_26(SHL26_in, SHL26_out);
    SHL2_32 shl2_32(SHL32_in, SHL32_out); 
    SigExt sigext(Sig16, Sig32);
    FIID fiid(.clk(clk), .NPC1in(Add1_c), .NPC1out(NPC1), .IRin(Inst), .IRout(FIIDout), .OP_Code(OP_Code), .Rs(Rs), .Rt(Rt), .Imm16(Imm16));
    IDEX idex(.clk(clk), .NPC1in(NPC1), .NPC1out(Add2_a), .Rsin(RF_R_data1), .Rtin(RF_R_data2), .Imm32in(Sig32), .IRin(FIIDout), .Rsout(ALU_a), .Rtout(IDEX_Rtout), .Imm32out(MUX32_2_2_i1), .IRout(IDEXout));
    EXMA exma(.clk(clk), .NPC3in(NPC3), .NPC2in(NPC2), .Rtin(IDEX_Rtout), .ALUOutin(ALU_c), .IRin(IDEXout), .NPC2out(MUX32_4_1_i2), .NPC3out(MUX32_4_1_i1), .ALUOutout(DM_Addr), .Rtout(EXMA_Rtout), .IRout(EXMAout), .Flagin(ALU_z), .Flagout(ZF));
    MAWB mawb(.clk(clk), .MEMOutin(DM_R_data), .ALUOutin(DM_Addr), .IRin(EXMAout), .MEMOutout(MUX32_2_1_i1), .ALUOutout(MUX32_2_1_i0), .IRout(MAWBout));

endmodule