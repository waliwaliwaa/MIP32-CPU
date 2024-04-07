`timescale 1ns/1ps
module Single (
    input clk,
    input reset
);
    
    wire [31:0] RF_R_data1, RF_R_data2, RF_W_data;
    wire [31:0] Add1_a, Add1_b, Add1_c, Add2_a, Add2_b, Add2_c;   
    wire [31:0] ALU_a, ALU_b, ALU_c;    
    wire [31:0] SHL32_in, SHL32_out;
    wire [31:0] MUX32_1_i0, MUX32_1_i1, MUX32_1_out;
    wire [31:0] MUX32_2_i0, MUX32_2_i1, MUX32_2_out;
    wire [31:0] MUX32_3_i0, MUX32_3_i1, MUX32_3_out;
    wire [31:0] MUX32_4_i0, MUX32_4_i1, MUX32_4_out;
    wire [31:0] Sig32;
    wire [31:0] DM_Addr, DM_R_data, DM_W_data; 
    wire [31:0] Inst_Addr, PC_in, PC_out;
    wire [27:0] SHL26_out;
    wire [25:0] Addr26, SHL26_in;
    wire [15:0] Imm16, Sig16;
    wire [5:0] OP_Code, Func, ALUCU_funct;
    wire [4:0] RF_R_Reg1, RF_R_Reg2, RF_W_Reg;
    wire [4:0] Rs, Rt, Rd, Shamt;
    wire [4:0] MUX5_i0, MUX5_i1, MUX5_out;
    wire [2:0] ALUCtrl, ALU_mod;
    wire [1:0] ALUOp;
    wire ALUSrc, RegDst, RegWr, MemtoReg, MemRd, MemWr, Jump, Branch;
    wire RF_W, DM_R, DM_W, ALU_z, ALU_o; 
    wire MUX32_1_ctrl, MUX32_2_ctrl, MUX32_3_ctrl, MUX32_4_ctrl, MUX5_ctrl;

    assign Add1_a = 4;
    assign Add1_b = PC_out;
    assign Add2_a = Add1_c;
    assign Add2_b = SHL32_out;
    assign ALU_a = RF_R_data1;
    assign ALU_b = MUX32_3_out;
    assign ALU_mod = ALUCtrl;
    assign ALUCU_funct = Func;
    assign DM_Addr = ALU_c;
    assign DM_W_data = RF_R_data2;
    assign DM_R  = MemRd;
    assign DM_W = MemWr;
    assign Inst_Addr = PC_out;
    assign MUX5_ctrl = RegDst;
    assign MUX5_i0 = Rt;
    assign MUX5_i1 = Rd;
    assign MUX32_1_ctrl = Jump;
    assign MUX32_1_i0 = MUX32_2_out;
    assign MUX32_1_i1 = {{Add1_c[31:28]}, {SHL26_out}};
    assign MUX32_2_ctrl = Branch & ALU_z;
    assign MUX32_2_i0 = Add1_c;
    assign MUX32_2_i1 = Add2_c;
    assign MUX32_3_ctrl = ALUSrc;
    assign MUX32_3_i0 = RF_R_data2;
    assign MUX32_3_i1 = Sig32;
    assign MUX32_4_ctrl = MemtoReg;
    assign MUX32_4_i0 = ALU_c;
    assign MUX32_4_i1 = DM_R_data;
    assign PC_in = MUX32_1_out;
    assign RF_W = RegWr;
    assign RF_W_data = MUX32_4_out;
    assign RF_W_Reg = MUX5_out;
    assign RF_R_Reg1 = Rs;
    assign RF_R_Reg2 = Rt;
    assign SHL26_in = Addr26;
    assign SHL32_in = Sig32;
    assign Sig16 = Imm16;

    ADD add1(Add1_a, Add1_b, Add1_c), add2(Add2_a, Add2_b, Add2_c);
    ALU alu(ALU_a, ALU_b, ALU_c, ALU_mod, ALU_z, ALU_o);
    ALUCU alucu(ALUOp, ALUCU_funct, ALUCtrl);
    DM dm(.R_Enable(DM_R), .W_Enable(DM_W), .Addr(DM_Addr), .R_data(DM_R_data), .W_data(DM_W_data));
    IM im(Inst_Addr, OP_Code, Rs, Rt, Rd, Shamt, Func, Imm16, Addr26);
    MCU mcu(.OP_Code(OP_Code), .ALUOp(ALUOp), .ALUSrc(ALUSrc), .RegDst(RegDst), .RegWr(RegWr), .MemtoReg(MemtoReg), .MemRd(MemRd), .MemWr(MemWr), .Jump(Jump), .Branch(Branch));
    MUX5 mux5(MUX5_ctrl, MUX5_i0, MUX5_i1, MUX5_out);
    MUX32 mux32_1(MUX32_1_ctrl, MUX32_1_i0, MUX32_1_i1, MUX32_1_out);
    MUX32 mux32_2(MUX32_2_ctrl, MUX32_2_i0, MUX32_2_i1, MUX32_2_out);
    MUX32 mux32_3(MUX32_3_ctrl, MUX32_3_i0, MUX32_3_i1, MUX32_3_out);
    MUX32 mux32_4(MUX32_4_ctrl, MUX32_4_i0, MUX32_4_i1, MUX32_4_out);
    PC pc(clk, reset, PC_in, PC_out);
    RF rf(.clk(clk), .W_Enable(RF_W), .W_Reg(RF_W_Reg), .W_data(RF_W_data), .R_Reg1(RF_R_Reg1), .R_Reg2(RF_R_Reg2), .R_data1(RF_R_data1), .R_data2(RF_R_data2));
    SHL2_26 shl2_26(SHL26_in, SHL26_out);
    SHL2_32 shl2_32(SHL32_in, SHL32_out); 
    SigExt sigext(Sig16, Sig32);

endmodule