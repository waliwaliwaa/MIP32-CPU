`timescale 1ns/1ps
module RF (
    input clk,
    input W_Enable,
    input [4:0] R_Reg1,
    input [4:0] R_Reg2,
    input [4:0] W_Reg,
    input [31:0] W_data,
    output [31:0] R_data1,
    output [31:0] R_data2
);
    reg [31:0] mem[0:31];
    integer fd1, err1, i;
    
    initial begin
        $readmemb("D:/CPU/RF.txt", mem);
    end
    
    assign R_data1 = mem[R_Reg1];
    assign R_data2 = mem[R_Reg2];
    
    always @(negedge clk) begin
        if (W_Enable == 1) begin
            mem[W_Reg] = W_data;
        end

        fd1 = $fopen("RF.txt");
        for (i = 0; i < 32; i = i + 1) begin
            $fdisplay(fd1, "%b", mem[i]);
            $fdisplay(fd1, "");
        end
        $fclose(fd1);
    end
endmodule