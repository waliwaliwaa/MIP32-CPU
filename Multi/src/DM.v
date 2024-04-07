`timescale 1ns/1ps
module DM (
    input R_Enable,
    input W_Enable,
    input IorD,
    input [31:0] Addr,
    input [31:0] W_data,
    output reg [31:0] R_data
);
    reg [31:0] mem[0:64];
    integer fd1, i;
    initial begin
        $readmemb("D:/CPU/DM.txt", mem);
    end
    always @(*) begin
        if (R_Enable == 1) begin
            if(IorD == 0)
                R_data = mem[8 + Addr];
            else
                R_data = mem[Addr];
        end
        if (W_Enable == 1) begin
            mem[Addr] = W_data;

            fd1 = $fopen("D:/CPU/DM.txt");
            for(i = 0; i <= 18; i = i + 1) begin
                $fdisplay(fd1, "%b", mem[i]);
                $fdisplay(fd1, "");
            end
            $fclose(fd1);
            
        end
    end
endmodule