`timescale 1ns/1ps
module Multi_tb (
);
    reg clk;
    reg reset;
    integer i;
    Multi Multi(clk, reset);

    initial begin
       $dumpfile("D:/CPU/build/multi.vcd"); 
	    $dumpvars(0, Multi_tb);
        clk = 0;
        reset = 1;
        # 5;
        reset = 0;   
        for(i = 0;i < 128; i = i + 1) begin
            clk = ~clk;
            #5;
        end  
        $finish;
    end
endmodule