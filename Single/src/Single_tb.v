`timescale 1ns/1ps
module Single_tb (
);
    reg clk;
    reg reset;
    integer i;
    Single Single(clk, reset);

    initial begin
        $dumpfile("D:/CPU/build/single.vcd"); 
	    $dumpvars(0, Single_tb);
        clk = 0;
        reset = 1;
        # 5;
        clk = ~clk;
        # 5;
        reset = 0;
        for(i = 0;i < 31; i = i + 1) begin
            clk = ~clk;
            #5;
        end  
        $finish;
    end
endmodule