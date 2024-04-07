`timescale 1ns/1ps
module Pipeline_tb (
);
    reg clk;
    reg reset;
    integer i;
    Pipeline Pipeline(clk, reset);

    initial begin
        $dumpfile("D:/CPU/build/pipeline.vcd"); 
	    $dumpvars(0, Pipeline_tb);
        clk = 0;
        reset = 1;
        # 5
        reset = 0;
        #5;
        for(i = 0;i < 64; i = i + 1) begin
            clk = ~clk;
            #5;
        end  
        $finish;
    end
endmodule