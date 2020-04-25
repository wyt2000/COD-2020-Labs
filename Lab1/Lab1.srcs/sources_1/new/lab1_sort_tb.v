`timescale 1ns / 100ps

module sort_tb;
    reg clk, rst;
    reg [3:0] x0, x1, x2, x3;
    wire [3:0] s0, s1, s2, s3;
    wire done;
    
parameter PERIOD = 10, 	//时钟周期长度
CYCLE = 50;		//时钟个数

    sort#(.WIDTH(4)) SORT(.CLK(clk), .RST(rst), .x0(x0), .x1(x1), .x2(x2), .x3(x3), .s0(s0), .s1(s1), .s2(s2), .s3(s3), .done(done));
    
    initial begin
        clk = 0;
        repeat (2 * CYCLE)
        	#(PERIOD/2) clk = ~clk;
        $finish;
    end
    
    initial begin
        rst=1;
        x0=-3;
        x1=2;
        x2=-2;
        x3=1;
        #10 rst=0;
        #80;
        
        rst=1;
        x0=-5;
        x1=7;
        x2=3;
        x3=-2;
        #10 rst=0;
        #80;
        
        rst=1;
        x0=-4;
        x1=-3;
        x2=4;
        x3=-5;
        #10 rst=0;
        #80; 
        
    end
endmodule
