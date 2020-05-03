`timescale 1ns / 1ps

module lab2_fifo_tb;
    reg clk, rst;	
    reg [7:0] din;		
    reg en_in; 		
    reg en_out;		
    wire [7:0] dout; 	
    wire [4:0] count;	
    

    fifo fifo(clk,rst,din,en_in,en_out,dout,count);
    
    initial begin
        clk=1;
        repeat (500)
            #(5) clk=~clk;
    end
    
    initial begin
        rst=1;
        din=0;
        en_in=0;
        en_out=0;
        #10;
        rst=0;
        en_out=0;
        en_in=1;
        din=0;
        repeat (34)
            #(20) begin en_in=~en_in; din=din+1; end
        en_out=1;
        en_in=0;
        repeat (34)
            #(20) en_out=~en_out;
        $finish;
    end
        
endmodule
