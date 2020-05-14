`timescale 1ns / 1ps

module lab3_cpu_tb();
    reg clk,rst;
    CPU CPU(clk,rst);
    initial begin
        clk=0;
        repeat (500)
            #(5) clk=~clk;
    end
    initial begin
        rst=1;
        #10
        rst=0;
    end
endmodule
