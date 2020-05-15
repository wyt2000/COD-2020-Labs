`timescale 1ns / 1ps

module lab3_cpu_tb();
    reg clk,rst,run;
    CPU CPU(clk,rst,run);
    initial begin
        clk=0;
        run=1;
        repeat (500)
            #(5) clk=~clk;
    end
    initial begin
        rst=1;
        #10
        rst=0;
    end
endmodule
