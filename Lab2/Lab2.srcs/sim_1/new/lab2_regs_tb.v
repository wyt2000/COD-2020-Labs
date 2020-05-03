`timescale 1ns / 1ps

module lab2_regs_tb;
    reg clk,we;
    reg [4:0] ra0,ra1,wa;
    reg [31:0] wd;
    wire [31:0] rd0,rd1;
    
    parameter PERIOD=10,CYCLE=50;
    register_file regs(clk,ra0,rd0,ra1,rd1,wa,we,wd);
    
    initial begin
        clk=0;
        repeat (2*CYCLE)
            #(PERIOD/2) clk=~clk;
        $finish;
    end
    initial begin
        ra0=0;
        ra1=1;
        wd=12;
        we=0;
        wa=0;
        #10
        ra0=0;
        ra1=1;
        wd=12;
        we=1;
        wa=0;
        #10
        ra0=0;
        ra1=1;
        wd=12;
        we=1;
        wa=1;
        #10
        ra0=0;
        ra1=1;
        wd=9;
        we=0;
        wa=0;
        #10;
    end
endmodule
