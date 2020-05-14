`timescale 1ns / 1ps

module lab3_dbu_tb();
    reg clk,rst,succ,step,m_rf,inc,dec;
    reg [2:0] sel;
    wire [11:0] led; 
    wire [7:0] CA,AN;
    DBU DBU(clk,rst,succ,step,sel,m_rf,inc,dec,led,CA,AN);
    initial begin
        clk=1;
        repeat (500)
            #(5) clk=~clk;
    end
    initial begin
        inc=1;
        repeat (16)
            #(10) inc=~inc;
    end
    initial begin
        rst=1;
        succ=0;
        step=0;
        sel=0;
        m_rf=1;
        dec=0;
        #10
        rst=0;
        succ=1;
        #40
        succ=0;
        #20
        step=1;
        #10
        step=0;
        #10
        step=1;
        #10
        step=0;
        #10
        succ=1;
        #80
        #100
        m_rf=0;
        #100
        sel=3;
        #100;
    end
endmodule
