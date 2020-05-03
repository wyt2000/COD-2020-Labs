`timescale 1ns / 1ps

module lab2_ram_tb;
    reg clk,we,en;
    reg [3:0] a;
    reg [7:0] d;
    wire [7:0] spo_dis;
    wire [7:0] spo_block_wf;
    wire [7:0] spo_block_rf;
    wire [7:0] spo_block_nc;
    
    parameter PERIOD=10,CYCLE=50;
    Distributed_RAM Distributed_RAM(.clk(clk),.a(a),.d(d),.we(we),.spo(spo_dis));
    Block_RAM_wf Block_RAM_wf(.clka(clk),.addra(a),.dina(d),.douta(spo_block_wf),.ena(en),.wea(we));
    Block_RAM_rf Block_RAM_rf(.clka(clk),.addra(a),.dina(d),.douta(spo_block_rf),.ena(en),.wea(we));
    Block_RAM_nc Block_RAM_nc(.clka(clk),.addra(a),.dina(d),.douta(spo_block_nc),.ena(en),.wea(we));
    initial begin
        clk=0;
        repeat (2*CYCLE)
            #(PERIOD/2) clk=~clk;
        $finish;
    end
    initial begin
        en=1;
        we=0;
        a=1;
        d=2;
        #10;
        we=1;
        a=2;
        d=4;
        #10;
        we=1;
        a=2;
        d=5;
        #10;
        we=0;
        a=2;
        d=0;
        #10
        we=0;
        a=0;
        d=0;
    end
endmodule