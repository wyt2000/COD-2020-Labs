`timescale 1ns / 1ps

module Register_File				//32 x 32 寄存器堆
#(parameter WIDTH = 32) 	//数据宽度
(   input clk,						//时钟（上升沿有效）
    input [4:0] ra1,				//读端口1地址
    output [WIDTH-1:0] rd1, 	//读端口1数据
    input [4:0] ra2, 				//读端口2地址
    output [WIDTH-1:0] rd2, 	//读端口2数据
    input [4:0] ra3, 				//读端口3地址
    output [WIDTH-1:0] rd3, 	//读端口3数据
    input [4:0] wa, 				//写端口地址
    input we,					//写使能，高电平有效
    input [WIDTH-1:0] wd 		//写端口数据
);
    reg [WIDTH-1:0] regs [31:0]; //32 32-bit binary codes
    integer i;
    initial begin
        for(i=0;i<=32;i=i+1) begin
            regs[i]=0;
        end
    end
    assign rd1 = regs[ra1];
    assign rd2 = regs[ra2];
    assign rd3 = regs[ra3];
    always@(negedge clk) begin
        if(we&&wa) regs[wa]<=wd;
    end
endmodule