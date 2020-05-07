`timescale 1ns / 1ps

module register_file				//32 x WIDTH寄存器堆
#(parameter WIDTH = 32) 	//数据宽度
(   input clk,						//时钟（上升沿有效）
    input [4:0] ra0,				//读端口0地址
    output [WIDTH-1:0] rd0, 	//读端口0数据
    input [4:0] ra1, 				//读端口1地址
    output [WIDTH-1:0] rd1, 	//读端口1数据
    input [4:0] wa, 				//写端口地址
    input we,					//写使能，高电平有效
    input [WIDTH-1:0] wd 		//写端口数据
);
    reg [WIDTH-1:0] regs [31:0]; //32 32-bit binary codes
    initial $readmemh("C:\\Users\\lenovo\\Desktop\\git\\COD-2020-Labs\\Lab2\\Lab2.srcs\\sources_1\\new\\init.txt",regs);
    assign rd0 = regs[ra0];
    assign rd1 = regs[ra1];
    always@(posedge clk) begin
        if(we&&wa) regs[wa]<=wd;
    end
endmodule