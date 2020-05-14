`timescale 1ns / 1ps

module Registers				//32 x 32 �Ĵ�����
#(parameter WIDTH = 32) 	//���ݿ��
(   input clk,						//ʱ�ӣ���������Ч��
    input [4:0] ra1,				//���˿�1��ַ
    output [WIDTH-1:0] rd1, 	//���˿�1����
    input [4:0] ra2, 				//���˿�2��ַ
    output [WIDTH-1:0] rd2, 	//���˿�2����
    input [4:0] ra3, 				//���˿�3��ַ
    output [WIDTH-1:0] rd3, 	//���˿�3����
    input [4:0] wa, 				//д�˿ڵ�ַ
    input we,					//дʹ�ܣ��ߵ�ƽ��Ч
    input [WIDTH-1:0] wd 		//д�˿�����
);
    reg [WIDTH-1:0] regs [31:0]; //32 32-bit binary codes
    initial $readmemh("C:\\Users\\lenovo\\Desktop\\git\\COD-2020-Labs\\Lab3\\Lab3.srcs\\sources_1\\new\\init.txt",regs);
    assign rd1 = regs[ra1];
    assign rd2 = regs[ra2];
    assign rd3 = regs[ra3];
    always@(posedge clk) begin
        if(we&&wa) regs[wa]<=wd;
    end
endmodule