`timescale 1ns / 1ps

module register_file				//32 x WIDTH�Ĵ�����
#(parameter WIDTH = 32) 	//���ݿ��
(   input clk,						//ʱ�ӣ���������Ч��
    input [4:0] ra0,				//���˿�0��ַ
    output [WIDTH-1:0] rd0, 	//���˿�0����
    input [4:0] ra1, 				//���˿�1��ַ
    output [WIDTH-1:0] rd1, 	//���˿�1����
    input [4:0] wa, 				//д�˿ڵ�ַ
    input we,					//дʹ�ܣ��ߵ�ƽ��Ч
    input [WIDTH-1:0] wd 		//д�˿�����
);
    reg [WIDTH-1:0] regs [31:0]; //32 32-bit binary codes
    initial $readmemh("C:\\Users\\lenovo\\Desktop\\git\\COD-2020-Labs\\Lab2\\Lab2.srcs\\sources_1\\new\\init.txt",regs);
    assign rd0 = regs[ra0];
    assign rd1 = regs[ra1];
    always@(posedge clk) begin
        if(we&&wa) regs[wa]<=wd;
    end
endmodule