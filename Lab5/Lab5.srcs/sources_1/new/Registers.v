`timescale 1ns / 1ps

module Register_File				//32 x 32 �Ĵ�����
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