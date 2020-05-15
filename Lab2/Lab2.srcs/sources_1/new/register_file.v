`timescale 1ns / 1ps

module register_file				//32 x WIDTH�Ĵ�����
#(parameter WIDTH = 256) 	//���ݿ��
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
    integer i;
    initial begin
        for(i=0;i<=32;i=i+1) begin
            regs[i]=0;
        end
    end
    assign rd0 = regs[ra0];
    assign rd1 = regs[ra1];
    always@(posedge clk) begin
        if(we&&wa) regs[wa]<=wd;
    end
endmodule