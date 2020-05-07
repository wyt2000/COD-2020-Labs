`timescale 1ns / 1ps

module  ram_16x8			//16x8λ���˿�RAM
(input  clk, 			//ʱ�ӣ���������Ч��
input en, we,				//ʹ�ܣ�дʹ��
input [3:0]  addr,	//��ַ
input [7:0]  din,		//��������
output [7:0]  dout	//�������
);
reg [3:0] addr_reg;
reg [7:0] mem[15:0];

initial $readmemh("C:\\Users\\lenovo\\Desktop\\git\\COD-2020-Labs\\Lab2\\Lab2.srcs\\sources_1\\new\\init.txt",mem);
assign dout = mem[addr_reg];

always@(posedge clk) begin
  if(en) begin
    addr_reg <= addr;
    if(we)
      mem[addr] <= din;
  end
end
endmodule
