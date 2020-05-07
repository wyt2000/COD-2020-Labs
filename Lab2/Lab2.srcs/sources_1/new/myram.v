`timescale 1ns / 1ps

module  ram_16x8			//16x8位单端口RAM
(input  clk, 			//时钟（上升沿有效）
input en, we,				//使能，写使能
input [3:0]  addr,	//地址
input [7:0]  din,		//输入数据
output [7:0]  dout	//输出数据
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
