`timescale 1ns / 1ps

module ALU  
    #(parameter WIDTH = 32, //数据宽度
    ADD=3'b000,
    SUB=3'b001,
    AND=3'b010,
    OR=3'b011,
    XOR=3'b100
    ) 	
    (output reg [WIDTH-1:0] y, 		//运算结果
    output reg zf, 					//零标志
    output reg cf, 					//进位/借位标志
    output reg of, 					//溢出标志
    output reg sf,                 //结果的最高位
    input [WIDTH-1:0] a, b,		//两操作数
    input [2:0] m						//操作类型
    );

always@(*) begin
    y=32'h0;
    {zf,cf,of}=3'h0;
    case(m)
        ADD: begin 
            {cf,y}=a+b;
            of = (~a[WIDTH-1] & ~b[WIDTH-1] & y[WIDTH-1]) | (a[WIDTH-1] & b[WIDTH-1] & ~y[WIDTH-1]);
            end
        SUB: begin
            {cf,y}=a-b;
            of = (~a[WIDTH-1] & b[WIDTH-1] & y[WIDTH-1]) | (a[WIDTH-1] & ~b[WIDTH-1] & ~y[WIDTH-1]) ;            
            end
        AND: begin
            y=a&b;
            end
        OR: begin
            y=a|b;
            end
        XOR: begin
            y=a^b;
            end
    endcase
    zf=~|y;
    sf=y[WIDTH-1];
end

endmodule