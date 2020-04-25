`timescale 1ns / 1ps

module ALU  
    #(parameter WIDTH = 32, //���ݿ��
    ADD=3'b000,
    SUB=3'b001,
    AND=3'b010,
    OR=3'b011,
    XOR=3'b100
    ) 	
    (output reg [WIDTH-1:0] y, 		//������
    output reg zf, 					//���־
    output reg cf, 					//��λ/��λ��־
    output reg of, 					//�����־
    output reg sf,                 //��������λ
    input [WIDTH-1:0] a, b,		//��������
    input [2:0] m						//��������
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