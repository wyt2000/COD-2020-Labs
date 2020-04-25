`timescale 1ns / 1ps

module sort
    #(parameter WIDTH = 4, //数据宽度
      LOAD=3'b000,
      CX01_1=3'b001,
      CX12_1=3'b010,
      CX23_1=3'b011,
      CX01_2=3'b100,
      CX12_2=3'b101,
      CX01_3=3'b110,
      HLT=3'b111
    )
    (output [WIDTH-1:0] s0, s1, s2, s3, 	//排序后的四个数据（递增）
    output reg done, 				//排序结束标志
    input [WIDTH-1:0] x0, x1, x2, x3,	//原始输入数据
    input CLK, RST				//时钟（上升沿有效）、复位（高电平有效）
    );
    wire [3:0]  i0,i1,i2,i3,i4,i5,r0,r1,r2,r3;
    wire of,sf,less;    
    reg m0,m1,m2,m3,m4,m5,en0,en1,en2,en3;
    reg [2:0] SUB=3'b001;
    reg [2:0] current_state,next_state;
    //Data Path
    Register R0(i2,CLK,en0,RST,r0);
    Register R1(i3,CLK,en1,RST,r1);
    Register R2(i4,CLK,en2,RST,r2);
    Register R3(i5,CLK,en3,RST,r3);
    ALU #(.WIDTH(WIDTH)) ALU(.a(i0),.b(i1),.of(of),.sf(sf),.m(SUB));
    MUX M0(m0,r0,r2,i0);
    MUX M1(m1,r1,r3,i1);
    MUX M2(m2,x0,r1,i2);
    MUX M3(m3,i0,x1,i3);
    MUX M4(m4,i1,x2,i4);
    MUX M5(m5,r2,x3,i5);
    // Control Unit
    assign s0=r0;
    assign s1=r1;
    assign s2=r2;
    assign s3=r3;
    assign less=of^sf;
    always@(posedge CLK or posedge RST) begin
        if(RST) current_state<=LOAD; 
        else current_state<=next_state;
    end
    always@(*) begin
        if(current_state!=HLT) next_state=current_state+1;
    end
    always@(*) begin
        {m0,m1,m2,m3,m4,m5,en0,en1,en2,en3,done}=11'h0;
        case(current_state)
            LOAD:{m3,m4,m5,en0,en1,en2,en3}=7'b111_1111;
            CX01_1,CX01_2,CX01_3:begin m2=1;en0=~less;en1=~less; end
            CX12_1,CX12_2:begin m0=1;en1=less;en2=less; end
            CX23_1:begin m0=1;m1=1;en2=~less;en3=~less; end
            HLT: done=1;
        endcase
    end
endmodule
