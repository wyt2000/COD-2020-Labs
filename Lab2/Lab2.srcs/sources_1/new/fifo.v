`timescale 1ns / 1ps

module fifo #(parameter WIDTH = 4, //数据宽度
      DONOTHING=3'b000,
      QIN=3'b001,
      QOUT=3'b010
    )
    (
    input clk, rst,	//时钟（上升沿有效）、异步复位（高电平有效）
    input [7:0] din,		//入队列数据
    input en_in, 		//入队列使能，高电平有效
    input en_out,		//出队列使能，高电平有效
    output [7:0] dout, 	//出队列数据
    output [3:0] count //队列数据计数
);
    wire en,en_i,en_o;
    wire [3:0] head,tail;
    wire isfull,isempty;
    wire select;
    reg [1:0] op_h,op_t,op_c;
    wire [3:0] addr;
    reg [2:0] state,next_state;
    
    //Data Path
    assign en=1;
    assign isfull=&count;
    assign isempty=~|count;
    assign select=(en_i&~en_o);
    
    EDG EDG_i(clk,rst,en_in,en_i);
    EDG EDG_o(clk,rst,en_out,en_o);
    counter counter_h(clk,rst,op_h,head);
    counter counter_t(clk,rst,op_t,tail);
    counter counter_count(clk,rst,op_c,count);  
    Block_RAM_wf Block_RAM_wf(.addra(addr),.clka(clk),.dina(din),.douta(dout),.ena(en),.wea(en_i));
    MUX MUX(select,head,tail,addr);
    
    //Control Unit
    //state logic
    always@(posedge clk or posedge rst) begin
        if(rst) state<=DONOTHING; 
        else state<=next_state;
    end
    //next state logic
    always@(*) begin
        next_state=DONOTHING;
        case({en_i,en_o})
            2'b10: if(~isfull) begin next_state=QIN; end
            2'b01: if(~isempty) begin next_state=QOUT; end
        endcase
    end
    //output logic
    always@(*) begin
        {op_t,op_h,op_c}=6'h0;
        case(state)
            QIN: begin op_t=2'b01;op_c=2'b01;end
            QOUT: begin op_h=2'b01;op_c=2'b10;end
        endcase
    end
endmodule