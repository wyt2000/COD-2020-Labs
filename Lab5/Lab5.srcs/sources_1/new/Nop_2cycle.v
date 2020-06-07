`timescale 1ns / 1ps

module Nop_2cycle(
    input clk,
    input rst,
    input [5:0] ins,
    input [5:0] op,
    input is_lw_stall,
    output reg control,
    output reg stall
    );
    reg [1:0] state,next_state;
    always@(posedge clk, posedge rst) begin
        if(rst) state<=0;
        else state<=next_state;
    end
    always@(*) begin
        next_state=state;
        case(state)
            0: if(ins==op&&~is_lw_stall) next_state=1;
            1: next_state=2;
            2: next_state=0;
        endcase
    end
    always@(*) begin
        if(ins==op&&state!=2) control=0;
        else control=1;
        if(state) stall=1;
        else stall=0; 
    end
endmodule
