`timescale 1ns / 1ps

module Nop_1cycle(
    input clk,
    input rst,
    input flag,
    output reg control,
    output reg stall,
    output reg is_lw_stall
    );
    
    reg state,next_state;
    always@(posedge clk, posedge rst) begin
        if(rst) state<=0;
        else state<=next_state;
    end
    always@(*) begin
        next_state=state;
        case(state)
            0: if(flag) next_state=1;
            1: next_state=0;
        endcase
    end
    always@(*) begin
        if(flag&&~state) {control,stall,is_lw_stall}=3'b011;
        else {control,stall,is_lw_stall}=3'b100;
    end

endmodule
