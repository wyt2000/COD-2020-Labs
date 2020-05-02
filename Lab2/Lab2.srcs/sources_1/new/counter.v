`timescale 1ns / 1ps

module counter(
    input clk,rst,
    input [1:0] op,
    output reg [3:0] cnt
);
    parameter INC=2'b01,DEC=2'b10;
    always@(posedge clk, posedge rst) begin
        if(rst) cnt<=4'b0000;
        else begin
        case(op)
            INC: cnt<=cnt+1;
            DEC: cnt<=cnt-1;
            default: cnt<=cnt;
        endcase
        end
    end
endmodule
