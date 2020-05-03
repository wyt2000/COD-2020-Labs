`timescale 1ns / 1ps

module counter(
    input clk,rst,
    input [1:0] op,
    output reg [4:0] cnt
);
    parameter INC=2'b01,DEC=2'b10;
    reg tag=0;
    always@(posedge clk, posedge rst) begin
        if(rst) cnt<=4'b0000;
        else begin
        case(op)
            INC: cnt<=cnt+1;
            DEC: tag<=~tag;
            default: cnt<=cnt;
        endcase
        end
        if(tag) begin cnt<=cnt-1;tag<=~tag; end
    end
endmodule
