`timescale 1ns / 1ps

module MUX(
    input  s,
    input [3:0] w0,w1,
    output reg [3:0] o
    );
    always @(*) begin
        if(s) o=w1;
        else o=w0;
    end
endmodule
