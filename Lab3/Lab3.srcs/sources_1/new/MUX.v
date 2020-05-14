`timescale 1ns / 1ps

module MUX#(parameter WIDTH = 32)(
    input  s,
    input [WIDTH-1:0] w0,w1,
    output reg [WIDTH-1:0] o
    );
    always @(*) begin
        if(s) o=w1;
        else o=w0;
    end
endmodule
