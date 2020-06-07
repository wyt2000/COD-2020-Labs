`timescale 1ns / 1ps

module MUX#(parameter WIDTH = 32)(
    input [1:0] s,
    input [WIDTH-1:0] w0,w1,w2,w3,
    output reg [WIDTH-1:0] o
    );
    always @(*) begin
        case(s)
            2'b00: o=w0;
            2'b01: o=w1;
            2'b10: o=w2;
            2'b11: o=w3;
        endcase
    end
endmodule