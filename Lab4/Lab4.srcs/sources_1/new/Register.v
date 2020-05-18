`timescale 1ns / 1ps

module Register #(parameter WIDTH = 32)(
    input [WIDTH-1:0] d,
    input clk,en,rst,
    output reg [WIDTH-1:0] q
    );
    always @(posedge clk or posedge rst) begin
        if(rst) q<=0;
        else if(en) q<=d;
    end
endmodule