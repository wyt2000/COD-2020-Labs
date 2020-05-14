`timescale 1ns / 1ps

module PC_Register #(parameter WIDTH = 32)(
    input [WIDTH-1:0] D,
    input CLK,EN,RST,
    output reg [WIDTH-1:0] Q
    );
    always @(posedge CLK or posedge RST) begin
        if(RST) Q<=0;
        else if(EN) Q<=D;
    end
endmodule