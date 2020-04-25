`timescale 1ns / 1ps

module Register(
    input [3:0] D,
    input CLK,EN,RST,
    output reg [3:0] Q
    );
    always @(posedge CLK or posedge RST) begin
        if(RST) Q<=0;
        else if(EN) Q<=D;
    end
endmodule
