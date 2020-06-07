`timescale 1ns / 1ps

module Forwarding_Unit(
    input [4:0] ID_EX_Rs,
    input [4:0] ID_EX_Rt,
    input [4:0] EX_MEM_Rd,
    input [4:0] MEM_WB_Rd,
    input EX_MEM_RegWrite,
    input MEM_WB_RegWrite,
    output reg [1:0] select_a,
    output reg [1:0] select_b,
    output reg [1:0] select_c
    );
    always @(*) begin
        if(ID_EX_Rs&&ID_EX_Rs==EX_MEM_Rd && EX_MEM_RegWrite) select_a=1;
        else if(ID_EX_Rs&&ID_EX_Rs==MEM_WB_Rd && MEM_WB_RegWrite) select_a=2;
        else select_a=0;
        if(ID_EX_Rt&&ID_EX_Rt==EX_MEM_Rd && EX_MEM_RegWrite) select_b=1;
        else if(ID_EX_Rt&&ID_EX_Rt==MEM_WB_Rd && MEM_WB_RegWrite) select_b=2;
        else select_b=0;
        if(ID_EX_Rt&&ID_EX_Rt==EX_MEM_Rd && EX_MEM_RegWrite) select_c=1;
        else if(ID_EX_Rt&&ID_EX_Rt==MEM_WB_Rd && MEM_WB_RegWrite) select_c=2;
        else select_c=0;
    end
endmodule
