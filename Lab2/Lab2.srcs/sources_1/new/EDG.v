`timescale 1ns / 1ps

module EDG(
    input clk,rst,y,
    output p 
);
    reg [1:0] state,next_state;
    parameter S0=0,S1=1,S2=2; 
    //output logic
    assign p=(state==S1);
    
    //state logic
    always@(posedge clk, posedge rst) begin
        if(rst) state<=S0;
        else state<= next_state;
    end
    
    //next state logic
    always@(*) begin
        next_state=state;
        case(state)
            S0:if(y) next_state=S1;
            S1:begin
                if(y) next_state=S2;
                else next_state=S0;
               end
            S2:begin
                if(y) next_state=S2;
                else next_state=S0; 
               end
            default:next_state=S0;
        endcase
    end
endmodule
