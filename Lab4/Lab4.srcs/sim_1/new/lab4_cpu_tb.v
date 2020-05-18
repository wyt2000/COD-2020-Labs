`timescale 1ns / 1ps

module lab3_cpu_tb();
    reg clk,rst;
    reg [7:0] m_rf_addr;
    wire [0:207] status;
    wire [31:0] m_data; 
    wire [31:0] rf_data;  
    CPU CPU(clk,rst,m_rf_addr,status,m_data,rf_data);
    initial begin
        m_rf_addr=10'h0;
        clk=0;
        repeat (500)
            #(5) clk=~clk;
    end
    initial begin
        rst=1;
        #5
        rst=0;
    end
endmodule
