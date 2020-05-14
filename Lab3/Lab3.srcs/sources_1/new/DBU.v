`timescale 1ns / 1ps

module counter(
    input clk,
    input rst,
    output reg [2:0] id
);
    reg [13:0] cnt;
    always@(posedge clk,posedge rst) begin
        if(rst) begin cnt<=0;id<=0; end
        cnt<=cnt+1;
        if(~cnt) id<=id+1;
    end
endmodule

module DBU(
    input clk,
    input rst,
    input succ,
    input step,
    input [2:0] sel,
    input m_rf,
    input inc,
    input dec,
    output reg [11:0] led, //用后8个 LED 灯显示地址  
    output reg [7:0] CA,
    output reg [7:0] AN
    );
    wire step_y,run;
    wire [0:235] status;
    wire [31:0] m_data,rf_data;
    wire [2:0] id;
    wire [7:0] res [7:0];
    wire inc_y,dec_y;
    reg [7:0] m_rf_addr;
    reg [31:0] seg;
    
    EDG EDG_1(clk,rst,step,step_y);
    EDG EDG_2(clk,rst,inc,inc_y);
    EDG EDG_3(clk,rst,dec,dec_y);
    CPU CPU(clk,rst,run,m_rf_addr,status,m_data,rf_data);
    
    counter counter(clk,rst,id);
    hex_segment_display S0(.a(seg[3:0]), .spo(res[0]));
    hex_segment_display S1(.a(seg[7:4]), .spo(res[1]));
    hex_segment_display S2(.a(seg[11:8]), .spo(res[2]));
    hex_segment_display S3(.a(seg[15:12]), .spo(res[3]));
    hex_segment_display S4(.a(seg[19:16]), .spo(res[4]));
    hex_segment_display S5(.a(seg[23:20]), .spo(res[5]));
    hex_segment_display S6(.a(seg[27:24]), .spo(res[6]));
    hex_segment_display S7(.a(seg[31:28]), .spo(res[7]));

     always @ (posedge clk, posedge rst) begin
        if(rst) AN<=8'b1111_1111;
        case(id)
            3'b000: begin AN<=8'b1111_1110;CA<=res[0]; end
            3'b001: begin AN<=8'b1111_1101;CA<=res[1]; end
            3'b010: begin AN<=8'b1111_1011;CA<=res[2]; end
            3'b011: begin AN<=8'b1111_0111;CA<=res[3]; end
            3'b100: begin AN<=8'b1110_1111;CA<=res[4]; end
            3'b101: begin AN<=8'b1101_1111;CA<=res[5]; end
            3'b110: begin AN<=8'b1011_1111;CA<=res[6]; end
            3'b111: begin AN<=8'b0111_1111;CA<=res[7]; end
        endcase
    end
    
    assign run=succ|step_y;
    always@(posedge clk,negedge rst) begin
        if(rst) begin 
            m_rf_addr<=0;
            led<=0;
            CA<=0;
            m_rf_addr<=0;
            seg<=0;
        end
        else begin
            led=status[0:11];
            case(sel)
                3'b000: begin
                    if(inc_y) m_rf_addr<=m_rf_addr+1;
                    if(dec_y) m_rf_addr<=m_rf_addr-1;
                    led[11:8]<=4'b0000;
                    led[7:0]<=m_rf_addr;
                    if(m_rf) seg<=m_data;
                    else seg<=rf_data;
                end
                3'b001: seg<=status[12:43];
                3'b010: seg<=status[44:75];
                3'b011: seg<=status[76:107];
                3'b100: seg<=status[108:139];
                3'b101: seg<=status[140:171];
                3'b110: seg<=status[172:203];
                3'b111: seg<=status[204:235];
            endcase
         end
    end
endmodule
