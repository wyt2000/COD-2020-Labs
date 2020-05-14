`timescale 1ns / 1ps

module CPU( //单周期 CPU
    input clk, //上升沿有效
    input rst, //异步复位，高电平有效
    input run, //是否继续运行
    input [7:0] m_rf_addr, // MEM/RF 的调试读口地址，复位时为零
    output [0:235] status,  //用一个大数组来装12个控制信号和7*32位供sel选择的数据
    output [31:0] m_data,  // MEM 的数据
    output [31:0] rf_data  // RF 的数据
    );
    parameter add=6'b000000;
    parameter addi=6'b001000;
    parameter lw=6'b100011;
    parameter sw=6'b101011;
    parameter beq=6'b000100;
    parameter j=6'b000010;

    wire en=1;
    wire [31:0] ALU1_res,ALU2_res,ALU2_a,ALU2_b,DataMem_rd,M2_o,M3_o,Regs_rd1,Regs_rd2,Sign_extend,M4_o,M5_o,Jump_addr;
    wire ALU1_zf,M4_s;
    wire [4:0] M1_o;
    wire [31:0] npc,pc,ins;
    reg [2:0] ALUop;
    reg Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,RegDst; //control signals
    
    //Data Path
    assign ALU2_a=pc+4;
    assign Sign_extend={ { 16{ins[15]} } , ins[15:0]};
    assign ALU2_b=Sign_extend<<2;
    assign M4_s=Branch&ALU1_zf;
    assign Jump_addr={ALU2_a[31:28],ins[25:0]<<2};
    assign npc=run?M5_o:pc;
    assign status={Jump, Branch, RegDst,RegWrite, MemRead, MemtoReg,MemWrite,ALUop, ALUSrc, ALU1_zf, npc, pc, ins, Regs_rd1, Regs_rd2, ALU1_res, DataMem_rd};
    
    PC_Register PC_Register(npc,clk,en,rst,pc);
    Instruction_Memory Instruction_Memory(.a(pc[31:2]),.spo(ins));
    Data_Memory Data_Memory(.a(ALU1_res[31:2]),.d(Regs_rd2),.clk(clk),.we(MemWrite),.spo(DataMem_rd), .dpra({10'h0,m_rf_addr[7:2]}),.dpo(m_data));
    Registers Registers(.clk(clk), .ra1(ins[25:21]), .ra2(ins[20:16]), .ra3(m_rf_addr[4:0]), .wa(M1_o), .wd(M3_o), .we(RegWrite), .rd1(Regs_rd1), .rd2(Regs_rd2), .rd3(rf_data));
    ALU ALU1(.y(ALU1_res),.m(ALUop),.a(Regs_rd1),.b(M2_o),.zf(ALU1_zf));
    ALU ALU2(.y(ALU2_res),.m(3'b000),.a(ALU2_a),.b(ALU2_b));
    
    MUX  #(.WIDTH(5)) M1(.s(RegDst), .w0(ins[20:16]), .w1(ins[15:11]), .o(M1_o));
    MUX  #(.WIDTH(32)) M2(.s(ALUSrc), .w0(Regs_rd2), .w1(Sign_extend), .o(M2_o));
    MUX  #(.WIDTH(32)) M3(.s(MemtoReg), .w0(ALU1_res), .w1(DataMem_rd), .o(M3_o)); 
    MUX  #(.WIDTH(32)) M4(.s(M4_s), .w0(pc+4), .w1(ALU2_res),.o(M4_o));
    MUX  #(.WIDTH(32)) M5(.s(Jump), .w0(M4_o), .w1(Jump_addr),.o(M5_o));
    
    //Control Unit
     always@(*) begin
            {Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,RegDst}=9'b0;
            ALUop<=3'b111;
            case(ins[31:26])
                add: begin
                    ALUop<=3'b000;
                    RegWrite<=1;
                    RegDst<=1;
                end
                addi:begin
                    ALUop<=3'b000;
                    RegWrite<=1;
                    ALUSrc<=1;
                end
                lw: begin
                    ALUop<=3'b000;
                    MemRead<=1;
                    MemtoReg<=1;
                    ALUSrc<=1;
                    RegWrite<=1;
                end
                sw: begin
                    ALUop<=3'b000;
                    MemWrite<=1;
                    ALUSrc<=1;
                end
                beq: begin
                    Branch<=1;
                    ALUop<=3'b001;
                end
                j: Jump<=1;
            endcase
        end
endmodule
