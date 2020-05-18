module CPU( //多周期 CPU
    input clk, //上升沿有效
    input rst, //异步复位，高电平有效
    input [7:0] m_rf_addr, // MEM/RF 的调试读口地址，复位时为零
    output [0:207] status,  //用一个大数组来装12个控制信号和7*32位供sel选择的数据
    output [31:0] m_data,  // MEM 的数据
    output [31:0] rf_data  // RF 的数据
    );
    parameter add=6'b000000;
    parameter addi=6'b001000;
    parameter lw=6'b100011;
    parameter sw=6'b101011;
    parameter beq=6'b000100;
    parameter j=6'b000010;
    
    //Data Path
    reg PCWriteCond,PCWrite,lorD,MemRead,MemWrite,MemtoReg,IRWrite,
        ALUSrcA,RegWrite,RegDst,B_en;
    reg [1:0] ALUSrcB,ALUOp,PCSource;
    reg [3:0] state,next_state;
    reg [31:0] pc_out;
    wire PCwe,ALU_zf;
    wire [31:0] npc,pc,M1_out,Mem_out,MDR_out,
        M3_out,Regs_rd1,Regs_rd2,A_out,B_out,M4_out,
        M5_out,ALU_res,Jump_addr,
        ALUOut_out,M6_out,ins,Sign_extend;
    wire [4:0] M2_out;
    reg [2:0] ALUm;
    
    assign PCwe=(ALU_zf&PCWriteCond)|PCWrite; 
    assign npc=M6_out;
    assign Sign_extend={ { 16{ins[15]} } , ins[15:0]};
    assign Jump_addr={pc[31:28],ins[25:0]<<2};
    assign status={PCSource,PCwe,lorD,MemWrite,IRWrite,
    RegDst,MemtoReg,RegWrite,ALUm,ALUSrcA,ALUSrcB,ALU_zf,
    pc_out,ins,MDR_out,A_out,B_out,ALUOut_out};
    
    Register PC_Register (.d(npc), .clk(clk), .en(PCwe), .rst(rst), .q(pc));
    Register Instruction_Register (.d(Mem_out), .clk(clk), .en(IRWrite), .rst(rst), .q(ins));
    Register MDR (.d(Mem_out), .clk(clk), .en(1), .rst(rst), .q(MDR_out));
    Register A (.d(Regs_rd1), .clk(clk), .en(1), .rst(rst), .q(A_out));
    Register B (.d(Regs_rd2), .clk(clk), .en(B_en), .rst(rst), .q(B_out));
    Register ALUOut (.d(ALU_res), .clk(clk), .en(1), .rst(rst), .q(ALUOut_out));
    
    MUX M1 (.s({0,lorD}), .w0(pc), .w1(ALUOut_out), .o(M1_out));
    MUX #(.WIDTH(5)) M2 (.s({0,RegDst}), .w0(ins[20:16]), .w1(ins[15:11]), .o(M2_out));
    MUX M3 (.s({0,MemtoReg}), .w0(ALUOut_out), .w1(MDR_out), .o(M3_out));
    MUX M4 (.s({0,ALUSrcA}), .w0(pc), .w1(A_out), .o(M4_out));
    MUX M5 (.s(ALUSrcB), .w0(B_out), .w1(4), .w2(Sign_extend), .w3(Sign_extend<<2), .o(M5_out));
    MUX M6 (.s(PCSource), .w0(ALU_res), .w1(ALUOut_out), .w2(Jump_addr), .o(M6_out));

    Memory Memory (.a(M1_out[31:2]), .d(B_out), .clk(clk), .we(MemWrite), .spo(Mem_out), .dpra({10'h0,m_rf_addr[7:2]}),.dpo(m_data));
    Register_File Registers (.clk(clk), .ra1(ins[25:21]), .ra2(ins[20:16]), .ra3(m_rf_addr[4:0]), .wa(M2_out), .wd(M3_out), .we(RegWrite), .rd1(Regs_rd1), .rd2(Regs_rd2), .rd3(rf_data));
    ALU ALU (.y(ALU_res), .m(ALUm), .a(M4_out), .b(M5_out), .zf(ALU_zf),.rst(rst));
    
    //Control Unit
    always@(posedge clk, posedge rst) begin
        if(rst) state<=0;
        else state<=next_state;
    end
    
    always@(*) begin
        next_state=state;
        case(state)
            0: next_state=1;
            1: begin
                case(ins[31:26])
                    lw,sw,addi: next_state=2;
                    add: next_state=6;
                    beq: next_state=8;
                    j: next_state=9;
                endcase
            end
            2: begin
                case(ins[31:26])
                    lw: next_state=3;
                    sw: next_state=5;
                    addi: next_state=10;
                endcase
            end
            3: next_state=4;
            4,5,7,8,9,10: next_state=0;
            6: next_state=7;
        endcase
    end
    
    always@(*) begin
     {PCWriteCond,PCWrite,lorD,MemRead,MemWrite,MemtoReg,IRWrite,
     ALUSrcA,RegWrite,RegDst,ALUSrcB,ALUOp,PCSource}=10'h0;
     B_en=1;
     ALUm=3'b000;
        case(state)
            0: begin
                MemRead=1;
                IRWrite=1;
                ALUSrcB=2'b01;
                PCWrite=1;
                pc_out=pc;
            end
            1: begin
                ALUSrcB=2'b11;
            end
            2: begin
                ALUSrcA=1;
                ALUSrcB=2'b10;
            end
            3: begin
                MemRead=1;
                lorD=1;
            end
            4: begin
                RegWrite=1;
                MemtoReg=1;
            end
            5: begin
                MemWrite=1;
                lorD=1;
            end
            6: begin
                ALUSrcA=1;
                ALUOp=2'b10;
            end
            7: begin
                RegDst=1;
                RegWrite=1;
            end
            8: begin
                ALUSrcA=1;
                ALUOp=2'b01;
                ALUm=3'b001;
                PCWriteCond=1;
                PCSource=2'b01;
            end
            9: begin
                PCWrite=1;
                PCSource=2'b10;
            end
            10: begin
                RegWrite=1;
            end
        endcase
    end
    
endmodule