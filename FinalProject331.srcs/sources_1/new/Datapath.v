`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Tristan Napier
// 
// Create Date: 12/08/2023 02:31:12 PM
// Design Name: 
// Module Name: Datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Datapath(
    input clk, 
    
    
    //new lab 4 outputs
    output wire mwreg, mm2reg,
    output wire [4:0] mdestReg,
    output wire [31:0] mr, mqb, mdo,
    //MEMWB outputs
    output wire wwreg, wm2reg,
    output wire [4:0] wdestReg,
    output wire [31:0] wr, wdo,
    output wire [31:0] r,

    
    //lab 5 new inputs
    output wire [31:0] wbData,
    output wire [31:0] qa, qb
);

 wire [31:0] pc, dinstOut;
 wire ewreg, em2reg, ewmem;
     wire [3:0] ealuc;
     wire ealuimm;
     wire [4:0] edestReg;
     wire [31:0] eqa, eqb, eimm32;

//extra needed wires
    wire [31:0] nextPC;
    wire [31:0] instrOut;
    wire wreg, m2regm, wmem, aluimm, regrt;
    wire [3:0] aluc;
    wire [4:0] destReg;
    wire [31:0] imm32;
    wire [31:0] b;
    //wire [31:0] r;
    wire [1:0] fwda, fwdb;
    wire [31:0] fa, fb;

//wire all modules together
    PC pc1(clk, nextPC, pc);
    pcAdd pcadd(pc, nextPC);
    insMem insmem( pc, instrOut);
    IFID ifid(clk, instrOut, dinstOut);
    ctrUnit ctrunit(dinstOut[31:26], dinstOut[5:0], dinstOut[25:21], dinstOut[20:16], mdestReg, mm2Reg, mwreg, edestReg, em22reg, ewreg, wreg, m2reg, wmem, aluimm, regrt, aluc, fwda, fwdb);
    regrtMul regrtmul(regrt, dinstOut[20:16], dinstOut[15:11], destReg);
    RegFile regfile(dinstOut[25:21], dinstOut[20:16], qa, qb, wdestReg, wbData, wwreg, clk);

    fwdaMul fwdamul(fwda, qa, r, mr, mdo, fa);
    fwdbMul fwdbmul(fwdb, qb, r, mr, mdo, fb);
    immExt immext(dinstOut[15:0], imm32);
    IDEXE idexe(clk, wreg, m2reg, wmem, aluimm, aluc, fa, fb, destReg, imm32, ewreg, em2reg, ewmem, ealuimm, ealuc, eqa, eqb, edestReg, eimm32);
    AluMux alumux(eqb, eimm32, ealuimm, b);
    Alu alu(eqa, b, ealuc, r);
    EXEMEM exemem(ewreg, em2reg, ewmem, edestReg, r, eqb, clk, mwreg, mm2reg, mwmem, mdestReg, mr, mqb);
    DataMem datamem(mr, mqb, mwmem, clk, mdo);
    MEMWB memwb(mwreg, mm2reg, mdestReg, mr, mdo, clk, wwreg, wm2reg, wdestReg, wr, wdo);
    WbMux wbmux(wr, wdo, wm2reg, wbData);
    

endmodule

module PC(
    input clk,
    input [31:0] nextPc,
    output reg [31:0] pc
    );
    initial begin
    //sets pc to start at 100
    pc = 100;
    end

    always@(posedge clk) begin
        pc = nextPc;
    end
endmodule

module insMem(
 input [31:0] pc,
    output reg [31:0] instOut
    );

    reg [31:0] memory[0:63];
    
    initial begin 
    memory[25] ={
    6'b000000,
    5'b00000,
    5'b00001,
    5'b00010,
    5'b00000,
    6'b100000
    };
    memory[26] ={
    6'b000000,
    5'b01000,
    5'b00010,
    5'b00011,
    5'b00000,
    6'b100010
    };
    memory[27] ={
    6'b000000,
    5'b00010,
    5'b01000,
    5'b00100,
    5'b00000,
    6'b100101
    };
    memory[28] ={
    6'b000000,
    5'b00010,
    5'b01000,
    5'b00101,
    5'b00000,
    6'b100110
    };
    memory[29] ={
    6'b000000,
    5'b00010,
    5'b01000,
    5'b00110,
    5'b00000,
    6'b100100
    };
    end
    
    always@(pc) begin
    instOut = memory[pc[7:2]];
    end
endmodule

module pcAdd(
    input [31:0] pc,
    output reg [31:0] nextPc);

        always@(pc) begin
            nextPc = pc + 4;
        end
endmodule

module IFID(
 input clk, 
    input [31:0] instOut,//input new PC reg instructions
    output reg [31:0] dinstOut
    );

    always@(posedge clk) begin
        dinstOut = instOut;
    end
    
endmodule

module ctrUnit(

    //left side input
    input [5:0] op, 
    input [5:0] func,
    input [4:0] rs, rt,
    //right side input
    input [4:0] mdestReg,
    input mm2reg, mwreg,
    input [4:0] edestReg,
    input em2reg, ewreg,
    output reg wreg, m2reg, wmem, aluimm, regrt,
    output reg [3:0] aluc,
    output reg [1:0] fwda, fwdb
    );

    always@(op,func,rs,rt) begin
    
    fwda = 2'b00;
        fwdb = 2'b00;

    
    if(ewreg && (edestReg != 0) && (edestReg == rs)) begin
        fwda = 2'b01;
    end
    if(ewreg && (edestReg != 0) && (edestReg == rt)) begin
        fwdb = 2'b01;
    end
    if(mwreg && (mdestReg != 0) && (mdestReg == rs)) begin
        fwda = 2'b10;
    end
    if(mwreg && (mdestReg != 0) && (mdestReg == rt)) begin
        fwdb = 2'b10;
    end

    //curently only have case of load word
        case(op)
            6'b100011: begin //Load Word
                aluimm = 1;
                aluc = 4'b0010;
                wreg = 1;
                m2reg = 1;
                wmem = 0;
                regrt = 1;
            end
            6'b000000: begin
                case(func)
                    6'b100000: begin 
                        aluc = 4'b0010;
                        aluimm = 0;
                        wreg = 1;
                        m2reg = 0;
                        wmem = 0;
                        regrt = 0;
                    end
                    6'b100010: begin
                        aluc = 4'b0110;
                        aluimm = 0;
                        wreg = 1;
                        m2reg = 0;
                        wmem = 0;
                        regrt = 0;
                    end
                    6'b100101: begin
                        aluc = 4'b0001;
                        aluimm = 0;
                        wreg = 1;
                        m2reg = 0;
                        wmem = 0;
                        regrt = 0;
                    end
                    6'b100100: begin
                        aluc = 4'b0000;
                        aluimm = 0;
                        wreg = 1;
                        m2reg = 0;
                        wmem = 0;
                        regrt = 0;
                    end
                    6'b100110: begin
                        aluc = 4'b1001;
                        aluimm = 0;
                        wreg = 1;
                        m2reg = 0;
                        wmem = 0;
                        regrt = 0;
                    end
                endcase
                end
            default: begin
                aluc = 4'b0010;
                aluimm = 0;
                wreg = 1;
                m2reg = 0;
                wmem = 0;
                regrt = 0;
            end
        endcase
    end
endmodule

module regrtMul( 
    input regrt,
    input [4:0] rt,
    input [4:0] rd, 
    output [4:0] destReg
    ); 

        assign destReg = regrt?rt:rd; //if regrt = 1, set to rt, else set to rd
    
endmodule

module fwdaMul(
    input [1:0] fwda,
    input [31:0] qa, r, mr, mdo,
    output reg [31:0] fa
    );
    
    always@(fwda, r, mr, qa) begin
    
    case(fwda)
        2'b00: begin
            fa = qa;
        end
        2'b01: begin
            fa = r;
        end
        2'b10: begin
            fa = mr;
        end
        2'b11: begin
            fa = mdo;
        end
        
    endcase
    
    end
    
endmodule

module fwdbMul(
    input [1:0] fwdb,
    input [31:0] qb, r, mr, mdo,
    output reg [31:0] fb
    );
    
    always@(fwdb, r, mr, qb) begin

    case(fwdb)
        2'b00: begin
            fb = qb;
        end
        2'b01: begin
            fb = r;
        end
        2'b10: begin
            fb = mr;
        end
        2'b11: begin
            fb = mdo;
        end
        
    endcase
    
    end
    
endmodule
    
module immExt(
    input [15:0] imm,
    output reg [31:0] imm32
    );

    always@(imm) begin
        
        //if last digit is 1 extend next 16 bits to 1
        if(imm[15] == 1) begin
            imm32[31:16] = 16'hffff;
            imm32[15:0] = imm;
            end
        //else just extend imm to 32 bits
        else begin
            imm32[31:16] = 16'h0000;
            imm32[15:0] = imm;
        end
    end
endmodule


module IDEXE(
    input clk,wreg, m2reg, wmem,aluimm,
    input [3:0] aluc,
    input [31:0] qa, qb,
    input [4:0] destReg,
    input [31:0] imm32,
    output reg ewreg, em2reg, ewmem, ealuimm,
    output reg [3:0] ealuc,
    output reg [31:0] eqa, eqb,
    output reg [4:0] edestReg,
    output reg [31:0] eimm32
    );  

//store values for next cycle
    always@(posedge clk) begin
        ewreg = wreg;
        em2reg = m2reg;
        ewmem = wmem;
        ealuc = aluc;
        ealuimm = aluimm;
        edestReg = destReg;
        eqa = qa;
        eqb = qb;
        eimm32 = imm32;
    end
endmodule


module AluMux(
    input [31:0] eqb, eimm32,
    input ealumimm,
    output reg [31:0] b );
    
    always@(ealumimm, eqb, eimm32) begin
        case (ealumimm)
            1: begin
                b = eimm32;
                end
            default: begin
                b = eqb;
                end
            endcase
        end
endmodule

module Alu(
    input [31:0] eqa, b,
    input [3:0] ealuc,
    output reg [31:0] r);
    
    always@(*) begin

        case (ealuc)
            4'b0010: begin
                r = eqa + b;
            end
            4'b0110: begin
                r = eqa - b;
            end
            4'b0001: begin
                r = eqa | b;
            end
            4'b0000: begin
                r = eqa & b;
            end
            4'b1001: begin
                r = eqa ^ b;
            end                
            default: begin
            //not adding any additiaonal things yet
                r = 0;
            end
        endcase
    end
endmodule
            
module EXEMEM(
    input ewreg, em2reg, ewmem,
    input [4:0] edestReg,
    input [31:0] r, eqb,
    input clk,
    output reg mwreg, mm2reg, mwmem,
    output reg [4:0] mdestReg,
    output reg [31:0] mr, mqb);

    always@(posedge clk) begin
        mwreg = ewreg;
        mm2reg = em2reg;
        mwmem = ewmem;
        mdestReg = edestReg;
        mr = r;
        mqb = eqb;
    end
endmodule

module DataMem(
    input [31:0] mr, mqb,
    input mwmem, clk,
    output reg [31:0] mdo);
    
    reg [31:0] dataMem [0:63];
    
    initial begin
//        dataMem[0] = 32'hA00000AA;
//        dataMem[4] = 32'h10000011;
//        dataMem[8] = 32'h20000022;
//        dataMem[12] = 32'h30000033;
//        dataMem[16] = 32'h40000044;
//        dataMem[20] = 32'h50000055;
//        dataMem[24] = 32'h60000066;
//        dataMem[28] = 32'h70000077;
//        dataMem[32] = 32'h80000088;
//        dataMem[36] = 32'h90000099;
        end
        
        always@(mr, mqb, mwmem, clk) begin
            mdo = dataMem[mr];
        end
        always@(negedge clk) begin
            if (mwmem == 1) begin
                dataMem[mr] = mqb;
            end
        end
endmodule
    
module MEMWB(
    input mwreg, mm2reg,
    input[4:0] mdestReg,
    input [31:0] mr, mdo,
    input clk,
    output reg wwreg, wm2reg,
    output reg [4:0] wdestReg,
    output reg [31:0] wr, wdo);
    
    always@(posedge clk) begin
        wwreg = mwreg;
        wm2reg = mm2reg;
        wdestReg = mdestReg;
        wr = mr;
        wdo = mdo;
    end
endmodule

module WbMux(
    input [31:0] wr, wdo,
    input wm2reg,
    output reg [31:0] wbData);
    
    always@(wr, wdo, wm2reg) begin
        if (wm2reg == 0) begin
            wbData = wr;
           end
       else begin
            wbData = wdo;
        end
    end
endmodule

module RegFile(

    input [4:0] rs, 
    input [4:0] rt, 
    output reg [31:0] qa, qb,
    //lab 5 new inputs
    input [4:0] wdestReg,
    input [31:0] wbData,
    input wwreg, clk
    );
    reg [31:0] registers [0:31];
    //set all registers to 0
    initial begin                   
        registers[0] = 32'h00000000;
        registers[1] = 32'hA00000AA;
        registers[2] = 32'h10000011;
        registers[3] = 32'h20000022;
        registers[4] = 32'h30000033;
        registers[5] = 32'h40000044;
        registers[6] = 32'h50000055;
        registers[7] = 32'h60000066;  
        registers[8] = 32'h70000077;
        registers[9] = 32'h80000088;
        registers[10] = 32'h90000099;
        registers[11] = 0;
        registers[12] = 0;
        registers[13] = 0;
        registers[14] = 0;
        registers[15] = 0;       
    end
    always@(negedge clk) begin
         qa = registers[rs];   //grabs value from register file at rs/rt
         qb = registers[rt];
    end 
    //lab 5
    always@(posedge clk) begin
        if (wwreg) begin
            registers[wdestReg] = wbData;
        end
    end
endmodule
