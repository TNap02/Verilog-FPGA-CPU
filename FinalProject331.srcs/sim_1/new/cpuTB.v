`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Tristan Napier
// 
// Create Date: 12/08/2023 05:22:53 PM
// Design Name: 
// Module Name: cpuTB
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


module cpuTB();
    reg clk;
     wire mwreg, mm2reg;
     wire [4:0] mdestReg;
     wire [31:0] mr, mdo;
    //MEMWB outputs
     wire wwreg, wm2reg;
     wire [4:0] wdestReg;
     wire [31:0] wr, mqb, wdo;
    wire [31:0] r;

    
    //lab 5 new inputs
     wire [31:0] wbData;
     wire [31:0] qa, qb;
    
    //make datapath
    Datapath datapath( clk, mwreg, mm2reg, mdestReg, mr, mqb, mdo, wwreg, wm2reg, wdestReg, wr, wdo, r, wbData, qa, qb);
    
    initial begin
    //start clk at 0
    clk = 0;
    end
    always 
    //every 100 ns toggle clock
       #100 clk = ! clk;
endmodule
