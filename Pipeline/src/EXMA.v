`timescale 1ns/1ps
module EXMA (
    input clk,
    input [31:0] NPC3in,
    output reg [31:0] NPC3out,
    input [31:0] NPC2in,
    output reg [31:0] NPC2out,
    input Flagin,
    input [31:0] Rtin,
    input [31:0] ALUOutin,
    input [31:0] IRin,
    output reg Flagout,
    output reg [31:0] Rtout,
    output reg [31:0] ALUOutout,
    output reg [31:0] IRout
);
    reg [31:0] NPC3;
    reg [31:0] NPC2;
    reg [31:0] Rt;
    reg [31:0] ALUOut;
    reg [31:0] IR;
    reg Flag;
    always @(posedge clk) begin
        NPC3 <= NPC3in;
        NPC3out <= NPC3;
        NPC2 <= NPC2in;
        NPC2out <= NPC2;
        Rt <= Rtin;
        Rtout <= Rt;
        ALUOut <= ALUOutin;
        ALUOutout <= ALUOut;
        IR <= IRin;
        IRout <= IR;
        Flag <= Flagin;
        Flagout <= Flag;
    end
endmodule