`timescale 1ns/1ps
module IDEX (
    input clk,
    input [31:0] NPC1in,
    output reg [31:0] NPC1out,
    input [31:0] Rsin,
    input [31:0] Rtin,
    input [31:0] Imm32in,
    input [31:0] IRin,
    output reg [31:0] Rsout,
    output reg [31:0] Rtout,
    output reg [31:0] Imm32out,
    output reg [31:0] IRout
);
    reg [31:0] NPC1;
    reg [31:0] Rs;
    reg [31:0] Rt;
    reg [31:0] Imm32;
    reg [31:0] IR;
    always @(posedge clk ) begin
        NPC1 <= NPC1in;
        NPC1out <= NPC1;
        Rs <= Rsin;
        Rsout <= Rs;
        Rt <= Rtin;
        Rtout <= Rt;
        Imm32 <= Imm32in;
        Imm32out <= Imm32;
        IR <= IRin;
        IRout <= IR;
    end
endmodule