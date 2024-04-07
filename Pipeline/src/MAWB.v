`timescale 1ns/1ps
module MAWB (
    input clk,
    input [31:0] MEMOutin,
    input [31:0] ALUOutin,
    input [31:0] IRin,
    output reg [31:0] MEMOutout,
    output reg [31:0] ALUOutout,
    output reg [31:0] IRout
);
    reg [31:0] MEMOut;
    reg [31:0] ALUOut;
    reg [31:0] IR;
    always @(posedge clk ) begin
        MEMOut <= MEMOutin;
        MEMOutout <= MEMOut;
        ALUOut <= ALUOutin;
        ALUOutout <= ALUOut;
        IR <= IRin;
        IRout <= IR;
    end
endmodule