`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2025 09:01:22 PM
// Design Name: 
// Module Name: whole_system_tb
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


module whole_system_tb();

reg clk, rst, start; 
reg [2:0] rom_addr1, rom_addr2, ram_addr;
wire done;
wire [7:0]data;

whole_system dut(.clk(clk), .rst(rst), .start(start), .rom_addr1(rom_addr1),
                 .rom_addr2(rom_addr2), .ram_addr(ram_addr), 
                 .done(done), .data(data));
                              
initial begin
    clk = 0;
    rst = 1;
    start = 0;
    rom_addr1 = 0;
    rom_addr2 = 0;
    ram_addr = 0;
    #20 rst = 0;
end
always #5 clk = ~clk;

//drag signals from "Scope" into waveform to add

initial begin
    #20 rom_addr1 = 3; rom_addr2 = 6; ram_addr = 2;
    #10 start = 1; #10 start = 0;
    @(posedge done);   //wait for done signal
    rom_addr1 = 1; rom_addr2 = 2; ram_addr = 3;
    #15 start = 1; #10 start = 0;
    @(posedge done);
    rom_addr1 = 0; rom_addr2 = 7; ram_addr = 5;
    #15 start = 1; #10 start = 0;
    @(posedge done);
    #70 $finish;    
end
endmodule
