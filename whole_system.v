`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2025 09:00:27 PM
// Design Name: 
// Module Name: whole_system
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


//top module
module whole_system(input clk, rst, start, input [2:0] rom_addr1, rom_addr2, 
                    ram_addr, output done, output reg [7:0]data);

wire [2:0] rom_addr; 
wire reg_w_en, ram_w_en, reg_w_addr;    //register file
wire reg_r_addr1, reg_r_addr2;
wire [3:0] rom_data, multiplicand, operand; //multiplier
wire [7:0] product;
wire [7:0] current_val;   //for holding result until next update


register_file file(.clk(clk), .rst(rst), .w_en(reg_w_en), .w_addr(reg_w_addr), 
                   .r_addr1(reg_r_addr1), .r_addr2(reg_r_addr2), .w_data(rom_data), 
                   .r_data1(multiplicand), .r_data2(operand));
                   
RAM8bit ram(.clk(clk), .rst(rst), .write_en(ram_w_en), .address(ram_addr), 
            .w_data(product), .r_data(current_val));    //using current_val

ROM4bit rom(.address(rom_addr), .data(rom_data));

multiplier_comb multi(.a(multiplicand), .b(operand), .p(product));

controller fsm(
    .clk(clk),
    .rst(rst),
    .start(start),
    .rom_addr1(rom_addr1),
    .rom_addr2(rom_addr2),
    .rom_addr(rom_addr),
    .ram_addr(ram_addr),
    .reg_w_en(reg_w_en),
    .ram_w_en(ram_w_en),
    .reg_w_addr(reg_w_addr),
    .reg_r_addr1(reg_r_addr1),
    .reg_r_addr2(reg_r_addr2),
    .done(done));
    
always @(posedge clk) begin
    if(rst)
        data <= 0;
    else if(done)
        data <= current_val;    //hold output until next done signal
end

endmodule


//FSM
module controller(
    input clk, rst, start,
    input [2:0] rom_addr1, rom_addr2, ram_addr,
    output reg reg_w_en, reg_w_addr, ram_w_en,
    output reg reg_r_addr1, reg_r_addr2,
    output reg [2:0] rom_addr,
    output reg done
);

reg [3:0]PS,NS;

parameter IDLE = 3'b000;
parameter GET_A = 3'b001;
parameter GET_B = 3'b010;
parameter MULTIPLY = 3'b011;
parameter WRITE = 3'b100;
parameter READ = 3'b101;
parameter DONE = 3'b110;

//present state updates
always @(posedge clk) begin
    if(rst) 
        PS <= IDLE;
    else
        PS <= NS;
end

//next state updates
always @(*) begin
    case(PS)
        IDLE: NS = start ? GET_A : IDLE;    //only continue when start = 1
        GET_A: NS = GET_B;
        GET_B: NS = MULTIPLY;
        MULTIPLY: NS = WRITE;
        WRITE: NS = READ;
        READ: NS = DONE;
        DONE: NS = IDLE; //loop for now
        default: NS = IDLE;
    endcase
end

always @(*) begin
    case(PS)
        IDLE: begin
            reg_w_en = 0;
            reg_w_addr = 0; 
            ram_w_en = 0;
            reg_r_addr1 = 0;
            reg_r_addr2 = 1;
            rom_addr = 0;
            done = 0;
        end
        GET_A: begin
            reg_w_en = 1;
            reg_w_addr = 0; 
            rom_addr = rom_addr1;
        end
        GET_B: begin
            reg_w_en = 1;
            reg_w_addr = 1; 
            rom_addr = rom_addr2;
        end
        MULTIPLY: begin
            reg_w_en = 0;
            ram_w_en = 0;       
        end
        WRITE: begin
            ram_w_en = 1;
        end
        READ: begin
            ram_w_en = 0;
        end
        DONE: begin
            done = 1;
        end
    endcase
end
endmodule


//register file to store ROM values (2 registers)
module register_file(input clk, rst, w_en, w_addr, r_addr1, 
                     r_addr2, input [3:0]w_data, output reg [3:0]r_data1, 
                     r_data2);

reg[3:0] mem[0:1];  //2 4-bit memory addresses

always @(posedge clk) begin
    if(rst) begin
        mem[0] <= 0;
        mem[1] <= 0;
    end else begin
        if(w_en)
            mem[w_addr] <= w_data;
    end
end

always @(*) begin
    r_data1 = mem[r_addr1];
    r_data2 = mem[r_addr2];
end
endmodule


//8 bit data width, 8 different addresses
module RAM8bit(clk, rst, write_en, address, w_data, r_data);

parameter DATA_W = 8;
parameter ADDR_W = 3;
parameter SIZE = 8;   //2^ADDR_W = 8

input clk,rst,write_en;
input [ADDR_W-1:0]address;
input [DATA_W-1:0]w_data;
output reg [DATA_W-1:0]r_data;

reg[DATA_W-1:0] mem[0:SIZE-1];  //8 8-bit memory addresses
integer i;

always @(posedge clk) begin     
    if(rst) begin
        for(i=0; i<SIZE; i=i+1) 
            mem[i] <= 0;
    end else begin
        if(write_en)
            mem[address] <= w_data;
        else
            r_data <= mem[address];  //synchronous reads
    end
end
endmodule


//4 bit data width, 8 different addressses
module ROM4bit(input [2:0]address, output reg [3:0]data);

always @(*) begin
    case(address)
        3'd0: data = 4'b1101;
        3'd1: data = 4'b0010;
        3'd2: data = 4'b1001;
        3'd3: data = 4'b1010;
        3'd4: data = 4'b1111;
        3'd5: data = 4'b1000;
        3'd6: data = 4'b0111;
        3'd7: data = 4'b1011;
    endcase
end
endmodule


//multiplier
module multiplier_comb(input [3:0]a, b, output [7:0]p);

    wire [3:0]pp0;
    wire [4:0]pp1;
    wire [5:0]pp2;
    wire [6:0]pp3;
    
    assign pp0 = b[0] ? a : 0;  //partial product 0 is a when b[0] = 1, 0 else
    assign pp1 = b[1] ? (a << 1) : 0; //partial product 1 is (a << 1) when b[1] = 1, 0 else
    assign pp2 = b[2] ? (a << 2) : 0; //partial product 2 is (a << 2) when b[2] = 1, 0 else
    assign pp3 = b[3] ? (a << 3) : 0; //partial product 3 is (a << 3) when b[3] = 1, 0 else
    assign p = pp0 + pp1 + pp2 + pp3; //sum partial products

endmodule
