`timescale 1ns/100ps

module TSME_tb;

parameter H=320;
parameter V=240;
parameter S=10; // logH
parameter blocksize=8;
parameter NP=6; // log(blocksize**2);
parameter BH=H/blocksize;
parameter BV=V/blocksize;
parameter N=12;

reg clk;
reg rst;
wire [N*BH*BV-1:0] vector_x;
wire [N*BH*BV-1:0] vector_y;

TSME tsme(.clk(clk), .rst(rst), .vector_x(vector_x), .vector_y(vector_y));

always begin #5 clk=1'b0; #5 clk=1'b1; end
initial begin rst=1'b0; #20 rst=1'b1; #38 rst=1'b0; end

endmodule 


