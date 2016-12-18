`timescale 1ns/100ps

module DSME(clk, rst, vector_x, vector_y);

parameter H=320;
parameter V=240;
parameter S=10; // logH
parameter blocksize=8;
parameter NP=6; // log(blocksize**2);
parameter BH=H/blocksize;
parameter BV=V/blocksize;
parameter N=12;
parameter M=4; // N/3

input clk, rst;
wire [N*H*V-1:0] in_present, in_next;
output reg [N*BH*BV-1:0] vector_x;
output reg [N*BH*BV-1:0] vector_y;

wire [N-1:0] bi_present [H*V-1:0];
wire [N-1:0] bi_next [H*V-1:0];
wire [N-1:0] present[H+blocksize-1:0][V+blocksize-1:0]; 
wire [N-1:0] next[H+blocksize-1:0][V+blocksize-1:0]; 
reg [N-1:0] vector_multi[BH-1:0][BV-1:0];
reg [N-1:0] temp_vector_x[BH*BV-1:0];
reg [N-1:0] temp_vector_y[BH*BV-1:0];
reg [N-1:0] current_in[H*V-1:0];
reg [N-1:0] next_in[H*V-1:0];

///////Data In////////
initial begin
	$readmemh ("out32.txt", current_in); // 4bit per 1segment
	$readmemh ("out42.txt", next_in);
end
genvar i, j, n;
generate for(i=H-1; i>=0; i=i-1) begin : G_i
	for(j=V-1; j>=0; j=j-1) begin : G_j
		for(n=N-1; n>=0; n=n-1) begin : G_n
			assign in_present[((H*j*N) + (i*N))+n] = current_in[((H*j) + (i))][n];
			assign in_next[((H*j*N) + (i*N))+n] = next_in[((H*j) + (i))][n];
		end
	end
end endgenerate

///////from 3 to 1 binary////////
genvar c, d;
generate for(c=0; c<H; c=c+1) begin : HV_c
   for(d=0; d<V; d=d+1) begin : HV_d
   	assign bi_present[c*V+d] = 1*in_present[(c*V+d)*N+M-1 : (c*V+d)*N] + 10*in_present[(c*V+d)*N+2*M-1 : (c*V+d)*N+M] + 100*in_present[(c*V+d)*N+3*M-1 : (c*V+d)*N+2*M];
   	assign bi_next[(c*V+d)] = 1*in_next[(c*V+d)*N+M-1 : (c*V+d)*N] + 10*in_next[(c*V+d)*N+2*M-1 : (c*V+d)*N+M] + 100*in_next[(c*V+d)*N+3*M-1 : (c*V+d)*N+2*M];
   end
end endgenerate

///////from 1D to 2D////////
genvar a, b;
generate for(a=0; a<H+blocksize; a=a+1) begin : x_axis
   for(b=0; b<V+blocksize; b=b+1) begin : y_axis	
	if( (a < blocksize/2) || (a > H+blocksize/2-1) || (b < blocksize/2) || (b > V+blocksize/2-1) ) begin
	   assign present[a][b] = {N{1'b1}};
	   assign next[a][b] = {N{1'b1}};
	end
	else begin
	   assign present[a][b] = bi_present[a-blocksize/2+(b-blocksize/2)*H];
 	   assign next[a][b] = bi_next[a-blocksize/2+(b-blocksize/2)*H];
	end
   end
end endgenerate


///////useful registers///////
reg finish;
reg [20:0] cnt;
reg [20:0] sum, temp;
reg [10:0] start_x, start_y;
reg [10:0] present_x, present_y;
reg [10:0] next_x, next_y;
reg [1:0] ps, ns;
reg f_finish, s_finish;
//reg [10:0] temp_fi, temp_fj;
//reg [10:0] temp_si, temp_sj;

localparam IDLE = 2'b00, LOAD = 2'b01, FIRST = 2'b10, SECOND = 2'b11;

///////update state///////
always @(posedge clk) begin
   if(rst) ps <= IDLE;
   else ps <= ns;
end
///////determine state///////
always @(*) begin
   case(ps)
	IDLE : if(finish) ns <= IDLE; else ns <= LOAD;
	LOAD : ns <= FIRST;
	FIRST : if(f_finish) ns <= SECOND; else ns <= FIRST;
	SECOND : if(finish) ns <= IDLE; else if(s_finish) ns <= LOAD; else ns <= SECOND;
   endcase
end
///////determine RTL///////
always @(posedge clk) begin
   if(rst) begin
	finish <= 0;
   end
   case(ps)
	IDLE :  cnt <= 0;
	LOAD :  begin 
		   sum <= 0; temp <= 0; f_finish <= 0; s_finish <= 0;  
		   start_x <= ((cnt-1)%BH)*blocksize+(blocksize/2);
		   start_y <= ((cnt-1)/BH)*blocksize+(blocksize/2);
		   present_x <= ((cnt-1)%BH)*blocksize+(blocksize/2);
		   present_y <= ((cnt-1)/BH)*blocksize+(blocksize/2);
		   cnt = cnt + 1;
		end
	FIRST : if(!f_finish) begin
		 	first_step(present_x, present_y);
		end
		else ;
	SECOND : if(s_finish == 1) begin
	   		temp_vector_x[cnt-1] <= next_x-start_x;
   	   		temp_vector_y[cnt-1] <= next_y-start_y;
	   		if(cnt == BH*BV) finish <= 1'b1;
		 end
		 else begin
			second_step(present_x, present_y);
		 end
   endcase
end

///////ABS calculation///////
function [N-1:0]abs(input [N-1:0]a, input [N-1:0]b);
   begin
	if(a>b) abs=a-b;
	else abs=b-a;
   end
endfunction






///////SAD calculation///////
function [N+NP-1:0]SAD(input [S-1:0] px, input [S-1:0] py, input [S-1:0] nx, input [S-1:0] ny);
   integer sx, sy;
   begin
   	SAD = 0;
  	for(sx=0; sx<blocksize; sx=sx+1) begin
	   for(sy=0; sy<blocksize; sy=sy+1) begin
	   	SAD = SAD + abs(present[px+sx][py+sy], next[nx+sx][ny+sy]);
	   end
   	end
   end 
endfunction

///////first step///////
task first_step(input [S-1:0] fpx, input [S-1:0] fpy);
   begin
	sum = SAD(start_x, start_y, fpx, fpy); next_x = fpx; next_y = fpy;
	f_finish = 1'b1;
	temp = SAD(start_x, start_y, fpx+2, fpy); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx+2; next_y = fpy;   f_finish = 0; end
	temp = SAD(start_x, start_y, fpx+1, fpy+1);	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx+1; next_y = fpy+1; f_finish = 0; end
	temp = SAD(start_x, start_y, fpx, fpy+2); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx;   next_y = fpy+2; f_finish = 0; end
	temp = SAD(start_x, start_y, fpx-1, fpy+1); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx-1; next_y = fpy+1; f_finish = 0; end
	temp = SAD(start_x, start_y, fpx-2, fpy); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx-2; next_y = fpy;   f_finish = 0; end
	temp = SAD(start_x, start_y, fpx-1, fpy-1); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx-1; next_y = fpy-1; f_finish = 0; end
	temp = SAD(start_x, start_y, fpx, fpy-2); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx;   next_y = fpy-2; f_finish = 0; end
	temp = SAD(start_x, start_y, fpx+1, fpy-1); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = fpx+1; next_y = fpy-1; f_finish = 0; end
	
	present_x = next_x; present_y = next_y;
   end
endtask
///////second step///////
task second_step(input [S-1:0] spx, input [S-1:0] spy);
   begin
	sum = SAD(start_x, start_y, spx, spy); next_x = spx; next_y = spy;
	s_finish = 1'b1;
	temp = SAD(start_x, start_y, spx+1, spy); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = spx+1; next_y = spy;   s_finish = 1'b0; end
	temp = SAD(start_x, start_y, spx, spy+1);	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = spx;   next_y = spy+1; s_finish = 1'b0; end
	temp = SAD(start_x, start_y, spx-1, spy); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = spx-1; next_y = spy;   s_finish = 1'b0; end
	temp = SAD(start_x, start_y, spx, spy-1); 	if( (temp < sum)  && (sum-temp>128) ) begin sum = temp; next_x = spx;   next_y = spy-1; s_finish = 1'b0; end
	
	present_x = next_x; present_y = next_y;
   end
endtask

/*
integer ii,jj,f;
initial begin
 #150000 f=$fopen("output.txt");
end
initial begin
#150000
for(ii=0; ii<30; ii=ii+1) begin
for(jj=0; jj<40; jj=jj+1) begin
$fwrite(f,"%d ", $signed(temp_vector_y[ii*40+jj]));
end
$fwrite(f,"\n");
end
end
*/

endmodule