`timescale 1ns/100ps

module TSME(clk, rst, vector_x, vector_y);

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

// Data In
initial begin
	$readmemh ("out32.txt", current_in); // 4bit per 1segment
	$readmemh ("out42.txt", next_in);
end
genvar i, j, n;
generate for(i=H-1; i>=0; i=i-1) begin : datain_i
	for(j=V-1; j>=0; j=j-1) begin : datain_j
		for(n=N-1; n>=0; n=n-1) begin : datain_n
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
reg [10:0] temp_fi, temp_fj;
reg [10:0] temp_si, temp_sj;
reg [10:0] temp_ti, temp_tj;

///////make counter///////
always @(posedge clk) begin
   if(rst) begin
	cnt <= 0;
	vector_x <= 0;
	vector_y <= 0;
	finish <= 0;
   end
   else begin
	if( cnt == BH*BV ) begin finish <= 1'b1; end
	else cnt <= cnt + 1;	   
   end
end



///////main always block////////
always @(cnt) begin
   if( !rst && (cnt!=0) && (finish==0) ) begin
   sum = 0;
   temp = {N{1'b1}};
   
   temp_fi = ((cnt-1)%BH)*blocksize+(blocksize/2);
   temp_fj = ((cnt-1)/BH)*blocksize+(blocksize/2);
   temp_si = ((cnt-1)%BH)*blocksize+(blocksize/2);
   temp_sj = ((cnt-1)/BH)*blocksize+(blocksize/2);
   temp_ti = ((cnt-1)%BH)*blocksize+(blocksize/2);
   temp_tj = ((cnt-1)/BH)*blocksize+(blocksize/2);

   first_step( ((cnt-1)%BH)*blocksize+(blocksize/2), ((cnt-1)/BH)*blocksize+(blocksize/2) );
   second_step( ((cnt-1)%BH)*blocksize+(blocksize/2), ((cnt-1)/BH)*blocksize+(blocksize/2), temp_fi, temp_fj );
   third_step( ((cnt-1)%BH)*blocksize+(blocksize/2), ((cnt-1)/BH)*blocksize+(blocksize/2), temp_si, temp_sj );

   temp_vector_x[cnt-1] <= temp_ti-(((cnt-1)%BH)*blocksize+(blocksize/2));
   temp_vector_y[cnt-1] <= temp_tj-(((cnt-1)/BH)*blocksize+(blocksize/2));
   end
end


///////from 2D vector to 1D output vector///////
/*
genvar e, f;
generate for(c=0; c<H*V; c=c+1) begin : HV
   bitsum = 0;
   for(d=0; d<N; d=d+1) begin : N
	bitsum = bitsum + (2**(N-1-d))*in_present[d+N*c];
   end
   bi_present[c] = bitsum;
end endgenerate
*/

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
   integer fsi, fsj;
   begin
	temp = SAD(fpx, fpy, fpx, fpy);
	for(fsi=0; fsi<3; fsi = fsi+1) begin
	   for(fsj = 0; fsj<3; fsj = fsj+1) begin
		if( (fsi!=1) || (fsj!= 1) ) begin
		   sum = SAD(fpx, fpy, fpx+(fsi-1)*(blocksize/2), fpy+(fsj-1)*(blocksize/2));
		   if( (temp > sum) && (temp-sum>128) ) begin
			temp = sum;
		   	temp_fi = fpx+(fsi-1)*(blocksize/2);
		   	temp_fj = fpy+(fsj-1)*(blocksize/2);
		   end
		end
	   end
	end
   end
endtask
///////second step///////
task second_step( input [S-1:0] ispx, input [S-1:0] ispy, input [S-1:0] spx, input [S-1:0] spy);
   integer ssi, ssj;
   begin
	temp_si = spx;
	temp_sj = spy;
	for(ssi=0; ssi<3; ssi = ssi+1) begin
	   for(ssj = 0; ssj<3; ssj = ssj+1) begin
		if( (ssi!=1) || (ssj!= 1) ) begin
		   sum = SAD(ispx, ispy, spx+(ssi-1)*(blocksize/4), spy+(ssj-1)*(blocksize/4));
		   if( (temp > sum) && (temp-sum>128) ) begin
		   	temp = sum;
		   	temp_si = spx+(ssi-1)*(blocksize/4);
		   	temp_sj = spy+(ssj-1)*(blocksize/4);
		   end
		end
	   end
	end
   end
endtask
///////second step///////
task third_step( input [S-1:0] itpx, input [S-1:0] itpy, input [S-1:0] tpx, input [S-1:0] tpy);
   integer tsi, tsj;
   begin
	temp_ti = tpx;
	temp_tj = tpy;
	for(tsi=0; tsi<3; tsi = tsi+1) begin
	   for(tsj = 0; tsj<3; tsj = tsj+1) begin
		if( (tsi!=1) || (tsj!= 1) ) begin
		   sum = SAD(itpx, itpy, tpx+(tsi-1)*(blocksize/8), tpy+(tsj-1)*(blocksize/8));
		   if( (temp > sum) && (temp-sum>128) ) begin
		   	temp = sum;
		   	temp_ti = tpx+(tsi-1)*(blocksize/8);
		   	temp_tj = tpy+(tsj-1)*(blocksize/8);
		   end
		end
	   end
	end
   end
endtask

integer ii,jj,f,g;
initial begin
#15000 f=$fopen("output_TSS_x.txt"); g=$fopen("output_TSS_y.txt");
end
initial begin
#15000
for(ii=0; ii<30; ii=ii+1) begin
for(jj=0; jj<40; jj=jj+1) begin
$fwrite(f,"%d ", $signed(temp_vector_x[ii*40+jj]));
$fwrite(g,"%d ", $signed(temp_vector_y[ii*40+jj]));
end
$fwrite(f,"\n");
$fwrite(g,"\n");
end
end


endmodule
