module sc_display(clock, out_port0, out_port1, out_port2, hex0,hex1,hex2,hex3,hex4,hex5);
input clock;
input [31:0] out_port0, out_port1,out_port2;
output [6:0] hex0,hex1,hex2,hex3,hex4,hex5;

reg [31:0] num0, num1, num2, num3, num4, num5;

always @(posedge clock)
begin
	if(out_port0<10)
	begin
		num0 = 0;
		num1 = out_port0;
	end
	else
	begin
		num0 = 1;
		num1 = out_port0-10;
	end

	if(out_port1<10)
	begin
		num2 = 0;
		num3 = out_port1;
	end
	else
	begin
		num2 = 1;
		num3 = out_port1-10;
	end

	
	num4 = out_port2/10;
	num5 = out_port2 - num4*10;

end

sevenseg display_0_high (num0, hex0);
sevenseg display_0_low (num1, hex1);
sevenseg display_1_high (num2, hex4);
sevenseg display_1_low (num3, hex5);
sevenseg display_2_high (num4, hex2);
sevenseg display_2_low (num5, hex3);

endmodule