/////////////////////////////////////////////////////////////
//                                                         //
// School of Software of SJTU                              //
// anthor by Hanbing                                       //
//                                                         //
/////////////////////////////////////////////////////////////

module sc_computer (resetn,mem_clk,
		,out_port0,out_port1,out_port2,
		in_port0,in_port1,in_port2,
		hex0, hex1, hex2, hex3, hex4, hex5);
   
    input resetn,mem_clk;
    input [3:0] in_port0, in_port1;
    input in_port2;

    output [6:0] hex0, hex1, hex2, hex3, hex4, hex5;
    output [31:0] out_port0, out_port1, out_port2;

    wire [31:0] pc,inst,aluout,memout,mem_dataout,io_read_data, data;
    wire        imem_clk,dmem_clk,clock,wmem; // all these "wire"s are used to connect or interface the cpu,dmem,imem and so on.
   
	  generate_clock clock_50M (mem_clk, clock);

    sc_cpu cpu (clock,resetn,inst,memout,pc,wmem,aluout,data);          // CPU module.

    sc_instmem  imem (pc,inst,clock,mem_clk,imem_clk);                  // instruction memory.

    sc_datamem_io dmem(aluout,data,memout,wmem,clock,mem_clk,dmem_clk,resetn,
        out_port0,out_port1,out_port2,in_port0,in_port1,in_port2,mem_dataout,io_read_data);

    sc_display show(clock, out_port0, out_port1, out_port2, hex0,hex1,hex2,hex3,hex4,hex5);
endmodule

module generate_clock(mem_clk, clock);

input mem_clk;
output clock;
reg clock;

initial
begin
  clock = 0;
end

always @(posedge mem_clk)
begin
	clock = ~clock;
end
endmodule

