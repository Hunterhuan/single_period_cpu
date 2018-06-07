module sc_datamem_io (addr,datain,dataout,we,clock,mem_clk,dmem_clk,clrn,
out_port0,out_port1, out_port2, in_port0,in_port1,in_port2,mem_dataout,io_read_data);

	input [31:0] addr;
	input [31:0] datain;
	input we, clock,mem_clk,clrn;
	//三个输入端口
	input [3:0] in_port0,in_port1;
	input in_port2;

	output [31:0] dataout;
	output dmem_clk;
	//三个输出端口
	output [31:0] out_port0,out_port1, out_port2;
	output [31:0] mem_dataout;
	output [31:0] io_read_data;

	wire [31:0] dataout;
	wire dmem_clk;
	wire write_enable;
	wire wrire_datamem_enable;
	wire [31:0] mem_dataout;

	//计算I/O的相关控制信号
	assign dmem_clk = mem_clk & ( ~ clock) ; //注意
	assign write_enable = we & ~clock; //注意
	assign write_datamem_enable = write_enable & ( ~ addr[7]); //注意
	assign write_io_output_reg_enable = write_enable & ( addr[7]); //注意

	mux2x32 mem_io_dataout_mux(mem_dataout,io_read_data,addr[7],dataout);

	//将datamem.mif文件中的数据放入寄存器。
	lpm_ram_dq_dram dram(addr[6:2],dmem_clk,datain,write_datamem_enable,mem_dataout);

	//调用io_output_reg模块，每次将output的内容从寄存器中取出来，供display使用。
	io_output_reg io_output_regx2 (addr,datain,write_io_output_reg_enable,dmem_clk,clrn,out_port0,out_port1, out_port2);

	//调用io_input_reg模块，每次将input的内容放入寄存器中，供cpu运算使用
	io_input_reg io_input_regx2(addr,dmem_clk,io_read_data,in_port0,in_port1,in_port2);
endmodule