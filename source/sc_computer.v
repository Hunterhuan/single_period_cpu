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
    
    //设定input，resetn：重置信号，mem_clk：memory_clock;
    input resetn,mem_clk;
    //设定三个input_port，两个4位二进制，一个控制位，控制
    input [3:0] in_port0, in_port1;
    input in_port2;

    //hex0-hex5，为6个数码显像管,输出结果。
    output [6:0] hex0, hex1, hex2, hex3, hex4, hex5;
    //三个输出端口。
    output [31:0] out_port0, out_port1, out_port2;

    wire [31:0] pc,inst,aluout,memout,mem_dataout,io_read_data, data;
    wire        imem_clk,dmem_clk,clock,wmem; // all these "wire"s are used to connect or interface the cpu,dmem,imem and so on.
    
    //由memory_clock, 生成系统时钟clock
	generate_clock clock_50M (mem_clk, clock);

    //调用cpu模块，（系统时钟，重置信号，指令，memory的输出，PC，写信号.....）
    sc_cpu cpu (clock,resetn,inst,memout,pc,wmem,aluout,data);          // CPU module.

    //调用sc_instruction_meomory,随着clock将instruction提取出来
    sc_instmem  imem (pc,inst,clock,mem_clk,imem_clk);                  // instruction memory.

    //调用i/O 模块
    sc_datamem_io dmem(aluout,data,memout,wmem,clock,mem_clk,dmem_clk,resetn,
        out_port0,out_port1,out_port2,in_port0,in_port1,in_port2,mem_dataout,io_read_data);

    //调用数码管，显示出数字
    sc_display show(clock, out_port0, out_port1, out_port2, hex0,hex1,hex2,hex3,hex4,hex5);
endmodule


//由memory_clock, 生成系统时钟clock
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

