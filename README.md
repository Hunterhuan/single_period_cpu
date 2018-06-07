# 《基本单周期CPU设计》实验报告

姓名：韩冰

学号：516030910523

***

## 1.实验目的

1. 理解计算机 5 大组成部分的协调工作原理，理解存储程序自动执行的原理。    
2. 掌握运算器、存储器、控制器的设计和实现原理。重点掌握控制器设计原理和实现方法。
3. 掌握 I/O 端口的设计方法，理解 I/O 地址空间的设计方法。
4. 会通过设计 I/O 端口与外部设备进行信息交互。

## 2.实验内容

1. 采用 Verilog HDL 在 quartusⅡ中实现基本的具有 20 条 MIPS 指令的单周期 CPU 设计。    
2. 利用实验提供的标准测试程序代码，完成仿真测试。    
3. 采用 I/O 统一编址方式，即将输入输出的 I/O 地址空间，作为数据存取空间 的一部分，实现 CPU 与外部设备的输入输出端口设计。实验中可采用高端 地址。    
4. 利用设计的 I/O 端口，通过 lw 指令，输入 DE2 实验板上的按键等输入设备 信息。即将外部设备状态，读到 CPU 内部寄存器。
5. 利用设计的 I/O 端口，通过 sw 指令，输出对 DE2 实验板上的 LED 灯等输出 设备的控制信号（或数据信息）。即将对外部设备的控制数据，从 CPU 内部 的寄存器，写入到外部设备的相应控制寄存器（或可直接连接至外部设备的 控制输入信号）。
6. 利用自己编写的程序代码，在自己设计的 CPU 上，实现对板载输入开关或 按键的状态输入，并将判别或处理结果，利用板载 LED 灯或 7 段 LED 数码 管显示出来。
7. 例如，将一路 4bit 二进制输入与另一路 4bit 二进制输入相加，利用两组分别 2 个 LED 数码管以 10 进制形式显示“被加数”和“加数”，另外一组 LED 数码管以 10 进制形式显示“和”等。（具体任务形式不做严格规定，同学可 自由创意）。
8. 在实验报告中，汇报自己的设计思想和方法；并以汇编语言的形式，提供采用以上自行设计的指令集的作品应用功能的程序设计代码，并提供程序主要流程图。

## 3.设计分析

​	本实验要求设计一个单周期CPU，实现该CPU部分简单指令，并对其进行仿真，最后设计I/O端口，使用该CPU进行一个简单的功能实现。

​	大部分代码已经给出，自己所需完成就是根据真值表，在理解代码的基础上，将alu.v 和 sc_cu.v中未完成代码补全，并在Quartus中编译，看是否有语法错误，检查出并修正。

​	其次是对该代码进行仿真，先行学习modelsim的使用方法，利用sc_computer_sim.v 测试桩，在modelsim中仿真得到波形图，测试该cpu是否能正常使用。

​	最后是对I/O进行设计。I/O根据实验指导书第3个实验，设计sc_datamen.v模块，进行I/O设计，最后设计程序，用汇编语言实现，生成mif文件，使CPU能够输出运算结果。

## 4.程序设计

### 1.顶层模块

顶层模块sc_computer.v 包括变量的定义，调用CPU模块，I/O模块和display模块。

模块功能在注释中已经给出。

```verilog
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

```

### 2.底层模块

底层模块只显示包括自己添加的和补全的部分。

#### 1.alu模块

alu模块并没有给全，需要自己补全，以下为补全的代码。

```verilog
module alu (a,b,aluc,s,z);
   //两个输入数据
   input [31:0] a,b;
   //进行什么操作标记
   input [3:0] aluc;
   //操作结果
   output [31:0] s;
   //标记是否正常操作
   output        z;
   reg [31:0] s;
   reg        z;
   always @ (a or b or aluc) 
      begin                                   // event
         casex (aluc)
             4'bx000: s = a + b;              //x000 ADD
             4'bx100: s = a - b;              //x100 SUB
             4'bx001: s = a & b;              //x001 AND
             4'bx101: s = a | b;              //x101 OR
             4'bx010: s = a ^ b;              //x010 XOR
             4'bx110: s = b << 16;            //x110 LUI: imm << 16bit             
             4'b0011: s = b << a;             //0011 SLL: rd <- (rt << sa)
             4'b0111: s = b >> a;             //0111 SRL: rd <- (rt >> sa) (logical)
             4'b1111: s = $signed(b) >>> a;   //1111 SRA: rd <- (rt >> sa) (arithmetic)
             default: s = 0;
         endcase
         if (s == 0 )  z = 1;
            else z = 0;         
      end      
endmodule 
```



#### 2.sc_cu模块

该模块主要用于CPU中根据instruction来进行控制信号的生成，源代码也没有给全，补全代码如下。(根据真值表填写)

```verilog
module sc_cu (op, func, z, wmem, wreg, regrt, m2reg, aluc, shift,
              aluimm, pcsource, jal, sext);
   input  [5:0] op,func;
   input        z;
   output       wreg,regrt,jal,m2reg,shift,aluimm,sext,wmem;
   output [3:0] aluc;
   output [1:0] pcsource;
   wire r_type = ~|op;
   wire i_add = r_type & func[5] & ~func[4] & ~func[3] &
                ~func[2] & ~func[1] & ~func[0];          //100000
   wire i_sub = r_type & func[5] & ~func[4] & ~func[3] &
                ~func[2] &  func[1] & ~func[0];          //100010
      
   //  please complete the deleted code.
   
   wire i_and = r_type & func[5] & ~func[4] & ~func[3] &func[2] & ~func[1] & ~func[0];   
   wire i_or  = r_type & func[5] & ~func[4] & ~func[3] &func[2] & ~func[1] & func[0];   
   wire i_xor = r_type & func[5] & ~func[4] & ~func[3] &func[2] & func[1] & ~func[0];   
   wire i_sll = r_type & ~func[5] & ~func[4] & ~func[3] &~func[2] & ~func[1] & ~func[0];   
   wire i_srl = r_type & ~func[5] & ~func[4] & ~func[3] &~func[2] & func[1] & ~func[0];   
   wire i_sra = r_type & ~func[5] & ~func[4] & ~func[3] &~func[2] & func[1] & func[0];   
   wire i_jr  = r_type & ~func[5] & ~func[4] & func[3] &~func[2] & ~func[1] & ~func[0];   
                
   wire i_addi = ~op[5] & ~op[4] &  op[3] & ~op[2] & ~op[1] & ~op[0]; //001000
   wire i_andi = ~op[5] & ~op[4] &  op[3] &  op[2] & ~op[1] & ~op[0]; //001100
   
   wire i_ori  = ~op[5] & ~op[4] &  op[3] &  op[2] & ~op[1] & op[0];
   wire i_xori = ~op[5] & ~op[4] &  op[3] &  op[2] & op[1] & ~op[0];
   wire i_lw   = op[5] & ~op[4] &  ~op[3] &  ~op[2] & op[1] & op[0];
   wire i_sw   = op[5] & ~op[4] &  op[3] &  ~op[2] & op[1] & op[0];
   wire i_beq  = ~op[5] & ~op[4] &  ~op[3] &  op[2] & ~op[1] & ~op[0];
   wire i_bne  = ~op[5] & ~op[4] &  ~op[3] &  op[2] & ~op[1] & op[0];
   wire i_lui  = ~op[5] & ~op[4] &  op[3] &  op[2] & op[1] & op[0];
   wire i_j    = ~op[5] & ~op[4] &  ~op[3] &  ~op[2] & op[1] & ~op[0];
   wire i_jal  = ~op[5] & ~op[4] &  ~op[3] &  ~op[2] & op[1] & op[0];
   
  
   assign pcsource[1] = i_jr | i_j | i_jal;
   assign pcsource[0] = ( i_beq & z ) | (i_bne & ~z) | i_j | i_jal ;
   
   assign wreg = i_add | i_sub | i_and | i_or   | i_xor  |
                 i_sll | i_srl | i_sra | i_addi | i_andi |
                 i_ori | i_xori | i_lw | i_lui  | i_jal;
   
   assign aluc[3] = i_sra;
   assign aluc[2] = i_sub | i_or | i_srl | i_sra | i_ori | i_lui;
   assign aluc[1] = i_xor | i_sll | i_srl | i_sra | i_xori | i_beq | i_bne | i_lui;
   assign aluc[0] = i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
   assign shift   = i_sll | i_srl | i_sra ;

   assign aluimm  = i_addi | i_andi | i_ori | i_xori | i_lw | i_sw | i_lui;
   assign sext    = i_addi | i_lw | i_sw | i_beq | i_bne;
   assign wmem    = i_sw;
   assign m2reg   = i_lw;
   assign regrt   = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui;
   assign jal     = i_jal;

endmodule
```
#### 3.I/O模块

参照实验3中给出的sc_datamen，我设计了自己的I/O模块，在该模块中，随着clock，每个周期，将input_port的数据放入寄存器中，将output_port的数据从寄存器中取出。I/O的总体设计如下：

```verilog
module sc_datamem_io (addr,datain,dataout,we,clock,mem_clk,dmem_clk,clrn,
out_port0,out_port1, out_port2, in_port0,in_port1,in_port2,mem_dataout,io_read_data);

	input [31:0] addr;
	input [31:0] datain;
	input we, clock,mem_clk,clrn;
    input [3:0] in_port0,in_port1; //三个输入端口
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
    //相关代码也在下面给出
	io_output_reg io_output_regx2 (addr,datain,write_io_output_reg_enable,dmem_clk,clrn,out_port0,out_port1, out_port2);

	//调用io_input_reg模块，每次将input的内容放入寄存器中，供cpu运算使用
    //相关模块的代码也在下面给出
	io_input_reg io_input_regx2(addr,dmem_clk,io_read_data,in_port0,in_port1,in_port2);
endmodule
```
```verilog
module io_output_reg (addr,datain,write_io_enable,io_clk,clrn,out_port0,out_port1,out_port2);

input [31:0] addr,datain;
input write_io_enable,io_clk;
//clrn是reset信号
input clrn;
//reset signal. if necessary,can use this signal to reset the output to 0.

output [31:0] out_port0,out_port1,out_port2;
reg [31:0] out_port0; // output 操作数1
reg [31:0] out_port1; // output 操作数2
reg [31:0] out_port2; // output 结果
always @(posedge io_clk or negedge clrn)
begin
	if (clrn == 0)
		begin //如果现在是reset状态，那么就不读取任何数字,将output结果置零。
			out_port0 <=0;
			out_port1 <=0;
			out_port2 <=0; // reset all the output port to 0.
		end
	else
		begin
		if (write_io_enable == 1)
		case (addr[7:2])
			6'b100000: out_port0 <= datain; // 80h
			6'b100001: out_port1 <= datain; // 84h
			6'b100010: out_port2 <= datain; // 88h
		// more ports，可根据需要设计更多的输出端口。
		endcase
		end
end
endmodule
```
```verilog
module io_input_reg (addr,io_clk,io_read_data,in_port0,in_port1,in_port2);

input [31:0] addr;
input io_clk;
input [3:0] in_port0,in_port1;
input in_port2;
output [31:0] io_read_data;

reg [31:0] in_reg0; // input 操作数1
reg [31:0] in_reg1; // input 操作数2
reg [31:0] in_reg2; // input 操作类型信号

//调用I/O多选器，根据address将寄存器中对应数据读出。
io_input_mux io_imput_mux2x32(in_reg0,in_reg1,in_reg2,addr[7:2],io_read_data);

always @(posedge io_clk)
begin
	in_reg0 <= in_port0; // 输入端口在 io_clk 上升沿时进行数据锁存
	in_reg1 <= in_port1; // 输入端口在 io_clk 上升沿时进行数据锁存
	in_reg2 <= in_port2; // 输入端口在 io_clk 上升沿时进行数据锁存
// more ports，可根据需要设计更多的输入端口。
end
endmodule

module io_input_mux(a0,a1,a2,sel_addr,y);
input [31:0] a0,a1,a2;
input [ 5:0] sel_addr;
output [31:0] y;
reg [31:0] y;
always @ *
case (sel_addr)
	6'b110000: y = a0;
	6'b110001: y = a1;
	6'b110010: y = a2;
// more ports，可根据需要设计更多的端口。
endcase
endmodule
```
#### 4.display模块

以上完成了cpu模块，I/O模块，display模块就是根据I/O output 的数据将要显示的数据显示给用户。因为是用显像管，所以需要将二进制数字转为十进制。主要内容就是进制转换

```verilog
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
```
## 5.实验结果

1. 首先完成sc_cu.v和alu的补全，在ModelSim中进行仿真，如图所示：![Alt text](https://github.com/Hunterhuan/single_period_cpu/raw/master/Screenshots/1.png)

可以观察到，每个信号的周期比，clock:mem_clk=2:1, 而且在每个clock的最后四分之一周期，imem_clk为上升状态，dmem_clk在前四分之一周期为上升状态，pc随着指令运行，每次一次+4。仿真结果正确，开始进行I/O设计。

2. 我设计的I/O的思路已经在上个章节中给出，我实现的是一个计算器（具有加减功能），利用10个开关，第一个为1时计算器处于运行状态，第二个（1为减法，0为加法），后面每4个是二进制操作数的输入，表示二进制数，最后的结果在显像管中显示出来。

我的汇编语言mif文件设计如下：

```
DEPTH = 32;           % Memory depth and width are required %
WIDTH = 32;           % Enter a decimal number %
ADDRESS_RADIX = HEX;  % Address and value radixes are optional %
DATA_RADIX = HEX;     % Enter BIN, DEC, HEX, or OCT; unless %
                      % otherwise specified, radixes = HEX %
CONTENT
BEGIN
[0..1F] : 00000000;   % Range--Every address from 0 to 1F = 00000000 %

 1 : 20010030;        % (04)       addi $1, $0, 48   #  %
 2 : 00010880;        % (08)       sll  $1, $1, 2    #  %
 3 : 8c220000;        % (0c)       lw   $2, 0($1)    #  %
 4 : 8c230004;        % (10)       lw   $3, 4($1)    #  %
 5 : 8c240008;        % (14)       lw   $4, 8($1)    #  %
 6 : 10800002;        % (18)       beq  $4, $0, Else #  %
 7 : 00432822;        % (1c)       sub  $5, $2, $3   #  %
 8 : 0c00000b;        % (20)       jal  Exit         #  %
 A : 00432820;        % (28)       add  $5, $2, $3   #  %
 C : 20010020;        % (30)       addi $1, $0, 32   #  %
 D : 00010880;        % (34)       sll  $1, $1, 2    #  %
 E : ac220000;        % (38)       sw   $2, 0($1)    #  %
 F : ac230004;        % (3c)       sw   $3, 4($1)    #  %
10 : ac250008;        % (40)       sw   $5, 8($1)    #  %
11 : 0c000000;        % (44)       jal  Loop         #  %
END ;
```

这是一个始终在运行的循环，首先是从寄存器中读入操作数和操作类型，beq条件分支，判断是加法还是减法，然后运算后存到寄存器中。流程图如下：

![Alt text](https://github.com/Hunterhuan/single_period_cpu/raw/master/Screenshots/2.png)

## 6.实验感想

​	这是用verilog语言完成的第二次作业，直接就进行单周期CPU的设计，还是觉得很困难的。所幸是大部分代码都已经给出，自己所需了解单周期的运行流程再补全代码，实现相应的I/O功能，烧录到板子上。应用CPU完成一个小功能。

​	在这次实验中还是遇到了很多的问题的，verilog的仿真遇到各种各样的错误，代码也出现各种error，幸运的是也都查到了错误原因，吸取经验，及时纠正。

​	这次实验让我对CPU整体运行流程有了更清晰的认识，为下次的流水线CPU奠定基础。其次，对verilog语言的调试和仿真也有了进一步的认识。能力得到了提升！

​	很感谢老师和助教的解答和帮助，让我们能够顺利的完成这次实验并得到能力的提升！
