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