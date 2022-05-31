module InstMemory #(parameter MEM_DEPTH = 16384) (input reset,
                                                  input clk,
                                                  input [31:0] addr,   // address of the instruction memory
                                                  output [31:0] dout); // instruction at addr
  integer i;
  // Instruction memory
  reg [31:0] mem[0:MEM_DEPTH - 1];
  // Do not touch imem_addr
  wire [31:0] imem_addr;
  assign imem_addr = {2'b00, addr >> 2};

  // Asynchronously read instruction from the memory 
  assign dout = mem[imem_addr];

  // Initialize instruction memory (do not touch except path)
  always @(posedge clk) begin
    if (reset) begin
      for (i = 0; i < MEM_DEPTH; i = i + 1)
          mem[i] = 32'b0;
      // Provide path of the file including instructions with binary format
      $readmemh("./opt_matmul_unroll.mem", mem);
    end
  end

endmodule
