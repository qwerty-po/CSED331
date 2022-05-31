module DataMemory #(parameter MEM_DEPTH = 16384,
                    parameter DELAY = 50,
                    parameter BLOCK_SIZE = 16) (
    input reset,
    input clk,

    // Inputs from the cache
    input is_input_valid,                   // is request valid?
    input [31:0] addr,                      // address of the memory
    input mem_read,                         // is read signal driven?
    input mem_write,                        // is write signal driven?
    input [BLOCK_SIZE * 8 - 1:0] din,      // data to be written

    // outputs from the data memory
    output is_output_valid,                 // is output valid?
    output [BLOCK_SIZE * 8 - 1:0] dout,    // output data
    output mem_ready);

  integer i;

  // Memory
  reg [BLOCK_SIZE * 8 - 1:0] mem[0: MEM_DEPTH - 1];

  // delay counter used to delay the memory accesses
  reg [31:0] delay_counter;

  // Used to store the status of the previous memory request
  reg [31:0] _mem_addr;
  reg _mem_read;
  reg _mem_write;
  reg [BLOCK_SIZE * 8 - 1:0] _din;

  wire request_arrived;

  assign request_arrived = ((mem_read | mem_write) && is_input_valid);

  assign dout = (_mem_read && (delay_counter == 0)) ? mem[_mem_addr] : 0;
  assign is_output_valid = (_mem_read && delay_counter == 0);

  // Do not have to check `_mem_read == 0 & _mem_write == 0`
  assign mem_ready = delay_counter == 0;

  always @(posedge clk) begin
    // Initialize data memory
    if (reset) begin
       for (i = 0; i < MEM_DEPTH; i = i + 1)
         mem[i] = 0;
    end
    // Write data to the memory
    else if (_mem_write && delay_counter == 0) begin
      mem[_mem_addr] <= _din;
    end
  end

  always @(posedge clk) begin
    if (reset) begin
      delay_counter <= 0;
      _mem_read <= 0;
      _mem_write <= 0;
      _mem_addr <= 0;
      _din <= 0;
    end
    else if (request_arrived && delay_counter == 0) begin
      delay_counter <= DELAY;
      _mem_read <= mem_read;
      _mem_write <= mem_write;
      _mem_addr <= addr;
      _din <= din;
    end
    else if (delay_counter > 0) begin
      delay_counter <= delay_counter - 1;
    end
    else begin
      delay_counter <= 0;
      _mem_read <= 0;
      _mem_write <= 0;
      _mem_addr <= 0;
      _din <= 0;
    end
  end
endmodule
