`include "CLOG2.v"

module Cache #(parameter LINE_SIZE = 16,
               parameter NUM_SETS = 1,
               parameter NUM_WAYS = 16) (
    input reset,
    input clk,

    input is_input_valid,
    input [31:0] addr,
    input mem_read,
    input mem_write,
    input [31:0] din,

    output is_ready,
    output is_output_valid,
    output [31:0] dout,
    output is_hit);

  integer i;
  integer j;

  // Wire declarations
  wire is_data_mem_ready;

  // Reg declarations
  reg [23:0] TAG [15:0];
  reg [31:0] DATA [15:0] [3:0];
  reg VALID [15:0];
  reg DIRTY [15:0];
  reg [3:0] Ntimeclk [15:0];

  reg is_hit_reg;
  reg is_output_valid_reg;
  reg [31:0] dout_reg;

  reg [31:0] data_mem_addr;
  reg [127:0] data_mem_din;
  reg [127:0] data_mem_dout;

  reg is_data_mem_read;
  reg is_data_mem_write;
  reg is_data_mem_input_valid;
  reg is_data_mem_output_valid;

  reg is_waiting_mem_to_cache_load;

  // You might need registers to keep the status.

  assign is_ready = is_data_mem_ready;
  assign is_hit = ((mem_read | mem_write) && VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8]);
  assign is_output_valid = 1'b1;
  assign dout = ((mem_read | mem_write) && VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8]) ? DATA[addr[7:4]][addr[3:2]] : 0;

  always @(posedge clk) begin
    if(reset) begin
      for(i = 0; i < 16; i = i + 1) begin
        TAG[i] <= 24'b0;
        for(j = 0; j < 4; j = j + 1)
          DATA[i][j] <= 32'b0;
        VALID[i] <= 1'b0;
        DIRTY[i] <= 1'b0;
        Ntimeclk[i] <= 4'b0;
      end
      dout_reg <= 0;
      is_hit_reg <= 0;
      is_output_valid_reg <= 0;
      is_data_mem_input_valid <= 0;
      is_waiting_mem_to_cache_load <= 0;
    end
    if(~is_input_valid);
    else if(is_waiting_mem_to_cache_load) begin // waiting mem to cache (regardless of read or write)
      if(is_data_mem_output_valid) begin
        VALID[addr[7:4]] <= 1; TAG[addr[7:4]] <= addr[31:8];
        // $display("get - %x, row - %x", data_mem_dout, addr);
        DATA[addr[7:4]][0] <= data_mem_dout[31:0];
        DATA[addr[7:4]][1] <= data_mem_dout[63:32];
        DATA[addr[7:4]][2] <= data_mem_dout[95:64];
        DATA[addr[7:4]][3] <= data_mem_dout[127:96];

        is_waiting_mem_to_cache_load <= 0;
        is_data_mem_input_valid <= 0;
      end
    end
    else if(mem_write && VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8]) begin
      // $display("memwrite - %x, %x", addr, din);
      
      if(DATA[addr[7:4]][addr[3:2]] != din) begin // if change -> dirty / not change -> maintain
        DATA[addr[7:4]][addr[3:2]] <= din[31:0];
        DIRTY[addr[7:4]] <= 1'b1;
      end
      // $display("cachewrite - %x, %x", addr, DATA[addr[7:4]][addr[3:2]]);
    end
    else if(mem_read && VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8]) begin
      dout_reg <= DATA[addr[7:4]][addr[3:2]];
      $display("memread - %x, %x", addr, dout_reg);
    end
    else begin
      if(VALID[addr[7:4]]) begin 
        if(TAG[addr[7:4]] != addr[31:8]) begin // conflict miss -> update cache
          if(~is_data_mem_ready); // data mem needs more time

          else if(DIRTY[addr[7:4]]) begin // if dirty -> evict cache and load(load will be occured under else if paragraph)
            is_data_mem_input_valid <= 1; is_data_mem_read <= 0; is_data_mem_write <= 1;
            data_mem_addr <= {4'b0, addr[31:4]};
            data_mem_din <= {DATA[addr[7:4]][3][31:0], DATA[addr[7:4]][2][31:0], DATA[addr[7:4]][1][31:0], DATA[addr[7:4]][0][31:0]};
            // $display("put - %x, row - %x", data_mem_din, addr[31:4]<<4);

            DIRTY[addr[7:4]] <= 0;
          end

          else begin // conflicted but clean -> just load on cache
            is_data_mem_input_valid <= 1; is_data_mem_read <= 1; is_data_mem_write <= 0;

            data_mem_addr <= {4'b0, addr[31:4]};
            is_waiting_mem_to_cache_load <= 1;
          end
        end
      end
      else begin // cold miss
        if(~is_data_mem_ready); // data mem needs more time
        else begin // just load on cache
          is_data_mem_input_valid <= 1; is_data_mem_read <= 1; is_data_mem_write <= 0;

          data_mem_addr <= {4'b0, addr[31:4]};
          is_waiting_mem_to_cache_load <= 1;
        end
      end
    end
  end

  // Instantiate data memory
  DataMemory #(.BLOCK_SIZE(LINE_SIZE)) data_mem(
    .reset(reset),
    .clk(clk),
    .is_input_valid(is_data_mem_input_valid),
    .addr(data_mem_addr),
    .mem_read(is_data_mem_read),
    .mem_write(is_data_mem_write),
    .din(data_mem_din),
    // is output from the data memory valid?
    .is_output_valid(is_data_mem_output_valid),
    .dout(data_mem_dout),
    // is data memory ready to accept request?
    .mem_ready(is_data_mem_ready)
  );
endmodule
