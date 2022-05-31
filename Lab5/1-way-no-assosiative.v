`include "CLOG2.v"

`define STATE_READY   3'd0
`define STATE_READ    3'd1
`define STATE_WRITE   3'd2
`define STATE_MEM_RD  3'd3
`define STATE_MEM_WR  3'd4
`define STATE_READY_2 3'd5

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

  reg [2:0]stat;

  // You might need registers to keep the status.
  assign is_ready = is_data_mem_ready;
  assign is_hit = (VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8] && is_output_valid) ? 1 : 0;
  assign dout = dout_reg;
  assign is_output_valid = (stat == `STATE_READY_2);

  always @(*) begin
    if (is_data_mem_ready && stat == `STATE_READY) begin
      if(mem_read)
        stat = `STATE_READ;
      else if(mem_write)
        stat = `STATE_WRITE;
    end
    
    if(is_data_mem_ready && VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8] && (stat == `STATE_READ)) begin
      is_output_valid_reg = 1;
      dout_reg = DATA[addr[7:4]][addr[3:2]];
      $display("read %x at %x", dout_reg, addr);
      stat = `STATE_READY_2;
    end
    else if(stat == `STATE_READ) begin
      is_output_valid_reg = 0;
    end
  end

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
      stat <= `STATE_READY;
    end
    if(stat == `STATE_READY_2)
      stat <= `STATE_READY;
    is_output_valid_reg <= 0;

    if(stat == `STATE_READ && is_data_mem_ready) begin
      if(VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8]);
      else begin
        if(VALID[addr[7:4]] == 0 || (VALID[addr[7:4]] == 1 && DIRTY[addr[7:4]] == 0)) begin
          data_mem_addr <= {4'b0, addr[31:4]};
          is_data_mem_read <= 1; is_data_mem_write <= 0; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_RD;
        end
        else begin
          data_mem_addr <={4'b0, TAG[addr[7:4]], addr[7:4]};
          data_mem_din <= {DATA[addr[7:4]][3][31:0], DATA[addr[7:4]][2][31:0], DATA[addr[7:4]][1][31:0], DATA[addr[7:4]][0][31:0]};
          is_data_mem_read <= 0; is_data_mem_write <= 1; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_WR;
        end
      end
    end

    if(stat == `STATE_WRITE && is_data_mem_ready) begin
      if(VALID[addr[7:4]] && TAG[addr[7:4]] == addr[31:8]) begin
        DATA[addr[7:4]][addr[3:2]] <= din[31:0];
        $display("write %x at %x", din, addr);
        DIRTY[addr[7:4]] <= 1;
        is_output_valid_reg <= 1;
        stat <= `STATE_READY_2;
      end
      else begin
        if(VALID[addr[7:4]] == 0 || (VALID[addr[7:4]] == 1 && DIRTY[addr[7:4]] == 0)) begin
          data_mem_addr <= {4'b0, addr[31:4]};
          is_data_mem_read <= 1; is_data_mem_write <= 0; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_RD;
        end
        else begin
          data_mem_addr <= {4'b0, TAG[addr[7:4]], addr[7:4]};
          data_mem_din <= {DATA[addr[7:4]][3][31:0], DATA[addr[7:4]][2][31:0], DATA[addr[7:4]][1][31:0], DATA[addr[7:4]][0][31:0]};
          is_data_mem_read <= 0; is_data_mem_write <= 1; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_WR;
        end
      end
    end

    if(stat == `STATE_MEM_RD && is_data_mem_ready && is_data_mem_output_valid) begin
        $display("load %x actually %x", addr[7:4], addr);
        VALID[addr[7:4]] <= 1; TAG[addr[7:4]] <= addr[31:8]; DIRTY[addr[7:4]] <= 0;
        DATA[addr[7:4]][0] <= data_mem_dout[31:0];
        DATA[addr[7:4]][1] <= data_mem_dout[63:32];
        DATA[addr[7:4]][2] <= data_mem_dout[95:64];
        DATA[addr[7:4]][3] <= data_mem_dout[127:96];
        is_data_mem_read <= 0; is_data_mem_write <= 0; data_mem_addr <= 0;
        is_data_mem_input_valid <= 0;
        $display("info : %x", data_mem_dout);
        if(mem_read)
          stat <= `STATE_READ;
        else if(mem_write)
          stat <= `STATE_WRITE;
    end

    if(stat == `STATE_MEM_WR && is_data_mem_ready) begin
      $display("evict %x actually %x", addr[7:4], addr);
      $display("valid - %x, TAG - TAG : %x -> %x", VALID[addr[7:4]], TAG[addr[7:4]], addr[31:8]);
      $display("info : %x", data_mem_din);
      data_mem_addr <= {4'b0, addr[31:4]};
      is_data_mem_read <= 1; is_data_mem_write <= 0; 
      is_data_mem_input_valid <= 1;
      stat <= `STATE_MEM_RD;
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
