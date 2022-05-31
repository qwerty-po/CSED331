`include "CLOG2.v"

`define STATE_READY   3'd0
`define STATE_READ    3'd1
`define STATE_WRITE   3'd2
`define STATE_MEM_RD  3'd3
`define STATE_MEM_WR  3'd4
`define STATE_READY_2 3'd5

`define blk 3:2
`define way 6:4
`define tag 31:7

module Cache #(parameter LINE_SIZE = 16,
               parameter NUM_SETS = 2,
               parameter NUM_WAYS = 8) (
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
  integer k;

  // Wire declarations
  wire is_data_mem_ready;

  // Reg declarations
  reg [23:0] TAG [1:0] [8:0];
  reg [31:0] DATA [1:0] [8:0] [3:0];
  reg VALID [1:0] [8:0];
  reg DIRTY [1:0] [8:0];
  reg Ntimeclk [8:0];

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
  assign is_hit = ((VALID[0][addr[`way]] && TAG[0][addr[`way]] == addr[`tag]) || (VALID[1][addr[`way]] && TAG[1][addr[`way]] == addr[`tag])) ? 1 : 0;
  assign dout = dout_reg;
  assign is_output_valid = (stat == `STATE_READY_2);

  always @(*) begin
    if (is_data_mem_ready && stat == `STATE_READY) begin
      if(mem_read)
        stat = `STATE_READ;
      else if(mem_write)
        stat = `STATE_WRITE;
    end
    
    if(is_data_mem_ready && is_hit && (stat == `STATE_READ)) begin
      if(VALID[0][addr[`way]] && TAG[0][addr[`way]] == addr[`tag]) begin
        dout_reg = DATA[0][addr[`way]][addr[`blk]];
        Ntimeclk[addr[`way]] = 1;
      end
      else if(VALID[1][addr[`way]] && TAG[1][addr[`way]] == addr[`tag]) begin
        dout_reg = DATA[1][addr[`way]][addr[`blk]];
        Ntimeclk[addr[`way]] = 0;
      end

      stat = `STATE_READY_2;
    end
    else if(stat == `STATE_READ) begin
      is_output_valid_reg = 0;
    end
  end

  always @(posedge clk) begin
    if(reset) begin
      for(k = 0; k < 2; k = k + 1) begin
        for(i = 0; i < 8; i = i + 1) begin
          TAG[k][i] <= 24'b0;
          for(j = 0; j < 4; j = j + 1)
            DATA[k][i][j] <= 32'b0;
          VALID[k][i] <= 1'b0;
          DIRTY[k][i] <= 1'b0;
          Ntimeclk[i] <= 4'b0;
        end
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
      if(is_hit);
      else begin
        if(VALID[Ntimeclk[addr[`way]]][addr[`way]] == 0 || (VALID[Ntimeclk[addr[`way]]][addr[`way]] == 1 && DIRTY[Ntimeclk[addr[`way]]][addr[`way]] == 0)) begin
          data_mem_addr <= {4'b0, addr[31:4]};
          is_data_mem_read <= 1; is_data_mem_write <= 0; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_RD;
        end
        else begin
          data_mem_addr <={4'b0, TAG[Ntimeclk[addr[`way]]][addr[`way]], addr[`way]};
          data_mem_din <= {DATA[Ntimeclk[addr[`way]]][addr[`way]][3][31:0], DATA[Ntimeclk[addr[`way]]][addr[`way]][2][31:0], DATA[Ntimeclk[addr[`way]]][addr[`way]][1][31:0], DATA[Ntimeclk[addr[`way]]][addr[`way]][0][31:0]};
          is_data_mem_read <= 0; is_data_mem_write <= 1; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_WR;
        end
      end
    end

    if(stat == `STATE_WRITE && is_data_mem_ready) begin
      if(is_hit) begin
        if(VALID[0][addr[`way]] && TAG[0][addr[`way]] == addr[`tag]) begin
          if(DATA[0][addr[`way]][addr[`blk]] != din[31:0])
            DIRTY[0][addr[`way]] <= 1;
          DATA[0][addr[`way]][addr[`blk]] <= din[31:0];
          Ntimeclk[addr[`way]] <= 1;
        end
        else if(VALID[1][addr[`way]] && TAG[1][addr[`way]] == addr[`tag]) begin
          if(DATA[1][addr[`way]][addr[`blk]] != din[31:0])
            DIRTY[1][addr[`way]] <= 1;
          DATA[1][addr[`way]][addr[`blk]] <= din[31:0];
          Ntimeclk[addr[`way]] <= 0;
        end
        is_output_valid_reg <= 1;
        stat <= `STATE_READY_2;
      end
      else begin
        if(VALID[Ntimeclk[addr[`way]]][addr[`way]] == 0 || (VALID[Ntimeclk[addr[`way]]][addr[`way]] == 1 && DIRTY[Ntimeclk[addr[`way]]][addr[`way]] == 0)) begin
          data_mem_addr <= {4'b0, addr[31:4]};
          is_data_mem_read <= 1; is_data_mem_write <= 0; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_RD;
        end
        else begin
          data_mem_addr <= {4'b0, TAG[Ntimeclk[addr[`way]]][addr[`way]], addr[`way]};
          data_mem_din <= {DATA[Ntimeclk[addr[`way]]][addr[`way]][3][31:0], DATA[Ntimeclk[addr[`way]]][addr[`way]][2][31:0], DATA[Ntimeclk[addr[`way]]][addr[`way]][1][31:0], DATA[Ntimeclk[addr[`way]]][addr[`way]][0][31:0]};
          is_data_mem_read <= 0; is_data_mem_write <= 1; 
          is_data_mem_input_valid <= 1;
          stat <= `STATE_MEM_WR;
        end
      end
    end

    if(stat == `STATE_MEM_RD && is_data_mem_ready && is_data_mem_output_valid) begin
        VALID[Ntimeclk[addr[`way]]][addr[`way]] <= 1; TAG[Ntimeclk[addr[`way]]][addr[`way]] <= addr[`tag]; DIRTY[Ntimeclk[addr[`way]]][addr[`way]] <= 0;
        DATA[Ntimeclk[addr[`way]]][addr[`way]][0] <= data_mem_dout[31:0];
        DATA[Ntimeclk[addr[`way]]][addr[`way]][1] <= data_mem_dout[63:32];
        DATA[Ntimeclk[addr[`way]]][addr[`way]][2] <= data_mem_dout[95:64];
        DATA[Ntimeclk[addr[`way]]][addr[`way]][3] <= data_mem_dout[127:96];
        is_data_mem_read <= 0; is_data_mem_write <= 0; data_mem_addr <= 0;
        is_data_mem_input_valid <= 0;
        // $display("info : %x", data_mem_dout);
        if(mem_read)
          stat <= `STATE_READ;
        else if(mem_write)
          stat <= `STATE_WRITE;
    end

    if(stat == `STATE_MEM_WR && is_data_mem_ready) begin
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
