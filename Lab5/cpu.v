// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify modules (except InstMemory, DataMemory, and RegisterFile)
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required

module CPU(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted); // Whehther to finish simulation
  /***** Wire declarations *****/
  wire [31:0] current_pc; wire [31:0] next_pc;
  wire [31:0] inst;
  wire [31:0] reg_rs1; wire [31:0] reg_rs2;
  wire mem_read; wire mem_to_reg; wire mem_write; wire alu_src; wire write_enable; wire pc_to_reg; wire alu_op; wire is_ecall;   
  wire [31:0] imm_gen_out;  
  wire [31:0] alu_opcode; wire [31:0] alu_in_2; wire [31:0] alu_out; wire alu_bcond;
  wire [31:0] read_data;
  wire [31:0] wb_data;
  wire [4:0] X17_or_rs1;
  wire [1:0] Forwarding_rs1; wire [1:0] Forwarding_rs2;
  wire [31:0] alu_forwarded_rs1; wire [31:0] alu_forwarded_rs2;
  wire is_stall;
  wire [1:0] next_counter;
  wire permanent_stall_wire;

  /***** Register declarations *****/
  // You need to modify the width of registers
  // In addition, 
  // 1. You might need other pipeline registers that are not described below
  // 2. You might not need registers described below
  /***** IF/ID pipeline registers *****/
  reg [31:0] IF_ID_inst;           // will be used in ID stage
  /***** ID/EX pipeline registers *****/
  // From the control unit
  reg ID_EX_alu_op;         // will be used in EX stage
  reg ID_EX_alu_src;        // will be used in EX stage
  reg ID_EX_is_ecall;       // will be used in EX stage
  reg ID_EX_mem_write;      // will be used in MEM stage
  reg ID_EX_mem_read;       // will be used in MEM stage
  reg ID_EX_mem_to_reg;     // will be used in WB stage
  reg ID_EX_reg_write;      // will be used in WB stage
  // From others
  reg [4:0] ID_EX_rs1;
  reg [4:0] ID_EX_rs2;
  reg [31:0] ID_EX_rs1_data;
  reg [31:0] ID_EX_rs2_data;
  reg [31:0] ID_EX_imm;
  reg [31:0] ID_EX_ALU_ctrl_unit_input;
  reg [4:0] ID_EX_rd;

  /***** EX/MEM pipeline registers *****/
  // From the control unit
  reg EX_MEM_mem_write;     // will be used in MEM stage
  reg EX_MEM_mem_read;      // will be used in MEM stage
  reg EX_MEM_is_branch;     // will be used in MEM stage
  reg EX_MEM_mem_to_reg;    // will be used in WB stage
  reg EX_MEM_reg_write;     // will be used in WB stage
  // From others
  reg [31:0] EX_MEM_alu_out;
  reg [31:0] EX_MEM_dmem_data;
  reg [4:0] EX_MEM_rd;

  /***** MEM/WB pipeline registers *****/
  // From the control unit
  reg MEM_WB_mem_to_reg;    // will be used in WB stage
  reg MEM_WB_reg_write;     // will be used in WB stage
  // From others
  reg [31:0] MEM_WB_mem_to_reg_src_1;
  reg [31:0] MEM_WB_mem_to_reg_src_2;
  reg [4:0] MEM_WB_rd;

  reg [31:0] FAR_linking_value;
  reg [4:0] FAR_linking_rd;
  reg FAR_reg_write;

  reg permanent_stall;
  reg [1:0] current_counter;

  reg full_stall;
  reg full_stall_flush;

  reg [31:0] EX_MEM_inst;
  
  ADDER adder(
    .current_pc(current_pc),
    .is_stall(is_stall),
    .full_stall(full_stall),
    .imm(4),
    .next_pc(next_pc)
  );

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  PC pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(next_pc),     // input
    .full_stall_flush(full_stall_flush),
    .current_pc(current_pc)   // output
  );
  
  // ---------- Instruction Memory ----------
  InstMemory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(current_pc),    // input
    .dout(inst)     // output
  );

  // Update IF/ID pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      IF_ID_inst <= 0; full_stall_flush <= 0;
    end
    else if(full_stall) begin
      full_stall_flush <= 1;
    end
    else if(full_stall_flush) begin
      full_stall_flush <= 0;
    end
    else if(!is_stall) begin
      IF_ID_inst <= inst; full_stall_flush <= 0;
    end
  end

  MUX_2_1 change_rs1(
    .if_switch_off(IF_ID_inst[19:15]),
    .if_switch_on(5'b10001),
    .switch(is_ecall),
    .result(X17_or_rs1)
  );

  // ---------- Register File ----------
  RegisterFile reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (X17_or_rs1),          // input
    .rs2 (IF_ID_inst[24:20]),          // input
    .rd (MEM_WB_rd),           // input
    .rd_din (wb_data),       // input
    .write_enable (MEM_WB_reg_write),    // input
    .rs1_dout (reg_rs1),     // output
    .rs2_dout (reg_rs2)      // output
  );


  // ---------- Control Unit ----------
  ControlUnit ctrl_unit (
    .part_of_inst(IF_ID_inst[6:0]),  // input
    .mem_read(mem_read),      // output
    .mem_to_reg(mem_to_reg),    // output
    .mem_write(mem_write),     // output
    .alu_src(alu_src),       // output
    .write_enable(write_enable),  // output
    .pc_to_reg(pc_to_reg),     // output
    .alu_op(alu_op),        // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );
  
  STALL stall(
    .ID_rs1(X17_or_rs1),
    .ID_rs2(IF_ID_inst[24:20]),
    .ID_EX_rd(ID_EX_rd),
    .ID_EX_mem_read(ID_EX_mem_read),
    .permanent_stall(permanent_stall),
    .is_stall(is_stall)
  );

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(IF_ID_inst[31:0]),  // input
    .imm_gen_out(imm_gen_out)    // output
  );

  // Update ID/EX pipeline registers here
  always @(posedge clk) begin
    if(reset) begin
      FAR_linking_rd <= 0; FAR_linking_value <= 0; FAR_reg_write <= 0;
    end
    if(full_stall);
    else if (reset || is_stall || full_stall_flush) begin
      ID_EX_rs1_data <= 0; ID_EX_rs2_data <= 0; ID_EX_rd <= 0;
      ID_EX_mem_read <= 0; ID_EX_mem_to_reg <= 0; ID_EX_mem_write <= 0; ID_EX_is_ecall <= 0;
      ID_EX_alu_src <= 0; ID_EX_alu_op <= 0; ID_EX_ALU_ctrl_unit_input <= 0;
      ID_EX_imm <= 0;
      ID_EX_reg_write <= 0;
      ID_EX_rs1 <= 0; ID_EX_rs2 <= 0;
    end
    else begin
      ID_EX_rs1_data <= reg_rs1; ID_EX_rs2_data <= reg_rs2; ID_EX_rd <= IF_ID_inst[11:7];
      ID_EX_mem_read <= mem_read; ID_EX_mem_to_reg <= mem_to_reg; ID_EX_mem_write <= mem_write; ID_EX_is_ecall <= is_ecall;
      ID_EX_alu_src <= alu_src; ID_EX_alu_op <= alu_op; ID_EX_ALU_ctrl_unit_input <= IF_ID_inst;
      ID_EX_imm <= imm_gen_out;
      ID_EX_reg_write <= write_enable;
      ID_EX_rs1 <= X17_or_rs1; ID_EX_rs2 <= IF_ID_inst[24:20];
      FAR_linking_rd <= MEM_WB_rd; FAR_linking_value <= wb_data; FAR_reg_write <= MEM_WB_reg_write;
    end
  end

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit (
    .part_of_inst(ID_EX_ALU_ctrl_unit_input),  // input
    .alu_op(alu_opcode)         // output
  );

  MUX_2_1 imm_or_reg_data(
    .if_switch_off(alu_forwarded_rs2),
    .if_switch_on(ID_EX_imm),
    .switch(ID_EX_alu_src),
    .result(alu_in_2)
  );

  MUX_4_1 Forwarding_for_rs1(
    .if_0(ID_EX_rs1_data),
    .if_1(wb_data),
    .if_2(EX_MEM_alu_out),
    .if_3(FAR_linking_value),
    .mode(Forwarding_rs1),
    .result(alu_forwarded_rs1)
  );

  MUX_4_1 Forwarding_for_rs2(
    .if_0(ID_EX_rs2_data),
    .if_1(wb_data),
    .if_2(EX_MEM_alu_out),
    .if_3(FAR_linking_value),
    .mode(Forwarding_rs2),
    .result(alu_forwarded_rs2)
  );

  // ---------- ALU ----------
  ALU alu (
    .alu_op(alu_opcode),      // input
    .alu_in_1(alu_forwarded_rs1),    // input  
    .alu_in_2(alu_in_2),    // input
    .alu_result(alu_out),  // output
    .alu_bcond(alu_bcond)     // output
  );

  Halt_Check halt_check(
    .is_ecall(ID_EX_is_ecall),
    .X17(EX_MEM_alu_out),
    .permanent_stall(permanent_stall_wire)
  );
  
  always @(*) begin
    permanent_stall = permanent_stall_wire;
  end

  Evict_ALL evict_all(
    .reset(reset),
    .clk(clk),
    .permanent_stall(permanent_stall),
    .current_counter(current_counter),
    .next_counter(next_counter),
    .is_halted(is_halted)
  );

  always @(*) begin
    current_counter = next_counter;
  end

  // Update EX/MEM pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      EX_MEM_alu_out <= 0; EX_MEM_is_branch <= 0;
      EX_MEM_dmem_data <= 0; EX_MEM_mem_read <= 0; EX_MEM_mem_to_reg <= 0; EX_MEM_mem_write <= 0;
      EX_MEM_rd <= 0; EX_MEM_reg_write <= 0;
    end
    else if(full_stall) begin
    end
    else begin
      EX_MEM_inst <= ID_EX_ALU_ctrl_unit_input;
      EX_MEM_alu_out <= alu_out; EX_MEM_is_branch <= alu_bcond;
      EX_MEM_dmem_data <= alu_forwarded_rs2; EX_MEM_rd <= ID_EX_rd; 
      EX_MEM_mem_to_reg <= ID_EX_mem_to_reg; EX_MEM_mem_write <= ID_EX_mem_write; EX_MEM_reg_write <= ID_EX_reg_write;
      EX_MEM_mem_read <= ID_EX_mem_read;
    end
  end

  // // ---------- Data Memory ----------
  // DataMemory dmem(
  //   .reset (reset),      // input
  //   .clk (clk),        // input
  //   .addr (EX_MEM_alu_out),       // input
  //   .din (EX_MEM_dmem_data),        // input
  //   .mem_read (EX_MEM_mem_read),   // input
  //   .mem_write (EX_MEM_mem_write),  // input
  //   .dout (read_data)        // output
  // );

  Cache cache(
    .reset(reset),
    .clk(clk),

    .is_input_valid(1'b0),
    .addr(EX_MEM_alu_out),
    .mem_read(EX_MEM_mem_read),
    .mem_write(EX_MEM_mem_write),
    .din(EX_MEM_dmem_data),

    .is_ready(is_cache_ready),
    .is_output_valid(is_cache_output_valid),
    .dout(read_data),
    .is_hit(is_cache_hit)
  );
  always @(*) begin
    full_stall = ~(is_cache_ready & is_cache_output_valid & is_cache_hit) & (EX_MEM_mem_read | EX_MEM_mem_write);
  end
  
  // Update MEM/WB pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      MEM_WB_mem_to_reg <= 0; MEM_WB_reg_write <= 0;
      MEM_WB_mem_to_reg_src_1 <= 0; MEM_WB_mem_to_reg_src_2 <= 0; MEM_WB_rd <= 0;
    end
    else if(full_stall);
    else begin
      MEM_WB_mem_to_reg <= EX_MEM_mem_to_reg; MEM_WB_reg_write <= EX_MEM_reg_write;
      MEM_WB_mem_to_reg_src_1 <= EX_MEM_alu_out; MEM_WB_mem_to_reg_src_2 <= read_data; MEM_WB_rd <= EX_MEM_rd;
    end
  end

  MUX_2_1 WB_data(
    .if_switch_off(MEM_WB_mem_to_reg_src_1),
    .if_switch_on(MEM_WB_mem_to_reg_src_2),
    .switch(MEM_WB_mem_to_reg),
    .result(wb_data)
  );

  Fowarding_Unit fowarding_unit(
    .ID_EX_rs1(ID_EX_rs1),
    .ID_EX_rs2(ID_EX_rs2),
    .EX_MEM_rd(EX_MEM_rd),
    .EX_MEM_reg_write(EX_MEM_reg_write),
    .MEM_WB_rd(MEM_WB_rd),
    .MEM_WB_reg_write(MEM_WB_reg_write),
    .FAR_linking_rd(FAR_linking_rd),
    .FAR_reg_write(FAR_reg_write),
    .Forwarding_rs1(Forwarding_rs1),
    .Forwarding_rs2(Forwarding_rs2)
  );
  
endmodule

module Halt_Check(input is_ecall, input [31:0] X17, output permanent_stall);
  assign permanent_stall = (is_ecall & (X17 == 10)) ? 1 : 0;
endmodule

module Evict_ALL(input reset, input clk, input permanent_stall, input [1:0] current_counter, output reg [1:0] next_counter, output reg is_halted);
  always @(posedge clk) begin
    if(reset || (permanent_stall == 0 && current_counter == 2'b00)) begin
      next_counter <= 2'b00; is_halted <= 0;
    end
    else if(current_counter < 2'b01) begin
      next_counter <= current_counter + 1; is_halted <= 0;
    end
    else begin
      next_counter <= 2'b01; is_halted <= 1;
    end
  end
endmodule

module MUX_2_1(input [31:0]if_switch_off, input [31:0]if_switch_on, input switch, output [31:0]result);
  assign result = switch ? if_switch_on : if_switch_off;
endmodule

module MUX_4_1(input [31:0]if_0, input [31:0]if_1, input [31:0]if_2, input [31:0]if_3, input [1:0]mode, output reg [31:0]result);

  always @(*) begin
    
    case (mode)
      2'b00:
        result = if_0;
      2'b01:
        result = if_1;
      2'b10:
        result = if_2;
      2'b11:
        result = if_3;
    endcase

  end

endmodule

module STALL(input [4:0] ID_rs1, input [4:0] ID_rs2, input [4:0] ID_EX_rd, input ID_EX_mem_read, input permanent_stall, output reg is_stall);
  always @(*) begin
    is_stall = 0;
    if(ID_rs1 != 0 && ID_rs1 == ID_EX_rd && ID_EX_mem_read)
      is_stall = 1;
    if(ID_rs2 != 0 && ID_rs2 == ID_EX_rd && ID_EX_mem_read)
      is_stall = 1;
    if(permanent_stall)
      is_stall = 1;
  end
endmodule

module ADDER(input [31:0] current_pc, input [31:0] imm, input is_stall, input full_stall, output reg [31:0] next_pc);
  always @(*) begin
    if(is_stall)
      next_pc = current_pc;
    else if(full_stall)
      next_pc = current_pc;
    else
      next_pc = current_pc + imm;
  end
endmodule

module PC(input reset, input clk, input [31:0]next_pc, input full_stall_flush, output reg [31:0]current_pc);

  always @(posedge clk) begin
    if(reset)
      current_pc <= 0;
    else if(!full_stall_flush)begin
      current_pc <= next_pc;
    end
  end
endmodule

module ImmediateGenerator(input [31:0]part_of_inst, output reg [31:0]imm_gen_out);

  reg [31:0]imm_gen_out1;
  reg [6:0] opcode;
  always @(*) begin
    opcode = part_of_inst[6:0];
    
    case (opcode)
      7'b0010011:
        imm_gen_out1 = {part_of_inst[31:20]};
    
      7'b0000011:
        imm_gen_out1 = {part_of_inst[31:20]};

      7'b0100011:
        imm_gen_out1 = {part_of_inst[31:25], part_of_inst[11:7]};

      7'b1101111:
        imm_gen_out1 = {part_of_inst[31], part_of_inst[19:12], part_of_inst[20], part_of_inst[30:21]}<<1;

      7'b1100111:
        imm_gen_out1 = {part_of_inst[31:20]};

      7'b1100011:
        imm_gen_out1 = {part_of_inst[31], part_of_inst[7], part_of_inst[30:25], part_of_inst[11:8]}<<1;
    endcase
    
    if(opcode == 7'b1101111) begin
      if(part_of_inst[31])
        imm_gen_out = 32'hffe00000|imm_gen_out1;
      else
        imm_gen_out = imm_gen_out1;
    end
    else if(opcode == 7'b1100011) begin
      if(part_of_inst[31])
        imm_gen_out = 32'hffffe000|imm_gen_out1;
      else
        imm_gen_out = imm_gen_out1;
    end
    else if(part_of_inst[31])
      imm_gen_out = 32'hfffff000|imm_gen_out1;
    else
      imm_gen_out = imm_gen_out1;
  end

endmodule

module ControlUnit(input clk, input [6:0]part_of_inst, output reg is_jal, output reg is_jalr, output reg branch,
  output reg mem_read, output reg mem_to_reg, output reg mem_write, output reg alu_src, output reg write_enable,
  output reg pc_to_reg, output reg alu_op, output reg is_ecall);

  always @(*) begin
    is_jal = 0; is_jalr = 0; branch = 0; mem_read = 0; mem_write = 0;
    mem_to_reg = 0; alu_src = 0; write_enable = 0; pc_to_reg = 0; is_ecall = 0; alu_op = 0;

    case (part_of_inst)
      7'b0110011:
        write_enable = 1;

      7'b0010011: begin
        write_enable = 1; alu_src = 1;
      end
      
      7'b0000011: begin
        mem_to_reg = 1; mem_read = 1; alu_src = 1; write_enable = 1;
      end
      
      7'b0100011: begin
        mem_write = 1; alu_src = 1;
      end 

      7'b1101111: begin
        is_jal = 1; mem_to_reg = 1; pc_to_reg = 1; write_enable = 1;
      end

      7'b1100111: begin
        is_jalr = 1; alu_src = 1; pc_to_reg = 1; write_enable = 1;
      end

      7'b1100011:
        branch = 1;

      7'b1110011:
        is_ecall = 1;
    endcase
  end
endmodule

module ALUControlUnit(input [31:0] part_of_inst, output reg [31:0]alu_op);
  always @(*) begin
    alu_op = {part_of_inst[31:25], part_of_inst[14:12], part_of_inst[6:0], 2'b00};
  end
endmodule

module ALU(input [31:0]alu_op, input signed [31:0]alu_in_1, input signed [31:0]alu_in_2, output reg [31:0]alu_result, output reg alu_bcond);

  reg [1:0] cond;
  reg [6:0] opcode;
  reg [2:0] funct3;
  reg [6:0] funct7;

  always @(*) begin

    alu_result = 0;
    alu_bcond = 0;
    cond = alu_op[1:0];
    opcode = alu_op[8:2];
    funct3 = alu_op[11:9];
    funct7 = alu_op[18:12];

    case(opcode)
      7'b0110011:
        if(funct7 == 7'b0000000) begin
          if(funct3 == 3'b000)
            alu_result = alu_in_1 + alu_in_2;

          else if(funct3 == 3'b111)
            alu_result = alu_in_1 & alu_in_2;
          
          else if(funct3 == 3'b110)
            alu_result = alu_in_1 | alu_in_2;

          else if(funct3 == 3'b100)
            alu_result = alu_in_1 ^ alu_in_2;

          else if(funct3 == 3'b001)
            alu_result = alu_in_1 << alu_in_2;

          else if(funct3 == 3'b101)
            alu_result = alu_in_1 >> alu_in_2;

          else
            alu_result = 0;

        end
        else if(funct7 == 7'b0100000 && funct3 == 3'b000)
          alu_result = alu_in_1 - alu_in_2;

        else
          alu_result = 0;

      7'b0010011:
        if(funct3 == 3'b000)
          alu_result = alu_in_1 + alu_in_2;

        else if(funct3 == 3'b111)
          alu_result = alu_in_1 & alu_in_2;

        else if(funct3 == 3'b110)
          alu_result = alu_in_1 | alu_in_2;
        
        else if(funct3 == 3'b100)
          alu_result = alu_in_1 ^ alu_in_2;

        else if(funct3 == 3'b001)
          alu_result = alu_in_1 << alu_in_2;

        else if(funct3 == 3'b101)
          alu_result = alu_in_1 >> alu_in_2;

        else
          alu_result = 0; 

      7'b0000011:
        if(funct3 == 3'b010)
          alu_result = alu_in_1 + alu_in_2;

        else
          alu_result = 0;

      7'b0100011:
        if(funct3 == 3'b010)
          alu_result = alu_in_1 + alu_in_2;
        else
          alu_result = 0;

      7'b1101111:
        alu_result = 0;

      7'b1100111:
        if(funct3 == 0)
          alu_result = alu_in_1 + alu_in_2;
        else 
          alu_result = 0;

      7'b1100011:
        if(funct3 == 3'b000) begin
          if(alu_in_1 == alu_in_2)
            alu_bcond = 1;
          else
            alu_bcond = 0;
        end

        else if(funct3 == 3'b001) begin
          if(alu_in_1 == alu_in_2)
            alu_bcond = 0;
          else
            alu_bcond = 1;
        end

        else if(funct3 == 3'b100) begin
          if(alu_in_1 < alu_in_2)
            alu_bcond = 1;
          else
            alu_bcond = 0;
        end

        else if(funct3 == 3'b101) begin
          if(alu_in_1 >= alu_in_2)
            alu_bcond = 1;
          else
            alu_bcond = 0;
        end

        else
          alu_bcond = 0;

      default:
        alu_bcond = 0;
    endcase
  end

endmodule

module Fowarding_Unit(input [4:0] ID_EX_rs1, input [4:0] ID_EX_rs2, input [4:0] EX_MEM_rd, 
input EX_MEM_reg_write, input [4:0] MEM_WB_rd, input MEM_WB_reg_write, input [4:0] FAR_linking_rd, input FAR_reg_write,
  output reg [1:0] Forwarding_rs1, output reg [1:0] Forwarding_rs2);

  always @(*) begin
    if(ID_EX_rs1 != 0 && ID_EX_rs1 == EX_MEM_rd && EX_MEM_reg_write)
      Forwarding_rs1 = 2;
    else if(ID_EX_rs1 != 0 && ID_EX_rs1 == MEM_WB_rd && MEM_WB_reg_write)
      Forwarding_rs1 = 1;
    else if(ID_EX_rs1 != 0 && ID_EX_rs1 == FAR_linking_rd && FAR_reg_write)
      Forwarding_rs1 = 3;
    else
      Forwarding_rs1 = 0;

    if(ID_EX_rs2 != 0 && ID_EX_rs2 == EX_MEM_rd && EX_MEM_reg_write)
      Forwarding_rs2 = 2;
    else if(ID_EX_rs2 != 0 && ID_EX_rs2 == MEM_WB_rd && MEM_WB_reg_write)
      Forwarding_rs2 = 1;
    else if(ID_EX_rs2 != 0 && ID_EX_rs2 == FAR_linking_rd && FAR_reg_write)
      Forwarding_rs2 = 3;
    else
      Forwarding_rs2 = 0;
  end

endmodule