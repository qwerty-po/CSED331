// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify the module.
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required

module CPU(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted); // Whehther to finish simulation
  /***** Wire declarations *****/

  wire [32-1:0] pc_nxt;
  wire [32-1:0] pc_add_4_out;
  wire [32-1:0] pc_add_imm_out;
  // InstMemory unit
  wire [32-1:0] full_instruction;
  
  // Regfile input unit
  wire [4:0] rs1;
  wire [4:0] rs2;
  wire [4:0] rd;
  wire [6:0] funct7;
  wire [2:0] funct3;
  wire [6:0] opcode;

  wire [32-1:0] rd_din;
  wire RegWrite; 
  
  // Regfile output unit
  wire [32-1:0] rs2_dout;

  // Recognize instruction type and opcode

  // Ctrl Unit instruction saver
  wire is_jal, is_jalr, branch, mem_read, mem_to_reg, mem_write;
  wire alu_src, write_enable, pc_to_reg, is_ecall;

  // Imm generator output
  wire [32-1:0] imm;

  // ALU calculated output
  wire signed [32-1:0] alu_in_1;
  wire signed [32-1:0] alu_in_2;
  wire [32-1:0] alu_out;

  // Datamemory output
  wire [32-1:0] ext;

  /***** Register declarations *****/
  // PC control unit
  reg [32-1:0] rip;

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  PC pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(pc_nxt),     // input
    .current_pc(rip)   // output
  );
  
  // ---------- Instruction Memory ----------  -> implemented at Memory.v but needs more
  InstMemory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(rip),    // input
    .dout(full_instruction)     // output
  );

  // ---------- Instruction Parser ----------
  InstParser iparser(
    .reset(reset),
    .clk(clk),
    .full_inst(full_instruction),
    .rs1(rs1),
    .rs2(rs2),
    .rd(rd),
    .funct7(funct7),
    .funct3(funct3),
    .opcode(opcode)
  );

  // ---------- Register File ----------  -> implemented at RegisterFile.v but needs more
  RegisterFile reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (rs1),          // input
    .rs2 (rs2),          // input
    .rd (rd),           // input
    .rd_din (rd_din),       // input
    .write_enable (write_enable),    // input
    .rs1_dout (alu_in_1),     // output
    .rs2_dout (rs2_dout)      // output
  );

  // ---------- Control Unit ----------
  ControlUnit ctrl_unit (
    .part_of_inst(opcode),  // input
    .is_jal(is_jal),        // output
    .is_jalr(is_jalr),       // output
    .branch(branch),        // output
    .mem_read(mem_read),      // output
    .mem_to_reg(mem_to_reg),    // output
    .mem_write(mem_write),     // output
    .alu_src(alu_src),       // output
    .write_enable(write_enable),     // output
    .pc_to_reg(pc_to_reg),     // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(full_instruction),  // input
    .opcode(opcode),
    .imm_gen_out(imm)    // output
  );

  // // ---------- ALU Control Unit ----------
  // ALUControlUnit alu_ctrl_unit (
  //   .part_of_inst(full_instruction),  // input
  //   .alu_op(opcode)         // output
  // );

  Switch alusrc_switch(
    .if_switch_off(rs2_dout),
    .if_switch_on(imm),
    .switch(alu_src),
    .result(alu_in_2)
  );

  // ---------- ALU ----------
  ALU alu (
    .alu_op(opcode),      // input
    .alu_funct7(funct7),
    .alu_funct3(funct3),
    .alu_in_1(alu_in_1),    // input  
    .alu_in_2(alu_in_2),    // input
    .alu_result(alu_out),  // output
    .alu_bcond(branch)     // output
  );

  // ---------- Data Memory ----------  -> implemented at Memory.v but needs more
  DataMemory dmem(
    .reset (reset),      // input
    .clk (clk),        // input
    .addr (alu_out),       // input
    .din (rs2_dout),        // input
    .mem_read (mem_read),   // input
    .mem_write (mem_write),  // input
    .dout (ext)        // output
  );

  Switch mem_to_reg_switch(
    .if_switch_off(alu_out),
    .if_switch_on(ext),
    .switch(mem_to_reg),
    .result(write_to_reg)
  );

  Switch write_to_rd_switch(
    .if_switch_off(write_to_reg),
    .if_switch_on(pc_add_4_out),
    .switch(pc_to_reg),
    .result(rd_din)
  );

  Switch pc_src_1_switch(
    .if_switch_off(pc_add_4_out),
    .if_switch_on(pc_add_imm_out),
    .switch(pc_src_1),
    .result(pc_might_nxt)
  );

  Switch pc_src_2_switch(
    .if_switch_off(pc_might_nxt),
    .if_switch_on(alu_out),
    .switch(pc_src_2),
    .result(pc_nxt)
  );

  Add pc_4_adder(
    .pc(pc),
    .imm(1'h00000004),
    .pc_nxt(pc_add_4_out)
  );

  Add pc_imm_adder(
    .pc(pc),
    .imm(imm),
    .pc_nxt(pc_add_imm_out)
  );

endmodule

module Switch(input [32-1:0]if_switch_off, input [32-1:0]if_switch_on, input switch, output reg [32-1:0]result);

  always @(*) begin
    if(switch)
      result = if_switch_on;
    else
      result = if_switch_off;
  end

endmodule

module Add(input [32-1:0] pc, input [32-1:0] imm, output reg [32-1:0] pc_nxt);

  assign pc_nxt = pc+imm;

endmodule

module PC(input reset, input clk, input [32-1:0]next_pc, output reg [32-1:0]current_pc);

  always @(posedge clk) begin
    if(!reset) begin
      current_pc <= 0;
    end
    else
      current_pc <= next_pc;
  end
endmodule

module InstParser(input reset, input clk, input [32-1:0] full_inst, 
  output reg[4:0]rs1, output reg[4:0]rs2, output reg[4:0]rd, output reg[6:0]funct7, 
  output reg[2:0]funct3, output reg[6:0]opcode);

  always @(*) begin
    rs1 = full_inst[19:15];
    rs2 = full_inst[24:20]; 
    rd = full_inst[11:7];

    funct7 = full_inst[31:25];
    funct3 = full_inst[14:12];

    opcode = full_inst[6:0];
  end
endmodule

module ControlUnit(input [6:0]part_of_inst, output reg is_jal, output reg is_jalr, output reg branch,
  output reg mem_read, output reg mem_to_reg, output reg mem_write, output reg alu_src, output reg write_enable,
  output reg pc_to_reg, output reg is_ecall);

  always @(*) begin
    is_jal = 0; is_jalr = 0; branch = 0; mem_read = 0; mem_write = 0;
    mem_to_reg = 0; alu_src = 0; write_enable = 0; pc_to_reg = 0; is_ecall = 0;

    case (part_of_inst)
      ARITHMETIC:
        write_enable = 1;

      ARITHMETIC_IMM: begin
        write_enable = 1; alu_src = 1;
      end
      
      LOAD: begin
        mem_read = 1; alu_src = 1; write_enable = 1;
      end
      
      STORE: begin
        mem_write = 1; alu_src = 1;
      end 

      JAL: begin
        is_jal = 1; pc_to_reg = 1;
      end

      JALR: begin
        is_jalr = 1; alu_src = 1; pc_to_reg = 1;
      end

      BRANCH:
        branch = 1;
    endcase
  end
endmodule

module ImmediateGenerator(input [32-1:0]part_of_inst, input [6:0]opcode, output reg [32-1:0]imm_gen_out);

  always @(*) begin
    case (opcode)
      ARITHMETIC_IMM:
        imm_gen_out = {part_of_inst[31:20]};
    
      LOAD:
        imm_gen_out = {part_of_inst[31:20]};

      STORE:
        imm_gen_out = {part_of_inst[31:25], part_of_inst[11:7]};

      JAL:
        imm_gen_out = {part_of_inst[31], part_of_inst[19:12], part_of_inst[20], part_of_inst[30:21]};

      JALR:
        imm_gen_out = {part_of_inst[31:20]};

      BRANCH:
        imm_gen_out = {part_of_inst[31], part_of_inst[7], part_of_inst[30:25], part_of_inst[11:8]}<<1;

      default:
        imm_gen_out = 0;
    endcase
  end

endmodule

module ALU(input [6:0]alu_op, input [6:0]alu_funct7, input [2:0]alu_funct3, input signed [32-1:0]alu_in_1, 
  input signed [32-1:0]alu_in_2, output reg [32-1:0]alu_result, output reg alu_bcond);

  always @(*) begin

    alu_result = 0;
    alu_bcond = 0;

    case(alu_op)
      ARITHMETIC:
        if(funct7 == FUNCT7_OTHERS) begin
          if(funct3 == FUNCT3_ADD)
            alu_result = alu_in_1 + alu_in_2;

          else if(funct3 == FUNCT3_AND)
            alu_result = alu_in_1 & alu_in_2;
          
          else if(funct3 == FUNCT3_OR)
            alu_result = alu_in_1 | alu_in_2;

          else if(funct3 == FUNCT3_XOR)
            alu_result = alu_in_1 ^ alu_in_2;

          else if(funct3 == FUNCT3_SLL)
            alu_result = alu_in_1 << alu_in_2;

          else if(funct3 == FUNCT3_SRL)
            alu_result = alu_in_1 >> alu_in_2;

          else
            alu_result = 0;

        end
        else if(funct7 == FUNCT7_SUB && funct3 == FUNCT3_SUB)
          alu_result = alu_in_1 - alu_in_2;

        else
          alu_result = 0;

      ARITHMETIC_IMM:
        if(funct3 == FUNCT3_ADD)
          alu_result = alu_in_1 + alu_in_2;

        else if(funct3 == FUNCT3_AND)
          alu_result = alu_in_1 & alu_in_2;

        else if(funct3 == FUNCT3_OR)
          alu_result = alu_in_1 | alu_in_2;
        
        else if(funct3 == FUNCT3_XOR)
          alu_result = alu_in_1 ^ alu_in_2;

        else
          alu_result = 0; 

      LOAD:
        if(funct3 == FUNCT3_LW)
          alu_result = alu_src_1 + alu_src_2;

        else
          alu_result = 0;

      STORE:
        if(funct3 == FUNCT3_SW)
          alu_result = alu_src_1 + alu_src_2;
        else
          alu_result = 0;

      JAL:
        alu_result = 0;

      JALR:
        if(funct3 == 0)
          alu_result = alu_in_1 + alu_in_2;
        else 
          alu_result = 0;

      BRANCH:
        if(funct3 == FUNCT3_BEQ) begin
          if(alu_in_1 == alu_in_2)
            alu_bcond = 1;
          else
            alu_bcond = 0;
        end

        else if(funct3 == FUNCT3_BNE) begin
          if(alu_in_1 == alu_in_2)
            alu_bcond = 0;
          else
            alu_bcond = 1;
        end

        else if(funct3 == FUNCT3_BLT) begin
          if(alu_in_1[32-1:0] < alu_in_2[32-1:0])
            alu_bcond = 1;
          else
            alu_bcond = 0;
        end

        else if(funct3 == FUNCT3_BGE) begin
          if(alu_in_1[32-1:0] >= alu_in_2[32-1:0])
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