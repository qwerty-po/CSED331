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
  wire is_pc_change; wire [31:0] current_pc; wire [31:0] next_pc; wire [31:0] aluout; wire [31:0] real_pc;
  wire [31:0] mem_data; wire [31:0] WR_data;

  wire PCWR_Not_Cond; wire PCWrite; wire IorD; wire Mem_Read; wire Mem_Write; wire Mem_to_Reg; wire IRWrite; wire RegWrite; wire ALUSrcA; wire is_ecall;
  wire [1:0] ALUSrcB; wire [1:0] ALUOp; wire PCSource; wire [31:0] alu_op;

  wire [31:0] aluout_addr; wire [31:0] real_addr; wire [31:0] write_mem_info;
  wire [31:0] imm_out;
  wire [31:0] Alu_in_1; wire [31:0] Alu_in_2;
  wire [31:0] alu_out; wire bcond;
  wire [31:0] X17_or_rs1;

  /***** Register declarations *****/
  reg [31:0] IR; // instruction register
  reg [31:0] MDR; // memory data register
  reg [31:0] A; // Read 1 data register
  reg [31:0] B; // Read 2 data register
  reg [31:0] ALUOut; // ALU output register
  // Do not modify and use registers declared above.

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  IS_PCJump is_pcjump(
    .bcond(bcond),
    .pcwr_not_cond(PCWR_Not_Cond),
    .pcwrite(PCWrite),
    .is_pc_change(is_pc_change)
  );
  
  PC pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .is_pc_change(is_pc_change),
    .next_pc(next_pc),     // input
    .current_pc(current_pc)   // output
  );

  MUX_2_1 Addr_MUX(
    .if_switch_off(current_pc),
    .if_switch_on(ALUOut),
    .switch(IorD),
    .result(real_pc)
  );
  
  MUX_2_1 WR_or_MEM(
    .if_switch_off(ALUOut),
    .if_switch_on(MDR),
    .switch(Mem_to_Reg),
    .result(WR_data)
  );

  MUX_2_1 change_rs1(
    .if_switch_off(IR[19:15]),
    .if_switch_on(5'b10001),
    .switch(is_ecall),
    .result(X17_or_rs1)
  );

  // ---------- Register File ----------
  RegisterFile reg_file(
    .reset(reset),        // input
    .clk(clk),          // input
    .rs1(X17_or_rs1),          // input
    .rs2(IR[24:20]),          // input
    .rd(IR[11:7]),           // input
    .rd_din(WR_data),       // input
    .write_enable(RegWrite),    // input
    .rs1_dout(A),     // output
    .rs2_dout(B)      // output
  );

  // ---------- Memory ----------
  Memory memory(
    .reset(reset),        // input
    .clk(clk),          // input
    .addr(real_pc),         // input
    .din(B),          // input
    .mem_read(Mem_Read),     // input
    .mem_write(Mem_Write),    // input
    .dout(mem_data)          // output
  );

  Wire_Reg_Connector WRC1(
    .wire_in(mem_data),
    .ctrl_unit(IRWrite),
    .reg_out(IR)
  );

  Wire_Reg_Connector WRC2(
    .wire_in(mem_data),
    .ctrl_unit(Mem_Read),
    .reg_out(MDR)
  );

  // ---------- Control Unit ----------
  ControlUnit ctrl_unit(
    .reset(reset),
    .clk(clk),
    .part_of_inst(IR[6:0]),  // input
    .bcond(bcond),
    .pcwr_not_cond(PCWR_Not_Cond),
    .pcwrite(PCWrite),
    .iord(IorD),
    .mem_read(Mem_Read),      // output
    .mem_to_reg(Mem_to_Reg),    // output
    .mem_write(Mem_Write),     // output
    .irwrite(IRWrite),
    .pcsource(PCSource),
    .aluop(ALUOp),
    .alusrcA(ALUSrcA),
    .alusrcB(ALUSrcB),
    .regwrite(RegWrite),
    .ALUOut_update(ALUOut_update),
    .is_ecall(is_ecall)       // output (ecall inst) -> need to change as other wire
  );

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(IR),  // input
    .imm_gen_out(imm_out)    // output
  );

  MUX_2_1 ALU_in_A(
    .if_switch_off(current_pc),
    .if_switch_on(A),
    .switch(ALUSrcA),
    .result(Alu_in_1)
  );

  MUX_4_1 ALU_in_B(
    .if_0(B),
    .if_1(4),
    .if_2(imm_out),
    .if_3(0),
    .mode(ALUSrcB),
    .result(Alu_in_2)
  );

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit(
    .part_of_inst(IR),  // input
    .ALUOp_ctrl_unit(ALUOp),
    .alu_op(alu_op)         // output
  );

  // ---------- ALU ----------
  ALU alu(
    .alu_op(alu_op),      // input
    .alu_in_1(Alu_in_1),    // input  
    .alu_in_2(Alu_in_2),    // input
    .alu_result(alu_out),  // output
    .alu_bcond(bcond)     // output
  );

  Wire_Reg_Connector WRC4(
    .wire_in(alu_out),
    .ctrl_unit(ALUOut_update),
    .reg_out(ALUOut)
  );

  MUX_2_1 ALU_result_OR_ALUOut(
    .if_switch_off(alu_out),
    .if_switch_on(ALUOut),
    .switch(PCSource),
    .result(next_pc)
  );

  Halt_Check halt_check(
    .is_ecall(is_ecall),
    .X17(A),
    .is_halted(is_halted)
  );
  
endmodule

module Assigner(input [31:0] src, output reg [31:0] dest);
  always @(*) begin
    dest = src;
  end
endmodule

module Halt_Check(input is_ecall, input [31:0] X17, output reg is_halted);
  always @(*) begin
    if(is_ecall) begin
      if(X17 == 10)
        is_halted = 1;
      else
        is_halted = 0;
    end
  end
endmodule

module MUX_2_1(input [31:0]if_switch_off, input [31:0]if_switch_on, input switch, output reg [31:0]result);

  always @(*) begin
    if(switch)
      result = if_switch_on;
    else
      result = if_switch_off;
  end

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

module Wire_Reg_Connector (input [31:0]wire_in, input ctrl_unit, output reg [31:0]reg_out);
  always @(*) begin
    if(ctrl_unit)
      reg_out = wire_in;
  end
endmodule

module CLK_updater(input clk, input [31:0]wire_in, input ctrl_unit, output reg [31:0]reg_out);
  always @(posedge clk) begin
    if(ctrl_unit)
      reg_out = wire_in;
  end
endmodule

module IS_PCJump (input bcond, input pcwr_not_cond, input pcwrite, output is_pc_change);
  assign is_pc_change = (!pcwr_not_cond & !bcond) | pcwrite;
endmodule

module PC(input reset, input clk, input is_pc_change, input [31:0]next_pc, output reg [31:0]current_pc);

  always @(posedge clk) begin
    if(reset)
      current_pc <= 0;
    else if(is_pc_change) begin
      current_pc <= next_pc;
    end
  end

endmodule

module ControlUnit (input reset, input clk, input bcond, input [6:0]part_of_inst, output reg pcwr_not_cond, output reg pcwrite, output reg iord, output reg mem_read, output reg mem_to_reg, output reg mem_write, 
                    output reg irwrite, output reg pcsource, output reg [1:0] aluop, output reg alusrcA, output reg [1:0] alusrcB, output reg regwrite, output reg ALUOut_update, output reg is_ecall);
  
  reg [4:0]current_status;
  
  always @(posedge clk) begin
    if(reset) begin
      current_status <= 0; 
      pcwrite <= 0; iord <= 0; mem_read <= 0; mem_to_reg <= 0; mem_write <= 0; irwrite <= 0;
      pcsource <= 0; alusrcA <= 0; regwrite <= 0; is_ecall <= 0; aluop <= 0; alusrcB <= 0;
      pcwr_not_cond <= 1;
      ALUOut_update <= 0;
    end

    else begin
      pcwrite <= 0; mem_read <= 0; mem_write <= 0; irwrite <= 0; regwrite <= 0; is_ecall <= 0; 
      pcwr_not_cond <= 1; ALUOut_update <= 0;
      case (current_status)
        0: begin
          mem_read <= 1; irwrite <= 1; alusrcA <= 0; alusrcB <= 2'b01; aluop <= 0;
          pcsource <= 0; iord <= 0; ALUOut_update <= 0;
          current_status <= 1;
        end

        1: begin
          alusrcA <= 0; alusrcB <= 2'b01; aluop <= 0; ALUOut_update <= 1;

          if(part_of_inst == 7'b1110011) begin
            is_ecall <= 1;
            current_status <= 6;
          end
          else if(part_of_inst == 7'b0000011 || part_of_inst == 7'b0100011)
            current_status <= 2;
          else if(part_of_inst == 7'b1100011)
            current_status <= 8;
          else
            current_status <= 6;
        end

        2: begin
          alusrcA <= 1; alusrcB <= 2'b10; aluop <= 2'b00; ALUOut_update <= 1;

          if(part_of_inst == 7'b0000011) begin
            current_status <= 3;
          end
          else
            current_status <= 5;
        end

        3: begin
          mem_read <= 1; iord <= 1;
          current_status <= 4;
        end

        4: begin
          regwrite <= 1; mem_to_reg <= 1; alusrcA <= 0; alusrcB <= 2'b01; pcsource <= 0; pcwrite <= 1;
          current_status <= 0;
        end

        5: begin
          mem_write <= 1; iord <= 1; alusrcA <= 0; alusrcB <= 2'b01; pcsource <= 0; pcwrite <= 1;
          current_status <= 0;
        end

        6: begin
          if(part_of_inst == 7'b1110011) begin
            is_ecall <= 1; alusrcA <= 0; alusrcB <= 2'b01; aluop <= 0; pcwrite <= 1;
            current_status <= 0;
          end
          else begin
            aluop <= 2'b10;
            if(part_of_inst == 7'b0110011) begin
              alusrcA <= 1; alusrcB <= 2'b00; current_status <= 7; ALUOut_update <= 1;
            end
            else if(part_of_inst == 7'b0010011) begin
              alusrcA <= 1; alusrcB <= 2'b10; current_status <= 7; ALUOut_update <= 1;
            end
            else if(part_of_inst == 7'b1101111) begin
              alusrcA <= 0; alusrcB <= 2'b10; current_status <= 0; ALUOut_update <= 0; regwrite <= 1; mem_to_reg <= 0;
              pcsource <= 0; pcwrite <= 1; regwrite <= 1; mem_to_reg <= 0;
            end
            else begin
              alusrcA <= 1; alusrcB <= 2'b10; current_status <= 0; ALUOut_update <= 0; regwrite <= 1; mem_to_reg <= 0;
              pcsource <= 0; pcwrite <= 1; regwrite <= 1; mem_to_reg <= 0;
            end
          end
        end

        7: begin
          regwrite <= 1; mem_to_reg <= 0; pcwrite <= 1; alusrcA <= 0; alusrcB <= 2'b01; pcsource <= 0;
          current_status <= 0; aluop <= 2'b00;
        end

        8: begin
          alusrcA <= 1; alusrcB <= 2'b00; aluop <= 2'b01; pcsource <= 1; ALUOut_update <= 0; pcwr_not_cond <= 0;
          current_status <= 9;
        end

        9: begin
          if(bcond) begin
            mem_read <= 1; irwrite <= 1; alusrcA <= 0; alusrcB <= 2'b10; aluop <= 0; pcwrite <= 1;
            pcsource <= 0; iord <= 0; ALUOut_update <= 0;
            current_status <= 0;
          end
          else begin
            mem_read <= 1; irwrite <= 1; alusrcA <= 0; alusrcB <= 2'b01; aluop <= 0; pcwrite <= 1;
            pcsource <= 1; iord <= 0; ALUOut_update <= 0;
            current_status <= 1;
          end
        end

      endcase
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

module ALUControlUnit(input [31:0] part_of_inst, input [1:0]ALUOp_ctrl_unit, output reg [31:0]alu_op);
  always @(*) begin
    alu_op = {part_of_inst[31:25], part_of_inst[14:12], part_of_inst[6:0], ALUOp_ctrl_unit[1:0]};
  end
endmodule

// alu_op = funct7/funct3/opcode/aluop_by_condition
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

    if (cond == 0) begin
      if(funct7 == 7'b0000011) begin
        if(funct3 == 3'b010)
          alu_result = alu_in_1 + alu_in_2;

        else
          alu_result = 0;
      end

      else if(funct7 == 7'b0100011) begin
        if(funct3 == 3'b010)
          alu_result = alu_in_1 + alu_in_2;

        else
          alu_result = 0;
      end
      
      else
        alu_result = alu_in_1 + alu_in_2;
    end

    else begin
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

          else if(funct3 == 3'b001) begin
            alu_result = alu_in_1 << alu_in_2;
          end

          else if(funct3 == 3'b101)
            alu_result = alu_in_1 >> alu_in_2;

          else
            alu_result = 0; 

        7'b1101111: begin
          alu_result = alu_in_1 + alu_in_2; 
        end

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
  end

endmodule