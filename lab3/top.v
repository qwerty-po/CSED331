// Do not submit this file.
`include "cpu.v"

module top; 
  reg reset;
  reg clk;
  wire is_halted;
  reg [31:0] total_cycle;

  CPU cpu(
    .reset(reset), 
    .clk(clk),
    .is_halted(is_halted)
  );

  // Initialize values for simulation
  initial begin
    clk = 1'b0;
    reset = 0;
    total_cycle = 32'b0;
    #1 reset = 1;         // Drive 1 to reset register values
    #6 reset = 0;
  end

  // Generate clock
  always begin
    #5 clk = ~clk;
  end

  // Calculate total cycle
  always @(posedge clk) begin
    total_cycle <= total_cycle + 1;
  end

  // After simulation finishes.
  integer i;
  always @(posedge clk) begin
    if (is_halted) begin
      $display("TOTAL CYCLE %d\n", total_cycle);
      // Print register values
      for (i = 0; i < 32; i = i + 1)
        $display("%d %x\n", i, cpu.reg_file.rf[i]);
      $finish();
    end
  end

endmodule
