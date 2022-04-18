`include "vending_machine_def.v"

	

module check_time_and_coin(clk,reset_n,i_trigger_return,coin_value,is_clock_reset,wait_time);
	input clk;
	input reset_n;
	input wire [31:0] coin_value [`kNumCoins-1:0];
	input i_trigger_return;
	input is_clock_reset;
	output reg [31:0] wait_time;

	// initiate values
	initial begin
		// TODO: initiate values
		wait_time = 100;
	end


	// update coin return time
	always @(*) begin
		// TODO: update coin return time
		if(is_clock_reset) begin
			wait_time = 100;
		end
	end

	always @(posedge clk) begin
		if (!reset_n) begin
		// TODO: reset all states.
			wait_time <= 100;
		end
		else if(i_trigger_return) begin
			wait_time <= 0;
		end
		else begin
		// TODO: update all states.
			if(wait_time <= 100 && wait_time) begin
				wait_time <= wait_time - 1;
			end
		end
	end
endmodule 