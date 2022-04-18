
`include "vending_machine_def.v"
	

module calculate_current_state(i_input_coin,i_select_item,item_price,coin_value,current_total,
is_clock_reset,current_total_nxt,wait_time,o_available_item,o_output_item, o_return_coin);
	
	input [`kNumCoins-1:0] i_input_coin;
	input [`kNumItems-1:0] i_select_item;			
	input wire [31:0] item_price [`kNumItems-1:0];
	input wire [31:0] coin_value [`kNumCoins-1:0];	
	input [`kTotalBits-1:0] current_total;
	input [31:0] wait_time;
	output reg [`kNumItems-1:0] o_available_item,o_output_item;
	output reg [`kTotalBits-1:0] current_total_nxt;
	output reg [`kNumCoins-1:0] o_return_coin;
	output reg is_clock_reset;
	integer i;	
	integer temp;
	integer victim;
	
	// Combinational logic for the next states
	always @(*) begin
		o_available_item = 0;
		o_output_item = 0;
		o_return_coin = 0;
		is_clock_reset = 0;
		current_total_nxt = 0;
		if(wait_time > 0 && wait_time <= 100) begin
			current_total_nxt = current_total;
			// TODO: current_total_nxt
			// You don't have to worry about concurrent activations in each input vector (or array).
			// Calculate the next current_total state.
			if(i_input_coin) begin
				for(i=0; i<`kNumCoins; i=i+1) begin
					if(i_input_coin[i]) begin
						victim = i;
					end
				end
				current_total_nxt = current_total + coin_value[victim];
				is_clock_reset = 1;
			end
			else if(i_select_item) begin
				for(i=0; i<`kNumItems; i = i+1) begin
					if(i_select_item[i]) begin
						victim = i;
					end
				end
				if(item_price[victim]<=current_total) begin
					current_total_nxt = current_total - item_price[victim];
					o_output_item = 1<<victim;
					is_clock_reset = 1;
				end
			end

			for(i=0; i<`kNumItems; i=i+1) begin
				if(current_total_nxt >= item_price[i]) begin
					o_available_item[i] = 1;
				end
				else begin
					o_available_item[i] = 0;
				end
			end
		end
		else begin
			temp = -1;
			for(i=0; i<`kNumCoins; i=i+1) begin
				if(current_total >= coin_value[i]) begin
					temp = i;
				end
			end
			if(temp != -1) begin
				o_return_coin[temp] = 1;
				current_total_nxt = current_total - coin_value[temp];
			end
			else begin
				is_clock_reset = 1;
			end
		end
	end
endmodule 