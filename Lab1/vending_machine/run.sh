#!/bin/bash

iverilog -g2012 -o vending_machine_tb vending_machine_tb.v vending_machine.v calculate_current_state.v change_state.v check_time_and_coin.v
timeout 1 ./vending_machine_tb