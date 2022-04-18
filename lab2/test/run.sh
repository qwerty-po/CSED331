iverilog -g2012 -o top Memory.v opcodes.v RegisterFile.v top.v
timeout 1 ./top