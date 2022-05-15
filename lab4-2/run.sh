iverilog -g2012 -o top Memory.v RegisterFile.v opcodes.v top.v
timeout 1 ./top