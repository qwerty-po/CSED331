iverilog -g2012 -o top Memory.v RegisterFile.v top.v
timeout 0.1 ./top