rm reg_dump
iverilog -g2012 -o top InstMemory.v DataMemory.v RegisterFile.v opcodes.v top.v CLOG2.v Cache.v
timeout 5 ./top