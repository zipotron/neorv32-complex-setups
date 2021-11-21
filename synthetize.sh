cp top/neorv32_ULX3S_BoardTop_Default.vhd neorv32/setups/osflow/board_tops/
make -C neorv32/setups/osflow BOARD=ULX3S Default ULX3S_VHDL="../../../hardware_dependent/ulx3s/wb_sdram.vhd ../../../hardware_dependent/ulx3s/wb_intercon.vhd ../../../hardware_dependent/ulx3s/ulx3s_sdram.vhd" ULX3S_VERILOG=../../../hardware_dependent/ulx3s/ecp5pll.sv
