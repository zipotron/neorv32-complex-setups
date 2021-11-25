cp top/neorv32_ULX3S_BoardTop_MinimalBoot.vhd neorv32/setups/osflow/board_tops/
make -C neorv32/setups/osflow BOARD=ULX3S MinimalBoot NEORV32_MEM_SRC_EXTRA="../../../hardware_dependent/ulx3s/wb_sdram.vhd ../../../hardware_dependent/ulx3s/wb_intercon.vhd ../../../hardware_dependent/ulx3s/ulx3s_sdram.vhd" NEORV32_VERILOG_SRC_EXTRA=../../../hardware_dependent/ulx3s/ecp5pll.sv
