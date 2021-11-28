-- #################################################################################################
-- # << NEORV32 - Example setup for the Radiona ULX3S FPGA Board >>                                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library ECP5;
use ECP5.components.all; -- for device primitives and macros

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_ULX3S_BoardTop_MinimalBoot is
  port (
    -- Clock and Reset inputs
    ULX3S_CLK   : in  std_logic;
    ULX3S_RST_N : in  std_logic;
    -- LED outputs
    ULX3S_LED0  : out std_logic;
    ULX3S_LED1  : out std_logic;
    ULX3S_LED2  : out std_logic;
    ULX3S_LED3  : out std_logic;
    ULX3S_LED4  : out std_logic;
    ULX3S_LED5  : out std_logic;
    ULX3S_LED6  : out std_logic;
    ULX3S_LED7  : out std_logic;
    -- UART0
    ULX3S_RX    : in  std_logic;
    ULX3S_TX    : out std_logic;
    -- SDRAM interface    
    sdram_clk   : out   std_logic;                        -- Master Clock
    sdram_cke   : out   std_logic;                        -- Clock Enable    
    sdram_csn  : out   std_logic;                        -- Chip Select
    sdram_rasn : out   std_logic;                        -- Row Address Strobe
    sdram_casn : out   std_logic;                        -- Column Address Strobe
    sdram_wen  : out   std_logic;                        -- Write Enable
    sdram_d    : inout std_logic_vector(15 downto 0);    -- Data I/O (16 bits)
    sdram_dqm  : out   std_logic_vector(1 downto 0);     -- Output Disable / Write Mask
    sdram_a  : out   std_logic_vector(12 downto 0);    -- Address Input (12 bits)
    sdram_ba  : out   std_logic_vector(1 downto 0)              -- Bank Address
  );
end entity;

architecture neorv32_ULX3S_BoardTop_MinimalBoot_rtl of neorv32_ULX3S_BoardTop_MinimalBoot is

  component ecp5pll
    generic (
      in_hz : in natural := 25*1000000;
      out0_hz : in natural := 125*1000000;
      out1_hz : in natural := 25*1000000;
      out2_hz : in natural := 100*1000000;                 -- SDRAM core
      out3_hz : in natural := 100*1000000;
      out3_deg : in natural := 180  -- SDRAM chip 45-330:ok 0-30:not
    );
    port (
      clk_i : in std_logic := '0';
      clk_o : out   std_logic_vector(03 downto 0);
      locked : out   std_logic
    );
  end component ecp5pll;
   
   --
   -- Wishbone Intercon
   --   
   component wb_intercon is
      port (  
         -- Syscon
         clk_i      : in  std_logic := '0';
         rst_i      : in  std_logic := '0';
      
         -- Wishbone Master
         wbm_stb_i  : in  std_logic := '0';
         wbm_cyc_i  : in  std_logic := '0';
         wbm_we_i   : in  std_logic := '0';
         wbm_ack_o  : out std_logic;
         wbm_adr_i  : in  std_logic_vector(31 downto 0) := (others => '0');
         wbm_dat_i  : in  std_logic_vector(31 downto 0) := (others => '0');
         wbm_dat_o  : out std_logic_vector(31 downto 0);
         wbm_sel_i  : in  std_logic_vector(03 downto 0) := (others => '0');
      
         -- Wishbone Slave x
         wbs_we_o   : out std_logic; 
         wbs_dat_o  : out std_logic_vector(31 downto 0);  
         wbs_sel_o  : out std_logic_vector(03 downto 0);  
      
         -- Wishbone Slave 1
         wbs1_stb_o : out std_logic;
         wbs1_ack_i : in  std_logic := '0';
         wbs1_adr_o : out std_logic_vector(27 downto 0);  
         wbs1_dat_i : in  std_logic_vector(31 downto 0) := (others => '0')
      
      );
   end component wb_intercon;


   --
   -- Wishbone SDRAM Controller
   --
   component wb_sdram is
      port (  
         -- System
         clk_i       : in  std_logic                      := '0';
         rst_i       : in  std_logic                      := '0';

         -- Wishbone
         wbs_stb_i   : in  std_logic                      := '0';
         wbs_we_i    : in  std_logic                      := '0';
         wbs_sel_i   : in  std_logic_vector(03 downto 0)  := (others => '0');
         wbs_adr_i   : in  std_logic_vector(27 downto 0)  := (others => '0');
         wbs_dat_i   : in  std_logic_vector(31 downto 0)  := (others => '0');
         wbs_dat_o   : out std_logic_vector(31 downto 0);  
         wbs_ack_o   : out std_logic;
      
         -- SDRAM
         sdram_addr  : out   std_logic_vector(12 downto 0);                    -- addr
         sdram_ba    : out   std_logic_vector(1 downto 0);                     -- ba
         sdram_cas_n : out   std_logic;                                        -- cas_n
         sdram_cke   : out   std_logic;                                        -- cke
         sdram_cs_n  : out   std_logic;                                        -- cs_n
         sdram_dq    : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
         sdram_dqm   : out   std_logic_vector(1 downto 0);                     -- dqm
         sdram_ras_n : out   std_logic;                                        -- ras_n
         sdram_we_n  : out   std_logic                                         -- we_n
      );
   end component wb_sdram;

  -- configuration --
  constant f_clock_c : natural := 25000000; -- clock frequency in Hz

  -- internal IO connection --
  signal con_pwm    : std_ulogic_vector(02 downto 0);
  signal con_gpio_o : std_ulogic_vector(63 downto 0);
  signal con_txd_o  : std_ulogic;
  signal con_rxd_i  : std_ulogic;

  -- wishbone bus --
  type wishbone_t is record
    addr  : std_ulogic_vector(31 downto 0); -- address
    wdata : std_ulogic_vector(31 downto 0); -- host write data
    rdata : std_ulogic_vector(31 downto 0); -- host read data
    we    : std_ulogic;                     -- write enable
    sel   : std_ulogic_vector(03 downto 0); -- byte enable
    stb   : std_ulogic;                     -- strobe
    cyc   : std_ulogic;                     -- valid cycle
    ack   : std_ulogic;                     -- transfer acknowledge
  end record;
--- UNCONNECTED
  signal wb_soc : wishbone_t;

  signal wb_sel_int       : std_logic_vector(03 downto 0);   -- byte enable
   signal wb_adr_int       : std_logic_vector(31 downto 0);   -- address
	signal wb_dat_read      : std_logic_vector(31 downto 0);
   signal wb_dat_write_int : std_logic_vector(31 downto 0);   -- write data

   -- Wishbone slave
   signal wbs_we_i         : std_logic;
   signal wbs_dat_i        : std_logic_vector(31 downto 0);
   signal wbs_sel_i        : std_logic_vector(03 downto 0);
   signal wbs1_stb_i       : std_logic;
   signal wbs1_ack_o       : std_logic;
   signal wbs1_adr_i       : std_logic_vector(27 downto 0);
   signal wbs1_dat_o       : std_logic_vector(31 downto 0);
--- END UNCONNECTED

   signal pll_locked : std_logic;
   signal clocks : std_logic_vector(03 downto 0);
   
   --signal sys_clk          : std_logic := '0';
   --signal SDRAM_CLK          : std_logic := '0';
   
begin
  ecp5pll_inst : ecp5pll
    generic map (
      in_hz => 25*1000000,
      out0_hz => 125*1000000,
      out1_hz => 25*1000000,
      out2_hz => 100*1000000,                 -- SDRAM core
      out3_hz => 100*1000000,
      out3_deg => 180  -- SDRAM chip 45-330:ok 0-30:not
    )
    port map (
      clk_i => ULX3S_CLK,
      clk_o => clocks,
      locked => pll_locked
      );
  
  sdram_clk <= clocks(3);
  
  
  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => f_clock_c,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => true,        -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => true,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_M        => true,        -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zicsr    => true,        -- implement CSR system?

    -- Extension Options --
    FAST_MUL_EN                  => true,        -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => true,        -- use barrel shifter for shift operations

    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => true,        -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => 16*1024,     -- size of processor-internal instruction memory in bytes

    -- Internal Data memory --
    MEM_INT_DMEM_EN              => true,        -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => 8*1024,      -- size of processor-internal data memory in bytes

    -- External memory interface (WISHBONE) --
    MEM_EXT_EN                   => true,        -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              => 255,         -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            => false,       -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode

    -- Processor peripherals --
    IO_GPIO_EN                   => true,        -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => true,        -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => true,        -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_PWM_NUM_CH                => 3,           -- number of PWM channels to implement (0..60); 0 = disabled
    IO_WDT_EN                    => true         -- implement watch dog timer (WDT)?
  )
  port map (
    -- Global control --
    clk_i       => std_ulogic(ULX3S_CLK),        -- global clock, rising edge
    rstn_i      => std_ulogic(ULX3S_RST_N),      -- global reset, low-active, async

    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o    => open,                         -- request tag
    wb_adr_o    => wb_soc.addr,                  -- address
    wb_dat_i    => wb_soc.rdata,                 -- read data
    wb_dat_o    => wb_soc.wdata,                 -- write data
    wb_we_o     => wb_soc.we,                    -- read/write
    wb_sel_o    => wb_soc.sel,                   -- byte enable
    wb_stb_o    => wb_soc.stb,                   -- strobe
    wb_cyc_o    => wb_soc.cyc,                   -- valid cycle
    wb_lock_o   => open,                         -- exclusive access request
    wb_ack_i    => wb_soc.ack,                   -- transfer acknowledge
    wb_err_i    => '0',                          -- transfer error

    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o      => con_gpio_o,                   -- parallel output
    gpio_i      => (others => '0'),              -- parallel input

    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => con_txd_o,                    -- UART0 send data
    uart0_rxd_i => con_rxd_i,                    -- UART0 receive data
    uart0_rts_o => open,                         -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i => '0',                          -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- PWM (available if IO_PWM_EN = true) --
    pwm_o       => con_pwm                       -- pwm channels
  );
  
  wb_sel_int       <= To_StdLogicVector( wb_soc.sel );
  wb_adr_int       <= To_StdLogicVector( wb_soc.addr );
  wb_soc.rdata    <= std_ulogic_vector( wb_dat_read );
  wb_dat_write_int <= To_StdLogicVector( wb_soc.wdata );
  
   --
   -- Wishbone Intercon
   --   
  inst_wb_intercon : wb_intercon
      port map (  
         -- System
         clk_i      => std_ulogic(ULX3S_CLK),
         rst_i      => std_ulogic(ULX3S_RST_N),
      
         -- Wishbone Master
         wbm_stb_i  => wb_soc.stb,
         wbm_cyc_i  => wb_soc.cyc,
         wbm_we_i   => wb_soc.we,
         wbm_ack_o  => wb_soc.ack,
         wbm_adr_i  => wb_adr_int,
         wbm_dat_i  => wb_dat_write_int,
         wbm_dat_o  => wb_dat_read,
         wbm_sel_i  => wb_sel_int,
      
         -- Wishbone Slave x
         wbs_we_o   => wbs_we_i,
         wbs_dat_o  => wbs_dat_i,
         wbs_sel_o  => wbs_sel_i,
      
         -- Wishbone Slave 1
         wbs1_stb_o => wbs1_stb_i,
         wbs1_ack_i => wbs1_ack_o,
         wbs1_adr_o => wbs1_adr_i,
         wbs1_dat_i => wbs1_dat_o 
      );


   --
   -- Wishbone SDRAM Controller
   --
  inst_wb_sdram: wb_sdram
      port map (  
         -- System
         clk_i       => std_ulogic(ULX3S_CLK),
         rst_i       => std_ulogic(ULX3S_RST_N),

         -- Wishbone
         wbs_stb_i   => wbs1_stb_i,
         wbs_we_i    => wbs_we_i,
         wbs_sel_i   => wbs_sel_i,
         wbs_adr_i   => wbs1_adr_i,
         wbs_dat_i   => wbs_dat_i,
         wbs_dat_o   => wbs1_dat_o,
         wbs_ack_o   => wbs1_ack_o,

         -- SDRAM
         sdram_addr  => sdram_a,
         sdram_ba    => sdram_ba,
         sdram_cas_n => sdram_casn,
         sdram_cke   => sdram_cke,
         sdram_cs_n  => sdram_csn,
         sdram_dq    => sdram_d,
         sdram_dqm   => sdram_dqm,
         sdram_ras_n => sdram_rasn,
         sdram_we_n  => sdram_wen
      );
   
   --  sdram_ba <= sdram_ba;
   --  sdram_dqm <= sdram_dqm;  


  -- IO Connection --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ULX3S_LED0 <= std_logic(con_gpio_o(0));
  ULX3S_LED1 <= std_logic(con_gpio_o(1));
  ULX3S_LED2 <= std_logic(con_gpio_o(2));
  ULX3S_LED3 <= std_logic(con_gpio_o(3));
  ULX3S_LED4 <= '0'; -- unused
  ULX3S_LED5 <= std_logic(con_pwm(0));
  ULX3S_LED6 <= std_logic(con_pwm(1));
  ULX3S_LED7 <= std_logic(con_pwm(2));

  con_rxd_i <= std_ulogic(ULX3S_RX);
  ULX3S_TX  <= std_logic(con_txd_o);

end architecture;
