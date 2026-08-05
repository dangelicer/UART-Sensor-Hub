## basys3_uv_ir_top.xdc
## Pin names verified against Digilent's official Basys3-Master.xdc
## (https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc)

## Clock signal (100 MHz onboard oscillator)
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk100mhz]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk100mhz]

## Reset button (btnC, center button)
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports btn_rst]

## UV sensor -- its own UART line, Pmod JA pins 1 & 2
## (plain push-pull UART now, not I2C -- no external pull-up resistors needed)
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports uv_uart_tx] ;# JA1, FPGA -> UV sensor
set_property -dict { PACKAGE_PIN L2 IOSTANDARD LVCMOS33 } [get_ports uv_uart_rx] ;# JA2, UV sensor -> FPGA

## IR sensor -- its own, separate UART line, Pmod JB pins 1 & 2
set_property -dict { PACKAGE_PIN A14 IOSTANDARD LVCMOS33 } [get_ports ir_uart_tx] ;# JB1, FPGA -> IR sensor
set_property -dict { PACKAGE_PIN A16 IOSTANDARD LVCMOS33 } [get_ports ir_uart_rx] ;# JB2, IR sensor -> FPGA

## Debug LEDs
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN V19 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {led[8]}]
set_property -dict { PACKAGE_PIN V3  IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3  IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3  IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3  IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3  IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1  IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1  IOSTANDARD LVCMOS33 } [get_ports {led[15]}]

## Configuration options (standard for all Basys3 designs)
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
