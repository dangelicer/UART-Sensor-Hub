## main.xdc  --  constraints for the UART Sensor Hub top-level (module `main`)
## Pin names verified against Digilent's official Basys3-Master.xdc.
##
## IMPORTANT (Vivado): set `main` as the top module and make THIS file the
## active constraints set. Exclude basys3_sensors_top.xdc from the build --
## it targets basys3_uv_ir_top, which is not the top here, and its get_ports
## calls will error against `main`.
##
## Power/ground reminder: the sensor/BT power pins in the wiring plan
## (JA6/JA12 = VCC, JA5/JA11 = GND, JC6 = VCC, JC5 = GND) are the Pmod
## connector's fixed 3.3V/GND rails -- they are NOT FPGA I/O and have no
## constraints here. Only signal pins are listed below.

## Clock signal (100 MHz onboard oscillator)
set_property -dict { PACKAGE_PIN W5  IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Reset button (btnC, center button, active-high)
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports rst]

## Channel-disable switch (high/up = sensor OFF)
set_property -dict { PACKAGE_PIN R2  IOSTANDARD LVCMOS33 } [get_ports sw_uv_off] ;# SW15

## UV sensor -- Pmod JA, two-wire UART (triggered)
set_property -dict { PACKAGE_PIN J2  IOSTANDARD LVCMOS33 } [get_ports uv_dt] ;# JA3 , UV -> FPGA (D-T)
set_property -dict { PACKAGE_PIN G2  IOSTANDARD LVCMOS33 } [get_ports uv_cr] ;# JA4 , FPGA -> UV (C-R)

## HM-10 Bluetooth -- Pmod JC
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports bt_tx] ;# JC4 , FPGA -> HM-10 RXD
set_property -dict { PACKAGE_PIN N17 IOSTANDARD LVCMOS33 } [get_ports bt_rx] ;# JC3 , HM-10 TXD -> FPGA (unused)

## Debug status LEDs (user LEDs LD0-LD15)
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]  ;# LD0  heartbeat
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]  ;# LD1  UV input activity
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]  ;# LD2  (unused)
set_property -dict { PACKAGE_PIN V19 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]  ;# LD3  UV word captured
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]  ;# LD4  (unused)
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]  ;# LD5  UV FIFO non-empty
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]  ;# LD6  (unused)
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]  ;# LD7  BT TX activity
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {led[8]}]  ;# LD8  UV trigger firing
set_property -dict { PACKAGE_PIN V3  IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3  IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3  IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3  IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3  IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1  IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1  IOSTANDARD LVCMOS33 } [get_ports {led[15]}] ;# LD15 reset

## Configuration options (standard for all Basys3 designs)
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
