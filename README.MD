# Hiscores_MiSTer

This is a module for MiSTer arcade cores which enables high score save/load based on the MAME hiscore plugin.

Written by @alanswx, further development by @jimmystones

## Features
- Reads hiscore.dat entries from MRA (ioctl index = 3)
- Loads and saves high score data in \<MRA name\>.nvm dump (ioctl index = 4)
- Configurable delay before high score load to game RAM 
- Configurable delay between hiscore entry start/end checks
- Configurable delay during hiscore RAM read/write
- Pause signal to halt CPU for during multiplex with cores already using dual-port RAM

## Implementation

- Add hiscore.sv to rtl/ folder and files.qip

- Add the following code to the main core .sv file. 

```verilog
// HISCORE SYSTEM
// --------------

wire [15:0]hs_address;
wire [7:0]hs_to_ram;
wire hs_write;
wire hs_pause;

hiscore #(
	.HS_ADDRESSWIDTH(16),
	.CFG_ADDRESSWIDTH(4),
	.DELAY_CHECKWAIT(6'b111111),
	.DELAY_CHECKHOLD(1'b0)
	
) hi (
	.clk(clk_sys),
	.reset(reset),
	.delay(1'b0),
	.ioctl_upload(ioctl_upload),
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_din(ioctl_din),
	.ioctl_index(ioctl_index),
	.ram_address(hs_address),
	.data_to_ram(hs_to_ram),
	.ram_write(hs_write),
	.pause(hs_pause)
);
```
### Module parameters
- HS_ADDRESSWIDTH - Set to the widest memory address used by the core.  The upper size of hs_address should should be set to the same -1.
- CFG_ADDRESSWIDTH - Set to allow for the maximum number of hiscore.dat entry lines used by the core (e.g. 4 = 16 max)
- DELAY_CHECKWAIT - Number of cycles to wait between start/end checks (use in conjunction with pause to avoid locking up cpu)
- DELAY_CHECKHOLD - Number of cycles to wait during start/end checks (use to allow multiplexers to settle)

_Pause_

For cores which require multiplexing due to scores being split across multiple RAM banks, an optional active-high pause signal is triggered while the high score module is reading or writing game RAM.