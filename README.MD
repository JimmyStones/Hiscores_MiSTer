# Hiscores_MiSTer

This is a module for MiSTer arcade cores which enables high score save/load based on the MAME hiscore plugin.

Created by [alanswx](https://github.com/alanswx), further development by [JimmyStones](https://github.com/jimmystones)

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

### Core specific implementation

Add code to link the high score signals into the relevant RAM instances.  

In simple cases (see [Phoenix](https://github.com/MiSTer-devel/Arcade_Phoenix-MiSTer) core for an example) this will only involve converting a working RAM instance from single to dual port.  

If hiscores are located in multiple RAM areas then some multiplexing will be needed to merge/split based on hs_address.  

If RAM is already dual-ported then the pause signal can be used to switch inputs to one of the ports during highscore access (see [Sega System 1](https://github.com/MiSTer-devel/Arcade_SEGASYS1-MiSTer) for a particularly complex example!)


### Module parameters
- HS_ADDRESSWIDTH - Set to the widest memory address used by the core.  The upper size of hs_address should should be set to the same -1.
- CFG_ADDRESSWIDTH - Set to allow for the maximum number of hiscore.dat entry lines used by the core (e.g. 4 = 16 max)
- DELAY_CHECKWAIT - Number of cycles to wait between start/end checks (use in conjunction with pause to avoid locking up cpu)
- DELAY_CHECKHOLD - Number of cycles to wait during start/end checks (use to allow multiplexers to settle)

_Pause_

For cores which require multiplexing due to scores being split across multiple RAM banks, an optional active-high pause signal is triggered while the high score module is reading or writing game RAM.