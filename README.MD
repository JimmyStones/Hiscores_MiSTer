# Hiscores_MiSTer

This is a generic module for MiSTer arcade cores which enables high score save/load based on the MAME hiscore plugin data format (hiscore.dat).

Created by Alan Steremberg ([alanswx](https://github.com/alanswx))

Further development by Jim Gregory ([JimmyStones](https://github.com/jimmystones))

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
wire hs_access;

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
	.ram_access(hs_access)
);
```

### Core specific implementation

Add code to link the high score signals into the relevant RAM instances.  

In simple cases (see [Phoenix](https://github.com/MiSTer-devel/Arcade_Phoenix-MiSTer) core for an example) this will only involve converting a working RAM instance from single to dual port.  

If hiscores are located in multiple RAM areas then some multiplexing will be needed to merge/split based on hs_address.  

If RAM is already dual-ported then the access signal can be used to puase CPU and switch inputs to one of the ports during highscore access (see [Sega System 1](https://github.com/MiSTer-devel/Arcade_SEGASYS1-MiSTer) for a particularly complex example!)

### Module parameters
- HS_ADDRESSWIDTH - Set to the widest memory address used by the core.  The upper size of hs_address should should be set to the same -1.
- CFG_ADDRESSWIDTH - Set to allow for the maximum number of hiscore.dat entry lines used by the core (e.g. 4 = 16 max)
- CFG_LENGTHWIDTH - Set to 1 for single byte length, 2 for double
- DELAY_CHECKWAIT - Number of cycles to wait between start/end checks (use in conjunction with pause to avoid locking up cpu)
- DELAY_CHECKHOLD - Number of cycles to wait during start/end checks (use to allow multiplexers to settle)

### Module ports
| Port           | Direction | Description 
| -------------- | --------- | ----------- 
| clk            | in        | System clock.  Should match the HPS IO clock input 
| reset          | in        | Active-high reset signal from core
| delay          | in        | 32-bit value to override default delay before writing hiscores to RAM
| ioctl_upload   | in        | From HPS module
| ioctl_download | in        | From HPS module
| ioctl_wr       | in        | From HPS module
| ioctl_addr     | in        | From HPS module
| ioctl_dout     | in        | From HPS module
| ioctl_din      | in        | From HPS module
| ioctl_index    | in        | From HPS module
| ram_address    | out       | System RAM address for highscore read/write
| data_to_ram    | out       | Data to write to system RAM
| ram_write      | out       | Active-high signal to write to system RAM
| ram_access     | out       | Active-high signal used to pause CPU for complex RAM situations

### MRA data

To enable the high score module add the following sections to the MRA.

#### MRA sections
```xml
<rom index="3" md5="none">
	<part>
		00 00 90 5F 01 30 30 00 
		00 00 90 7F 01 30 30 00 
		00 00 90 9F 01 30 30 00 
		00 00 90 BF 01 30 30 00 
		00 00 90 DE 03 53 24 00 
		00 00 90 FE 03 2D 24 00 
		00 00 91 1F 01 00 00 00 
		00 00 91 3F 01 00 00 00 
		00 00 80 E2 04 00 00 00 
		00 00 81 00 96 00 FF 00 
	</part>
</rom>
<nvram index="4" size="166"/>
```

The ```<rom index="3">``` section contains data from the MAME hiscore.dat entries.  If CFG_LENGTHWIDTH=1 then the following structure is used:

- 4 bytes		Address of ram entry (in core memory map)
- 1 byte		Length of ram entry in bytes
- 1 byte		Start value to check for at start of address range before proceeding
- 1 byte		End value to check for at end of address range before proceeding
- 1 byte		(padding)

Below is the original hiscore.dat entry for the example above (Bomb Jack)
```
@:maincpu,program,905f,1,30,30
@:maincpu,program,907f,1,30,30
@:maincpu,program,909f,1,30,30
@:maincpu,program,90bf,1,30,30
@:maincpu,program,90de,3,53,24
@:maincpu,program,90fe,3,2d,24
@:maincpu,program,911f,1,00,00
@:maincpu,program,913f,1,00,00
@:maincpu,program,80e2,4,00,00
@:maincpu,program,8100,96,00,ff
```

If CFG_LENGTHWIDTH=2 then the following structure is used:

- 4 bytes		Address of ram entry (in core memory map)
- 2 byte		Length of ram entry in bytes
- 1 byte		Start value to check for at start of address range before proceeding
- 1 byte		End value to check for at end of address range before proceeding

The size attribute in ```<nvram index="4" size="?"> section should be the total of all lengths in the entry table.

### Notes

_Pause_

For cores which require multiplexing due to scores being split across multiple RAM banks, an optional active-high pause signal is triggered while the high score module is reading or writing game RAM.

_Calculating delays_

Delays should be calculated based on the clock passed to the hiscore module - e.g. for a 48MHz clock, a 10 second delay should be 48 \* 1000 \* 1000 \* 10 = 31'h1C9C3800;

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.