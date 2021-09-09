# Hiscores_MiSTer

This is a generic module for MiSTer arcade cores which enables high score save/load based on the MAME hiscore plugin data format (hiscore.dat).

Created by Alan Steremberg ([alanswx](https://github.com/alanswx))

Further development by Jim Gregory ([JimmyStones](https://github.com/jimmystones))

### Features
- Reads hiscore.dat entries from MRA (defaults to ioctl index = 3)
- Loads and saves high score data in \<MRA name\>.nvm dump (defaults to ioctl index = 4)
- Configurable delay before high score load to game RAM 
- Configurable hold time during hiscore RAM read/write
- Configurable single or double byte entry lengths
- Configurable pause padding around game RAM access
- Pause signal to halt CPU for during multiplex with cores already using dual-port RAM
- Scores are extracted every time OSD is opened into temporary buffer, if changed then they are copied to saveable data and auto-save is triggered (optional)

### History
#### 0001 - 2021-03-06
- First marked release
#### 0002 - 2021-03-06
- Added HS_DUMPFORMAT localparam to identify dump version (for future use).
- Add HS_CONFIGINDEX and HS_DUMPINDEX parameters to configure ioctl_indexes
#### 0003 - 2021-03-10
- Added WRITE_REPEATCOUNT and WRITE_REPEATDELAY to handle tricky write situations
#### 0004 - 2021-03-15
- Fix ram_access assignment
#### 0005 - 2021-03-18
- Add configurable score table width, clean up some stupid mistakes
#### 0006 - 2021-03-27
- Move 'tweakable' parameters into MRA data header
#### 0007 - 2021-04-17
- Convert state machine to use constants
- Add new 'pause padding' states
#### 0008 - 2021-05-12
- Feed back core-level pause to halt startup timer
#### 0009 - 2021-07-31
- Split hiscore extraction from upload (updates hiscore buffer on OSD open)
#### 0010 - 2021-08-03
- Add hiscore extraction buffer and change detection (ready for autosave!)
#### 0011 - 2021-08-07
- Optional auto-save on OSD open
#### 0012 - 2021-08-17
- Add variable length change detection mask
#### 0013 - 2021-09-01
- Output configured signal for autosave option menu masking

## Implementation instructions

- Add hiscore.v to ```rtl/``` folder and files.qip
- Add the following code to the main core .sv file.

```verilog
// HISCORE SYSTEM
// --------------

wire [15:0]hs_address;
wire [7:0]hs_data_out;
wire [7:0]hs_data_in;
wire hs_write_enable;
wire hs_access_read;
wire hs_access_write;
wire hs_pause;
wire hs_configured;

hiscore #(
	.HS_ADDRESSWIDTH(16),
	.CFG_ADDRESSWIDTH(4)
) hi (
	.*
	.clk(clk_sys),
	.paused(pause_cpu),
	.reset(reset),
	.autosave(status[28]),
	.ram_address(hs_address),
	.data_from_ram(hs_data_out),
	.data_to_ram(hs_data_in),
	.data_from_hps(ioctl_dout),
	.data_to_hps(ioctl_din),
	.ram_write(hs_write_enable),
	.ram_intent_read(hs_access_read),
	.ram_intent_write(hs_access_write),
	.pause_cpu(hs_pause),
	.configured(hs_configured)
);
```
 The module parameters in the example above are the bare minimum config required to customise behaviour for a core.  See [Module parameters](#Module-parameters) for further details.


### Core specific implementation


> **_NOTE:_** It is advisable to implement a working pause system beforehand so the pause_cpu signal can be used to pause the core while reading/writing to game RAM to avoid instability/corruption.

Add code to link the high score signals into the relevant RAM instances.  

In simple cases (see [Phoenix](https://github.com/MiSTer-devel/Arcade-Phoenix_MiSTer) core for an example) this will only involve converting a working RAM instance from single to dual port.  

If hiscores are located in multiple RAM areas then some multiplexing will be needed to merge/split based on hs_address.  

If RAM is already dual-ported then the hs_access_read/hs_access_write signals should be used to pause CPU and switch inputs to one of the ports during highscore access (see [Sega System 1](https://github.com/MiSTer-devel/Arcade_SEGASYS1-MiSTer) for a particularly complex example!)

### Module parameters

Parameters should be configured to allow the high score system to cope with all games supported by the core.  Maximum dump size, maximum number of entries and largest individual entry length need to be collated for all MRAs before selecting values.

| Parameter | Description 
| ----------| -----------
| HS_ADDRESSWIDTH | Set to the widest memory address used by the core.  The upper size of hs_address should should be set to the same -1.
| HS_SCOREWIDTH | Set to allow the size of the largest .nvm dump for this core  (default is 8, allowing 256 bytes)
| HS_CONFIGINDEX | Optional override of ioctl_index used for config download (default is 3)
| HS_DUMPINDEX | Optional override of ioctl_index used for nvram dump download (default is 4)
| CFG_ADDRESSWIDTH | Set to allow for the maximum number of hiscore.dat entry lines used by the core (e.g. 4 = 16 max)
| CFG_LENGTHWIDTH | Set to 1 for single byte length, 2 for double

### Module ports
| Port             | Direction | Description 
| ---------------- | --------- | ----------- 
| clk              | in        | System clock.  Should match the HPS IO clock input 
| reset            | in        | Active-high reset signal from core
| paused           | in        | Active-high signal from core confirming CPU is paused
| autosave         | in        | Active-high flag to enable autosave
| ioctl_upload     | in        | From HPS module
| ioctl_upload_req | in        | From HPS module
| ioctl_download   | in        | From HPS module
| ioctl_wr         | in        | From HPS module
| ioctl_addr       | in        | From HPS module
| ioctl_index      | in        | From HPS module
| ram_address      | out       | System RAM address for highscore read/write
| data_to_ram      | out       | Data to send to system RAM (restore scores)
| data_from_ram    | out       | Data from system RAM (extract scores)
| data_to_hps      | out       | Data to send to HPS (save scores)
| data_from_hps    | out       | Data from HPS (load scores)
| ram_write        | out       | Active-high signal to write to system RAM
| ram_intent_read  | out       | Active-high signal used to enable RAM read access
| ram_intent_write | out       | Active-high signal used to enable RAM write access
| pause_cpu        | out       | Active-high signal used to pause CPU
| configured       | out       | Active-high signal used to mask autosave menu item if game has no hiscore configuration

### MRA data

To enable the high score module add the following sections to the MRA.

#### MRA sections
```xml
<rom index="3" md5="none">
	<part>
		00 00 FF FF 00 FF 00 02 00 02 00 01 00 FF 02 00
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

The ```<rom index="3">``` section contains a header section with runtime tweakable parameters, and the entries from the MAME hiscore.dat file.

##### Header data
| Size (bytes) | Parameter | Description 
| --- | ---| ---
4 | START_WAIT | Number of cycles before beginning check process (usually 0)
2 | CHECK_WAIT | Number of cycles to wait between start/end checks (usually 256)
2 | CHECK_HOLD | Number of cycles to wait during start/end checks to allow addresses/multiplexers to settle (usually 2)
2 | WRITE_HOLD | Number of cycles to wait during write to game RAM to allow addresses/multiplexers to settle (usually 2)
2 | WRITE_REPEATCOUNT | Number of times to write score to game RAM - used to brute force cores that don't behave (usually 1)
2 | WRITE_REPEATWAIT | Number of cycles delay between subsequent write attempts
1 | ACCESS_PAUSEPAD | Number of cycles to pause CPU before and after RAM access attempts (usually 0)
1 | CHANGEMASK | Number of bytes to read after header into change detection mask

##### Entry data

If CFG_LENGTHWIDTH=1 then the following structure is used:

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

The size attribute in ```<nvram index="4" size="?">``` should be the **decimal** total of all lengths in the entry table.

### Notes

_Change detection_

Change detection is employed to limit unnecessary autosave events. An optional change detection mask can be provided to stop autosave being triggered by constantly changing data in the hiscore RAM areas.  The final byte of the header specifies the amount of bytes to read into the mask.  Only multiples of 8 are supported.

##### Example:

CHANGEMASK = 8 with the following 8 bytes:
F0 FF FF FF FF FF FF FF

This would result in a mask with the first 4 bits as zero - this would mean any change in the first 4 bytes of the hiscore data dump are ignored.

_Multiplexing_

To help support cores which require multiplexing due to scores being split across multiple RAM banks, or where RAM is already dual-ported, the followinG signals can be used:
 - ram_intent_read - High when hiscore module wants to read from game RAM
 - ram_intent_write - High when hiscore module wants to write to game RAM (ram_write needs to also be high to actually write to game RAM)

_Calculating delays_

Delays should be calculated based on the clock passed to the hiscore module - e.g. for a 48MHz clock, a 10 second delay should be 48 \* 1000 \* 1000 \* 10 = 31'h1C9C3800;

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
