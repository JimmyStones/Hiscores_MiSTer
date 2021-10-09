# Hiscores_MiSTer

## nvram.v

nvram.v is a generic module for MiSTer arcade cores which enables high score save/load for cores in which the original system had NVRAM (or similar) save support.

Created by Jim Gregory ([JimmyStones](https://github.com/jimmystones))

### Features
- Saves high score data in \<MRA name\>.nvm dump (defaults to ioctl index = 4)
- Configurable pause padding around game NVRAM access
- Pause signal to halt CPU for during multiplex with cores already using dual-port RAM
- Scores are extracted every time OSD is opened into temporary buffer, if changed then they are copied to saveable data and auto-save is triggered (optional)
- Optional change mask can be supplied to ignore NVRAM areas to avoid unnecessary autosave events

### History
#### 0001 - 2021-10-05
- First marked release
#### 0002 - 2021-10-09
- Add change mask support

## Implementation instructions

- Add nvram.v to ```rtl/``` folder and files.qip
- Add the following code to the main core .sv file.

```verilog
// HISCORE SYSTEM
// --------------
wire [7:0] hs_address;
wire [7:0] hs_data_out;
wire hs_pause;

nvram #(
	.DUMPWIDTH(8),
	.CONFIGINDEX(3),
	.DUMPINDEX(4),
	.PAUSEPAD(2)
) hi (
	.*,
	.clk(clk_sys),
	.paused(pause_cpu),
	.autosave(status[27]),
	.nvram_address(hs_address),
	.nvram_data_out(hs_data_out),
	.pause_cpu(hs_pause)
);
```
 The module parameters in the example above are the bare minimum config required to customise behaviour for a core.  See [Module parameters](#Module-parameters) for further details.


### Core specific implementation


> **_NOTE:_** It is advisable to implement a working pause system beforehand so the pause_cpu signal can be used to pause the core while reading/writing to game NVRAM to avoid instability/corruption.

Add code to link the high score signals into the relevant NVRAM instances.  

In simple cases (see [Phoenix](https://github.com/MiSTer-devel/Arcade-Phoenix_MiSTer) core for an example) this will only involve converting a working RAM instance from single to dual port.  

### Module parameters

Parameters should be configured to allow the high score system to cope with all games supported by the core.  Maximum dump size, maximum number of entries and largest individual entry length need to be collated for all MRAs before selecting values.

| Parameter | Description 
| ----------| -----------
| DUMPWIDTH | Set to the width of the NVRAM memory used by the core.  
| DUMPINDEX | Optional override of ioctl_index used for nvram dump download (default is 4)
| PAUSEPAD | Number of cycles to pause CPU before and after RAM access attempts (usually 0)

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
| nvram_address    | out       | System RAM address for NVRAM read
| nvram_data_out   | out       | Data from NVRAM
| pause_cpu        | out       | Active-high signal used to pause CPU
| configured       | out       | Active-high signal used to mask autosave menu item if game has no hiscore configuration

### MRA data

To enable the high score module, add the ```<nvram>``` section to the MRA with an index matching that specified in the DUMPINDEX parameter

```xml
<nvram index="4" size="256"/>
```

To enable change mask support, add a ```rom``` section with an index matching that specified in the CONFIGINDEX parameter.  The hex content should represent the bitmap of the check mask required.
```xml
<rom index="3">
	<part>
		FF FF FF FF FF FF FF FF FF FF FF ...
	</part>
</rom>
```

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
