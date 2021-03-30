//============================================================================
//  MAME hiscore.dat support for MiSTer arcade cores.
//
//  https://github.com/JimmyStones/Hiscores_MiSTer
//
//  Copyright (c) 2021 Alan Steremberg
//  Copyright (c) 2021 Jim Gregory
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 3 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================
/*
 Version history:
 0001 - 2021-03-06 -	First marked release
 0002 - 2021-03-06 -	Added HS_DUMPFORMAT localparam to identify dump version (for future use)
							Add HS_CONFIGINDEX and HS_DUMPINDEX parameters to configure ioctl_indexes
 0003 - 2021-03-10 -	Added WRITE_REPEATCOUNT and WRITE_REPEATWAIT to handle tricky write situations
 0004 - 2021-03-15 -	Fix ram_access assignment
 0005 - 2021-03-18 -	Add configurable score table width, clean up some stupid mistakes
 0006 - 2021-03-27 -	Move 'tweakable' parameters into MRA data header
============================================================================
*/

module hiscore 
#(
	parameter HS_ADDRESSWIDTH=10,							// Max size of game RAM address for highscores
	parameter HS_SCOREWIDTH=8,								// Max size of capture RAM For highscore data (default 8 = 256 bytes max)
	parameter HS_CONFIGINDEX=3,							// ioctl_index for config transfer
	parameter HS_DUMPINDEX=4,								// ioctl_index for dump transfer
	parameter CFG_ADDRESSWIDTH=4,							// Max size of RAM address for highscore.dat entries (default 4 = 16 entries max)
	parameter CFG_LENGTHWIDTH=1							// Max size of length for each highscore.dat entries (default 1 = 256 bytes max)
)
(
	input										clk,
	input										reset,

	input										ioctl_upload,
	input										ioctl_download,
	input										ioctl_wr,
	input		[24:0]						ioctl_addr,
	input		[7:0]							ioctl_dout,
	input		[7:0]							ioctl_din,
	input		[7:0]							ioctl_index,

	output	[HS_ADDRESSWIDTH-1:0]	ram_address,	// Address in game RAM to read/write score data
	output	[7:0]							data_to_ram,	// Data to write to game RAM
	output	reg							ram_write,		// Write to game RAM (active high)
	output									ram_access		// RAM read or write required (active high)
);

// Parameters read from config header
reg [31:0]	START_WAIT			=1'b0;		// Delay before beginning check process
reg [15:0]	CHECK_WAIT 			=8'hFF;		// Delay between start/end check attempts
reg [15:0]	CHECK_HOLD			=8'd2;		// Hold time for start/end check reads
reg [15:0]	WRITE_HOLD			=8'd2;		// Hold time for game RAM writes 
reg [15:0]	WRITE_REPEATCOUNT	=8'b1;		// Number of times to write score to game RAM
reg [15:0]	WRITE_REPEATWAIT	=8'b1111;	// Delay between subsequent write attempts to game RAM

/*
Hiscore config data structure (version 1)
-----------------------------------------
[16 byte header]
[8 byte * no. of entries]

- Header format
00 00 FF FF 00 FF 00 02 00 02 00 01 11 11 00 00 
[    SW   ] [ CW] [ CH] [ WH] [WRC] [WRW] [PAD]
4 byte		START_WAIT
2 byte		CHECK_WAIT
2 byte		CHECK_HOLD
2 byte		WRITE_HOLD
2 byte		WRITE_REPEATCOUNT
2 byte		WRITE_REPEATWAIT
2 byte		(padding/future use)

- Entry format (when CFG_LENGTHWIDTH=1)
00 00 43 0b  0f    10  01  00
00 00 40 23  02    04  12  00
[   ADDR  ] LEN START END PAD

4 bytes		Address of ram entry (in core memory map)
1 byte		Length of ram entry in bytes 
1 byte		Start value to check for at start of address range before proceeding
1 byte		End value to check for at end of address range before proceeding
1 byte		(padding)

- Entry format (when CFG_LENGTHWIDTH=2)
00 00 43 0b  00 0f    10  01
00 00 40 23  00 02    04  12
[   ADDR  ] [LEN ] START END

4 bytes		Address of ram entry (in core memory map)
2 bytes		Length of ram entry in bytes 
1 byte		Start value to check for at start of address range before proceeding
1 byte		End value to check for at end of address range before proceeding

*/

localparam HS_VERSION			=6;			// Version identifier for module
localparam HS_DUMPFORMAT		=1;			// Version identifier for dump format
localparam HS_HEADERLENGTH		=4'b1111;	// Size of header chunk (default=16 bytes)

// HS_DUMPFORMAT = 1 --> No header, just the extracted hiscore data

// Hiscore config and dump status 
wire				downloading_config;
wire				parsing_header;
wire				downloading_dump;
wire				uploading_dump;
reg				downloaded_config = 1'b0;
reg				downloaded_dump = 1'b0;
reg				uploaded_dump = 1'b0;
reg	[3:0]		initialised;
reg				writing_scores = 1'b0;
reg				checking_scores = 1'b0;

assign downloading_config = ioctl_download && (ioctl_index==HS_CONFIGINDEX);
assign parsing_header = downloading_config && (ioctl_addr<=HS_HEADERLENGTH);
assign downloading_dump = ioctl_download && (ioctl_index==HS_DUMPINDEX);
assign uploading_dump = ioctl_upload && (ioctl_index==HS_DUMPINDEX);
assign ram_access = uploading_dump | writing_scores | checking_scores;
assign ram_address = ram_addr[HS_ADDRESSWIDTH-1:0];

reg	[3:0]								state = 4'b0000;			// Current state machine index
reg	[3:0]								next_state = 4'b0000;	// Next state machine index to move to after wait timer expires
reg	[31:0]							wait_timer;					// Wait timer for inital/read/write delays

reg	[CFG_ADDRESSWIDTH-1:0]		counter = 1'b0;			// Index for current config table entry
reg	[CFG_ADDRESSWIDTH-1:0]		total_entries=1'b0;		// Total count of config table entries
reg										reset_last = 1'b0;		// Last cycle reset
reg	[7:0]								write_counter = 1'b0;	// Index of current game RAM write attempt

reg	[7:0]								last_ioctl_index;			// Last cycle HPS IO index
reg										last_ioctl_download=0;	// Last cycle HPS IO download
reg										last_ioctl_upload=0;		// Last cycle HPS IO upload
reg	[7:0]								last_ioctl_dout;			// Last cycle HPS IO data out
reg	[7:0]								last_ioctl_dout2;			// Last cycle +1 HPS IO data out
reg	[7:0]								last_ioctl_dout3;			// Last cycle +2 HPS IO data out

reg	[24:0]							ram_addr;					// Target RAM address for hiscore read/write
reg	[24:0]							old_io_addr;
reg	[24:0]							base_io_addr;
wire	[23:0]							addr_base;
wire	[(CFG_LENGTHWIDTH*8)-1:0]	length;
wire	[24:0]							end_addr = (addr_base + length - 1'b1);
reg	[HS_SCOREWIDTH-1:0]			local_addr;
wire	[7:0]								start_val;
wire	[7:0]								end_val;

wire [23:0]								address_data_in;
wire [(CFG_LENGTHWIDTH*8)-1:0]	length_data_in;

assign address_data_in = {last_ioctl_dout2, last_ioctl_dout, ioctl_dout};
assign length_data_in = (CFG_LENGTHWIDTH == 1'b1) ? ioctl_dout : {last_ioctl_dout, ioctl_dout};

wire address_we = downloading_config & ~parsing_header & (ioctl_addr[2:0] == 3'd3);
wire length_we = downloading_config & ~parsing_header & (ioctl_addr[2:0] == 3'd3 + CFG_LENGTHWIDTH);
wire startdata_we = downloading_config & ~parsing_header & (ioctl_addr[2:0] == 3'd4 + CFG_LENGTHWIDTH); 
wire enddata_we = downloading_config & ~parsing_header & (ioctl_addr[2:0] == 3'd5 + CFG_LENGTHWIDTH);

// RAM chunks used to store configuration data
// - address_table
// - length_table
// - startdata_table
// - enddata_table
dpram_hs #(.aWidth(CFG_ADDRESSWIDTH),.dWidth(24))
address_table(
	.clk(clk),
	.addr_a(ioctl_addr[CFG_ADDRESSWIDTH+2:3] - 2'd2),
	.we_a(address_we & ioctl_wr),
	.d_a(address_data_in),
	.addr_b(counter),
	.q_b(addr_base)
);
// Length table - variable width depending on CFG_LENGTHWIDTH
dpram_hs #(.aWidth(CFG_ADDRESSWIDTH),.dWidth(CFG_LENGTHWIDTH*8))
length_table(
	.clk(clk),
	.addr_a(ioctl_addr[CFG_ADDRESSWIDTH+2:3] - 2'd2),
	.we_a(length_we & ioctl_wr),
	.d_a(length_data_in),
	.addr_b(counter),
	.q_b(length)
);
dpram_hs #(.aWidth(CFG_ADDRESSWIDTH),.dWidth(8))
startdata_table(
	.clk(clk),
	.addr_a(ioctl_addr[CFG_ADDRESSWIDTH+2:3] - 2'd2),
	.we_a(startdata_we & ioctl_wr), 
	.d_a(ioctl_dout),
	.addr_b(counter),
	.q_b(start_val)
);
dpram_hs #(.aWidth(CFG_ADDRESSWIDTH),.dWidth(8))
enddata_table(
	.clk(clk),
	.addr_a(ioctl_addr[CFG_ADDRESSWIDTH+2:3] - 2'd2),
	.we_a(enddata_we & ioctl_wr),
	.d_a(ioctl_dout),
	.addr_b(counter),
	.q_b(end_val)
);

// RAM chunk used to store hiscore data
dpram_hs #(.aWidth(HS_SCOREWIDTH),.dWidth(8))
hiscoredata (
	.clk(clk),
	.addr_a(ioctl_addr[(HS_SCOREWIDTH-1):0]),
	.we_a(downloading_dump),
	.d_a(ioctl_dout),
	.addr_b(local_addr),
	.we_b(ioctl_upload), 
	.d_b(ioctl_din),
	.q_b(data_to_ram)
);

wire [3:0] header_chunk = ioctl_addr[3:0];

always @(posedge clk)
begin

	if (downloading_config)
	begin
		// Get header chunk data
		if(parsing_header)
		begin
			if(ioctl_wr & (header_chunk == 4'd3)) START_WAIT <= { last_ioctl_dout3, last_ioctl_dout2, last_ioctl_dout, ioctl_dout };
			if(ioctl_wr & (header_chunk == 4'd5)) CHECK_WAIT <= { last_ioctl_dout, ioctl_dout };
			if(ioctl_wr & (header_chunk == 4'd7)) CHECK_HOLD <= { last_ioctl_dout, ioctl_dout };
			if(ioctl_wr & (header_chunk == 4'd9)) WRITE_HOLD <= { last_ioctl_dout, ioctl_dout };
			if(ioctl_wr & (header_chunk == 4'd11)) WRITE_REPEATCOUNT <= { last_ioctl_dout, ioctl_dout };
			if(ioctl_wr & (header_chunk == 4'd13)) WRITE_REPEATWAIT <= { last_ioctl_dout, ioctl_dout };
		end
		else
		begin
			// Keep track of the largest entry during config download
			total_entries <= ioctl_addr[CFG_ADDRESSWIDTH+2:3] - 2'd2;
		end
	end

	// Track completion of configuration and dump download
	if ((last_ioctl_download != ioctl_download) && (ioctl_download == 1'b0))
	begin
		if (last_ioctl_index==HS_CONFIGINDEX) downloaded_config <= 1'b1;
		if (last_ioctl_index==HS_DUMPINDEX) downloaded_dump <= 1'b1;
	end

	// Track completion of dump upload
	if ((last_ioctl_upload != ioctl_upload) && (ioctl_upload == 1'b0))
	begin
		if (last_ioctl_index==HS_DUMPINDEX)
		begin
			uploaded_dump <= 1'b1;
			// Mark uploaded dump as readable in case of reset
			downloaded_dump <= 1'b1;
		end
	end

	// Track last ioctl values 
	last_ioctl_download <= ioctl_download;
	last_ioctl_upload <= ioctl_upload;
	last_ioctl_index <= ioctl_index;
	if(ioctl_download && ioctl_wr)
	begin
		last_ioctl_dout3 = last_ioctl_dout2;
		last_ioctl_dout2 = last_ioctl_dout;
		last_ioctl_dout = ioctl_dout;
	end

	if(downloaded_config)
	begin
		// Check for end of state machine reset to initialise state machine
		reset_last <= reset;
		if (reset_last == 1'b1 && reset == 1'b0)
		begin
			wait_timer <= START_WAIT;
			next_state <= 4'b0000;
			state <= 4'b1111;
			counter <= 1'b0;
			initialised <= initialised + 1'b1;
		end
		else
		begin
			// Upload scores to HPS
			if (uploading_dump == 1'b1)
			begin
				// generate addresses to read high score from game memory. Base addresses off ioctl_address
				if (ioctl_addr == 25'b0) begin
					local_addr <= 0;
					base_io_addr <= 25'b0;
					counter <= 1'b0000;
				end
				// Move to next entry when last address is reached
				if (old_io_addr!=ioctl_addr && ram_addr==end_addr[24:0])
				begin
					counter <= counter + 1'b1;
					base_io_addr <= ioctl_addr;
				end
				// Set game ram address for reading back to HPS
				ram_addr <= addr_base + (ioctl_addr - base_io_addr);
				// Set local addresses to update cached dump in case of reset
				local_addr <= ioctl_addr[HS_SCOREWIDTH-1:0];
			end
			
			if (ioctl_upload == 1'b0 && downloaded_dump == 1'b1 && reset == 1'b0)
			begin
				// State machine to write data to game RAM
				case (state)
					4'b0000: // Start state machine
						begin
						// Setup base addresses
						local_addr <= 0;
						base_io_addr <= 25'b0;
						// Reset entry counter and states
						counter <= 0;
						writing_scores <= 1'b0;
						checking_scores <= 1'b0;
						state <= 4'b0001;
						end

					4'b0001: // Start start/end check run
						begin
						checking_scores <= 1'b1;
						ram_addr <= {1'b0, addr_base};
						state <= 4'b0010;
						wait_timer <= CHECK_HOLD;
						end

					4'b0010: // Start check
						begin
							// Check for matching start value
							if(wait_timer != CHECK_HOLD & ioctl_din == start_val)
							begin
								// Prepare end check
								ram_addr <= end_addr;
								state <= 4'b0100;
								wait_timer <= CHECK_HOLD;
							end
							else
							begin
								ram_addr <= {1'b0, addr_base};
								if (wait_timer > 1'b0)
								begin
									wait_timer <= wait_timer - 1'b1;
								end
								else
								begin
									// - If no match after read wait then stop check run and schedule restart of state machine
									next_state <= 4'b0000;
									state <= 4'b1111;
									checking_scores <= 1'b0;
									wait_timer <= CHECK_WAIT;
								end
							end
						end

					4'b0100: // End check
						begin
							// Check for matching end value
							if (wait_timer != CHECK_HOLD & ioctl_din == end_val)
							begin
								if (counter == total_entries)
								begin
									// If this was the last entry then move to phase II, copying scores into game ram
									checking_scores <= 1'b0;
									state <= 4'b1001;
									counter <= 1'b0;
									write_counter <= 1'b0;
									ram_write <= 1'b0;
									ram_addr <= {1'b0, addr_base};
								end
								else
								begin
									// Increment counter and restart state machine to check next entry
									counter <= counter + 1'b1;
									state <= 4'b0001;
								end
							end
							else
							begin
								ram_addr <= end_addr;
								if (wait_timer > 1'b0)
								begin
									wait_timer <= wait_timer - 1'b1;
								end
								else
								begin
									// - If no match after read wait then stop check run and reset state machine
									state <= 4'b0000;
									checking_scores <= 1'b0;
									wait_timer <= CHECK_WAIT;
								end
							end
						end

					//
					//  this section walks through our temporary ram and copies into game ram
					//  it needs to happen in chunks, because the game ram isn't necessarily consecutive
					4'b0110:
						begin
							local_addr <= local_addr + 1'b1;
							if (ram_addr == end_addr)
							begin
								if (counter == total_entries) 
								begin 
									state <= 4'b1000;
								end
								else
								begin
									// Move to next entry
									counter <= counter + 1'b1;
									write_counter <= 1'b0;
									base_io_addr <= local_addr + 1'b1;
									state <= 4'b1001;
								end
							end 
							else 
							begin
								state <= 4'b1010;
							end
							ram_write <= 1'b0;
						end

					4'b1000: // Hiscore write to RAM completed
						begin
							ram_write <= 1'b0;
							writing_scores <= 1'b0;
							if(write_counter < WRITE_REPEATCOUNT)
							begin
								// Schedule next write
								state <= 4'b1111;
								next_state <= 4'b1001;
								local_addr <= 0;
								wait_timer <= WRITE_REPEATWAIT;
							end
						end

					4'b1001:  // Writing scores to game RAM begins
						begin
							writing_scores <= 1'b1; // indicate that writing has begun, will hold pause until write is complete if hooked up in core
							write_counter <= write_counter + 1'b1;
							state <= 4'b1010;
						end

					4'b1010: // local ram is correct
						begin
							state <= 4'b1110;
							ram_addr <= addr_base + (local_addr - base_io_addr);
							ram_write <= 1'b1;
							wait_timer <= WRITE_HOLD;
						end
						
					4'b1110: // hold write for wait_timer
						begin
							if (wait_timer > 1'b0)
							begin
								wait_timer <= wait_timer - 1'b1;
							end
							else
							begin
								state <= 4'b0110;
							end
						end
						
					4'b1111: // timer wait state
						begin
							if (wait_timer > 1'b0)
								wait_timer <= wait_timer - 1'b1;
							else
								state <= next_state;
						end
				endcase
			end
		end
	end
	old_io_addr<=ioctl_addr;
end

endmodule

module dpram_hs #(
	parameter dWidth=8,
	parameter aWidth=8
)(
	input								clk,

	input			[aWidth-1:0]	addr_a,
	input			[dWidth-1:0]	d_a,
	input								we_a,
	output reg	[dWidth-1:0]	q_a,

	input			[aWidth-1:0]	addr_b,
	input			[dWidth-1:0]	d_b,
	input								we_b,
	output reg	[dWidth-1:0]	q_b
);

reg [dWidth-1:0] ram [2**aWidth-1:0];

always @(posedge clk) begin
	if (we_a) begin 
		ram[addr_a] <= d_a;
		q_a <= d_a;
	end
	else
	begin
		q_a <= ram[addr_a];
	end

	if (we_b) begin 
		ram[addr_b] <= d_b;
		q_b <= d_b;
	end
	else
	begin
		q_b <= ram[addr_b];
	end
end

endmodule