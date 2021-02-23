module hiscore #(parameter ADDRESSWIDTH=10) (
	input				clk,
	input				reset,
	input	[31:0]	delay,
	
	input				ioctl_upload,
	input				ioctl_download,
	input				ioctl_wr,
	input	[24:0]	ioctl_addr,
	input	[7:0]		ioctl_dout,
	input	[7:0]		ioctl_din,
	input	[7:0]		ioctl_index,
	
	output	[ADDRESSWIDTH-1:0]	ram_address,
	output	[7:0]						data_to_ram,
	output	reg						ram_write,
	output	reg						pause
);

/*
00 00 00 0b 0f 10 01 00
00 00 00 23 0f 04 12 00
[ addr (4)]len start end pad
addr -> address of ram (in memory map)
len -> how many bytes
start -> wait for this value at start
end -> wait for this value at end
*/


//reg [25:0] delay_default = 21'h1FFFFF;
//reg [25:0] delay_default = 25'h1FFFFFF; // 2.8 seconds
//reg [25:0] delay_default = 24'hFFFFFF; // 1.4 seconds
//reg [25:0] delay_default = 24'h7FFFFF; // 0.7 seconds
//reg [25:0] delay_default = 24'h3FFFFF; // 0.35 seconds
//reg [25:0] delay_default = 24'h1FFFFF; // 0.175 seconds
reg [31:0] delay_default = 24'h7FFFF; // 0.04 seconds
reg [31:0] read_defaultwait = 24'h7FFFF; // 0.04 seconds
reg [31:0] read_defaultcheck = 4'b1111; 
reg [15:0] read_attempt = 1'b0;

assign ram_address = ram_addr[ADDRESSWIDTH-1:0];

//reg  [7:0] ioctl_dout_r;
reg	[7:0]		ioctl_dout_r2;
reg	[7:0]		ioctl_dout_r3;
reg	[3:0]		state = 4'b0000;
reg	[3:0]		next_state = 4'b0000;
reg	[31:0]	wait_timer;

reg				ram_read = 1'b0;
reg	[3:0]		counter = 4'b0;

reg				reset_last = 1'b0;

reg	[7:0]		last_index;
reg				last_ioctl_download=0;
reg	[24:0]	ram_addr;
reg	[3:0]		total_entries=4'b0;
reg	[24:0]	old_io_addr;
reg	[24:0]	base_io_addr;
reg	[24:0]	end_addr;
reg	[24:0]	local_addr;
wire	[23:0]	addr_base;
wire	[7:0]		length;
wire	[7:0]		start_val;
wire	[7:0]		end_val;

reg				downloading_config;
reg				downloading_dump;
reg				downloaded_config;
reg				downloaded_dump;
reg	[3:0]		initialised;

assign downloading_config = ioctl_download && ioctl_wr && (ioctl_index==3);
assign downloading_dump = ioctl_download && ioctl_wr && (ioctl_index==4);

// RAM chunks used to store configuration data
// - address_table
// - length_table
// - startdata_table
// - enddata_table
dpram #(.aWidth(4),.dWidth(24))
address_table(
	.addr_a(ioctl_addr[6:3]),
	.clk_a(clk),
	.d_a({ioctl_dout_r2,  ioctl_dout_r3, ioctl_dout}), // ignore first byte
	.we_a(downloading_config & ~ioctl_addr[2] &  ioctl_addr[1] & ioctl_addr[0]),
	.clk_b(clk),
	.q_b(addr_base),
	.addr_b(counter)
);

dpram #(.aWidth(4),.dWidth(8))
length_table(
	.addr_a(ioctl_addr[6:3]),
	.clk_a(clk),
	.d_a(ioctl_dout),
	.we_a(downloading_config & ioctl_addr[2] & ~ioctl_addr[1] & ~ioctl_addr[0]), // ADDR b100
	.clk_b(clk),
	.q_b(length),
	.addr_b(counter)
);
dpram #(.aWidth(4),.dWidth(8))
startdata_table(
	.addr_a(ioctl_addr[6:3]),
.clk_a(clk),
	.d_a(ioctl_dout),
	.we_a(downloading_config & ioctl_addr[2] & ~ioctl_addr[1] & ioctl_addr[0]), // ADDR b101
	.clk_b(clk),
	.q_b(start_val),
	.addr_b(counter)
);
dpram #(.aWidth(4),.dWidth(8))
enddata_table(
	.addr_a(ioctl_addr[6:3]),
	.clk_a(clk),
	.d_a(ioctl_dout),
	.we_a(downloading_config & ioctl_addr[2] & ioctl_addr[1] & ~ioctl_addr[0]), // ADDR b110
	.clk_b(clk),
	.q_b(end_val),
	.addr_b(counter)
);

// RAM chunk used to store hiscore data
dpram #(.aWidth(8),.dWidth(8))
hiscoredata (
	.clk_a(clk),
	.we_a(downloading_dump),
	.addr_a(ioctl_addr[7:0]),
	.d_a(ioctl_dout),
	.clk_b(clk),
	.addr_b(local_addr[7:0]),
	.we_b(ioctl_upload), 
	.d_b(ioctl_din),
	.q_b(data_to_ram)
);


always @(posedge clk)
begin
	if (downloading_config)
	begin
		// Save configuration data into tables
		if(ioctl_wr & ~ioctl_addr[2] & ~ioctl_addr[1] &  ioctl_addr[0]) ioctl_dout_r2 <= ioctl_dout;
		if(ioctl_wr & ~ioctl_addr[2] & ioctl_addr[1] & ~ioctl_addr[0]) ioctl_dout_r3 <= ioctl_dout;
		// Keep track of the largest entry during config download
		total_entries <= ioctl_addr[6:3];
	end
	
	// Track completion of configuration and dump download
	if ((last_ioctl_download != ioctl_download) && (ioctl_download == 1'b0))
	begin
		if (last_index==3) downloaded_config <= 1'b1;
		if (last_index==4) downloaded_dump <= 1'b1;
	end

	// Track last ioctl values 
	last_ioctl_download <= ioctl_download;
	last_index <= ioctl_index;

	// Generate last address of entry to check end value
	end_addr <= addr_base + length - 1'b1;

	if(downloaded_config)
	begin
		// Check for state machine initalise/reset
		if (initialised == 1'b0 || (reset_last == 1'b1 && reset == 1'b0))
		begin
			wait_timer = (delay > 1'b0) ? delay : delay_default;
			next_state <= 4'b0000;
			state <= 4'b1111;
			counter <= 4'b0;
			initialised <= initialised + 1'b1;
		end
		reset_last <= reset;

		// activate pause signal when necessary
		pause <= ioctl_upload | ram_write | ram_read;
		
		// Upload scores to HPS
//		if (ioctl_upload == 1'b1 && mode == 1'b0) 
		if (ioctl_upload == 1'b1)
		begin
		
			// generate addresses to read high score from game memory. Base addresses off ioctl_address
			if (ioctl_addr == 25'b0) begin
				local_addr <= 25'b0;
				base_io_addr <= 25'b0;
				counter <= 4'b0000;
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
			local_addr <= ioctl_addr;
			// Mark dump as readable
			downloaded_dump <= 1'b1;
		end
		
		if (ioctl_upload == 1'b0 && downloaded_dump == 1'b1 && reset == 1'b0)
		begin
			// State machine to write data to game RAM
			case (state)
				4'b0000: // Initialise state machine
				begin
					// Setup base addresses
					local_addr <= 25'b0;
					base_io_addr <= 25'b0;
					read_attempt <= 1'b0;
					// Set address for start check
					ram_read <= 1'b0;
					// Set wait timer
					next_state <= 4'b0001;
					state <= 4'b1111;
					wait_timer <= read_defaultwait;
				end

				4'b0001: // Start check prepare and wait
				begin
					// Set start check address, enable ram read and move to start check state
					ram_addr <= {1'b0, addr_base};
					ram_read <= 1'b1;
					state <= 4'b0010;
					wait_timer <= read_defaultcheck;
				end

				4'b0010: // Start check
					begin
						// Check for matching start value
						if(ioctl_din == start_val)
						begin
						// - If match then reset read attempts, stop ram_read and reset timer for end check
							read_attempt <= 1'b0;
							ram_read <= 1'b0;
							next_state <= 4'b0011;
							state <= 4'b1111;
							wait_timer <= read_defaultwait;
						end
						else
						begin
							if (wait_timer > 1'b0)
							begin
								wait_timer <= wait_timer - 1'b1;
							end
							else
							begin
								// - If no match after read wait then stop ram_read and retry
								next_state <= 4'b0001;
								state <= 4'b1111;
								read_attempt <= read_attempt + 1'b1;
								ram_read <= 1'b0;
								wait_timer <= read_defaultwait;
							end
						end
					end

				4'b0011: // End check prepare and wait
				begin
					// Set end check address, enable ram read and move to end check state
					ram_addr <= end_addr;
					ram_read <= 1'b1;
					state <= 4'b0100;
					wait_timer <= read_defaultcheck;
				end
					
					
				4'b0100: // End check
					begin
						// Check for matching end value
						// - If match then move to next state
						// - If no match then go back to previous state
						if (ioctl_din == end_val)
						begin
							if (counter == total_entries)
							begin
								// If this was the last entry then move to phase II, copying scores into game ram
								state <= 4'b1001;
								counter <= 1'b0;
								ram_write <= 1'b0;
								ram_read <= 1'b0;
								ram_addr <= {1'b0, addr_base};
							end
							else
							begin
								// Increment counter and restart state machine to check next entry
								counter <= counter + 1'b1;
								ram_read <= 1'b0;
								state <= 4'b0000;
							end
						end
						else
						begin
							if (wait_timer > 1'b0)
							begin
								wait_timer <= wait_timer - 1'b1;
							end
							else
							begin
								// - If no match after read wait then stop ram_read and retry
								next_state <= 4'b0011;
								state <= 4'b1111;
								read_attempt <= read_attempt + 1'b1;
								ram_read <= 1'b0;
								wait_timer <= read_defaultwait;
							end
						end
					end

				//
				//  this section walks through our temporary ram and copies into game ram
				//  it needs to happen in chunks, because the game ram isn't necessarily consecutive
				4'b0110:
					begin
						local_addr <= local_addr + 1'b1;
						if (ram_addr == end_addr[24:0])
						begin
							if (counter == total_entries) 
							begin 
								state <= 4'b1000;
							end
							else
							begin
								counter <= counter + 1'b1;
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

//				4'b0111:
//					begin // our local ram should be correct, 
//						state <= 4'b0110;
//						ram_addr <= {1'b0, addr_base};
//						ram_write <= 1'b1;
//					end

				4'b1000:
					begin
						// Hiscore write back complete
						ram_write <= 1'b0;
					end

				4'b1001:  // counter is correct, next state the output of our local ram will be correct
					begin
//						state <= 4'b0111;
						state <= 4'b1010;
					end

				4'b1010: // local ram is  correct
					begin
						state <= 4'b1110;
						ram_addr <= addr_base + (local_addr - base_io_addr);
						ram_write <= 1'b1;
					end
					
				4'b1110: // hold write for cycle
					begin
						state <= 4'b0110;
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
	old_io_addr<=ioctl_addr;
end

endmodule