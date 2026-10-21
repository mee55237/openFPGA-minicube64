// ---------------------------------------------------------------------------
// mf_pll_mc64 -- thin wrapper over mf_pll_mc64_0002.
//
// The same two-file split the APF's own PLL IP uses (mf_pllbase.v over
// mf_pllbase_0002.v): keep the megafunction settings in one file and a stable
// module name to instantiate in the other, so the settings can be retuned
// without touching core_top.
// ---------------------------------------------------------------------------
`timescale 1ns/10ps
module mf_pll_mc64 (
	input  wire refclk,   // clk_74a, 74.25 MHz
	input  wire rst,
	output wire outclk_0, // 6.4 MHz machine (CPU, RAM port B, APU, video counters)
	output wire outclk_1, // 1.32 MHz pixel clock (22,000 px = exactly 60 Hz)
	output wire outclk_2, // 1.32 MHz pixel clock, +90 deg (the scaler's clock)
	output wire outclk_3, // 6.4 MHz, +180 deg: RAM port A (the CPU's port)
	output wire outclk_4, // 12.184615 MHz audio mclk (VCO/52; not 12.288)
	output wire locked
);

	mf_pll_mc64_0002 mf_pll_mc64_inst (
		.refclk   (refclk),
		.rst      (rst),
		.outclk_0 (outclk_0),
		.outclk_1 (outclk_1),
		.outclk_2 (outclk_2),
		.outclk_3 (outclk_3),
		.outclk_4 (outclk_4),
		.locked   (locked)
	);

endmodule
