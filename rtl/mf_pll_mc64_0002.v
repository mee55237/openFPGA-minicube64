// ---------------------------------------------------------------------------
// mf_pll_mc64_0002 -- the one PLL the machine runs on.
//
// This is the `altera_pll` megafunction, hand-written in the same shape as
// C:/Code/Wasm4/VexiiRiscv/mf_pllbase_0002.v (a Pocket core that fits and runs
// on hardware). It is deliberately NOT an IP wizard file: nothing here needs
// regenerating, and there is no .qip to get out of step with the QSF.
//
// Outputs, and why each exists:
//
//   outclk_0   6.400000 MHz  -- the machine. CPU, RAM port B, the APU, the
//                               save-state bus, the video generator's counters.
//                               There is no master clock and nothing divides
//                               this: the CPU advances every cycle. VCO / 99.
//   outclk_1   1.320000 MHz  -- video_rgb_clock. One pixel per cycle. VCO / 480,
//                               i.e. 22,000 pixels per frame, i.e. EXACTLY
//                               60.000000 Hz. See THE VCO IS THE DESIGN below.
//   outclk_2   1.320000 MHz  -- video_rgb_clock_90, same rate, +90 degrees.
//                               THIS MUST BE A PLL OUTPUT. apf_top DDR-launches
//                               the RGB and sync bits on the 0-degree clock and
//                               the scaler's own clock on this one, so the
//                               quarter period IS the sampling margin. A
//                               fabric "shifted copy" passes every simulation
//                               and fails on the panel -- both VexiiRiscv and
//                               agg23's NES note the same trap.
//   outclk_3   6.400000 MHz  -- RAM PORT A's clock, +180 degrees. Same rate as
//                               the CPU, half a cycle later: that is what lets
//                               the RAM's own address register latch AB after
//                               it has settled, and a read registered at that
//                               edge (the only form Quartus infers an M10K
//                               from) puts the byte back inside the same CPU
//                               cycle. See mc64_ram.sv and mc64_top.sv.
//                               VCO / 99.
//   outclk_4  12.184615 MHz  -- audio_mclk (VCO / 52). NOT 12.288: see below.
//
// WHY IT USED TO BE 1.28 MHz, since the note in mc64_video.sv points here: at
// 614.4 MHz (the VCO the first fit of this core reported) the machine was
// 614.4/96, the pixel clock 614.4/480 = 1.28 and the audio mclk 614.4/50 =
// 12.288 -- everything exact, and 6.4/1.28 = 5 made a pixel a whole number of
// machine clocks, which mc64_top and the harnesses were written around. What it
// could not do was hit 60 Hz: the rate is the pixel clock divided by the pixel
// count, and with the pixel clock pinned at 1.28 MHz a frame could only hold a
// whole number of pixels, so 1.28e6/60 = 21,333.33 px was the target and the
// nearest whole number -- 21,333 -- is 60.0009 Hz, a frame the scaler drops
// every ~18 minutes. Moving the pixel clock so that a WHOLE number of pixels
// lands exactly on 60 Hz is the whole point here; 22,000 x 60 = 1,320,000.
//
// THE VCO IS THE DESIGN. 633.6 MHz = 74.25 MHz / 15 x 128, and that choice is
// what puts the frame rate on exactly 60.000000 Hz:
//
//     VCO 633.6 MHz / 99  = 6.400000 MHz   machine (the 6502's own rate)
//     VCO 633.6 MHz / 480 = 1.320000 MHz   pixel clock
//     frame = 1.320000 MHz / 22,000 pixels = EXACTLY 60.000000 Hz
//
// A PLL has ONE VCO and every output is that VCO divided by an integer, so the
// machine and pixel clocks can only BOTH be exact when their ratio is a ratio
// of integers -- and the frame rate is the pixel clock divided by the pixel
// count, which is ours to choose. 6.4/1.32 = 99/480 = 33/160 is small integers;
// 6.4/1.28 = 5 was too, and that is exactly what forced the old frame to
// 21,333.33 pixels and the rate to 60.0009 Hz. Refusing to move the pixel
// clock is what made "exactly 60 Hz" look unreachable; the frame length is the
// free parameter, not the pixel clock.
//
// 633.6 / 74.25 = 128/15 EXACTLY, so this needs no fractional-N residue at all:
// the reference divider is 15 and the multiplier 128, both integers, and the
// counters 99/480 are integers inside the range (the previous fit used 480 for
// the same 1.28 MHz output). The rate is exact because the RATIO is exact: both
// clocks are divisions of one VCO, not two separately-quantised frequencies
// that have to agree. fractional_vco_multiplier stays "true" the way the
// sibling core and the previous fit of this file have it, so the assembler is
// free to pick a fractional M/N if it scores better -- it should not need to,
// with an exact integer solution available. READ THE FIT REPORT for outclk_1:
// 1.320000 MHz at a 189.394 ns (90.000000 degree) shift, on a 633.6 MHz VCO.
// A pixel clock a few ppm off moves the frame by the same few ppm and the panel
// never sees it; a VCO that is not 633.6 MHz would mean outclk_1 is not the
// frequency this change exists for, and the counters are then the thing to
// read.
//
// WHAT IT COSTS: the pixel is no longer a whole number of machine clocks. It is
// 160/33 = 4.8485 of them, where 1.28 MHz gave exactly five. So the
// machine-domain pixel tick alternates 5 and 4 clk_core cycles in a fixed
// pattern (33 pixels per 160 machine clocks), which the video generator does not
// care about at all -- it counts PIXELS, one per ce -- and which the DDR
// serializer does not care about either, PROVIDED requirement 1 below is met --
// the pads are fed from this domain, and the pixel edges they change against
// are the pixel clock's:
//
// WHY 633.6 MHz IS THE ONLY VCO THAT CAN DO THIS, and what it cannot do. The
// machine must be exactly 6.4 MHz, the pixel clock must be 60 Hz x a whole pixel
// count (that is what makes the rate exact), and every output is the VCO divided
// by ONE INTEGER C counter, of which this device has 512. So VCO = 6.4k = 1.32m
// with m <= 512, i.e. 160k = 33m, i.e. k = 33n and m = 160n: n = 1 gives
// 633.6 MHz with the counters 99 and 480, and n = 2 gives 844.8 MHz with m = 640
// -- over the limit. One VCO, one answer. What that fixes is the OTHER
// direction: 633.6/6.4 = 99 VCO cycles per machine period is ODD, so an exact
// 180-degree output for the RAM's CPU port no longer exists -- the closest the
// phase shifter can place it is 49 or 50 VCO cycles, i.e. 178.18 or 181.82
// degrees, 0.79 ns from the ideal. (At the previous 614.4 MHz the count was 96
// and 180 was exact; the counter limit, not the phase, is what makes the pair
// 6.4 + 1.32 decide the VCO.) That is requirement 3 below, and it is harmless --
// the port-A path is a half cycle with tens of ns of margin either side -- but
// it is exactly the kind of thing to read back out of the fit report.
//
//   1. WHEN the video data is allowed to change, relative to the pixel edges --
//      and this is now DONE rather than described. The link is DDR: apf_top's
//      `isco` launches video_rgb[23:12] on one edge of video_rgb_clock and
//      [11:0] on the other, and the scaler latches both halves on the
//      90-degree output's edges -- a quarter period after each change, i.e. at
//      25% and 75% of the period. So a change that falls BETWEEN those two
//      captures puts one pixel's high bits on the wire with the next pixel's
//      low bits: a torn pixel, i.e. single-pixel colour specks at every colour
//      edge, on top of a perfectly valid envelope.
//
//      That is exactly what the panel showed, and it is why this core's pads are
//      now driven from the pixel clock's own domain: core_top's last video
//      stage (pix_rgb/pix_de/pix_hs/pix_vs) is ONE register on clk_pixel --
//      which is video_rgb_clock itself -- so all 28 bits change ON the
//      0-degree edge, the centre of the window between the two captures, and
//      are stable everywhere else in the period. That is the arrangement the
//      hardware-proven sibling core uses (VexiiRiscv generates its whole
//      scanout in the pixel domain, and its mf_pllbase_0002.v names the point
//      of the 90-degree clock as "0 deg samples exactly when the data
//      changes"). Before it, the pads were registered in the machine domain on
//      `ce_pix` (an edge detect of clk_pixel two flops into clk_core), so they
//      changed 1-2 machine clocks after every pixel edge: 21%-41% of a
//      4.85-clock period, i.e. inside the capture window for part of every
//      period -- and 40%-60% at the old 5-clocks-per-pixel ratio, which is
//      inside it for ALL of it, which is why the artifact was there before this
//      change as well. This ratio did not create the fault; it removed the one
//      thing that had been hiding its cause.
//
//      The crossing is free in timing terms (the source is stable ≈ 2.8 machine
//      clocks, ≈440 ns, before the capture edge, with no combinational path),
//      and it is asserted in simulation at the level a DDR-free simulator can
//      see: tools/boot_sim.sh now requires every change of {rgb,de,hs,vs,skip}
//      to coincide with a rising edge of video_rgb_clock (0 of ~4,500 changes
//      with this stage; reverting the pads to the machine domain makes 2,584 of
//      4,691 land off the edge, which is the check's own negative control).
//      VIDEO_LINK.md in C:/Code/Wasm4/VexiiRiscv records what the DDR halves
//      are, and why the capture phase must be a quarter period.
//   2. phase_shift2 must be one quarter of the pixel period: 757.5758 ns / 4 =
//      189.394 ns, so 189394 ps. If the pixel frequency ever moves, this moves
//      with it.
//   3. phase_shift3 is REQUESTED as half of the machine period: 156.25 ns / 2 =
//      78.125 ns, so 78125 ps -- and at this VCO it cannot be met exactly (see
//      WHY 633.6 MHz ... above): the machine period is 99 VCO cycles, so half
//      of it is 49.5. Expect the fit report to say 178.18 or 181.82 degrees and
//      NOT 180.000000; anything in that pair is the quantisation working, and
//      the thing to check is that it is one of THOSE and not, say, 0 or 90.
//
// outclk_4 is the one output that is NOT exact: 633.6/52 = 12.184615 MHz, 0.84%
// high (52 is the nearest the VCO can divide to 12.288 -- 633.6/12.288 =
// 51.5625). Nothing hears it: the APU is not in the fit, audio_dac is tied to
// 0, and when the APU is wired the core owns the divider that turns mclk into
// the sample rate, so an exact 48 kHz lrck is still available from this clock.
// The clean answer is a SECOND PLL for audio with its own 12.288 MHz VCO; it
// does not have to share the video VCO, and sharing cannot work: an exact 6.4,
// 1.32 and 12.288 together need a common VCO of 3,379.2 MHz (it is the LCM of
// 800/125, 165/125 and 1536/125 MHz), four times above this device's range. The
// frame rate is the deliverable here, so the audio mclk is the one that gives.
// ---------------------------------------------------------------------------
`timescale 1ns/10ps
module mf_pll_mc64_0002 (

	// interface 'refclk' -- clk_74a, 74.25 MHz
	input wire refclk,

	// interface 'reset'
	input wire rst,

	// interface 'outclk0' -- 6.4 MHz machine
	output wire outclk_0,

	// interface 'outclk1' -- 1.32 MHz pixel clock, 0 degrees
	output wire outclk_1,

	// interface 'outclk2' -- 1.32 MHz pixel clock, +90 degrees
	output wire outclk_2,

	// interface 'outclk3' -- 6.4 MHz, +180 degrees: RAM port A
	output wire outclk_3,

	// interface 'outclk4' -- 12.184615 MHz audio mclk (VCO/52)
	output wire outclk_4,

	// interface 'locked'
	output wire locked
);

	altera_pll #(
		.fractional_vco_multiplier("true"),
		.reference_clock_frequency("74.25 MHz"),
		.operation_mode("normal"),
		.number_of_clocks(5),

		// The machine. 6.4 MHz, the 6502's own rate (machine.c runs the CPU at
		// 6400000 cycles per second), and the RAM's port B clock.
		.output_clock_frequency0("6.400000 MHz"),
		.phase_shift0("0 ps"),
		.duty_cycle0(50),

		// Pixel clock: 22,000 pixels x 60 Hz. 757.5758 ns period. VCO / 480.
		.output_clock_frequency1("1.320000 MHz"),
		.phase_shift1("0 ps"),
		.duty_cycle1(50),

		// The scaler's capture clock: quarter of the pixel period.
		// 757575.8 ps / 4 = 189393.9 -> 189394 ps.
		.output_clock_frequency2("1.320000 MHz"),
		.phase_shift2("189394 ps"),
		.duty_cycle2(50),

		// RAM port A: the machine's own rate, half a cycle later.
		// 156250 ps / 2 = 78125 ps.
		.output_clock_frequency3("6.400000 MHz"),
		.phase_shift3("78125 ps"),
		.duty_cycle3(50),

		// Audio: VCO / 52. Requested as the exact quotient, not 12.288 (which
		// this VCO cannot reach -- see the header).
		.output_clock_frequency4("12.184615 MHz"),
		.phase_shift4("0 ps"),
		.duty_cycle4(50),

		// The IP wants the full clock list even when only five are used.
		.output_clock_frequency5("0 MHz"),
		.phase_shift5("0 ps"),
		.duty_cycle5(50),
		.output_clock_frequency6("0 MHz"),
		.phase_shift6("0 ps"),
		.duty_cycle6(50),
		.output_clock_frequency7("0 MHz"),
		.phase_shift7("0 ps"),
		.duty_cycle7(50),
		.output_clock_frequency8("0 MHz"),
		.phase_shift8("0 ps"),
		.duty_cycle8(50),
		.output_clock_frequency9("0 MHz"),
		.phase_shift9("0 ps"),
		.duty_cycle9(50),
		.output_clock_frequency10("0 MHz"),
		.phase_shift10("0 ps"),
		.duty_cycle10(50),
		.output_clock_frequency11("0 MHz"),
		.phase_shift11("0 ps"),
		.duty_cycle11(50),
		.output_clock_frequency12("0 MHz"),
		.phase_shift12("0 ps"),
		.duty_cycle12(50),
		.output_clock_frequency13("0 MHz"),
		.phase_shift13("0 ps"),
		.duty_cycle13(50),
		.output_clock_frequency14("0 MHz"),
		.phase_shift14("0 ps"),
		.duty_cycle14(50),
		.output_clock_frequency15("0 MHz"),
		.phase_shift15("0 ps"),
		.duty_cycle15(50),
		.output_clock_frequency16("0 MHz"),
		.phase_shift16("0 ps"),
		.duty_cycle16(50),
		.output_clock_frequency17("0 MHz"),
		.phase_shift17("0 ps"),
		.duty_cycle17(50),

		.pll_type("General"),
		.pll_subtype("General")
	) altera_pll_i (
		.rst      (rst),
		.outclk   ({outclk_4, outclk_3, outclk_2, outclk_1, outclk_0}),
		.locked   (locked),
		.fboutclk (),
		.fbclk    (1'b0),
		.refclk   (refclk)
	);
endmodule
