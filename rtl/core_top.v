//
// core_top -- Minicube64 for the Analogue Pocket (openFPGA / APF)
//
// apf_top instantiates this. The port list is the APF's, not ours: every pin
// and every bridge/logical connection apf_top drives has to be here, whether
// or not the machine uses it. Unused ones are tied off in the block below.
//
// WHAT THIS FILE IS
//   The glue between the platform and rtl/mc64_top.sv, which is the machine
//   (6502 + 64 KB RAM + framebuffer, verified bit-exact against the upstream
//   emulator in sim/). Core_top does four things:
//
//     1. clocks -- one PLL (mf_pll_mc64) makes 6.4 MHz for the machine, the
//        same 6.4 MHz a half cycle later for the RAM's CPU port, 1.32 MHz for
//        the pixel link, its 90-degree twin for the scaler, and 12.184615 MHz
//        for the audio master clock (VCO/52; not 12.288 -- see the PLL). There
//        is no master clock and no divided enable: the machine is 6.4 MHz and
//        the CPU advances every cycle. The pixel clock is deliberately NOT a
//        whole division of the machine (160/33, i.e. 4.8485 machine clocks per
//        pixel): that is what buys a frame of exactly 60.000000 Hz with a whole
//        number of pixels -- see the WHY 22,000 note in mc64_video.sv.
//     2. the video link -- forward the machine's picture to the scaler with
//        the sync timing the platform's rules require, and hand the pads to
//        the pixel clock's own domain at the last stage so the DDR halves the
//        platform launches always come from ONE pixel (see the two-stage note
//        at the pads; the machine's picture is generated in clk_core). What
//        the link carries is RGB565 left-aligned in the 24-bit word, with the
//        unused low bits zero -- generated in mc64_video (16-bit palette cache,
//        565 default palette, rgb565_to_wire()), not clamped here: this file
//        has no colour logic at all. The machine's RAM and its model stay 888.
//     3. the cartridge -- receive the .bin the operating system streams over
//        the bridge and write it into the machine's RAM at $0200.
//     4. the bridge, input and audio sides of the APF.
//
// NOT HERE YET (deliberately, not by omission)
//   * the APU: mc64_top currently drives audio_l/audio_r to zero, so audio_dac
//     is silent. The ported APU is rtl/apu.sv and is not in the fit.
//   * save states: savestate_supported is 0 and the handshake is tied off, so
//     the options in the OS show no save state. The MiSTer-style state stack
//     (savestates.vhd and friends) is in rtl/ and commented in the QSF; wiring
//     it up means presenting the machine's RAM through mc64_top's ss_* bus.
//   * real-time clock, target dataslots, data table writes. Tied off.
//
// THE VIDEO RULES, AND WHY THE TIMING LOOKS LIKE THIS
//   The scaler is fed one clock (video_rgb_clock_90) and 24 bits per pixel on
//   the DDR pads clocked by video_rgb_clock (0 degrees). Both clocks come
//   straight from the PLL and must stay that way: a fabric "90 degree copy"
//   passes simulation and fails on the panel -- documented twice over in
//   C:/Code/Wasm4/VexiiRiscv (VIDEO_LINK.md) and in the APF's own NES core.
//
//   The two gap rules that come from the same body of work -- hs at least 3
//   pixel cycles after vs, and at least one DE-low cycle on each side of hs --
//   are met by the GENERATOR itself: mc64_video.sv puts one HS per line in the
//   line's second-to-last cycle (one blank cycle after it), and the VS on the
//   frame's LAST line three cycles before that line's own HS -- so the
//   platform, which takes its window from
//   the vs/hs pair, opens the frame on the picture's row 0 rather than in blank
//   lines. (VS in the gap of the last PICTURE line -- what this core used to
//   do -- passes every counter-based check and puts the whole 64-row window in
//   blanking: sync perfect, picture absent.) The shaper that used to patch this
//   up here is gone, and with it the second clock domain in this file.
//
// THE HAND SHAKE, WHICH IS THE PART THE PANEL READS FIRST
//   Before the OS will show a file browser or stream a cartridge, it asks the
//   core "Request Status", and the answer comes from status_boot_done /
//   status_setup_done / status_running in the block below. Those must be
//   clk_74a signals. The PLL's `locked` is not one -- it is asynchronous to
//   clk_74a even though clk_74a is its reference -- so it is sampled into the
//   domain with two flops before it is used, exactly as the sibling core does.
//   Handing the raw level to the bridge is a status that can read 0 forever,
//   and a Pocket that refuses the handshake is a black panel with no file
//   list: indistinguishable, from the outside, from a dead video link. See
//   `pll_locked_s1/s2` and `core_ready` in the clocks section.
//
// HOW TO CHECK THIS FILE WITHOUT QUARTUS
//   Run the boot-path harness: tools/boot_sim.sh. It drives the APF bridge the
//   way the OS does -- cartridge words into data slot 1, all-complete and reset
//   exit as host commands -- and checks the things that fail on hardware as
//   "black screen with valid sync":
//
//     * that a picture is on the link BEFORE any cartridge exists, pixel-exact
//       against the bring-up blocks below (a dead link fails here rather than
//       passing silently);
//     * that the machine stays in reset until the cartridge has finished
//       arriving (read off `run_core` itself, since de is now live either way);
//     * that every cartridge byte appears in RAM at $0200 with the right value,
//       read back out of the video output -- which with framebuffer bank 0 is a
//       window onto RAM;
//     * that a cartridge loaded AFTER another one starts from the EMULATOR's RAM
//       and not from what the previous one left (the RAM clear below, read out
//       of RAM itself rather than inferred from a picture).
//
//   It runs three orderings, and all three must pass: reset exit present, reset
//   exit absent, and reset_n powered up LOW and left there (silicon's worst
//   case if the power-up value inside core_bridge_cmd were ever dropped).
//   tools/boot_sim.sh runs three more: a PLL that never locks, the overrun
//   control for the alert mark, and the reload ordering above (its own section
//   in that script).
//
//   tools/boot_sim.sh checks the core PACKAGE before it builds anything: the
//   slot address this file's loader listens on is window 1 (bridge_addr[31:28]
//   == 4'h1), so pocket/data.json -- the file the OS reads, and the one
//   tools/stage_bitstream.sh stages -- must say "address": "0x10000000", or the
//   OS streams the cartridge where this core is not listening. It also reports
//   the slot's `parameters`: bit6 ("Reset core while loading") decides whether
//   the OS sends [0010]/[0011] around a load, which is the difference between
//   the harness's orderings 1 and 2. The harness itself cannot see the package
//   (it drives its slot address as a constant), so the check lives in the
//   script rather than in a comment claiming the C++ does it.
//
//   Lint it with Verilator: --lint-only --top-module core_top +define+MC64_SIM
//   +incdir+rtl, over rtl/core_top.v, rtl/core_bridge_cmd.v, rtl/common.v and
//   the machine's five files -- and do NOT pass -Wno-MULTIDRIVEN. The fit
//   rejects multiple drivers with Error 10028 (two always blocks writing one
//   register), and that warning is the only place it shows up beforehand.
//
//   The CPU's frame is the emulator's: exec6502(6400000/60) = 106,666 machine
//   clocks per vblank, counted in mc64_top and compared against the emulator to
//   the cycle by tools/sim.sh. The VIDEO frame is the generator's own and is
//   22,000 pixels of the PLL's 1.32 MHz = 106,666.67 machine clocks =
//   60.000000 Hz exactly -- 0.67 of a clock under the CPU's frame, i.e. 6.3 ppm
//   -- so a game that draws straight into VRAM has its tear creep one picture
//   line per ~10.7 s (~11 minutes across the 64-line picture) where the panel's
//   136-clocks-per-frame geometry slewed a line every 2.5 frames. Every line
//   is 88 pixel cycles and the frame is 250 of them, so no line of a frame is
//   longer than any other. The rate is a
//   deliverable and not a by-product:
//   the geometry this replaced ran 318 uniform lines at 60.077 Hz, which every
//   check this core had passed and the panel reported anyway, and no pixel count
//   is exactly 60 Hz while a pixel is a whole number of 6.4 MHz machine clocks.
//   See the WHY 22,000 note in mc64_video.sv, and tools/video_sim.sh, which
//   asserts the rate to 0.01 Hz.

`default_nettype none

module core_top (

    //
    // physical connections
    //

    ///////////////////////////////////////////////////
    // clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

    input wire clk_74a,  // mainclk1
    input wire clk_74b,  // mainclk1

    ///////////////////////////////////////////////////
    // cartridge interface
    // switches between 3.3v and 5v mechanically
    // output enable for multibit translators controlled by pic32

    // GBA AD[15:8]
    inout  wire [7:0] cart_tran_bank2,
    output wire       cart_tran_bank2_dir,

    // GBA AD[7:0]
    inout  wire [7:0] cart_tran_bank3,
    output wire       cart_tran_bank3_dir,

    // GBA A[23:16]
    inout  wire [7:0] cart_tran_bank1,
    output wire       cart_tran_bank1_dir,

    // GBA [7] PHI#
    // GBA [6] WR#
    // GBA [5] RD#
    // GBA [4] CS1#/CS#
    //     [3:0] unwired
    inout  wire [7:4] cart_tran_bank0,
    output wire       cart_tran_bank0_dir,

    // GBA CS2#/RES#
    inout  wire cart_tran_pin30,
    output wire cart_tran_pin30_dir,
    // when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
    // the goal is that when unconfigured, the FPGA weak pullups won't interfere.
    // thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
    // and general IO drive this pin.
    output wire cart_pin30_pwroff_reset,

    // GBA IRQ/DRQ
    inout  wire cart_tran_pin31,
    output wire cart_tran_pin31_dir,

    // infrared
    input  wire port_ir_rx,
    output wire port_ir_tx,
    output wire port_ir_rx_disable,

    // GBA link port
    inout  wire port_tran_si,
    output wire port_tran_si_dir,
    inout  wire port_tran_so,
    output wire port_tran_so_dir,
    inout  wire port_tran_sck,
    output wire port_tran_sck_dir,
    inout  wire port_tran_sd,
    output wire port_tran_sd_dir,

    ///////////////////////////////////////////////////
    // cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)
    // Minicube64 has a flat 64 KB inside the FPGA and uses none of it.

    output wire [21:16] cram0_a,
    inout  wire [ 15:0] cram0_dq,
    input  wire         cram0_wait,
    output wire         cram0_clk,
    output wire         cram0_adv_n,
    output wire         cram0_cre,
    output wire         cram0_ce0_n,
    output wire         cram0_ce1_n,
    output wire         cram0_oe_n,
    output wire         cram0_we_n,
    output wire         cram0_ub_n,
    output wire         cram0_lb_n,

    output wire [21:16] cram1_a,
    inout  wire [ 15:0] cram1_dq,
    input  wire         cram1_wait,
    output wire         cram1_clk,
    output wire         cram1_adv_n,
    output wire         cram1_cre,
    output wire         cram1_ce0_n,
    output wire         cram1_ce1_n,
    output wire         cram1_oe_n,
    output wire         cram1_we_n,
    output wire         cram1_ub_n,
    output wire         cram1_lb_n,

    ///////////////////////////////////////////////////
    // sdram, 512mbit 16bit -- unused

    output wire [12:0] dram_a,
    output wire [ 1:0] dram_ba,
    inout  wire [15:0] dram_dq,
    output wire [ 1:0] dram_dqm,
    output wire        dram_clk,
    output wire        dram_cke,
    output wire        dram_ras_n,
    output wire        dram_cas_n,
    output wire        dram_we_n,

    ///////////////////////////////////////////////////
    // sram, 1mbit 16bit -- unused

    output wire [16:0] sram_a,
    inout  wire [15:0] sram_dq,
    output wire        sram_oe_n,
    output wire        sram_we_n,
    output wire        sram_ub_n,
    output wire        sram_lb_n,

    ///////////////////////////////////////////////////
    // vblank driven by dock for sync in a certain mode

    input wire vblank,

    ///////////////////////////////////////////////////
    // i/o to 6515D breakout usb uart

    output wire dbg_tx,
    input  wire dbg_rx,

    ///////////////////////////////////////////////////
    // i/o pads near jtag connector user can solder to

    output wire user1,
    input  wire user2,

    ///////////////////////////////////////////////////
    // RFU internal i2c bus

    inout  wire aux_sda,
    output wire aux_scl,

    ///////////////////////////////////////////////////
    // RFU, do not use
    output wire vpll_feed,


    //
    // logical connections
    //

    ///////////////////////////////////////////////////
    // video, audio output to scaler
    output wire [23:0] video_rgb,
    output wire        video_rgb_clock,
    output wire        video_rgb_clock_90,
    output wire        video_de,
    output wire        video_skip,
    output wire        video_vs,
    output wire        video_hs,

    output wire audio_mclk,
    input  wire audio_adc,
    output wire audio_dac,
    output wire audio_lrck,

    ///////////////////////////////////////////////////
    // bridge bus connection
    // synchronous to clk_74a
    output wire        bridge_endian_little,
    input  wire [31:0] bridge_addr,
    input  wire        bridge_rd,
    output reg  [31:0] bridge_rd_data,
    input  wire        bridge_wr,
    input  wire [31:0] bridge_wr_data,

    ///////////////////////////////////////////////////
    // controller data
    //
    // key bitmap:
    //   [0]    dpad_up
    //   [1]    dpad_down
    //   [2]    dpad_left
    //   [3]    dpad_right
    //   [4]    face_a
    //   [5]    face_b
    //   [6]    face_x
    //   [7]    face_y
    //   [8]    trig_l1
    //   [9]    trig_r1
    //   [10]   trig_l2
    //   [11]   trig_r2
    //   [12]   trig_l3
    //   [13]   trig_r3
    //   [14]   face_select
    //   [15]   face_start
    // joy values - unsigned
    //   [ 7: 0] lstick_x
    //   [15: 8] lstick_y
    //   [23:16] rstick_x
    //   [31:24] rstick_y
    // trigger values - unsigned
    //   [ 7: 0] ltrig
    //   [15: 8] rtrig
    //
    input wire [15:0] cont1_key,
    input wire [15:0] cont2_key,
    input wire [15:0] cont3_key,
    input wire [15:0] cont4_key,
    input wire [31:0] cont1_joy,
    input wire [31:0] cont2_joy,
    input wire [31:0] cont3_joy,
    input wire [31:0] cont4_joy,
    input wire [15:0] cont1_trig,
    input wire [15:0] cont2_trig,
    input wire [15:0] cont3_trig,
    input wire [15:0] cont4_trig

`ifdef MC64_SIM
    //
    // Simulation-only observation taps. Combinational reads of signals that
    // already exist, so they cannot perturb behaviour. The boot harness needs
    // both `run_core` (the machine's release) and `cart_ready` (the gate that
    // opens first, and the one the host's own all-complete announcement opens
    // without waiting for the quiet window) as separate observables.
    ,
    output wire        dbg_run_core,
    output wire [5:0]  dbg_boot_state,
    output wire        dbg_cart_ready,
    output wire        dbg_core_ready,
    // Cartridge writes the loader could not take (see the FIFO note in the
    // cartridge section). Must be 0: anything else means the file in RAM is
    // known to be incomplete and the picture cannot be trusted to mean
    // anything.
    output wire [15:0] dbg_cart_drop,
    // ...and the same fact as the alert mark sees it: after the one-bit latch
    // in clk_74a and the two flops into the pixel domain (`drop_pix`). The
    // harness checks this separately from the pixels so "the wiring is there"
    // and "the mark is on the link" cannot be confused for one another.
    output wire        dbg_cart_incomplete,
    // The machine's own 64 KB, through mc64_top's simulation-only back door
    // (mc64_ram's `assign dbg_data = mem[dbg_addr]`). The boot harness drives
    // this address and reads the byte, which is how the controller check reads
    // the machine's $0102 back out again: the cartridge it streams copies
    // $0102/$0103 to $0A00/$0A01 for exactly that (see sim/tb_boot.cpp), so a
    // key the core never passed on shows up as a byte the cartridge never
    // could have written. Combinational, simulation-only, absent from a fit.
    input  wire [15:0] dbg_ram_addr,
    output wire [ 7:0] dbg_ram_data,
    // The CPU's architectural state, for the save-state harness: the machine
    // resolves a state against these and so does the test
    // (sim/tb_savestate.cpp). `pc` is the core's own PC, i.e. the address being
    // fetched; a state's PC field is that minus one.
    output wire [15:0] dbg_mc64_pc,
    output wire  [7:0] dbg_mc64_a,
    output wire  [7:0] dbg_mc64_s,
    output wire  [7:0] dbg_mc64_p,
    /* The machine's five RAM MIRRORS out of mc64_top (its shadow block):
     * { dmc_bank, irqvec_hi, irqvec_lo, pal_bank, vram_bank }. Not RAM,
     * so the RAM dump every other check reads cannot see them -- which is
     * how a mirror can outlive the byte it mirrors, or never follow the CPU
     * at all, with the whole suite green. sim/tb_boot.cpp's reload ordering
     * reads this while each cartridge is up. Simulation-only. */
    output wire [39:0] dbg_ram_mirrors,
    /* The RASTER'S OWN PAIR: what the video is drawing through, not what the
     * machine's shadows say. `vram_bank_q`/`pal_bank_q` are latched once a
     * frame, so a load that restores the shadows and not these gives a wrong
     * picture on a machine whose RAM and mirrors are perfect -- and every check
     * above is blind to it. sim/tb_savestate.cpp's MC64_SS_BANK reads this.
     * Simulation-only, like the rest of this block. */
    output wire  [7:0] dbg_mc64_vram_bank_q,
    output wire  [7:0] dbg_mc64_pal_bank_q,
    /* WHAT THE VIDEO'S RAM PORT B ASKED FOR, AND WHAT IT GOT. The pixel the
     * link paints is `pal[ram_rdata]`, and a byte taken from the wrong place --
     * a fetch stolen by the DMC's sample borrow, an address a pipeline level
     * out, a bank the raster is not drawing through -- looks exactly like a
     * correct picture with a few wrong colours in it. sim/tb_savestate.cpp's
     * picture check reads these to say WHICH of those it is (2026-10-19).
     * Simulation-only, like the rest of this block. */
    output wire [15:0] dbg_mc64_vid_addr,
    output wire  [7:0] dbg_mc64_vid_rdata,
    /* ONE PULSE PER PICTURE PIXEL, in clk_core. `video_de` is a LEVEL held for a
     * whole pixel period, so a harness that counts DE-high clocks counts the
     * same pixel many times; this is the edge the video's own counters advance
     * on, i.e. the rate the scaler samples at. Simulation-only. */
    output wire        dbg_mc64_ce_pix,
    // The state path's two canaries: a read the engine had not produced yet,
    // and a write the FIFO could not take. Both must stay 0 -- a state that
    // went through either is known to be incomplete (see
    // rtl/save_state_controller.sv).
    output wire        dbg_ss_underrun,
    output wire        dbg_ss_overrun,
    // TEMPORARY (remove me): the pause's own two signals, so the harness can
    // see which source is holding the machine and whether the escape fired.
    output wire        dbg_pause_req,
    output wire        dbg_os_menu_en,
    output wire        dbg_ss_freeze,
    output wire        dbg_pause_hold,
    output wire        dbg_pause_freeze,
    output wire        dbg_pause_muted,
    output wire        dbg_ss_loading,
    /* The load's picture rule: the bytes the raster draws from are rewritten
     * only while `dbg_vid_blank` (the raster's blanking slot) is high, and
     * `dbg_ss_flushing` is that write. sim/tb_savestate.cpp checks it while a
     * load runs -- the panel's `it looks corrupted while it loads`. */
    output wire        dbg_ss_flushing,
    output wire        dbg_vid_blank,
    output wire        dbg_pause_low_sat,
    output wire [16:0] dbg_frame_cnt
`endif

);

  // ------------------------------------------------------------------ tie-offs
  //
  // Everything the machine does not use, driven to the state the Pocket expects
  // from a core that is present but not using the pin.

  // not using the IR port, so turn off both the LED, and
  // disable the receive circuit to save power
  assign port_ir_tx              = 0;
  assign port_ir_rx_disable      = 1;

  // bridge endianness: LITTLE. This is the one that decides the byte order of
  // the cartridge stream (see the loader below): with 1, the first file byte of
  // each 32-bit transfer arrives in bridge_wr_data[7:0]. VexiiRiscv declares 1
  // for the same reason.
  assign bridge_endian_little    = 1;

  // cart is unused, so set all level translators accordingly
  // directions are 0:IN, 1:OUT
  assign cart_tran_bank3         = 8'hzz;
  assign cart_tran_bank3_dir     = 1'b0;
  assign cart_tran_bank2         = 8'hzz;
  assign cart_tran_bank2_dir     = 1'b0;
  assign cart_tran_bank1         = 8'hzz;
  assign cart_tran_bank1_dir     = 1'b0;
  assign cart_tran_bank0         = 4'hf;
  assign cart_tran_bank0_dir     = 1'b1;
  assign cart_tran_pin30         = 1'b0;  // reset or cs2, we let the hw control it by itself
  assign cart_tran_pin30_dir     = 1'bz;
  assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
  assign cart_tran_pin31         = 1'bz;  // input
  assign cart_tran_pin31_dir     = 1'b0;  // input

  // link port is input only
  assign port_tran_so            = 1'bz;
  assign port_tran_so_dir        = 1'b0;  // SO is output only
  assign port_tran_si            = 1'bz;
  assign port_tran_si_dir        = 1'b0;  // SI is input only
  assign port_tran_sck           = 1'bz;
  assign port_tran_sck_dir       = 1'b0;  // clock direction can change
  assign port_tran_sd            = 1'bz;
  assign port_tran_sd_dir        = 1'b0;  // SD is input and not used

  // the two cellular psram chips and the external sram are not fitted to this
  // core at all; leave their buses high-Z and their controls deasserted
  assign cram0_a                 = 'h0;
  assign cram0_dq                = {16{1'bZ}};
  assign cram0_clk               = 0;
  assign cram0_adv_n             = 1;
  assign cram0_cre               = 0;
  assign cram0_ce0_n             = 1;
  assign cram0_ce1_n             = 1;
  assign cram0_oe_n              = 1;
  assign cram0_we_n              = 1;
  assign cram0_ub_n              = 1;
  assign cram0_lb_n              = 1;

  assign cram1_a                 = 'h0;
  assign cram1_dq                = {16{1'bZ}};
  assign cram1_clk               = 0;
  assign cram1_adv_n             = 1;
  assign cram1_cre               = 0;
  assign cram1_ce0_n             = 1;
  assign cram1_ce1_n             = 1;
  assign cram1_oe_n              = 1;
  assign cram1_we_n              = 1;
  assign cram1_ub_n              = 1;
  assign cram1_lb_n              = 1;

  assign dram_a                  = 'h0;
  assign dram_ba                 = 'h0;
  assign dram_dq                 = {16{1'bZ}};
  assign dram_dqm                = 2'h3;
  assign dram_clk                = 0;
  assign dram_cke                = 0;
  assign dram_ras_n              = 1;
  assign dram_cas_n              = 1;
  assign dram_we_n               = 1;

  assign sram_a                  = 'h0;
  assign sram_dq                 = {16{1'bZ}};
  assign sram_oe_n               = 1;
  assign sram_we_n               = 1;
  assign sram_ub_n               = 1;
  assign sram_lb_n               = 1;

  assign dbg_tx                  = 1'bZ;
  assign user1                   = 1'bZ;
  assign aux_scl                 = 1'bZ;
  assign vpll_feed               = 1'bZ;

  // the scaler is fed one picture at the frame origin and nothing else
  assign video_skip              = 1'b0;

  // ------------------------------------------------------------------- clocks
  //
  // One PLL, five outputs at three rates. See rtl/mf_pll_mc64_0002.v for the
  // settings and for the two relationships that must hold: the 90-degree output
  // at a quarter of the pixel period, and the RAM clock half a machine period
  // after the machine (both are PLL phase_shift values, not fabric delays).

  wire clk_core;       // 6.4 MHz, the machine: CPU, RAM port B, APU, video
  wire clk_ram;        // 6.4 MHz, +180 degrees: RAM port A, the CPU's port
  wire clk_pixel;      // 1.32 MHz, video link (0 degrees) -- 22,000 px/frame
  wire clk_pixel_90;   // 1.32 MHz, +90 degrees, the scaler's capture clock
  wire clk_audio;      // 12.184615 MHz, audio mclk (VCO/52; not 12.288)
  wire pll_locked_raw; // straight out of the megafunction (or the sim's model)
  wire pll_locked;     // ... and that level sampled into clk_74a, below

  reg  pll_reset = 1'b0;

  assign video_rgb_clock    = clk_pixel;
  assign video_rgb_clock_90 = clk_pixel_90;

`ifdef MC64_SIM
  // Simulation has no altera_pll (it is an Altera megafunction and only
  // resolves inside Quartus), so simulation divides clk_74a instead, and models
  // the lock. The ratios here are NOT the fit's and do not need to be: the
  // machine harness models the pixel tick itself (sim/tb_mc64.cpp), and the
  // only thing core_top does with clk_pixel is time a toggle that mc64_top
  // samples as `ce_pix` -- one pulse per pixel edge, whatever the ratio. The
  // real 160/33 is a PLL fact and has no model here. Nothing in this block
  // reaches the fit. The 90-degree output has no phase in simulation; core_top
  // only forwards it, so there is nothing to model.
  reg [4:0] simdiv  = 5'd0;
  reg [3:0] simlock = 4'd0;
  reg [2:0] simpix  = 3'd0;

  always @(posedge clk_74a) begin
    simdiv  <= simdiv + 5'd1;
    if (&simlock) simlock <= simlock; else simlock <= simlock + 4'd1;
  end

  assign clk_core     = simdiv[2];      // clk_74a / 8
  assign clk_ram      = ~clk_core;      // the half-cycle-later RAM clock
  assign clk_audio    = simdiv[3];      // clk_74a / 16, just has to tick

  // ---- the pixel clock, deliberately WITHOUT a phase model for its twin ----
  //
  // Five clk_core per pixel, not a power of two, because that is what the fit's
  // 160/33 = 4.8485 is: with a /5 counter the machine-domain picture changes
  // 2 of 5 clk_core (40%) after a pixel edge, which is INSIDE the window
  // between the two DDR launches, as the real ratio's 21-41% is -- so a harness
  // checking the DDR halves reproduces the torn-pixel fault instead of sitting
  // on the boundary of the test. (With the old /8 divider the change landed at
  // exactly 25%, i.e. exactly on a capture instant, and the check could not see
  // it: that is why this divider was changed with the check.)
  //
  // clk_pixel_90 gets NO PHASE, and that is a decision rather than an omission.
  // ON HARDWARE IT MUST BE A PLL OUTPUT -- a fabric copy of a clock phase fails
  // on the panel, documented twice in C:/Code/Wasm4/VexiiRiscv (VIDEO_LINK.md,
  // and v5/v6 in HARDWARE_TEST.md) -- and no simulator can validate a phase.
  // What simulation CAN check is the requirement the phase exists to serve, and
  // without modelling the phase at all: the link is DDR (apf_top launches
  // datain_h on one edge of clk_pixel and datain_l on the other), so the two
  // samples must carry ONE picture pixel -- see pad_sample in sim/tb_boot.cpp,
  // which compares the word at each rising edge with the word at the following
  // falling edge. That is convention-free (it does not matter which half is
  // which) and it is strictly the platform's requirement; a phase model here
  // would add nothing to it.
  wire [2:0] pix_phase = simpix[2:0];
  assign clk_pixel    = (pix_phase < 3'd2);  // 2 of 5 clk_core: 40% duty
  assign clk_pixel_90 = clk_pixel;           // no phase in simulation

  always @(posedge clk_core) simpix <= simpix + 3'd1;

`ifdef MC64_SIM_PLL_DEAD
  // A PLL that never locks. The one clock a real core always has is clk_74a, so
  // this is the case `core_ready` exists for: the OS must still get its
  // handshake, and the link must still carry the machine's own picture with the
  // PLL bit of `dbg_boot_state` dark. tools/boot_sim.sh builds and requires this
  // ordering -- without it the
  // timer half of `core_ready` is never exercised, because the lock always wins.
  assign pll_locked_raw = 1'b0;
`else
  assign pll_locked_raw = &simlock;
`endif
`else
  mf_pll_mc64 pll (
      .refclk   (clk_74a),
      .rst      (pll_reset),
      .outclk_0 (clk_core),       // 6.4 MHz, the machine
      .outclk_1 (clk_pixel),      // 1.32 MHz, the video link
      .outclk_2 (clk_pixel_90),   // 1.32 MHz, +90 degrees
      .outclk_3 (clk_ram),        // 6.4 MHz, +180 degrees: RAM port A
      .outclk_4 (clk_audio),      // 12.184615 MHz, audio mclk (VCO/52)
      .locked   (pll_locked_raw)
  );
`endif

  // ------------------------------------------------- the lock, into clk_74a
  //
  // THIS IS THE ONE THAT COST A BLACK SCREEN.
  //
  // The PLL's `locked` leaves the megafunction in the PLL's own output-clock
  // domain. Its refclk happens to be clk_74a, but locked asserts asynchronously
  // after the VCO settles -- it is not a clk_74a signal just because the
  // reference is. Everything that reads it here (status_boot_done,
  // status_setup_done, the power-on counter, run_core)
  // is in clk_74a, and the bridge forwards it *combinationally* to the OS.
  //
  // The sibling core that runs on this hardware found this the hard way and
  // recorded the symptom in its own source: "Without this the status reads 0
  // forever and the Pocket OS refuses the handshake (host commands ignored)" --
  // see C:/Code/Wasm4/VexiiRiscv/core_top.v, pll_locked_s1/s2. The Pocket OS
  // asks Request Status before it will load anything at all; a status that
  // never becomes 1 means no file browser, no cartridge, no picture, and a
  // panel that is black whether or not the machine underneath is healthy.
  //
  // Two flops, so the level is *sampled* rather than merely routed. Do not let
  // this be optimised into a wire: the whole value is that the reading is taken
  // on a clk_74a edge.
  reg pll_locked_s1 = 1'b0;
  reg pll_locked_s2 = 1'b0;

  always @(posedge clk_74a) begin
    pll_locked_s1 <= pll_locked_raw;
    pll_locked_s2 <= pll_locked_s1;
  end

  assign pll_locked = pll_locked_s2;

  // ---- and the lock, LATCHED, for everything that holds the machine down ----
  //
  // `pll_locked` is a LIVE level, and a lock detector on a fractional-N PLL can
  // drop for a few reference cycles and re-assert. This core used to build the
  // machine's only release out of that live level
  //
  //     run_core = pll_locked && por_done && cart_ready && ...
  //
  // so every glitch of the lock reset the whole machine: run_core dropped, the
  // POR window restarted, `machine_released` pulsed, the raster restarted and
  // the machine fell back to its held picture -- a picture that flickers instead
  // of a machine that runs, with a perfect bitstream, a locked PLL and every
  // harness green.
  //
  // The sibling core that runs on this hardware never does that: its CPU reset
  // is a timer alone (`por_reset_n` in C:/Code/Wasm4/VexiiRiscv/core_top.v) and
  // its lock signal feeds only the OS status bits. Same split here:
  //
  //   pll_locked          live, unsynchronised-to-the-machine: the OS status
  //                       bits, and bit 0 of `boot_state` (the debug tap), so a
  //                       PLL that never locks is still reported as one
  //   pll_locked_sticky   latched on first lock, never cleared: what the
  //                       power-on window and the machine's release gate use,
  //                       so no glitch can reset a running machine
  //
  // A PLL that is genuinely down is not made harder to see by this: every clock
  // in the machine comes from it, so the machine stops anyway, and bit 0 of
  // `boot_state` still reports the live level.
  reg pll_locked_sticky = 1'b0;

  always @(posedge clk_74a)
    if (pll_locked)
      pll_locked_sticky <= 1'b1;

  // ------------------------------------------------ what "ready" means here
  //
  // status_boot_done / status_setup_done are what the OS reads before it will
  // offer a cartridge. The framework's reference core and the sibling core both
  // take them straight from the PLL lock, and when the PLL locks that is the
  // right answer -- this keeps it the answer, because the lock wins the race in
  // well under a millisecond.
  //
  // What it protects against is the PLL that never locks. On that panel the
  // handshake would never happen, and the machine's picture -- which is clocked
  // by the PLL -- would not be visible either, so a dead PLL, a dead bridge and
  // a dead loader would all look exactly alike from the outside: black, with no
  // file list. Two panel tests have already been spent on that.
  //
  // clk_74a is the one clock that exists before any PLL, so a timer on it
  // reports the truth about the bridge whatever the PLL does. With it, the
  // failure tells itself apart:
  //
  //   file list appears, then the blocks   the handshake path is alive; read
  //                                        the blocks (block 0 dark = the PLL)
  //   no file list at all                  the bridge or clk_74a is the fault,
  //                                        and the machine is not implicated
  localparam [19:0] BOOT_WAIT_MAX = 20'h3FFFF;   // ~7 ms at 74.25 MHz

  reg  [19:0] boot_wait  = 20'h0;
  wire        boot_timer = (boot_wait == BOOT_WAIT_MAX);
  wire        core_ready = pll_locked | boot_timer;

  always @(posedge clk_74a) begin
    if (!boot_timer)
      boot_wait <= boot_wait + 20'd1;
  end

  // -------------------------------------------------------------- host control
  //
  // cart is not present until the OS has handed one over, and the machine is
  // held in reset while it is being handed over (mc64_top only accepts
  // cartridge writes while reset is asserted, which is what makes the loader
  // below safe).

  reg  [31:0] reset_delay = 0;

  // ONE always block, the way the framework's own core_top has it: the OS's
  // 0x050 "reset the core" command reloads the counter, and the counter runs
  // itself down. Two blocks writing this register is a multiple-constant-driver
  // error in Analysis & Synthesis (Error 10028), so keep them together.
  always @(posedge clk_74a) begin
    if (reset_delay > 0)
      reset_delay <= reset_delay - 1;

    if (bridge_wr && bridge_addr == 32'h00000050)
      reset_delay <= 32'h100000;
  end

  wire external_reset = reset_delay > 0;

  // bridge read data: the command block (0xF8xxxxxx, from core_bridge_cmd) and
  // the save-state buffer (0x4xxxxxxx, the window `savestate_addr` declares).
  // The buffer's bytes come straight out of the controller's holding register,
  // which is why this is combinational and not registered: the state byte the
  // host is being handed was made ready on the previous access, so the answer
  // is stable for the whole cycle it is asked in.
  //
  // 0x1xxxxxxx (the cartridge slot) is a write window and 0x2xxxxxxx (save RAM)
  // is not used by this machine, so both read as zero.
  wire [31:0] cmd_bridge_rd_data;

  // An if/else rather than the framework's casex: there are exactly two
  // regions, and casex would treat x/z address bits as don't-care.
  always @(*) begin
    if (bridge_addr[31:28] == 4'h4)
      bridge_rd_data = ss_rd_data;
    else if (bridge_addr[31:24] == 8'hF8)
      bridge_rd_data = cmd_bridge_rd_data;
    else
      bridge_rd_data = 32'h0;
  end

  wire reset_n;                 // from core_bridge_cmd: host has taken us out of reset

  wire dataslot_requestread;
  wire [15:0] dataslot_requestread_id;
  wire dataslot_requestwrite;
  wire [15:0] dataslot_requestwrite_id;
  wire [31:0] dataslot_requestwrite_size;
  wire dataslot_update;
  wire [15:0] dataslot_update_id;
  wire [31:0] dataslot_update_size;
  wire dataslot_allcomplete;

  wire savestate_start, savestate_load;

  // The save-state blob, in bytes: the machine's whole state (8 bytes of header,
  // 64 register words, 64 KB of RAM -- mc64_savestates' WORDS * 8), and the
  // maximum the OS may hand back for a restore, which is that PLUS 4 KB of
  // slack: the shape the sibling core that runs save states on this panel
  // declares (agg23's openfpga-NES: `savestate_size + 32'h1000`). The slack is
  // what lets a file written by an earlier build, or padded by the OS, still be
  // offered rather than refused before the engine's own header check can answer
  // -- and it is now also the size of the controller's staging buffer, so the
  // same constant appears on both sides of the promise (see the note there).
  localparam [31:0] SAVESTATE_SIZE          = 32'd66056;
  localparam [31:0] SAVESTATE_MAXLOAD_SIZE  = SAVESTATE_SIZE + 32'd4096;
  /* The state's FORMAT TAG, which is not the file's size: the header's upper
   * half is the engine's STATESIZE -- the bytes of state behind the header, 64
   * register words + 64 KB of RAM -- while the file is that plus the header
   * itself. The controller needs it to tell the host's two 32-bit halves apart
   * (see THE HOST'S HALF ORDER in rtl/save_state_controller.sv), and
   * rtl/mc64_savestates.sv builds the same number from the same two quantities.
   * If the state's shape ever changes, both move together -- and the ordering
   * that would catch it is the harness's, not this comment: a tag that does not
   * match the engine's makes every word of a panel-written file assemble the
   * wrong way round, which sim/tb_savestate.cpp's MC64_SS_STA ordering fails on
   * (it loads a state a real console wrote). */
  localparam [31:0] SAVESTATE_TAG           = 32'd66048;

  // The OS's notify level (0x00B0): the menu state, and the OS's own pause
  // request. BIT 0 PAUSES THE MACHINE, through core_top's escape latch -- a
  // level the OS owns that the user can always take back (see the PAUSE
  // section and spec 21.12). Bit 1 is not in the specification and is unread.
  wire        osnotify_inmenu;
  wire        osnotify_pause_req;

  wire [9:0]  datatable_addr  = 10'h0;
  wire        datatable_wren  = 1'b0;
  wire [31:0] datatable_data  = 32'h0;
  wire [31:0] datatable_q;

  core_bridge_cmd icb (
      .clk(clk_74a),
      .reset_n(reset_n),

      .bridge_endian_little(bridge_endian_little),
      .bridge_addr         (bridge_addr),
      .bridge_rd           (bridge_rd),
      .bridge_rd_data      (cmd_bridge_rd_data),
      .bridge_wr           (bridge_wr),
      .bridge_wr_data      (bridge_wr_data),

      // ready to receive a cartridge as soon as the PLL is locked (or, if it
      // never locks, as soon as the clk_74a timer says the bridge is alive and
      // has waited long enough); the machine itself stays in reset until one
      // arrives
      .status_boot_done (core_ready),
      .status_setup_done(core_ready),
      .status_running   (reset_n),

      .dataslot_requestread    (dataslot_requestread),
      .dataslot_requestread_id (dataslot_requestread_id),
      .dataslot_requestread_ack(1'b1),
      .dataslot_requestread_ok (1'b1),

      .dataslot_requestwrite    (dataslot_requestwrite),
      .dataslot_requestwrite_id (dataslot_requestwrite_id),
      .dataslot_requestwrite_size(dataslot_requestwrite_size),
      .dataslot_requestwrite_ack(1'b1),
      .dataslot_requestwrite_ok (1'b1),

      .dataslot_update     (dataslot_update),
      .dataslot_update_id  (dataslot_update_id),
      .dataslot_update_size(dataslot_update_size),

      .dataslot_allcomplete(dataslot_allcomplete),

      .rtc_epoch_seconds(),
      .rtc_date_bcd     (),
      .rtc_time_bcd     (),
      .rtc_valid        (),

      // Save states: supported, and the buffer is the 0x4 window. The size is
      // the state itself -- 8 bytes of header, 64 register words, 64 KB of RAM
      // (mc64_savestates' WORDS * 8).
      //
      // `savestate_maxloadsize` is that size PLUS 4 KB of slack, which is the
      // shape the sibling core that runs save states on this panel declares
      // (agg23's openfpga-NES: `savestate_size + 32'h1000`). It bounds the file
      // the OS will TALK BACK for a restore, so a file that is a little larger
      // than the state this build writes -- one written by an earlier build, or
      // padded by the OS -- is still offered rather than refused before the
      // engine's own header check can answer.
      .savestate_supported  (1'b1),
      .savestate_addr       (32'h40000000),
      .savestate_size       (SAVESTATE_SIZE),
      .savestate_maxloadsize(SAVESTATE_MAXLOAD_SIZE),

      .osnotify_inmenu    (osnotify_inmenu),
      .osnotify_pause_req (osnotify_pause_req),

      .savestate_start     (savestate_start),
      .savestate_start_ack (ss_start_ack),
      .savestate_start_busy(ss_start_busy),
      .savestate_start_ok  (ss_start_ok),
      .savestate_start_err (ss_start_err),

      .savestate_load     (savestate_load),
      .savestate_load_ack (ss_load_ack),
      .savestate_load_busy(ss_load_busy),
      .savestate_load_ok  (ss_load_ok),
      .savestate_load_err (ss_load_err_bridge),

      .target_dataslot_read      (1'b0),
      .target_dataslot_write     (1'b0),
      .target_dataslot_getfile   (1'b0),
      .target_dataslot_openfile  (1'b0),
      .target_dataslot_ack       (),
      .target_dataslot_done      (),
      .target_dataslot_err       (),
      .target_dataslot_id        (16'h0),
      .target_dataslot_slotoffset(32'h0),
      .target_dataslot_bridgeaddr(32'h0),
      .target_dataslot_length    (32'h0),
      .target_buffer_param_struct(32'h0),
      .target_buffer_resp_struct (32'h0),

      .datatable_addr(datatable_addr),
      .datatable_wren(datatable_wren),
      .datatable_data(datatable_data),
      .datatable_q   (datatable_q)
  );

  // ------------------------------------------------------------------ cartridge
  //
  // The OS streams the .bin over the bridge as 32-bit writes into data slot 1
  // (0x1xxxxxxx). Each write carries the byte address in bridge_addr[27:0] and
  // four bytes of file in bridge_wr_data -- with bridge_endian_little = 1 the
  // first file byte of the group is in bits [7:0]. The bytes land in the
  // machine's RAM through its rom port while it is held in reset, so the CPU
  // never sees a partially loaded cartridge.
  //
  // THE HOST CANNOT BE PACED, SO THE ONLY SAFE CAPTURE IS A FIFO. This is the
  // fault that made every cartridge render as its own private garbage on the
  // panel while every harness passed (2026-10-11).
  //
  // The bridge hands a slot write to the core as a ONE-CYCLE PULSE and never
  // waits: io_bridge_peripheral.v's own header says "writes are immediate", and
  // the sibling core that runs on this panel says the same about its cart path
  // ("this path has NO BACKPRESSURE ... a dropped cart word would be a silent,
  // un-debuggable boot failure", which is why it counts drops). So a write the
  // loader cannot take disappears. There is no handshake to slow the OS down
  // and no way to ask it to repeat.
  //
  // What this file used to do was latch the word into one register, raise a
  // flag, and clear that flag through an acknowledge round trip to the clk_74a
  // domain and back -- i.e. it could not accept the next word until it had
  // finished the previous one and the host had seen the ack. Measured against
  // the bridge's stated ceiling ("the worst-case read/write timing is every 88
  // cycles @ 74.25mhz", io_bridge_peripheral.v) that is 88 clk_74a cycles =
  // 1.185 us = 7.59 machine cycles per 4-byte word at 6.4 MHz, and the round
  // trip alone cost ~11 -- so on the panel the loader dropped roughly one word
  // in eight and the machine executed a file with holes in it. Different
  // cartridges lost different words, which is exactly the "every ROM shows its
  // own different noise" the panel reported.
  //
  // The harness could not see it because it paced the stream at 26 SIMULATED
  // machine cycles per word (sim/tb_boot.cpp), which was the real margin when
  // the machine ran at 25.6 MHz. When the machine clock became 6.4 MHz the
  // budget per word fell to 7.59 cycles and the harness kept handing the loader
  // 26; with the pace corrected to the hardware's own ceiling the old loader
  // fails loudly (127 of 1024 cartridge bytes wrong in RAM -- see the "cart
  // region" line in sim/out/boot*.log).
  //
  // THE SHAPE THAT CANNOT DROP A WORD: capture into a small dual-clock FIFO in
  // the bridge's own domain, drain in the machine's domain at four bytes per
  // word, and let the FIFO absorb the jitter. The drain (4 machine cycles =
  // 0.63 us per word) is faster than the host can offer words (1.185 us at the
  // bridge's ceiling), so the FIFO stays empty in steady state and only needs to
  // cover the difference when the host bursts. `cart_drop` is the canary -- the
  // sibling core's counter, for the same reason -- and it must stay 0; a
  // non-zero value means the cartridge in RAM is KNOWN to be incomplete, which
  // is a much better thing for the panel to tell us than a garbage picture.

  wire        slot1_write = bridge_wr && (bridge_addr[31:28] == 4'h1);

  // Verilog-2001 ONLY in this file: it is compiled by Quartus as .v, so a
  // SystemVerilog literal (`'0`), a `localparam int`, or any other 1800 feature
  // is a hard Analysis & Synthesis error (10839) that Verilator will happily
  // accept. Hence the explicit widths and the untyped localparams below.
  localparam CF_AW = 2;                      // 4 entries: see the note above
  localparam CF_N  = 1 << CF_AW;

  reg  [27:0] cf_addr [0:CF_N-1];
  reg  [31:0] cf_data [0:CF_N-1];

  reg  [CF_AW:0] cf_wbin  = 3'b0;           // write pointer, binary (clk_74a)
  reg  [CF_AW:0] cf_wgray = 3'b0;           // ... and gray-coded, to cross domains
  reg  [CF_AW:0] cf_rbin  = 3'b0;           // read pointer, binary (clk_core)
  reg  [CF_AW:0] cf_rgray = 3'b0;

  wire [CF_AW:0] cf_wbin_n  = cf_wbin + 1'b1;
  wire [CF_AW:0] cf_wgray_n = (cf_wbin_n >> 1) ^ cf_wbin_n;
  wire [CF_AW:0] cf_rbin_n  = cf_rbin + 1'b1;
  wire [CF_AW:0] cf_rgray_n = (cf_rbin_n >> 1) ^ cf_rbin_n;

  reg  [CF_AW:0] cf_rgray_s1 = 3'b0, cf_rgray_s2 = 3'b0;   // read ptr -> clk_74a
  reg  [CF_AW:0] cf_wgray_s1 = 3'b0, cf_wgray_s2 = 3'b0;   // write ptr -> clk_core

  /* Full and empty, the standard gray-pointer pair. Only one bit of a gray
   * pointer changes per step, so a pointer sampled mid-change reads as either
   * its old or its new value -- never as a third one -- and an entry's data is
   * written on the same edge as the pointer that publishes it, i.e. at least a
   * full synchroniser delay before the other side can act on it. */
  wire [CF_AW:0] cf_full_cmp = {~cf_rgray_s2[CF_AW],
                                ~cf_rgray_s2[CF_AW-1],
                                 cf_rgray_s2[CF_AW-2:0]};

  wire cf_full  = (cf_wgray_n == cf_full_cmp);
  wire cf_empty = (cf_rgray   == cf_wgray_s2);

  // The canary. Sticky: the cartridge in RAM is incomplete from here on.
  reg  [15:0] cart_drop = 16'd0;

  /* --------------------------------------------- the canary, made visible
   *
   * `cart_drop` is the number that says whether the frame in RAM is the file
   * the OS sent, and until now the only way to read it was a Verilator run of
   * sim/tb_boot.cpp. On the panel it was invisible, and that is why this fault
   * cost two flashes: a cartridge with holes in it still draws *something* --
   * per-ROM noise, or a partial picture -- which is indistinguishable from
   * every other reason a picture can be wrong. The number that names it
   * existed, and hardware could not report it.
   *
   * So the core now reports it in the only channel it owns: while any word has
   * been dropped, an 8x8 red square is painted in the top-left corner of the
   * picture -- on the machine's picture as well as the bring-up one, so it is
   * there whether or not the machine was released. Everything else about the
   * picture is untouched: with cart_drop == 0 (the normal case, and every
   * ordering tools/boot_sim.sh runs) the mark changes no pixel at all, which is
   * why the pixel-exact comparisons in sim/tb_boot.cpp, sim/compare.py and
   * tools/panel_check.py still hold -- and why the negative control that
   * proves the mark can appear has to OVERRUN the loader deliberately (see the
   * fifth ordering in tools/boot_sim.sh).
   *
   * ONE BIT, THEN TWO FLOPS. `cart_drop` is a 16-bit binary counter in
   * clk_74a, and a picture stage reading it directly would be sampling 16 bits
   * across a clock boundary: at a carry, one edge can see a mixture the counter
   * never held -- and the interesting transition here is 0 -> 1, whose mixed
   * reads are 0, i.e. exactly the ``nothing was dropped'' value. The picture
   * needs one boolean ("has a word ever been dropped"), so it is latched to one
   * bit in the counter's own domain and only that bit crosses, through the same
   * two-flop pattern as `run_pix` below. The crossing itself is already
   * declared false in rtl/core_constraints.sdc (the clk_74a <-> PLL pair), and
   * this is the structure that makes that declaration honest: one bit,
   * sampled, nothing combinational.
   */
  reg cart_incomplete = 1'b0;

  always @(posedge clk_74a)
    if (cart_drop != 16'd0)
      cart_incomplete <= 1'b1;

  wire rom_we;
  wire [15:0] rom_addr;
  wire [7:0]  rom_data;

  always @(posedge clk_74a) begin
    cf_rgray_s1 <= cf_rgray;
    cf_rgray_s2 <= cf_rgray_s1;

    if (slot1_write) begin
      if (!cf_full) begin
        cf_addr[cf_wbin[CF_AW-1:0]] <= bridge_addr[27:0];
        cf_data[cf_wbin[CF_AW-1:0]] <= bridge_wr_data;
        cf_wbin  <= cf_wbin_n;
        cf_wgray <= cf_wgray_n;
      end else begin
        cart_drop <= cart_drop + 16'd1;
      end
    end
  end

  // ---- drain side: one word per four machine cycles, no idle cycle between
  //      words, so the sustained rate is 4 machine cycles (0.63 us) per word.
  reg  [27:0] cf_raddr_q = 28'h0;
  reg  [31:0] cf_rdata_q = 32'h0;
  reg  [ 1:0] cf_byte    = 2'd0;
  reg         cf_active  = 1'b0;

  wire [CF_AW-1:0] cf_rd_i = cf_rbin[CF_AW-1:0];

  wire [16:0] cf_ramadr = 17'h00200 + {1'b0, cf_raddr_q[15:0]} + {15'b0, cf_byte};

  // The byte of the word to write this cycle, byte 0 (the first file byte)
  // first -- a variable shift rather than an indexed part-select. `+: 8` is
  // Verilog-2001 as well, but nothing else in this project uses one and the fit
  // is not the place to discover how the toolchain feels about it.
  wire [31:0] cf_byte_sel = cf_rdata_q >> {cf_byte, 3'b000};

  /* ------------------------------------------------ RAM clear (2026-10-11)
   *
   * THE MACHINE MUST START FROM THE EMULATOR'S RAM, NOT THE PREVIOUS CARTRIDGE'S.
   *
   * sim/golden/golden.c:84 is the reference: `memory zeroed, cartridge loaded at
   * $0200`. The emulator memsets all 64 KB and then lays the image on top, so
   * every byte outside the image is ZERO when the cartridge starts. M10K also
   * powers up zeroed -- but nothing here ever cleared it again, and this package
   * does NOT pulse reset between loads (its slot's data.json has bit 6 clear, so
   * the OS sends no Reset Enter/Exit around a load). So on the panel a cartridge
   * inherited whatever the previous one left, and three separate field reports
   * are that one fact:
   *
   *   * `cube` never writes $0101, so it drew with the PREVIOUS cartridge's
   *     palette bank -- `palettecycle`'s palette -- and any wireframe edge whose
   *     entry collided with the background vanished. Reported as "sides of the
   *     cube are missing" (docs/minicube64-fpga-spec.md 17.4).
   *   * `twister` writes $0100 but never $0101 either, so it drew with the
   *     previous palette too: green-yellow #AAFF55 came out PINK #FF55AA after a
   *     `palettecycle` run. Reported (2026-10-11, panel) as "twister is pink
   *     instead of green".
   *   * `boundary` (the DVD logo) reads $0007 as its sprite mask and NEVER WRITES
   *     IT -- emulator: 0, draw the glyph; panel: the previous cartridge's byte,
   *     so `cmp mask / beq` skipped the glyph and drew its complement. Reported
   *     as "the DVD is inverted and does not change colour".
   *
   * WHAT IS CLEARED, AND WHY IT IS NOT SIMPLY ALL 64 KB. The sweep runs while the
   * freshly arrived cartridge is already in RAM, so it must leave the image
   * alone. What it covers is exactly the emulator's zeroes: the low page
   * ($0000-$01FF: zero page, stack, IRQ/vector registers and the $0100/$0101
   * bank registers) and everything ABOVE the image ($0200+len .. $FFFF). The
   * loader never writes below $0200 (its addresses are $0200 + the file offset),
   * so the low page is swept first and without any interaction; the tail starts
   * at image_top + 1, where image_top is the highest address THIS load has
   * written (cf_ramadr tracked while the drain runs). Together they are
   * `memset(memory, 0, sizeof(memory))` followed by the load, byte for byte.
   *
   * ONE PORT, ONE WRITER AT A TIME. The sweep writes through the SAME port B the
   * loader uses (mc64_ram's `rom_*` port, which is free while the machine is held
   * in reset). The loader ALWAYS has priority: the sweep only drives the port in
   * cycles where the drain is idle, and stalls -- holding its address -- while a
   * word is in flight. So a load that arrives mid-sweep cannot lose a word, which
   * is the property that matters: the bridge cannot be paced, and a dropped word
   * is a silently incomplete cartridge (see the FIFO note above).
   *
   * AND THE SWEEP RE-ARMS RATHER THAN RACING. `cart_ready` is the machine-domain
   * view of "a cartridge load is complete and the bridge is quiet"; it drops the
   * instant a new slot write arrives. While it is low the sweep is idle and the
   * loader owns the port; when it rises the sweep runs from $0000. That single
   * rule covers every ordering: the first load after power-up, a reload (the
   * user picking another file), and a stream that STALLED long enough for the
   * quiet window to expire -- there the sweep starts against a partial image,
   * the first resumed word drops cart_ready and ABORTS it (the image_top skip is
   * what keeps that harmless), and the completed load's own sweep re-runs it
   * from scratch.
   *
   * THE RELEASE WAITS FOR IT. `run_core` gains `ram_clear_done`, so the CPU is
   * held until the sweep has finished -- the same shape the sibling core uses
   * for its scratch-RAM clear (C:/Code/Wasm4/VexiiRiscv/core_top.v, "Scratch-RAM
   * clear on Reset Core / end of load", where the CPU is held across the sweep).
   * Cost: 65,536 machine cycles = 10.2 ms at 6.4 MHz, once per load, entirely
   * inside the reset the machine is already in (the release gate has been open
   * since the host announced the load, or for the fallback quiet window, by
   * then). What the panel shows across it is the machine's own picture -- black,
   * at power-up -- not a picture this file draws.
   *
   * NOT cleared, deliberately: anything on a Reset Core pulse with no re-stream
   * (the console's soft reset keeps RAM -- and it must, because the cartridge's
   * own code lives in it), and the image region itself.
   */
  reg  [16:0] ram_clear_addr = 17'd0;      // 17 bits: 65536 means "past the end"
  reg         ram_clear_busy = 1'b0;
  reg         ram_clear_done = 1'b0;
  reg  [15:0] image_top      = 16'h0000;    // highest address THIS load has written

  reg         cart_ready_s1  = 1'b0;        // cart_ready into clk_core
  reg         cart_ready_s2  = 1'b0;
  reg         ram_clear_done_s1 = 1'b0;     // ram_clear_done into clk_74a
  reg         ram_clear_done_s2 = 1'b0;

  wire [15:0] cf_waddr = cf_ramadr[15:0];   // the address this cycle's write lands on

  /* The image region, and the one step that skips it. The sweep reaches $0200
   * and jumps to image_top + 1 -- so it never writes a byte the loader owns, and
   * never wastes 64K-image_top cycles walking over it. */
  wire        ram_clear_at_image = (ram_clear_addr == 17'd512)
                                   && (image_top >= 16'h0200);
  wire [16:0] ram_clear_next = ram_clear_at_image
                               ? ({1'b0, image_top} + 17'd1)
                               : (ram_clear_addr + 17'd1);

  wire        ram_clear_owns  = ram_clear_busy && !cf_active;
  wire        ram_clear_write = ram_clear_owns && !ram_clear_at_image;

  assign rom_we   = cf_active | ram_clear_write;
  assign rom_addr = cf_active ? cf_waddr : ram_clear_addr[15:0];
  assign rom_data = cf_active ? cf_byte_sel[7:0] : 8'h00;

  always @(posedge clk_74a) begin
    ram_clear_done_s1 <= ram_clear_done;
    ram_clear_done_s2 <= ram_clear_done_s1;
  end

  always @(posedge clk_core) begin
    cart_ready_s1 <= cart_ready;
    cart_ready_s2 <= cart_ready_s1;

    /* The image, as this load has built it so far. Accumulated on the drain's
     * OWN cycles, so it cannot be moved by anything else in this file. */
    if (cf_active && (cf_waddr > image_top))
      image_top <= cf_waddr;

    if (!cart_ready_s2) begin
      /* No complete load to clear for -- or a word has just arrived, which is
       * the abort above: drop everything and wait for the quiet window again. */
      ram_clear_busy <= 1'b0;
      ram_clear_done <= 1'b0;
      ram_clear_addr <= 17'd0;
    end else if (!ram_clear_done) begin
      ram_clear_busy <= 1'b1;
      if (ram_clear_owns) begin
        ram_clear_addr <= ram_clear_next;
        if (ram_clear_next >= 17'd65536) begin
          ram_clear_busy <= 1'b0;
          ram_clear_done <= 1'b1;
          image_top      <= 16'h0000;   // the next load measures its own image
        end
      end
    end
  end

  always @(posedge clk_core) begin
    cf_wgray_s1 <= cf_wgray;
    cf_wgray_s2 <= cf_wgray_s1;

    if (!cf_active) begin
      if (!cf_empty) begin
        cf_raddr_q <= cf_addr[cf_rd_i];
        cf_rdata_q <= cf_data[cf_rd_i];
        cf_rbin    <= cf_rbin_n;
        cf_rgray   <= cf_rgray_n;
        cf_byte    <= 2'd0;
        cf_active  <= 1'b1;
      end
    end else if (cf_byte == 2'd3) begin
      // last byte of this word: take the next one now if there is one, so the
      // four cycles are back to back with no gap
      if (!cf_empty) begin
        cf_raddr_q <= cf_addr[cf_rd_i];
        cf_rdata_q <= cf_data[cf_rd_i];
        cf_rbin    <= cf_rbin_n;
        cf_rgray   <= cf_rgray_n;
        cf_byte    <= 2'd0;
      end else
        cf_active <= 1'b0;
    end else
      cf_byte <= cf_byte + 2'd1;
  end

  // ------------------------------------------------------------------ run gate
  //
  // WHEN THE MACHINE IS ALLOWED TO RUN.
  //
  // The obvious rule -- release the machine when the framework reports that all
  // data slots are complete (host command 0x008F, which core_bridge_cmd exposes
  // as dataslot_allcomplete) -- is wrong, and it fails in a way that looks
  // exactly like a dead cartridge. That signal arrives while the file is still
  // queued behind the OS's file browser; VexiiRiscv's core_top documents the
  // same trap for its own data slot ("0x008F is exactly what arrives while slot
  // 0 is still queued behind the file browser"). A machine released on it starts
  // the 6502 on a RAM of zeros, the CPU BRK-loops forever, and the cartridge
  // that lands a moment later is never executed because nothing restarts the CPU
  // at $0200. The screen then shows the default palette's entry 0 -- black --
  // with perfectly valid sync, which is indistinguishable from a dead core.
  // sim/tb_boot.cpp reproduces that ordering and fails a core that does this.
  //
  // So the gate is on the data itself: wait until at least one cartridge byte has
  // arrived and the bridge has then been quiet for CART_QUIET_MAX cycles. Any
  // arriving byte restarts the timer, so a reload (the user picking another file)
  // puts the machine back into reset and restarts it cleanly with the new image.
  // No dependence on which commands the OS sends, and no dependence on their
  // order.
  //
  // The simulation value is short because only the logic is under test there.
  //
  // THE SYNTHESIS VALUE USED TO BE 2^26 = 0.90 s, AND THAT WAS THE LOADING
  // TIME. It is the fallback for a host that never announces the load's end
  // (see the announcement below), so it only has to be longer than the longest
  // gap a real stream contains. It is now 2^22 = 56.5 ms at 74.25 MHz: sixteen
  // times faster than the old value, and 64x the 0.88 ms the sibling core that
  // runs on this panel is willing to call "quiet" (C:/Code/Wasm4/VexiiRiscv,
  // `por_reset_n`) -- whose own history is why a SHORT quiet window cannot be
  // the only rule: that core released its CPU into a half-loaded ROM when a
  // gap inside a stream exceeded its 0.88 ms. Words arrive back to back on the
  // panel's cached push, so a 56.5 ms gap mid-file is not a thing; if one ever
  // happens the release is early, not wrong -- the next byte drops the gate and
  // restarts the machine, and the completed load's own sweep re-runs (see the
  // aborts above). This is the knob.
`ifdef MC64_SIM
  localparam [26:0] CART_QUIET_MAX = 27'd4096;
`else
  localparam [26:0] CART_QUIET_MAX = 27'd4194304;
`endif

  reg        cart_seen  = 1'b0;             // a cartridge byte has arrived
  reg [26:0] cart_quiet = CART_QUIET_MAX;   // cycles since the last one

  // ---- ...AND THE HOST'S OWN STATEMENT THAT THE LOAD IS DONE ---------------
  //
  // The quiet window above is a guess. The host has an answer, and it is the
  // difference between a ~1 s boot and an instant one.
  //
  // `dataslot_allcomplete` (core_bridge_cmd, command 0x008F) is the host's
  // "every data slot it asked for has been loaded", and it is a PAIR with
  // command 0x0082, which CLEARS it: the OS clears it when it starts streaming
  // a slot and raises it when that slot is complete. So it is low for the whole
  // of a slot load, gaps inside the stream included, and its RISING EDGE after
  // those bytes is the host saying "the slot I streamed is finished" rather
  // than a timer saying it has been quiet lately. That is the instant release
  // path -- and it is the rule the sibling core arrived at after its own panel
  // bug, in the same words (C:/Code/Wasm4/VexiiRiscv/core_top.v: "`dataslot_
  // allcomplete` is CLEARED by 0x0082 ... and RAISED by 0x008F ... So it is low
  // for the whole of a slot load -- including every gap inside it -- and
  // requiring it is the host's own statement that the slot it streamed is
  // finished").
  //
  // BOTH HALVES OF THAT PAIR ARE REQUIRED, and tools/boot_sim.sh's first
  // ordering is the reason: 0x008F ALONE arrives while a file is still
  // arriving -- the framework's own order, per that same core's note ("0x008F
  // is exactly what arrives while slot 0 is still queued behind the file
  // browser"). A rise with no 0x0082 before it says nothing about this load, so
  // `load_open` (set by the clear) is the proof that a stream was in flight and
  // `cart_written` is the proof that bytes of it reached this core. A rise
  // with either missing is ignored and the quiet window decides, exactly as
  // before.
  //
  // AND A BYTE AFTER THE RISE TAKES IT BACK. `written_since_announce` is what
  // keeps a host that announces from ever releasing the machine under a stream
  // that is still moving: the first byte after the announcement drops the gate
  // (and restarts the machine through the same abort path as the quiet timer's
  // own early release).
  reg        allcomplete_q = 1'b0;
  reg        load_open     = 1'b0;   // 0x0082 seen: a slot stream is in flight
  reg        cart_written  = 1'b0;   // ... and a byte of it has arrived
  reg        load_announced = 1'b0;  // ... and 0x008F has since said it is done
  reg        written_since_announce = 1'b0;

  wire allc_rise =  dataslot_allcomplete & ~allcomplete_q;
  wire allc_fall = ~dataslot_allcomplete &  allcomplete_q;

  always @(posedge clk_74a) begin
    allcomplete_q <= dataslot_allcomplete;

    if (slot1_write) begin
      cart_seen  <= 1'b1;
      cart_quiet <= 27'h0;
      cart_written <= 1'b1;
      written_since_announce <= 1'b1;
    end else if (cart_quiet != CART_QUIET_MAX)
      cart_quiet <= cart_quiet + 27'd1;

    if (allc_fall) begin
      // The OS is starting a slot load (0x0082). A previous announcement, and
      // the bytes it was about, belong to the previous load.
      load_open     <= 1'b1;
      cart_written  <= 1'b0;
      load_announced <= 1'b0;
      written_since_announce <= 1'b0;
    end
    else if (allc_rise) begin
      // The OS says the slot it was streaming is complete (0x008F) -- but only
      // if this core saw both the start of that stream and bytes of it.
      load_open     <= 1'b0;
      load_announced <= load_open & cart_written;
      written_since_announce <= 1'b0;
    end
  end

  /* AND THE DRAIN HAS TO HAVE FINISHED. The quiet timer is ~0.9 s after the
   * last slot write and a four-entry FIFO drains in microseconds, so in
   * practice this can never be the binding term -- but "the machine starts with
   * the whole cartridge in RAM" should be a property of the logic rather than
   * of one timer's arithmetic, and with the FIFO capture the release no longer
   * has anything else to say whether the last words have landed. */
  reg cf_idle_s1 = 1'b1;
  reg cf_idle_s2 = 1'b1;

  wire cf_idle = !cf_active && cf_empty;   // clk_core: nothing left to write

  always @(posedge clk_74a) begin
    cf_idle_s1 <= cf_idle;
    cf_idle_s2 <= cf_idle_s1;
  end

  // Two ways to open the gate: the host said so (announce_done), or the bridge
  // has been quiet for the fallback window (quiet_done). `cart_seen` still
  // gates both -- before any byte has arrived there is nothing to run.
  wire quiet_done    = (cart_quiet == CART_QUIET_MAX);
  wire announce_done = load_announced & ~written_since_announce;

  wire cart_ready = cart_seen && cf_idle_s2 && (quiet_done | announce_done);

  // ------------------------------------------------------- power-on reset
  //
  // The PLL's own `locked`, held high for a moment before the machine is
  // allowed to look at it. Same shape, and same purpose, as the 0.9 ms quiet
  // window in the sibling core that runs on this hardware (VexiiRiscv's
  // core_top `por_reset_n`): the lock is a level that can still be settling
  // when it first reads 1, and a machine released inside that window starts
  // from a clock that is not finished changing.
  localparam [15:0] POR_MAX = 16'hFFFF;   // 0.88 ms at 74.25 MHz

  reg [15:0] por_cnt  = 16'h0;
  reg        por_done = 1'b0;

  // Gated on the LATCHED lock, not the live one: a lock glitch must not restart
  // this window (which would drop run_core and reset a running machine -- see
  // the note at pll_locked_sticky).
  always @(posedge clk_74a) begin
    if (!pll_locked_sticky) begin
      por_cnt  <= 16'h0;
      por_done <= 1'b0;
    end else if (!por_done) begin
      por_cnt <= por_cnt + 16'd1;
      if (por_cnt == POR_MAX)
        por_done <= 1'b1;
    end
  end

  // ---------------------------------------------------- host "Reset Core"
  //
  // `reset_n` is used as an EDGE here, not as a level, and that is deliberate.
  //
  // As a level it made the machine's only release depend on a power-up value
  // inside another module and on a host command the OS may never send: the OS
  // sends [0010 Reset Enter]/[0011 Reset Exit] around a data-slot load only
  // when that slot's data.json `parameters` has bit6 set, and this package's
  // Cartridge slot is "0x01". Held that way, a reset_n that powers up 0 is a
  // permanently black panel, and one that powers up 1 makes the Reset Core
  // button do nothing until the OS happens to send Exit.
  //
  // As an edge it is right under both orderings: Reset Enter restarts the
  // machine for a fixed window, Reset Exit is not required, and a reset_n that
  // powers up 0 (what silicon does if the initial value is ever dropped) costs
  // one spurious 14 ms restart at power-up and nothing else. This is exactly
  // what the working sibling core does with the same host command -- see
  // C:/Code/Wasm4/VexiiRiscv/core_top.v, `reset_n_fall`.
  reg reset_n_q = 1'b1;                  // power-up 1: no edge without a command

  always @(posedge clk_74a)
    reset_n_q <= reset_n;

  wire reset_core_edge = reset_n_q & ~reset_n;

  localparam [19:0] RC_MAX = 20'hFFFFF;   // 14.1 ms at 74.25 MHz

  reg [19:0] rc_hold = 20'h0;

  always @(posedge clk_74a) begin
    if (reset_core_edge)       rc_hold <= RC_MAX;
    else if (rc_hold != 20'h0) rc_hold <= rc_hold - 20'd1;
  end

  wire reset_core = (rc_hold != 20'h0);

  // ---- one condition for "the machine may run", built in the clk_74a domain
  // and then synchronised into the machine's domain as a level (assert is
  // asynchronous, release is synchronised -- which is what a reset wants).
  //
  // Note what is NOT in here: `reset_n`. See above.
  // ... and this gate uses the LATCHED lock for the reason above: as the live
  // level it made the machine's life depend on the lock detector's chatter.
  wire run_core = pll_locked_sticky && por_done && cart_ready && ram_clear_done_s2
                  && !external_reset && !reset_core;

  reg [2:0] run_sync = 3'b000;

  always @(posedge clk_core) begin
    run_sync <= {run_sync[1:0], run_core};
  end

  wire machine_reset = !run_sync[2];

  // ------------------------------------------------------------------ status
  //
  // The six boot conditions, as a debug tap. The picture that used to display
  // them is gone (see the video stage); this is what tools/boot_sim.sh reads --
  // its PLL-dead ordering checks bit 0, and the rest are for the log.
  // Read as a ladder: 0 -> 1 -> 2 -> 3 is a cartridge arriving and being
  // released, and every one of them lit is a working boot.
  wire [5:0] boot_state = {
      external_reset,   // 5  bottom-right : the OS's 0x050 reset is counting down
      reset_n,          // 4  bottom-mid   : the host has not asked for Reset Enter
      run_core,         // 3  bottom-left  : THE GATE -- the machine may run
      cart_ready,       // 2  top-right    : the whole cartridge has arrived, bridge quiet
      cart_seen,        // 1  top-mid      : at least one byte reached slot window 1
      pll_locked        // 0  top-left     : the PLL locked
  };

  // ------------------------------------------------------------------- machine
  //
  // mc64_top is the whole machine: 6502 + RAM + framebuffer + the counter that
  // makes the 6.4 MHz cycle enable. It runs on the master clock and its video
  // outputs advance once per pixel enable.

  wire [23:0] m_video_rgb;
  wire        m_video_de;
  wire        m_video_hs;
  wire        m_video_vs;
  // The machine's audio. A LEVEL, not a signed sample: the APU's mixer output
  // is 0 for silence and adds up from there (mc64_top's audio note). Never
  // negative, so unsigned.
  wire [15:0] m_audio_l;
  wire [15:0] m_audio_r;

  // controllers, mapped from the Pocket's key bitmap onto the machine's:
  //   bit0 A(Z)  bit1 B(X)  bit2 C  bit3 Start  bit4 Up  bit5 Down  bit6 Left  bit7 Right
  //
  // THE CORE'S PAUSE IS `cont1_key[14]` (THE POCKET'S SELECT) AND THE MACHINE'S
  // Start BIT IS `cont1_key[15]` (THE POCKET'S START). FINAL, 2026-10-17.
  //
  // The panel settled this: Select pauses, Start is the button the cartridges
  // answer to. MiniCube64's pad (the emulator's `$0102` byte) has exactly one
  // Start bit, and every program that uses it -- `input_demo` prints the four
  // face buttons and Start -- wants it on the button actually labelled Start.
  // Select is the one left over: no cartridge reads it, so it carries the
  // core's own pause.
  //
  // So `[14]` = the core's pause, `[15]` = the machine's Start. This is the
  // third arrangement this file has had; the first two differed only in which
  // of the two small buttons carried the pause, and 18.2/18.5 in the spec
  // record why each was chosen. What was actually wrong with the first one was
  // never the mapping -- it was that `0x00B0` bit 1 latched a pause nothing
  // could clear (see the PAUSE section below).
  // See the PAUSE section below for what the pause does and why it is a level
  // with a guard. `sim/tb_boot.cpp`'s controller table is the same statement,
  // and its per-key sweep is the check that can disagree with it.
  wire [7:0] pad1_map = {
    cont1_key[3],   // Right
    cont1_key[2],   // Left
    cont1_key[1],   // Down
    cont1_key[0],   // Up
    cont1_key[15],  // Start  (the Pocket's Start)
    cont1_key[6],   // C   (face_x)
    cont1_key[5],   // B   (face_b)
    cont1_key[4]    // A   (face_a)
  };

  wire [7:0] pad2_map = {
    cont2_key[3],
    cont2_key[2],
    cont2_key[1],
    cont2_key[0],
    cont2_key[15],
    cont2_key[6],
    cont2_key[5],
    cont2_key[4]
  };

  // ------------------------------------------------------------------- pause
  //
  // `cont1_key[14]` -- THE POCKET'S SELECT -- PAUSES THE MACHINE. Press to
  // pause, press again to resume.
  //
  // Select is the button with no machine meaning (see pad1_map above), so it
  // carries the core's own pause; the machine's Start bit is on `[15]`, the
  // button labelled Start. The two small buttons carry one job each.
  //
  // The machine has no pause of its own and no game implements one, so this is
  // the core's: `pause_req` below becomes mc64_top's RDY hold on the CPU (the
  // raster keeps running -- see that module's port note), and while it is up
  // the panel shows the frozen frame and the machine resumes on the same cycle
  // it stopped. This is the shape the sibling core on this panel arrived at
  // (C:/Code/Wasm4/VexiiRiscv/core_top.v, "hardware pause (CPU freeze)"), and
  // the two things it learned the hard way are reproduced here:
  //
  //   * THE GUARD. The pause button held from power-on, or held across a Reset
  //     Core, must not freeze the next boot -- a pause that engages before a
  //     cartridge has run is indistinguishable from a dead core. The toggle
  //     only ARMS once the button has been seen RELEASED since the machine was
  //     last held, and after that only a fresh press (release -> press) toggles.
  //     `cont1_key` updates once per controller poll (~16 ms), so this needs no
  //     debouncing.
  //
  //   * THE MENU, WHICH PAUSES THE MACHINE -- AND CANNOT STRAND IT (2026-10-17,
  //     the fifth report: the game must stop behind the OS menu, and a load must
  //     still never leave it stopped).
  //
  //     The OS tells the core it is in its menu with command 0x00B0 bit 0
  //     (`osnotify_inmenu`, "the user is in the OS menus and the core has lost
  //     focus"), and the machine stops behind the menu while that level is up.
  //     What this file must NOT do is trust the level to come back down: the
  //     panel has reported a machine left stopped on the loaded frame four
  //     times, and a level the OS owns is a hold this core cannot release --
  //     if the matching `bit 0 = 0` is lost or arrives late, the machine is
  //     parked for ever (spec 21.7 through 21.11; `MC64_SS_MENU_GAP` in the
  //     harness reproduces it at every restore delay, which is why the window
  //     the earlier fixes tried cannot answer it).
  //
  //     So the menu pause is ESCAPABLE, and the escape is the user's own pause
  //     button. `os_menu_en` is the permission to pause on the notify:
  //
  //       * SET whenever the notify is seen LOW -- the OS saying it is out of
  //         its menu -- which is the only thing that can re-arm it;
  //       * CLEARED by a fresh Select press while the notify is up, which
  //         CONSUMES that press as "let me play" rather than toggling the
  //         user's own pause. A spurious level therefore costs one button
  //         press, and only ever when there is no menu on the screen to close
  //         (a real menu is closed with the OS's own button, which drops the
  //         level and re-arms this latch).
  //
  //     `osnotify_pause_req` (bit 1 of that word) is NOT in the specification at
  //     all, and stays unread: as a pause it is a latch with no release.
  //
  // A save-state command OVERRIDES the pause (mc64_top releases the hold while
  // the engine is running): a stopped CPU never reaches an instruction boundary,
  // so a state taken while paused would time out with the machine frozen. The
  // transfer wins, and the pause re-engages when it is over.
  reg pause_armed = 1'b0;   // the button has been seen released since the machine ran
  reg pause_prev  = 1'b0;   // the previous sample, for the fresh-press edge
  reg pause_req_q = 1'b0;   // the user's toggle itself
  reg os_menu_en  = 1'b1;   // ... and the OS's notify may stop the machine

  always @(posedge clk_74a) begin
    if (!run_core) begin
      // The machine is held (power-up, a cartridge arriving, the host's Reset
      // Core): a pause must not survive it, and the button held through it must
      // not arm the next one. The OS's notify permission comes back armed: the
      // OS is the one who will say it is in its menu, from a clean slate.
      pause_armed <= 1'b0;
      pause_prev  <= 1'b0;
      pause_req_q <= 1'b0;
      os_menu_en  <= 1'b1;
    end
    else begin
      pause_prev <= cont1_key[14];

      // The OS saying it is OUT of its menu is the only thing that re-arms the
      // notify's permission to stop the machine.
      if (!osnotify_inmenu)
        os_menu_en <= 1'b1;

      if (!cont1_key[14])
        pause_armed <= 1'b1;              // seen released: arm the guard
      else if (pause_armed && !pause_prev) begin
        // A fresh press. WHILE THE OS'S PAUSE IS HOLDING, THE PRESS IS THE
        // ESCAPE and is spent on it -- the machine resumes and the user's own
        // toggle is left where it was; otherwise it is the usual pause toggle.
        if (osnotify_inmenu && os_menu_en)
          os_menu_en <= 1'b0;
        else
          pause_req_q <= ~pause_req_q;
      end
    end
  end

  // ---- the two pause sources ------------------------------------------------
  //
  // The user's toggle, and the OS's menu notify THROUGH `os_menu_en` -- so the
  // machine stops behind the menu, and one Select press takes it back if that
  // level is ever left standing (see THE MENU above). Both are levels, and
  // mc64_top holds the machine for as long as either is up; a load hands it
  // back by itself (that module's `pause_muted`).
  wire pause_req = pause_req_q | (osnotify_inmenu & os_menu_en);

  // Controller state is a slow, human-speed vector, but it still crosses
  // domains, and eight independently synchronised bits can be captured on
  // different cycles. So the clk_74a side captures the whole vector and flips
  // a toggle; the machine side detects the toggle's edge through two flops and
  // latches the vector one cycle after that, by which time the vector has come
  // through its own two flops and has been stable for a full machine cycle.
  reg  [7:0] pad1_src = 8'h0;
  reg  [7:0] pad2_src = 8'h0;
  reg        pad_tog  = 1'b0;

  always @(posedge clk_74a) begin
    if ({pad2_map, pad1_map} != {pad2_src, pad1_src}) begin
      pad1_src <= pad1_map;
      pad2_src <= pad2_map;
      pad_tog  <= ~pad_tog;
    end
  end

  reg [7:0] pad1 = 8'h0;
  reg [7:0] pad2 = 8'h0;

  reg [7:0] pad1_s1 = 8'h0, pad1_s2 = 8'h0;
  reg [7:0] pad2_s1 = 8'h0, pad2_s2 = 8'h0;
  reg       pad_tog_s1 = 1'b0, pad_tog_s2 = 1'b0;
  reg       pad_tog_edge = 1'b0;

  always @(posedge clk_core) begin
    pad1_s1 <= pad1_src;
    pad1_s2 <= pad1_s1;
    pad2_s1 <= pad2_src;
    pad2_s2 <= pad2_s1;
    pad_tog_s1 <= pad_tog;
    pad_tog_s2 <= pad_tog_s1;

    // registered edge detect: by the time pad_tog_edge is high, pad1_s2 holds
    // the vector that went with the new toggle
    pad_tog_edge <= (pad_tog_s1 != pad_tog_s2);

    if (pad_tog_edge) begin
      pad1 <= pad1_s2;
      pad2 <= pad2_s2;
    end
  end

  // --------------------------------------------------------------------- video
  //
  // ONE PIXEL PER `ce_pix`, and `ce_pix` is derived from the pixel clock
  // itself, not from a fabric divider.
  //
  // The generator's counters run on ce_pix (in clk_core) and the platform's DDR
  // pads sample the result on video_rgb_clock's edges. Those are 6.4 MHz and
  // 1.32 MHz from the same PLL, so the two edge trains are locked -- but a
  // fabric counter dividing clk_core by 160/33 starts wherever its reset leaves
  // it, which would put the data changes at an arbitrary place relative to the
  // sampling edges. Deriving the tick from clk_pixel (a pixel-domain toggle,
  // two flops into clk_core, one edge detect) fixes the relationship: the data
  // changes 1-2 clk_core cycles after a pixel edge, EVERY period, and holds for
  // the rest of the period -- deterministic instead of lucky. WHERE inside the
  // period that is, and what the scaler needs of it, is the one thing this
  // structure does not settle: see the WHEN THE DATA CHANGES note in
  // rtl/mf_pll_mc64_0002.v, which is where the open question is recorded.

  reg pix_tog = 1'b0;

  always @(posedge clk_pixel)
    pix_tog <= ~pix_tog;

  reg pt_s1 = 1'b0;
  reg pt_s2 = 1'b0;

  always @(posedge clk_core) begin
    pt_s1 <= pix_tog;
    pt_s2 <= pt_s1;
  end

  wire ce_pix = pt_s1 ^ pt_s2;   // one clk_core cycle, once per pixel
`ifdef MC64_SIM
  assign dbg_mc64_ce_pix = ce_pix;   // the harness's pixel-time oracle
`endif

  // ------------------------------------------------------------- save states
  //
  // The core declares the state buffer at 0x40000000 (the constant above,
  // which is also what the OS reads back from command 0x00A0/0x00A4) and the
  // controller is what makes that window real: it answers the host's reads
  // with the state the engine has produced, takes the host's writes as the
  // state coming back, crosses both through the FIFOs below, and reports the
  // command handshake. See rtl/save_state_controller.sv for the protocol and
  // rtl/mc64_savestates.sv for what a state is; the two FIFOs are
  // rtl/mc64_async_fifo.sv, the same gray-pointer pair the cartridge loader
  // above uses inline.
  //
  // Nothing here is timed against clk_74a's own cycles: the crossings are the
  // two FIFOs and the two toggling request lines, all declared asynchronous in
  // rtl/core_constraints.sdc.
  // Verilog-2001 here: this file is a VERILOG_FILE in rtl/ap_core.qsf, so no
  // `logic` and no unsized literals (Error 10839).
  wire [63:0] ss_save_data;
  wire        ss_save_wr, ss_save_full;
  wire        ss_load_rd, ss_load_empty;
  wire [63:0] ss_load_data;
  wire        ss_save_tog, ss_load_tog;
  // The host's end giving up (its watchdog): the engine drops the state and
  // hands the machine back -- see the note on mc64_savestates' abort_tog.
  wire        ss_abort_tog;
  wire        ss_busy, ss_load_done, ss_load_err;
  wire [31:0] ss_rd_data;

  wire        ss_start_ack, ss_start_busy, ss_start_ok, ss_start_err;
  wire        ss_load_ack, ss_load_busy, ss_load_ok, ss_load_err_bridge;
  wire        ss_underrun, ss_overrun;

  // The CPU taps' internal names. Declared in both builds so the machine's
  // port list does not change; only the debug ports above vanish from a fit.
  wire [15:0] dbg_mc64_pc_w;
  wire [ 7:0] dbg_mc64_a_w, dbg_mc64_s_w, dbg_mc64_p_w;
  wire [ 7:0] dbg_mc64_vb_w, dbg_mc64_pb_w;
  wire        dbg_mc64_ss_freeze_w, dbg_mc64_pause_hold_w, dbg_mc64_pause_freeze_w;
  wire        dbg_mc64_pause_muted_w;
  wire        dbg_mc64_ss_loading_w, dbg_mc64_pause_low_sat_w;
  wire        dbg_mc64_ss_flushing_w, dbg_mc64_vid_blank_w;
  wire [16:0] dbg_mc64_frame_cnt_w;

`ifdef MC64_SIM
  assign dbg_ss_underrun = ss_underrun;
  assign dbg_ss_overrun  = ss_overrun;
`endif

  save_state_controller #(
      // THE STAGING BUFFER IS THE SIZE THE HOST IS TOLD IT MAY SEND. A load
      // cannot be paced (the APF's bridge has no backpressure for a write), so
      // the buffer has to hold whatever `savestate_maxloadsize` allows the OS to
      // hand over in one go -- here, 8769 64-bit words. Both come from the one
      // constant below, so the promise to the OS and the buffer that has to take
      // it cannot drift apart. See the staging note in the controller.
      .STAGE_WORDS (SAVESTATE_MAXLOAD_SIZE / 8),
      // ... AND THE FORMAT TAG THE SAME WAY. The controller decides which of a
      // host's two 32-bit writes is the upper half of a 64-bit word by seeing
      // which one carries this tag, because the OS's own state files store the
      // halves the other way round from this file's window (measured: spec
      // 21.5, tools/sta_inspect.py). One constant, so the tag the engine checks
      // and the tag the assembly is decided by cannot drift apart.
      .TAG         (SAVESTATE_TAG)
  ) save_state_controller (
      .clk_74a        (clk_74a),
      .reset_74a      (~core_ready),

      .bridge_wr      (bridge_wr),
      .bridge_rd      (bridge_rd),
      .bridge_addr    (bridge_addr),
      .bridge_wr_data (bridge_wr_data),
      .ss_rd_data     (ss_rd_data),

      .ss_start       (savestate_start),
      .ss_start_ack   (ss_start_ack),
      .ss_start_busy  (ss_start_busy),
      .ss_start_ok    (ss_start_ok),
      .ss_start_err   (ss_start_err),

      .ss_load        (savestate_load),
      .ss_load_ack    (ss_load_ack),
      .ss_load_busy   (ss_load_busy),
      .ss_load_ok     (ss_load_ok),
      .ss_load_err    (ss_load_err_bridge),

      .clk_core       (clk_core),
      .reset_core     (machine_reset),
      .save_tog       (ss_save_tog),
      .load_tog       (ss_load_tog),
      .abort_tog      (ss_abort_tog),
      .ss_busy        (ss_busy),
      .ss_load_done   (ss_load_done),
      .ss_engine_err  (ss_load_err),

      .ss_save_data   (ss_save_data),
      .ss_save_wr     (ss_save_wr),
      .ss_save_full   (ss_save_full),
      .ss_load_rd     (ss_load_rd),
      .ss_load_data   (ss_load_data),
      .ss_load_empty  (ss_load_empty),

      .underrun       (ss_underrun),
      .overrun        (ss_overrun)
  );

  mc64_top machine (
      .clk     (clk_core),
      .clk_ram (clk_ram),
      .ce_pix  (ce_pix),
      .reset   (machine_reset),

      .rom_we  (rom_we),
      .rom_addr(rom_addr),
      .rom_data(rom_data),

      // the sweep, told to mc64_top so the five bank/vector shadows are
      // cleared BY THE LOAD rather than only by the sweep's write to $0100/$0101
      // passing the loader port (see the mirror's own note in mc64_top.sv, and
      // core_top's RAM-clear section for the sweep itself)
      .rom_clear(ram_clear_busy),

      .pad1    (pad1),
      .pad2    (pad2),

      // the user's pause (key[14], the Pocket's Select) with the OS's escapable
      // menu notify (see the PAUSE section)
      .pause   (pause_req),

      .video_rgb     (m_video_rgb),
      .video_de      (m_video_de),
      .video_hs      (m_video_hs),
      .video_vs      (m_video_vs),

      .audio_l       (m_audio_l),
      .audio_r       (m_audio_r),

      // save states: the machine's end of the two FIFOs the controller owns,
      // plus the requests and the status (see the save-state section above)
      .ss_save_tog  (ss_save_tog),
      .ss_load_tog  (ss_load_tog),
      .ss_abort_tog (ss_abort_tog),
      .ss_save_data (ss_save_data),
      .ss_save_wr   (ss_save_wr),
      .ss_save_full (ss_save_full),
      .ss_load_rd   (ss_load_rd),
      .ss_load_data (ss_load_data),
      .ss_load_empty(ss_load_empty),
      .ss_busy      (ss_busy),
      .ss_load_done(ss_load_done),
      .ss_load_err (ss_load_err),

      .dbg_pc       (dbg_mc64_pc_w),
      .dbg_frame_end()
`ifdef MC64_SIM
      ,
      // straight through to this module's own ports of the same names
      .dbg_ram_addr (dbg_ram_addr),
      .dbg_ram_data (dbg_ram_data),
      .dbg_a        (dbg_mc64_a_w),
      .dbg_s        (dbg_mc64_s_w),
      .dbg_status   (dbg_mc64_p_w),
      .dbg_ram_mirrors (dbg_ram_mirrors),
      .dbg_vram_bank_q (dbg_mc64_vb_w),
      .dbg_pal_bank_q  (dbg_mc64_pb_w),
      .dbg_vid_addr    (dbg_mc64_vid_addr),
      .dbg_vid_rdata   (dbg_mc64_vid_rdata),
      .dbg_ss_freeze   (dbg_mc64_ss_freeze_w),
      .dbg_pause_hold  (dbg_mc64_pause_hold_w),
      .dbg_pause_freeze(dbg_mc64_pause_freeze_w),
      .dbg_pause_muted (dbg_mc64_pause_muted_w),
      .dbg_ss_loading  (dbg_mc64_ss_loading_w),
      .dbg_ss_flushing (dbg_mc64_ss_flushing_w),
      .dbg_vid_blank   (dbg_mc64_vid_blank_w),
      .dbg_pause_low_sat(dbg_mc64_pause_low_sat_w),
      .dbg_frame_cnt   (dbg_mc64_frame_cnt_w)
`endif
  );

`ifdef MC64_SIM
  assign dbg_mc64_pc = dbg_mc64_pc_w;
  assign dbg_mc64_a  = dbg_mc64_a_w;
  assign dbg_mc64_s  = dbg_mc64_s_w;
  assign dbg_mc64_p  = dbg_mc64_p_w;
  assign dbg_mc64_vram_bank_q = dbg_mc64_vb_w;
  assign dbg_mc64_pal_bank_q  = dbg_mc64_pb_w;
`endif

  // --------------------------------------------------------------------- video
  //
  // ce_pix is generated above, before the machine, because the generator's
  // whole timing hangs off it. THERE ARE TWO video stages in this file and the
  // second one is the one the pads see: this first stage runs in clk_core at
  // pixel rate and chooses between the machine's picture and the bring-up
  // picture (emitting 24'h0 -- not the last colour, and not an end-of-line
  // word -- after DE falls), and the second, at the very end, captures all 28
  // bits into the pixel clock's own domain so the DDR link's two launches
  // always come from one pixel. See the two-stage note at the pads.

  reg [23:0] v_rgb  = 24'h0;
  reg        v_de   = 1'b0;
  reg        v_hs   = 1'b0;
  reg        v_vs   = 1'b0;

  // ------------------------------------------------------------ the picture
  //
  // ONE PICTURE: THE MACHINE'S. This file used to present its own 64x64
  // bring-up picture -- six 20x20 status blocks, one per boot condition, white
  // = true -- in place of the machine's while the machine was held, to tell a
  // dead video link apart from a machine with nothing to draw. It was removed
  // 2026-10-15 for the reason its own comment gave: a picture had been
  // confirmed on the panel, so it was no longer a diagnostic but a LOADING
  // SCREEN -- the panel showed those blocks for the whole quiet window between
  // the OS finishing a cartridge and this core releasing the machine.
  //
  // WHAT IS ON THE LINK INSTEAD. The machine's own VRAM through its own
  // palette, in the machine's own sync. mc64_top keeps the generator alive
  // from its own power-on reset (`mc64_video` is reset by `vid_reset`, not by
  // the machine's reset), so the envelope is live from power-up whether or not
  // the machine is running; at power-up RAM is zeros, so palette[0] -- black --
  // is what a held machine shows. The link is therefore still fully
  // exercisable while held: sim/tb_boot.cpp measures the envelope AND all 4096
  // active pixels in its held orderings, against the machine's own RAM.
  //
  // The six boot conditions are not lost, they are just not a picture any more:
  // they are `boot_state` below, which tools/boot_sim.sh reads through
  // `dbg_boot_state`. It is the only thing that ever needed them.
  //
  // `sx`/`sy` remain for one reason: the alert mark is placed in the window's
  // own coordinates. NOTHING else uses them.
  reg [6:0] sx = 7'd0;          // active pixel within the machine's 64x64 window
  reg [6:0] sy = 7'd0;          // ... and the line

  /* The alert mark, in the window's own coordinates: the top-left 8x8 pixels.
   * It is drawn AFTER the picture itself, so it lands on whatever the machine
   * is driving, and only while DE is high (the platform's
   * blanking rule is asserted over the whole blanking by sim/tb_boot.cpp, and
   * this must not breach it). Note what it does NOT overlap: the boot
   * harness's cart region is RAM $0200 onwards, which with the framebuffer
   * bank at 0 is window pixel 0x200 -- row 8, column 0 -- so the mark's eight
   * rows sit above it and the byte-for-byte cartridge comparison is untouched.
   *
   * #FF0000 is a colour the default palette can reach (index 3 is exactly it),
   * so the mark is identified by SHAPE AND POSITION -- a solid 8x8 block at the
   * window origin -- not by its colour alone. Its authority is the counter it
   * comes from: the mark and sim/tb_boot.cpp's `cart writes dropped` are the
   * same fact, and a cartridge that draws a red square there while the counter
   * says 0 is a cartridge drawing a red square. */
  wire alert_mark = drop_pix[1] & (sx < 7'd8) & (sy < 7'd8);

  // No sync shaping here any more: the generator puts one HS per line in that
  // line's last gap cycle and the VS at x=0 of the frame's LAST line (whose own
  // HS is 65 cycles later), so the platform's gap rules are satisfied by
  // construction and the capture window it opens holds the picture (see the
  // frame-opening note in mc64_video.sv). This stage passes the machine's
  // picture through at pixel rate and emits 24'h0 after DE falls.

  // `run_core` is a slow level from the clk_74a domain; two flops into the
  // machine domain so the picture changes on a settled value rather than
  // mid-edge. Worst case the switch lands inside one frame, which costs one
  // torn frame at the handover.
  //
  /* The canary, into the pixel domain, because `alert_mark` is evaluated in the
   * generator's own cycle. (This used to be paired with a `run_pix`
   * synchroniser for the picture select; that picture is gone -- see the video
   * stage -- so the only thing that crosses here now is the mark.)
   *
   * A note the deletion keeps: this block once declared `reg [1:0] run_sync`,
   * in the same scope as the three-flop `run_sync` at the top of the file, and
   * Quartus rejects that outright --
   *
   *   Error (10149): ... identifier "run_sync" is already declared in the
   *   present scope, core_top.v:1173 (original at core_top.v:937)
   *
   * It got as far as a fit because nothing in the repo read this file with
   * errors left fatal: tools/lint.sh linted only mc64_top, and tools/boot_sim.sh
   * -- which does build core_top -- was reusing an object file from before the
   * declaration was duplicated, so its pass was a stale build of a different
   * core_top. Both are closed: lint.sh now lints core_top with errors fatal, and
   * deleting sim/obj_boot* forces the harness to rebuild. Do that when in doubt.
   *
   * ...and the mark, into the pixel domain for the same reason and by the
   * same pattern: see the note at `cart_incomplete` in the cartridge section.
   * Only the sticky one-bit half of it crosses. */
  reg [1:0] drop_pix = 2'b00;

  always @(posedge clk_core)
    drop_pix <= {drop_pix[0], cart_incomplete};

  always @(posedge clk_core) begin
    if (ce_pix) begin
      // the window's own coordinates -- the alert mark's box, nothing else:
      // one step per active pixel, reset by the frame's vs
      if (m_video_vs) begin
        sx <= 7'd0;
        sy <= 7'd0;
      end else if (m_video_de) begin
        if (sx == 7'd63) begin sx <= 7'd0; sy <= sy + 7'd1; end
        else                  sx <= sx + 7'd1;
      end

      v_de   <= m_video_de;
      v_hs   <= m_video_hs;
      v_vs   <= m_video_vs;

      /* RGB OUTSIDE THE PICTURE IS 24'h0 -- AND NOTHING ELSE.
       *
       * The platform's rule, verbatim (Analogue developer docs, "Bus
       * Communication" -> VIDEO, read 2026-10-10): "RGB pixel value should
       * remain all zero when DE is deasserted unless using the frame feature
       * bits or end-of-line bits." and "When driving the safe value of 24'h0
       * outside of visible display areas, the end-of-line function code Set
       * Scaler Slot to parameter 0 is the default."
       *
       * The end-of-line word is what may be clocked out on the cycle after DE
       * falls: endline[23:13] = parameter (the scaler-slot index),
       * endline[12:3] = 0 (reserved), endline[2:0] = 000 = "Set Scaler Slot".
       * So 24'h0 IS a valid end-of-line command -- Set Scaler Slot 0, this
       * core's only declared mode (pocket/video.json, one scaler mode) -- and
       * 0x002000, which this stage used to emit there, is Set Scaler Slot
       * **1**: a switch request against a mode that does not exist, repeated
       * on every line of every frame. The core that is proven pixel-exact on
       * this panel drives 24'h0 outside its picture and issues no end-of-line
       * commands at all (C:/Code/Wasm4/VexiiRiscv, VIDEO_LINK.md).
       *
       * The 0x002000 came from rtl/core_top_template.v, the stock scaffolding,
       * where bits 14/13 are named hide_overscan/square_pixels; those names are
       * not in the current documentation, and this core has no reason to
       * switch scaler slots, set interlace/frame feature bits, or rescan
       * frames, so it emits none of them.
       *
       * The other half of the rule -- that the register must not HOLD the last
       * picture colour through the blanking -- is what the default assignment
       * above provides. What was measured on the old stage: 17,211 cycles per
       * frame with a nonzero level while DE was low, i.e. the entire blanking.
       * sim/tb_video.cpp and sim/tb_boot.cpp now assert the whole blanking is
       * zero, not merely free of held colours.
       */
      v_rgb <= 24'h0;
      if (m_video_de) begin
        /* THE MACHINE'S PICTURE, AND NOTHING ELSE. What is on the link while
         * the machine is held is the machine's own VRAM through its own
         * palette -- palette[0], black, at power-up -- in the machine's own
         * sync, because mc64_top keeps the generator alive from its own POR and
         * mc64_video is the only timing source this file has.
         *
         * This replaces the six-block bring-up picture that used to sit here
         * (2026-10-15). That picture did its job -- it is what told the panel's
         * bring-up faults apart, block by block, and its own comment said to
         * delete it once a picture was confirmed -- but it was also the
         * "loading screen" the panel showed for the whole ~1 s between the OS
         * finishing a cartridge and this core releasing the machine, and it is
         * not the machine's output. The debug taps below (`dbg_boot_state`)
         * still carry the same six boot conditions for tools/boot_sim.sh, which
         * no longer needs a picture to read them. */
        v_rgb <= m_video_rgb;
        /* The alert mark, last, so it lands on the picture. See its note at
         * `alert_mark`.
         *
         * 24'hF80000, NOT 24'hFF0000, because this is the one colour in this
         * file that does not come from mc64_video and the link is RGB565: full
         * red is 31/31, left-aligned F8, and everything below zero. Emitting the
         * 888 spelling here put seven bits on the wire that the format says
         * cannot exist, and sim/tb_boot.cpp's `link format` check caught it on
         * the first run after the palette became 16-bit (64 of 8192 pixels,
         * first FF0000) -- which is exactly what that check is for. It is also
         * the same colour the machine's default palette reaches for index 3. */
        if (alert_mark)
          v_rgb <= 24'hF80000;
      end
    end
  end

`ifdef MC64_SIM
  assign dbg_pause_req   = pause_req;
  assign dbg_os_menu_en  = os_menu_en;
  assign dbg_ss_freeze   = dbg_mc64_ss_freeze_w;
  assign dbg_pause_hold  = dbg_mc64_pause_hold_w;
  assign dbg_pause_freeze = dbg_mc64_pause_freeze_w;
  assign dbg_pause_muted  = dbg_mc64_pause_muted_w;
  assign dbg_ss_loading   = dbg_mc64_ss_loading_w;
  assign dbg_ss_flushing  = dbg_mc64_ss_flushing_w;
  assign dbg_vid_blank    = dbg_mc64_vid_blank_w;
  assign dbg_pause_low_sat = dbg_mc64_pause_low_sat_w;
  assign dbg_frame_cnt   = dbg_mc64_frame_cnt_w;
  assign dbg_run_core    = run_core;
  assign dbg_boot_state  = boot_state;
  assign dbg_cart_ready  = cart_ready;
  assign dbg_core_ready  = core_ready;
  assign dbg_cart_drop   = cart_drop;
  assign dbg_cart_incomplete = drop_pix[1];
`endif

  // NOTE on reading the screen. There is one picture now, and it says which of
  // three states the boot is in:
  //
  //   no file list at all       the OS never accepted the handshake. Nothing
  //                             here ran: look at the bridge, clk_74a and the
  //                             package's data.json, not at the machine
  //   black (palette[0])        the handshake worked, the machine is held or
  //                             running with nothing drawn -- the state the
  //                             panel is in while a cartridge is being chosen
  //                             and while it is arriving
  //   a picture that moves      working
  //   any picture with a red
  //   8x8 square in its
  //   top-left corner            the cartridge in RAM is KNOWN to be incomplete
  //                             -- words were dropped while the OS streamed it
  //                             (cart_drop != 0), so whatever else the picture
  //                             looks like, this is not the cartridge's output
  //                             and no other fault should be read from it.
  //
  // "The panel is black" is a weaker statement than it used to be: the six
  // blocks that used to separate "the link is dead" from "the machine has
  // nothing to draw" are gone (see the picture note above), and black is what
  // both of those look like. What has NOT changed is the fix for the ambiguity
  // that actually cost this core panel tests: the OS handshake no longer depends
  // on the raw PLL lock (`core_ready` above), so a file list appearing at all is
  // the proof that clk_74a, the bridge and the package are good -- and the boot
  // conditions are still readable, as `dbg_boot_state`, by tools/boot_sim.sh.
  // Its held orderings still require a live link: the envelope and all 4096
  // active pixels are measured against the machine's own RAM while the machine
  // is held.
  // -------------------------------------------- the pads' last register (0 deg)
  //
  // WHY THERE ARE TWO STAGES, AND WHERE THE SPECKLED PICTURE CAME FROM.
  //
  // The stage above is in the MACHINE domain: it is registered on ce_pix, which
  // is an edge detect of the pixel clock two flops into clk_core, so its
  // outputs change 1-2 clk_core cycles after every pixel edge -- 21-41% of the
  // new 4.85-cycle pixel period (and 40-60% of the old 5-cycle one).
  //
  // The link is DDR: apf_top's `isco` launches video_rgb[23:12] on one edge of
  // video_rgb_clock and [11:0] on the other, and the scaler captures BOTH halves
  // on the 90-degree output's edges -- 25% and 75% of the pixel period, a
  // quarter period after each launch. So a change that falls anywhere BETWEEN
  // those two captures puts one pixel's high bits on the wire with the next
  // pixel's low bits: a torn pixel, i.e. a single-pixel colour error at every
  // colour edge on the screen. That is the speckled/glitched picture the panel
  // has been showing, it was there with the old pixel clock as well (40-60% is
  // inside the same window), and it is the one fault in this core that no
  // simulation can see -- Verilator has no DDR cell and no scaler, so the panel
  // is the only oracle for it.
  //
  // THE FIX IS TO PUT THE DATA WHERE THE HARDWARE-PROVEN CORE PUTS IT. The
  // sibling core that runs on this panel generates its whole scanout in the
  // pixel clock's own domain, so its pads change exactly ON the 0-degree edge
  // -- the centre of the window between the two captures -- and its own PLL
  // note names that as the property the 90-degree capture clock exists to
  // complement ("0 deg samples exactly when the data changes"). This stage is
  // that arrangement: ONE register on clk_pixel, which IS video_rgb_clock,
  // capturing all 28 bits together so de/hs/vs can never disagree with the
  // colour they frame.
  //
  // The crossing is clean in the direction that matters: the source holds each
  // pixel for a whole pixel period and has been stable since 1-2 clk_core after
  // the PREVIOUS pixel edge, i.e. ~2.8 cycles (≈440 ns) before this capture
  // edge, with no combinational path between the two domains. It costs one
  // pixel of latency, uniformly on rgb/de/hs/vs, so the envelope every harness
  // measures is unchanged; and where the pads change is now a property of the
  // clock tree alone -- no clk_core phase, no synchroniser latency, and no
  // dependence on the machine/pixel ratio, which is exactly why this became
  // worth doing once the pixel clock stopped being a whole division of the
  // machine (see the WHEN THE DATA CHANGES note in mf_pll_mc64_0002.v).
  /* ---- THE COLOUR FORMAT IS APPLIED IN mc64_video, NOT HERE --------------
   *
   * This stage is the domain crossing and nothing else: one register on
   * clk_pixel capturing all 28 bits, so the DDR link's two launches always come
   * from one pixel. The colour is already RGB565 by the time it arrives -- the
   * picture is generated in 565, not quantised on the way out: mc64_video holds
   * its palette as 16-bit entries (packed as the reload reads the machine's
   * three palette bytes), its default palette is computed as 565, and its output
   * stage left-aligns that into the 24-bit word this port carries, with the
   * unused low bits zero. So `video_rgb` here is 24 bits wide because the APF's
   * DDR link is (apf_top: [23:12] on one edge, [11:0] on the other), and the
   * picture inside it is 65,536 colours -- red [23:19], green [15:10], blue
   * [7:3], everything below zero. The whole low half of the bus, [11:0], is a
   * constant in active video, which sim/tb_boot.cpp asserts per pixel.
   *
   * There is deliberately NO second pack here. Two would be harmless (the
   * truncation is idempotent) but it would hide where the format is decided, and
   * the question "is this 565 or 888" has one answer in this tree now:
   * mc64_video's palette cache. */
  reg [23:0] pix_rgb = 24'h0;
  reg        pix_de  = 1'b0;
  reg        pix_hs  = 1'b0;
  reg        pix_vs  = 1'b0;

  always @(posedge clk_pixel) begin
    pix_rgb <= v_rgb;
    pix_de  <= v_de;
    pix_hs  <= v_hs;
    pix_vs  <= v_vs;
  end

  assign video_rgb = pix_rgb;
  assign video_de  = pix_de;
  assign video_hs  = pix_hs;
  assign video_vs  = pix_vs;

  // --------------------------------------------------------------------- audio
  //
  // The machine's half of the audio is in mc64_top: the APU, and the level it
  // produces. This is the other half, rtl/mc64_audio.sv -- the anti-alias
  // filter, the 48 kHz resample and the I2S serializer -- because it is the
  // clock that decides its shape and the audio clock is this module's.
  //
  // mclk is clk_audio: 12.184615 MHz (VCO/52 -- see the PLL and spec s.6), not
  // 12.288, so LRCK comes out 47.596 kHz rather than 48.000 and every sound is
  // 0.84% flat -- 14 cents, the one open item this leaves. The interface does
  // not care: the scaler is handed mclk and lrck by the core and takes the
  // frames at the rate they arrive, exactly as the sibling core's fractionally
  // generated mclk does. An exact 12.288 needs its own PLL and a fractional
  // VCO (agg23's openfpga-NES builds one); the video PLL's integer outputs can
  // never make it at 633.6 MHz, and spec s.6 says so.
  //
  // audio_adc is the scaler's own input to the core. Unused, as in the stock
  // template, and it stays an input: nothing here may drive it.

  mc64_audio audio_out (
      .clk       (clk_core),
      .reset     (machine_reset),
      .level     (m_audio_l),
      .clk_audio (clk_audio),
      .audio_lrck(audio_lrck),
      .audio_dac (audio_dac)
  );

  assign audio_mclk = clk_audio;

endmodule
