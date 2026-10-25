/*
 * Minicube64 machine top level.
 *
 * CLOCKING. TWO clocks, both from the PLL in core_top:
 *
 *   clk      6.4 MHz  -- the machine: 6502, RAM port B, APU, the save-state
 *                        bus, and the video generator's counters.
 *   clk_ram  6.4 MHz, 180 degrees -- RAM port A's clock ONLY. The CPU advances
 *                        every clk; the RAM's own address register latches AB
 *                        one half-cycle later, when AB has been stable since
 *                        the rising edge, and the read comes back REGISTERED at
 *                        that same edge (see mc64_ram.sv -- the register has to
 *                        be the array's own or Quartus cannot infer an M10K),
 *                        so the byte for the address the CPU presented is valid
 *                        for the rest of that CPU cycle. That is what lets the
 *                        machine run on one 6.4 MHz clock with no master and no
 *                        RDY stall.
 *
 * The 6502's RDY is tied HIGH: arlet's core is documented as expecting "valid
 * data (DI) expected on the cycle AFTER valid address", and DI is registered
 * once per CPU cycle below, which is exactly that contract. The old 25.6 MHz
 * master + 4-phase enable + RDY-as-stall existed only to pay for a RAM read; a
 * registered read off the 180-degree clock pays for it in half a cycle.
 *
 * The video generator counts PIXEL periods, not CPU cycles: `ce_pix` is one
 * pulse per pixel, generated in core_top from the pixel clock itself so its
 * position inside the pixel period is known. WHERE the picture this file
 * produces changes is deliberately not the same question: the pads are
 * re-timed into the pixel clock's own domain by core_top's last video stage,
 * so the 1-2 machine clocks of latency in this domain's register stop at that
 * stage and never reach the DDR link -- see the TWO STAGES note in core_top.v
 * and requirement 1 in mf_pll_mc64_0002.v. The pixel clock is 1.32 MHz against a 6.4 MHz
 * machine: 160/33 = 4.8485 machine cycles per pixel, not a whole number, so the
 * picture no longer ticks once per CPU cycle. The CPU itself is not divided; it
 * advances every machine cycle.
 *
 * THE CPU'S FRAME IS THE CPU'S OWN, measured by the counter below: 106,666
 * machine clocks, i.e. machine.c's exec6502(6400000/60), because that is the
 * budget the program is written against and the vblank interrupt is what tells
 * it the budget is over. The RASTER is the video's own and free-runs under it:
 * 22,000 pixels of the 1.32 MHz pixel clock = 106,666.67 machine clocks =
 * 60.000000 Hz EXACTLY, by construction. The two frames are 0.67 of a machine
 * clock apart -- 6.3 ppm -- and are deliberately not locked together; see the
 * frame counter below, and WHY 22,000 in mc64_video.sv for why the pixel count
 * is the free parameter and the 60 Hz is exact rather than approached.
 *
 * Reset and vectors. Minicube64 has no reset vector: machine.c calls
 * reset6502() and then assigns `pc = 0x200` outright, and its IRQ vector lives
 * at $010E rather than $FFFE. Both are handled here, in the read path, so the
 * vendored CPU core needs no behavioural modification:
 *
 *   read $FFFC/$FFFD -> 0x00/0x02      (so the stock reset sequence lands on
 *                                       PC = 0x0200, but ONLY while the reset
 *                                       sequence is running -- see res_seq)
 *   read $FFFE/$FFFF -> mirror of $010E/$010F, but only while an IRQ is
 *                       actually being taken, so a BRK instruction still reads
 *                       the real $FFFE from RAM as it should.
 *
 * Known, deliberate divergences (see docs/minicube64-fpga-spec.md s.9/s.7):
 *   - Illegal opcodes are not implemented (arlet is documented-only). No ROM in
 *     the upstream demos uses them; revisit if one turns up.
 *   - $0102/$0103 read back the live controller state instead of an emulated
 *     key-event accumulator. Identical for any ROM that polls input.
 */
`default_nettype none
`timescale 1ns / 1ps

module mc64_top (
    input  logic        clk,          // 6.4 MHz machine clock: CPU, RAM port B, APU, video
    input  logic        clk_ram,      // 6.4 MHz, 180 degrees: RAM port A address latch
    input  logic        ce_pix,       // one pulse per pixel period (see core_top)
    input  logic        reset,        // active high; also gates cartridge load

    // ---- cartridge load, driven while reset is held ----
    input  logic        rom_we,
    input  logic [15:0] rom_addr,
    input  logic [ 7:0] rom_data,
    /* ---- the RAM clear is in progress (core_top's sweep) ----
     * A LEVEL that is high for the whole of the sweep the load's arrival starts,
     * and only then. It exists for ONE reason: the five shadows below mirror RAM
     * bytes, and while this is high they are held at zero the way the bytes they
     * mirror are being written to zero -- so a load's register state comes from
     * the SWEEP rather than from the sweep's write to a particular address
     * landing next to a mirror that only listens to two of the 65,536 addresses
     * it walks past. The RAM byte and the register still end at the same value,
     * the CPU is held across the whole sweep either way, and the loader cannot
     * address these five -- so this changes WHEN the zero lands and nothing else.
     * (Spec 18.4 is the port-mirror half of the same guarantee; that mirror
     * stays, and stays exact, for every other write.) */
    input  logic        rom_clear,

    // ---- controllers, active high ----
    // bit0 A(Z)  bit1 B(X)  bit2 C  bit3 Start  bit4 Up  bit5 Down  bit6 Left  bit7 Right
    input  logic [ 7:0] pad1,
    input  logic [ 7:0] pad2,

    /* ---- pause (2026-10-16) ----
     * A LEVEL from core_top: the machine's execution is to stop where it is and
     * stay stopped (core_top's pause button is a toggle -- key[14], the Pocket's
     * Select, which is NOT the bit the games' Start input is on [that is key 15,
     * Start]; see its PAUSE section). The OS's menu notify (0x00B0 bit 0) is
     * part of this level, through core_top's `os_menu_en` -- so the machine
     * stops behind the menu, and one Select press takes it back if that level is
     * ever left standing (spec 21.12).
     *
     * It is NOT the save-state freeze. That freeze has to stop the machine at an
     * INSTRUCTION BOUNDARY (so a state can be taken), so it is built here from
     * the CPU's own dispatch cycle and it holds the machine's frame counter with
     * it. A pause has neither requirement and one it must not break: the CPU
     * stops, everything else -- the raster, the frame counter, the phase lock
     * that keeps the raster reading finished frames -- keeps running, because
     * the Pocket's scaler is synced to this core's raster. Freezing the raster
     * would take the picture off the panel instead of holding it. */
    input  logic        pause,

    // ---- APF video ----
    output logic [23:0] video_rgb,
    output logic        video_de,
    output logic        video_hs,
    output logic        video_vs,

    // ---- audio ----
    // The APU's mixer level (0 == silence), the same value on both channels.
    // A level and not a signed sample -- see the audio section at the end of
    // this file -- so it is unsigned, and core_top's mc64_audio turns it into
    // the Pocket's I2S stream.
    output logic [15:0] audio_l,
    output logic [15:0] audio_r,

    // ---- save states (spec s.10/s.11) ----
    // The framework's REGISTER bus (ss_reg_*) stays inside this file: the
    // machine's own registers and the APU's sit on it, and mc64_savestates
    // drives it. What crosses this boundary is the state STREAM -- the two
    // ends of the FIFOs core_top's save_state_controller owns -- plus the
    // requests and the freeze.
    input  logic        ss_save_tog,    // a state is wanted (a toggling level from clk_74a)
    input  logic        ss_load_tog,    // ... or one is being restored
    input  logic        ss_abort_tog,   // ... or the host has given up on it
    output logic [63:0] ss_save_data,   // one word of the state, into the save FIFO
    output logic        ss_save_wr,
    input  logic        ss_save_full,
    output logic        ss_load_rd,
    input  logic [63:0] ss_load_data,
    input  logic        ss_load_empty,
    output logic        ss_busy,        // a state is in flight
    output logic        ss_load_done,   // ... and this pulse says one was restored
    output logic        ss_load_err,    // ... or that the file was not this build's

    // ---- debug / testbench ----
    output logic [15:0] dbg_pc,
    output logic        dbg_frame_end
`ifdef MC64_SIM
    ,
    output logic [ 7:0] dbg_a,
    output logic [ 7:0] dbg_x,
    output logic [ 7:0] dbg_y,
    output logic [ 7:0] dbg_s,
    output logic [ 7:0] dbg_status,
    input  logic [15:0] dbg_ram_addr,
    output logic [ 7:0] dbg_ram_data,
    output logic [15:0] dbg_apu_dma_addr,  // the DMC's sample fetch: its address
    output logic [ 7:0] dbg_apu_dma_byte,  // ... the byte that read returned
    output logic        dbg_apu_dma_ack,   // ... and one pulse per byte accepted
    output logic        dbg_apu_ce,        // the APU's clock enable (2A03 rate)
    output logic        dbg_apu_get,       // ... and get_or_put, its phase
    output logic [15:0] dbg_vid_addr,   // the video's RAM port B address
    output logic [ 7:0] dbg_vid_rdata,  // the byte its port B is presenting
    /* The banks the video LATCHED for the frame it is drawing (mc64_video's
     * frame-end latch): the palette and framebuffer a captured picture was
     * actually drawn through. Not the same thing as the RAM bytes at
     * $0100/$0101, which is what sim/tb_mc64.cpp used to reconstruct them
     * from -- and which is one bank stale exactly when a cartridge stores
     * those addresses between the VS and the frame's last cycle. */
    output logic [ 7:0] dbg_vram_bank_q,
    output logic [ 7:0] dbg_pal_bank_q,
    /* The palette reload's own read window: the pixel cycle on which the video
     * consumes a palette byte and its index into the 768. A harness needs both
     * to say what the palette cache actually holds -- it is filled from one
     * read port over ~8.7 lines, so sampling it at the frame boundary is not
     * the same thing (see the port note in rtl/mc64_video.sv). */
    output logic        dbg_ld_valid,
    output logic [ 9:0] dbg_ld_idx,
    /* TEMPORARY (remove me): the machine's own hold latches and its frame
     * counter, so a harness can say WHICH of them is holding a frozen machine
     * and whether the counter that releases them is still rolling. */
    output logic        dbg_ss_freeze,
    output logic        dbg_pause_hold,
    output logic        dbg_pause_freeze,
    output logic        dbg_pause_muted,  // TEMPORARY (remove me): the load's hand-back
    output logic        dbg_ss_loading,   // TEMPORARY (remove me): a state is being restored
    // The load's picture rule, so a harness can hold it to account: the bytes
    // the raster is drawing from are written only while `dbg_vid_blank` (the
    // raster's blanking slot) is high, and `dbg_ss_flushing` is that write.
    output logic        dbg_ss_flushing,
    output logic        dbg_vid_blank,
    output logic        dbg_pause_low_sat,// TEMPORARY (remove me): the level has been low a frame
    output logic [16:0] dbg_frame_cnt,
    output logic        dbg_ce,        // one pulse per 6502 cycle
    output logic [ 5:0] dbg_state,     // arlet microcode state (13 == FETCH)
    output logic        dbg_instr_start,  // first cycle of every instruction
    output logic        dbg_int_entry,    // this 6502 cycle is interrupt entry
    output logic        dbg_int_done,     // interrupt vector fully fetched
    /* The five RAM mirrors (see the shadow block below), in the order the
     * savestate word carries them: { dmc_bank, irqvec_hi, irqvec_lo, pal_bank,
     * vram_bank }. They are NOT RAM, so no harness sees them through the
     * machine's RAM dump -- which is exactly why a fault in them (a mirror that
     * outlives the byte it mirrors, or one that never follows the CPU at all)
     * survives every RAM comparison in this tree. This tap is what makes them
     * observable: sim/tb_boot.cpp's reload ordering reads it while each
     * cartridge is up. Combinational, simulation-only, absent from a fit. */
    output logic [39:0] dbg_ram_mirrors
`endif
);

    // The save-state register indices the APU takes as parameters (rtl/
    // regs_savestates.sv, MiSTer's table). Imported here rather than passed as
    // literals so the indices have one definition in the tree.
    import regs_savestates::*;

    /* ------------------------------------------------------------------ *
     * Save states: the signals, declared here and driven at the end of the
     * file (the engine is the last thing in it). Two of the machine's own
     * registers sit on the framework's register bus -- the CPU's architectural
     * state and the machine's own counters/shadows -- and the APU's four are
     * already there (it takes their indices from regs_savestates.sv).
     * ------------------------------------------------------------------ */
    logic [63:0] ss_reg_din, ss_reg_dout;   // the wired-or of every register
    logic [ 9:0] ss_reg_adr;
    logic        ss_reg_wren, ss_reg_rst;
    logic        ss_sleep;                  // the engine wants the machine stopped
    logic        ss_freeze;                 // ... and it is
    logic        ss_stop;                   // ... this cycle is the stop itself
    logic        ss_cpu_we;                 // one pulse: put the CPU's state back
    logic        ss_loading;                // a state is being restored (the APU wants this)
    logic        ss_flushing;               // ... and the picture's own bytes are being written
    logic        ss_reset;                  // the engine's reset pulse inside a load

    // The machine's own reset, which is the platform's plus the engine's. A
    // load ends with this: everything that comes back through a reset path (the
    // 6502's register file, the APU's channels, the frame counter) re-reads its
    // savestate register as it comes out, which is how the framework's
    // registers are meant to work -- see the save-state section at the end.
    wire rst = reset | ss_reset;

    // The engine's RAM port, which borrows port A of mc64_ram while it runs:
    // the CPU is stopped for the whole transfer and its port is idle, so the
    // state's byte-at-a-time walk costs the machine nothing.
    logic [15:0] ss_ram_addr;
    logic        ss_ram_own;                // the port belongs to the engine
    logic        ss_ram_rden;
    logic        ss_ram_wren;
    /* FOUR BYTES A CLOCK (2026-10-18). The engine moves a whole 32-bit chunk of
     * the machine's memory per clock into port A, which is 16,384 x 32 now; the
     * CPU's byte access is a byte enable and a mux on the same port. See
     * rtl/mc64_ram.sv for why that is the same 64 M10K blocks. */
    logic [31:0] ss_ram_wdata;

    /* ------------------------------------------------------------------ *
     * The VIDEO's reset, which is NOT `reset`.
     *
     * `reset` holds the CPU and gates the cartridge writes. The video generator
     * must not be held with it, because the link carries the MACHINE's picture
     * while the machine is held (no cartridge has been released yet) -- its VRAM
     * through its own palette, black at power-up -- and that picture rides this
     * generator's sync. The link has to be alive from power-up, which is the
     * whole point of showing anything at all.
     *
     * So the video is held for exactly two things: the four clocks after
     * power-up (enough for the block RAMs and the position counters; the PLL is
     * what really decides when there are clocks), and ONE cycle at the machine's
     * release.
     *
     * The one cycle at the machine's release restarts the raster's position
     * counters. The raster and the CPU's frame are separate things now (a pixel
     * is 160/33 machine clocks, so the raster is not a count of them; the CPU's
     * frame is 106,666 of them), so this is
     * no longer about aligning the two -- it is about making the raster's phase
     * relative to the machine's release a fixed, known thing rather than
     * whatever the power-up order happened to leave, which is what the bring-up
     * picture in core_top and the boot harness's captured frame both see.
     *
     * The CPU is unaffected either way: it is held by `reset` directly, and the
     * write strobe below is gated by `~reset` so a free-running phase counter
     * cannot put a byte into RAM from an address the CPU does not own yet.
     * ------------------------------------------------------------------ */
    logic [3:0] vid_por = 4'd0;

    always_ff @(posedge clk)
        if (!(&vid_por))
            vid_por <= vid_por + 4'd1;

    reg reset_q;

    always_ff @(posedge clk)
        reset_q <= reset;

    wire machine_released = reset_q & ~reset;

    wire vid_reset = !(&vid_por) | machine_released;

    // The RAM answers inside the cycle: nothing to wait for, EXCEPT while a
    // save state is being transferred. The engine stops the machine at the
    // DECODE cycle that dispatches an instruction (see the save-state section at
    // the end of this file), which is the one point where a stopped 6502 can be
    // left and resumed without carrying any microcode state. `ss_stop` is the
    // cycle itself, `ss_freeze` the hold: RDY must drop DURING the stop cycle
    // (a registered stop would land one cycle late, after the dispatch).
    //
    // `pause` is the third way to hold this CPU, and it is the user's (core_top
    // turns the pause button and the OS's escapable menu notify into one level
    // -- see the pause port's note and spec 21.12).
    // It is TWO stages, and the second one is why the CPU is never stopped in
    // the middle of an instruction:
    //
    //   pause_hold   the request, latched on the machine's frame boundary so a
    //                pause holds a whole frame (assigned below);
    //   pause_stop   this cycle, the first instruction boundary after the
    //                request -- the same DECODE cycle the save-state engine
    //                stops on, i.e. the one point where a stopped 6502 carries
    //                no microcode state;
    //   pause_freeze the hold for as long as the request lasts.
    //
    // The CPU then TAKES NO BUS CYCLES AT ALL: RDY low holds the whole core, so
    // while the user has it paused it neither reads nor writes the machine's
    // memory. That is the property a NOP feed was wanted for, and it cannot be
    // had by injecting $EA onto DI instead: at DECODE, DI already holds the
    // NEXT opcode, prefetched by the previous instruction's last cycle, so
    // substituting a NOP there overwrites an instruction rather than idling the
    // CPU. That was tried and measured -- boot_sim reported 258 of 1024
    // cartridge bytes wrong after the pause and a machine that never resumed.
    // Holding RDY at the boundary is the same thing done safely: nothing is
    // half-executed on either side of the pause, so there is no state to unwind
    // and no cycle count for a resume to get wrong, which is the whole reason
    // the stop is taken at a boundary instead of wherever the frame boundary
    // happened to land.
    //
    // A WRITE CYCLE IS NEVER THE STOP POINT, and that is the 6502's own rule as
    // much as it is this core's: RDY low stops the processor on a read cycle. A
    // DECODE cycle is a read, so the pause can always take it there and the old
    // `& ~WE` special case is gone with the mid-instruction hold that needed it.
    /* Declared here rather than at their assignment, because `cpu_rdy` below is
     * built out of them and this file keeps one definition of each: the pause's
     * two stages, and the two CPU taps its stop is taken on. All five are
     * assigned further down (the taps are outputs of the CPU instance). */
    logic pause_hold;        // the request, latched on a frame boundary
    logic pause_freeze;      // the hold, taken on the next instruction boundary
    logic pause_stop;        // this cycle: that boundary
    logic cpu_instr_start;   // CPU tap: the DECODE cycle that dispatches
    logic cpu_int_entry;     // CPU tap: an interrupt entry's own DECODE

    assign pause_stop = pause_hold & ~pause_freeze & ~ss_sleep & ~rst
                        & cpu_instr_start & ~cpu_int_entry;

    wire cpu_rdy = ~(ss_freeze | ss_stop | pause_stop | pause_freeze);

    /* ------------------------------------------------------------------ *
     * CPU
     * ------------------------------------------------------------------ */
    wire [15:0] AB;
    wire [ 7:0] DO;
    logic [7:0] DI;      // driven by the read-substitution block below
    wire        WE;

    logic [7:0] cpu_P;
    logic [5:0] cpu_state;
    logic       cpu_int_done;

    logic irq_req;

    // The CPU's architectural state, tapped for the state file (and for the
    // harness). `dbg_*` are read-only taps inside the vendored core; the
    // restore side is the ss_* port group that core was given for this.
    wire [15:0] cpu_dbg_pc;
    wire [ 7:0] cpu_dbg_a, cpu_dbg_x, cpu_dbg_y, cpu_dbg_s;
    logic [63:0] ss_cpu_word;    // the register as it was saved / restored

    /* ---- the pause, in the machine's own domain ------------------------
     * Two flops for the level (it is a slow, human-speed signal -- the
     * controller's own poll is ~16 ms -- but it crosses clock domains, so it is
     * synchronised like every other level in this file).
     *
     * `pause_hold` ENGAGES ON THE FRAME BOUNDARY and that is not cosmetic: the
     * raster reads the frame the CPU has just finished, so stopping the CPU
     * anywhere inside a frame leaves the user looking at the half-drawn frame
     * the old code path had already started to write. At `frame_cnt ==
     * FRAME_CLOCKS-1` the cartridge has finished the frame (the vblank interrupt
     * is what tells it so) and the next one has not begun, so what is held is a
     * whole frame. It costs at most one frame of delay between the button and
     * the freeze, which is what the sibling core on this panel pays too.
     *
     * It RELEASES the moment the level drops -- there is nothing to unwind, the
     * CPU is held on its own RDY -- and while `ss_sleep` is up, because a
     * save-state transfer needs the CPU to run on to its next instruction
     * boundary (a paused CPU never reaches one, so the state command would time
     * out with the machine frozen). The state transfer wins; when it is over the
     * pause re-engages at the next boundary if the level is still up -- EXCEPT
     * after a LOAD, which hands the machine back instead (see A LOAD HANDS THE
     * MACHINE BACK below, and the panel's `loads don't unpause`).
     *
     * The frame counter is NOT held: it, the raster and the phase lock between
     * them keep running, which is what keeps the panel's picture up while the
     * machine is stopped. */
    logic pause_s1, pause_s2;

    always_ff @(posedge clk) begin
        pause_s1 <= pause;
        pause_s2 <= pause_s1;
    end

    /* ---- A LOAD HANDS THE MACHINE BACK (2026-10-17, the panel's `loads
     * don't unpause`) --------------------------------------------------
     *
     * The override further down releases the pause for the length of a state
     * transfer, and it used to let the level re-engage the moment the transfer
     * ended. That is right for the user's own Select toggle, which they can
     * release, and it is wrong for a LOAD, because a load IS the resume action:
     * the user asked for the machine to be put back where it was, and the state
     * they chose is the answer, not a reason to hold it.
     *
     * What made that a panel failure rather than a nicety is that the level used
     * to be the OS's too: an unpause was the OS closing its menu, and when that
     * notify did not reach this core on the load path the pause was up for ever
     * -- the machine stopped, the loaded frame on screen, and nothing the user
     * could press got it back. That is why core_top no longer lets the menu
     * notify stop the machine at all (spec 21.11). What is left is the button,
     * so a LOAD mutes the pause until the button has been seen RELEASED since
     * the load: a level that was already up when the load finished cannot hold
     * the machine again, and the next press pauses it exactly as before.
     *
     * A save mutes nothing: the user may well be holding pause across it, and
     * the button is doing its job there.
     *
     * WHAT COUNTS AS RELEASED (2026-10-17, the fifth report: the game must stop
     * behind the OS menu, and a load must still never leave it stopped).
     *
     * Two fixes before this one disagreed about WHEN the OS drops its menu
     * notify around a state command -- a single low cycle, then a frame, then a
     * frame plus eight frames of quiet -- and `sim/tb_savestate.cpp` now sweeps
     * that delay (`MC64_SS_MENU_GAP`): EVERY restore later than the window pins
     * all three panel files at their own PCs (40 frames does, and 120). No
     * window can settle it, because from inside this core "the OS is still in
     * its menu" and "the OS's exit notification was lost" are the same level.
     *
     * So the level is no longer the OS's ALONE: `core_top` gates the notify with
     * `os_menu_en`, a latch a single Select press clears (the escape) and only
     * the notify going LOW re-arms. The level that reaches this module is
     * therefore one the user can always take back -- which is what makes the
     * simple rule below safe rather than lucky.
     *
     * The rule: a load hands the machine back, and a level that was already up
     * when the load finished cannot hold it again until that level has been seen
     * LOW *since the load* -- one frame of it, the unit `pause_hold` is latched
     * in. The counter is CLEARED by the load's own falling edge, so a release
     * that began before the transfer does not count as one that happened after
     * it. Load a state while the machine is paused and it comes back RUNNING;
     * the next press pauses it exactly as before.
     */
    logic        ss_loading_q;   // the previous cycle's "a state is being restored"
    logic        pause_muted;    // a load just ended: the level must drop first
    logic [16:0] pause_low_cnt;  // clocks the level has been low SINCE the load

    // One frame of a level that is DOWN, counted from the load that handed the
    // machine back. There is no second window any more: the level is the user's
    // button (see WHAT COUNTS AS RELEASED), so a release is a release -- there
    // is no command window to debounce and no OS notify that can come back by
    // itself. A very short low is still ignored, because RDY is released on a
    // frame boundary and a release has to be visible to the frame counter.
    localparam logic [16:0] PAUSE_SETTLE = 17'(FRAME_CLOCKS);

    always_ff @(posedge clk) begin
        ss_loading_q <= ss_loading;

        if (rst || (ss_loading_q && !ss_loading) || pause_s2)
            pause_low_cnt <= 17'd0;    /* the load's own edge clears it too: a
                                        * release that began before the transfer
                                        * is not one that happened after it   */
        else if (pause_low_cnt != PAUSE_SETTLE)
            pause_low_cnt <= pause_low_cnt + 17'd1;

        if (rst)
            pause_muted <= 1'b0;
        else if (ss_loading_q && !ss_loading)
            pause_muted <= 1'b1;   /* the load is over: hand the machine back */
        else if (pause_low_cnt == PAUSE_SETTLE)
            pause_muted <= 1'b0;   /* ... and the button has been released
                                    * since the load: the user's level, and
                                    * the only one that can pause again     */
    end

    cpu_6502 cpu (
        .clk      (clk),
        .reset    (rst),
        .AB       (AB),
        .DI       (DI),
        .DO       (DO),
        .WE       (WE),
        .IRQ      (irq_req),
        .NMI      (1'b0),
        .RDY      (cpu_rdy),
        .ss_we    (ss_cpu_we),
        .ss_PC    (ss_cpu_word[15:0]),
        .ss_A     (ss_cpu_word[23:16]),
        .ss_X     (ss_cpu_word[31:24]),
        .ss_Y     (ss_cpu_word[39:32]),
        .ss_S     (ss_cpu_word[47:40]),
        .ss_P     (ss_cpu_word[55:48]),
        .dbg_PC   (cpu_dbg_pc),
        .dbg_A    (cpu_dbg_a),
        .dbg_X    (cpu_dbg_x),
        .dbg_Y    (cpu_dbg_y),
        .dbg_S    (cpu_dbg_s),
        .dbg_P    (cpu_P),
        .dbg_state(cpu_state),
        .dbg_instr_start(cpu_instr_start),
        .dbg_int_entry  (cpu_int_entry),
        .dbg_int_done   (cpu_int_done)
    );
    assign dbg_pc = cpu_dbg_pc;

`ifdef MC64_SIM
    assign dbg_a       = cpu_dbg_a;
    assign dbg_x       = cpu_dbg_x;
    assign dbg_y       = cpu_dbg_y;
    assign dbg_s       = cpu_dbg_s;
    assign dbg_status  = cpu_P;
    /* The address still names the framebuffer slot this pixel is at -- the same
     * one the store was filled from -- but the BYTE has to be the one the colour
     * lookup consumed, which after the first capture is the store's and not the
     * one on the RAM bus (rtl/mc64_video.sv's THE FRAME STORE). tb_mc64 records
     * both for compare.py's picture check. */
    assign dbg_vid_addr  = vid_ram_addr;
    assign dbg_vid_rdata = video_pic_byte;
    // the video's latched banks are real nets now (the state engine's deferred
    // window reads them); the harness keeps its own names for them
    assign dbg_vram_bank_q = video_cur_vram_bank;
    assign dbg_pal_bank_q  = video_cur_pal_bank;
    assign dbg_ld_valid   = video_ld_valid;
    assign dbg_ld_idx     = video_ld_idx;
    assign dbg_ss_freeze   = ss_freeze;
    assign dbg_pause_hold  = pause_hold;
    assign dbg_pause_freeze = pause_freeze;
    assign dbg_pause_muted  = pause_muted;
    assign dbg_ss_loading   = ss_loading;
    assign dbg_ss_flushing  = ss_flushing;
    assign dbg_vid_blank    = video_blank_slot;
    assign dbg_pause_low_sat = (pause_low_cnt == PAUSE_SETTLE);
    assign dbg_frame_cnt   = frame_cnt;
    assign dbg_ram_mirrors = {dmc_bank, irqvec_hi, irqvec_lo, pal_bank, vram_bank};
`endif

    // Mirror of arlet's own internal `res` flag: set while reset is asserted,
    // cleared on the first instruction start, i.e. once the reset sequence is
    // over. Driven by a semantic tap from the CPU rather than a microcode state
    // number, so it cannot silently rot if the state encoding changes.
    logic res_seq;

    always_ff @(posedge clk) begin
        if (ss_load_done)
            res_seq <= ss_aux_word[17];   // the reset sequence this state was in
        else if (rst)
            res_seq <= 1'b1;
        else if (cpu_instr_start)
            res_seq <= 1'b0;
    end

    /* ------------------------------------------------------------------ *
     * CPU-side read pipeline
     *
     * arlet's core "assumes a synchronous memory. This means that valid data
     * (DI) is expected on the cycle AFTER valid address" (upstream README).
     *
     * The RAM delivers that directly, and the mechanism is in mc64_ram.sv: port
     * A is clocked by clk_ram (the same 6.4 MHz, half a cycle later), so the
     * array's own address register latches AB once AB has settled, and its
     * registered read output presents mem[AB] from that edge onward -- i.e. for
     * the whole second half of the cycle, for the address the CPU put out at the
     * start of it.
     *
     * DI then registers it at the cycle boundary, which is what makes the byte
     * stable for the WHOLE of the next cycle. That stability is not a luxury:
     * AB is combinational and is itself built from DI (the ABMUX expressions in
     * cpu_6502.sv), so a DI that drifted part-way through a cycle would move AB
     * with it, and the core would sample the wrong byte at the boundary and form
     * addresses from it. (That is the bug the old master-clock design fought:
     * its comment records the JSR operand pair being swapped this way.)
     *
     * So the substitutions below are indexed by AB -- the live address of the
     * cycle being closed, whose byte is the one about to be handed over -- NOT
     * by a delayed copy. irq_req and res_seq are likewise sampled pre-edge, i.e.
     * still holding the values that qualified this address: irq_req is the
     * interesting case, since it is dropped by cpu_int_done at the very edge
     * that captures the vector's high byte, so the substitution covers the
     * $FFFF read and the request is only retired once the byte is in DI.
     * ------------------------------------------------------------------ */

    // One write strobe per 6502 cycle. WE is valid for the whole cycle, and the
    // RAM's port A latches it on the 180-degree clock (mid-cycle), so no phase
    // mask is needed -- and none is possible: there is only one clock.
    //
    // `~reset`: `vid_por` free-runs from the video's power-on reset (see above),
    // so without this a WE left asserted by the reset sequence could strobe a
    // write from an address the CPU does not own.
    wire cpu_write = WE & ~rst;

    // ---- shadows of the bytes the CPU owns that other blocks need ----
    // The only way any of these change is a CPU write, so mirroring them is exact.

    // The initial values are the hardware's, not a simulation convenience: a
    // register powers up at 0 and the RAM these five mirror powers up at 0 too
    // (and the RAM clear writes zeros through them -- see below). Without them
    // the simulator holds X until the first write, and an X on the video's
    // palette-bank select reads as palette[0]'s colour, i.e. black, which is a
    // picture fault that exists in no fitted bitstream.
    logic [ 7:0] irqvec_lo = 8'h0;       // $010E, the vblank handler's low byte
    logic [ 7:0] irqvec_hi = 8'h0;       // $010F, its high byte
    logic [ 7:0] vram_bank = 8'h0;       // $0100
    logic [ 7:0] pal_bank  = 8'h0;       // $0101 -- bank 0 is the DEFAULT palette
    logic [ 7:0] dmc_bank  = 8'h0;       // $0104, the DMC's sample bank (see the APU)

    always_ff @(posedge clk) begin
        /* THE CARTRIDGE PORT IS THE OTHER WAY THE RAM IS WRITTEN -- AND IT IS
         * THE ONE THE CLEAR USES (2026-10-16). These five are shadows of RAM
         * bytes, but they are NOT RAM: they were written only by a CPU store
         * snoop and by a restored state, and `core_top`'s RAM clear writes its
         * zeros through the CARTRIDGE LOADER's port (rom_we/rom_addr/rom_data,
         * see its RAM-clear note), which the snoop below never sees. So the
         * clear emptied the RAM byte and left the register that mirrors it, and
         * a cartridge that never writes `$0101` -- `cube` sets `$0100` and
         * relies on `$0101` being 0, which is what the emulator's zeroed 64 KB
         * gives it (`sim/golden/golden.c:84`) -- could be drawn through the
         * bank the PREVIOUS cartridge had left in it. Spec 17.4 removed the
         * inherited BYTES; this removes the inherited REGISTERS, by mirroring
         * the port that removed them.
         *
         * Why the loader port and not the machine's reset: a Reset Core pulse
         * with no re-stream deliberately KEEPS the RAM (`core_top` says so),
         * and this machine's registers have to keep it with it. The port is
         * exactly the event that zeroes those bytes: the sweep drives it with
         * rom_data = 0 for every address outside the image, so a mirror that
         * follows it lands on the same zero at the same instant -- and the
         * LOADER, which shares the port, only ever writes the image at
         * $0200 and above, so it can never touch these five.
         *
         * NOT PROVEN ON HARDWARE. The harnesses cannot see these registers (the
         * boot harness's reload ordering compares RAM bytes, which the clear
         * already fixed, and a cartridge's own store to $0101 turned out not to
         * move the video's palette bank in simulation -- see the note in
         * docs/minicube64-fpga-spec.md 18.4). The panel is the test: load
         * `palettecycle` and then `cube`, which must come up in the DEFAULT
         * palette (a fresh core load is the control). */
        /* THE SWEEP CLEARS THEM BY NAME, not only through the port. It comes
         * FIRST, not after the address-keyed mirror: the sweep drives rom_we on
         * almost every cycle of its walk (that is what a sweep is), so a second
         * branch would be reached only in the cycles it is not writing -- i.e.
         * never. An address-keyed mirror is also one compare away from missing
         * the write it depends on, and this is that guarantee made unconditional.
         * The sweep always covers $0000-$01FF, so these five end at the
         * emulator's zeros either way -- here they are simply zero for its whole
         * duration. Nothing else writes them in that window: the CPU is held by
         * the reset the sweep runs under, and the loader is idle (the sweep only
         * runs once the bridge has gone quiet). */
        if (rom_clear) begin
            vram_bank  <= 8'h0;
            pal_bank   <= 8'h0;
            dmc_bank   <= 8'h0;
            irqvec_lo  <= 8'h0;
            irqvec_hi  <= 8'h0;
        end
        else if (rom_we) begin
            if (rom_addr == 16'h0100) vram_bank <= rom_data;
            if (rom_addr == 16'h0101) pal_bank  <= rom_data;
            if (rom_addr == 16'h0104) dmc_bank  <= rom_data;
            if (rom_addr == 16'h010E) irqvec_lo <= rom_data;
            if (rom_addr == 16'h010F) irqvec_hi <= rom_data;
        end
        // A restored state carries these five bytes as they were, rather than
        // re-reading them out of the RAM it has just written: they are shadows
        // of addresses the CPU owns, and the file has them at the same moment
        // the RAM does.
        else if (ss_load_done) begin
            irqvec_lo <= ss_aux_word[50:43];
            irqvec_hi <= ss_aux_word[58:51];
            vram_bank <= ss_aux_word[26:19];
            pal_bank  <= ss_aux_word[34:27];
            dmc_bank  <= ss_aux_word[42:35];
        end
        else if (cpu_write) begin
            if (AB == 16'h010E) irqvec_lo <= DO;
            if (AB == 16'h010F) irqvec_hi <= DO;
            if (AB == 16'h0100) vram_bank  <= DO;
            if (AB == 16'h0101) pal_bank   <= DO;
            if (AB == 16'h0104) dmc_bank   <= DO;
        end
    end

    /* ------------------------------------------------------------------ *
     * The APU's window, decoded where the RAM needs it
     *
     * machine.c diverts every read and write in $0110..$0130 into the APU and
     * returns before memory is touched, so those writes are the APU's and NOT
     * the RAM's: here the array's write port is gated off for the window, which
     * is the same thing (the byte the cartridge wrote there is in the register,
     * and RAM keeps the zeroes the clear left -- spec 17.4). Without the gate
     * the register byte would also land in RAM and the golden RAM comparison in
     * sim/ would see it the first time a cartridge writes an audio register.
     *
     * The window's map is the NES's own, relocated by $2F0: $0110 + n == $4000
     * + n, so the 5-bit APU address is simply the low five bits of AB minus 16.
     * ------------------------------------------------------------------ */
    wire [15:0] apu_addr_sel = AB;
    wire        apu_cs       = (apu_addr_sel >= 16'h0110) && (apu_addr_sel <= 16'h0130);
    wire [ 4:0] apu_reg      = apu_addr_sel[4:0] - 5'd16;   // $0110 -> index 0

    // ---- port A: the CPU, unless a save state has taken it --------------
    // The engine raises one strobe per byte it reads or writes and the machine
    // is frozen for the whole transfer, so ownership is just "the engine has a
    // strobe up". The CPU's own address and strobe cannot change while it is
    // stopped (RDY is low), and its write is blocked either way.
    wire [15:0] ram_a_addr  = ss_ram_own ? ss_ram_addr  : AB;
    wire        ram_a_we    = ss_ram_own ? ss_ram_wren  : (cpu_write && !apu_cs);
    // The engine's writes are whole words; the CPU's are one byte, at the lane
    // its own address selects. The byte's bits are replicated into every lane so
    // that the one the enable picks up already has them (the enables are what
    // decide, not the data).
    wire [31:0] ram_a_wdata = ss_ram_own ? ss_ram_wdata : {4{DO}};
    wire [ 3:0] ram_a_be    = ss_ram_own ? 4'b1111 : (4'b0001 << AB[1:0]);


    /* ------------------------------------------------------------------ *
     * Main memory
     * ------------------------------------------------------------------ */
    wire [ 7:0] ram_rdata_a;
    wire [31:0] ram_rdata_a_word;   // ... the same read, whole, for the engine
    wire [ 7:0] ram_rdata_b;
    wire [15:0] vid_ram_addr;
    wire [ 7:0] video_pic_byte;
    wire [15:0] video_cp_addr;
    wire        video_cp_busy;

    // The DMC's sample fetch borrows one port-B cycle from the video. Both
    // signals are driven in the APU section below, where the fetch lives and
    // where its reasoning is; they are declared here because this mux needs
    // them (and because a port-B address put out by an earlier section is what
    // makes the borrow a one-cycle interrupt of the video's stream rather than
    // a change to its own address register).
    wire        dmc_take;
    wire [15:0] dmc_addr;

    // Port B: cartridge loader while reset is held, video scan-out otherwise,
    // and the DMC's byte in the single cycle the fetch asks for.
    /* ... and the video's frame-store capture, which borrows the whole bus for
     * four thousand clocks at the end of the frame, one byte each (see
     * rtl/mc64_video.sv's THE CAPTURE). It sits BELOW `dmc_take` so a sample
     * fetch is never lost; the video re-issues the read it lands on. */
    wire [15:0] ram_b_addr  = rom_we ? rom_addr
                            : dmc_take ? dmc_addr
                            : video_cp_busy ? video_cp_addr
                            : vid_ram_addr;
    wire        ram_b_we    = rom_we;
    // Port B writes ONE BYTE, into one of mc64_ram's four byte-wide banks -- it
    // picks the bank with the low address bits itself (see that file's header:
    // the banks are four separate arrays because a byte-enabled 32-bit true dual
    // port is a shape Quartus will not infer). Only the loader writes port B.
    wire [ 7:0] ram_b_wdata = rom_data;

    mc64_ram ram (
        .clk_a   (clk_ram),   // port A: mid-cycle latch, registered read
        .clk_b   (clk),       // port B: video scan-out, or the cartridge loader
        .a_addr  (ram_a_addr),
        .a_we    (ram_a_we),
        .a_be    (ram_a_be),
        .a_wdata (ram_a_wdata),
        .a_rdata (ram_rdata_a),
        .a_rdata_word (ram_rdata_a_word),
        .b_addr  (ram_b_addr),
        .b_we    (ram_b_we),
        .b_wdata (ram_b_wdata),
        .b_rdata (ram_rdata_b)
`ifdef MC64_SIM
        ,
        .dbg_addr (dbg_ram_addr),
        .dbg_data (dbg_ram_data)
`endif
    );

    /* ------------------------------------------------------------------ *
     * APU ($0110-$0125 == NES $4000-$4015)
     *
     * THE MACHINE'S AUDIO IS THE NES APU. machine.c's makefile builds with
     * -DNES_APU, so the emulator runs Conte's Nofrendo APU rather than the wsg.c
     * fallback, and its registers are the NES's own relocated by $2F0 -- the
     * table is in memory_map.md and docs/minicube64-fpga-spec.md s.6: $0119,
     * $011D and $0124 ($4009, $400D, $4014) do not exist.
     *
     * rtl/apu.sv is agg23/openfpga-NES's rtl/upstream/apu.sv -- Kitrinx's
     * cycle-based rewrite of the Ludde/sorgelig APU -- and it is here for two
     * reasons: it is the design the Pocket's own NES core ships, and it is
     * built out of the NES's own arithmetic rather than a sample-rate model.
     *
     * THE CLOCK ENABLE, and why there is one. This machine runs ONE 6.4 MHz
     * clock and the 6502 advances every cycle, but the APU's pitch, envelopes,
     * the frame counter's LFSR, the noise LFSR and the DMC's rate table are all
     * scaled in 2A03 CPU cycles: 1.7897727 MHz (machine.c's
     * apu_create(0, 44100, 60, 16) -> APU_BASEFREQ in apu/nes_apu.h, the NTSC
     * rate). The two are not commensurate -- 6.4/1.7897727 = 3.5759, not an
     * integer -- and no output of this VCO is 1.7897727 either (633.6 MHz / 354
     * = 1.7898, 32 ppm off, and that would cost a third clock domain for the
     * register writes). So the APU is clocked by clk with `ce` generated by a
     * fractional accumulator: STEP/2^24 per cycle, i.e. 1789772.797 Hz on
     * average -- 0.04 ppm from the emulator's base frequency -- with no edge
     * ever more than one 6.4 MHz cycle (156 ns) from where the ideal one falls.
     * The jitter is 1/7 of a square channel's tick at 894 kHz, entirely above
     * the audio band, and the error does not accumulate: it is a bounded phase
     * error, not the drift a fixed ratio would give (6.4/4 = 1.6 MHz would be
     * 10.6% flat, an audible wrong pitch on every note).
     *
     * THE PHASES. Kitrinx's APU derives its internal tick structure from the two
     * signals a 2A03's APU sees from the CPU: PHI2 (the bus access's own phase)
     * and get_or_put (which half of the pair of ticks inside a CPU cycle this
     * is). It builds
     *     aclk1         = ce & get_or_put       -- pulse/noise/DMC timers
     *     aclk1_delayed = ce & ~get_or_put      -- square timers, length counters
     *     aclk2         = phi2_ce & ~get_or_put -- the frame counter's odd cycles
     * and toggling get_or_put once per ce, with PHI2 = ce, reproduces the NES
     * rates exactly:
     *     square/noise/DMC timers at ce/2 = 894.886 kHz (the CPU/2 they are on
     *     the NES), the triangle and DMC timers at ce = 1.7897727 MHz, and one
     *     phi2_ce pulse per ce (ce pulses are three to four cycles apart, so no
     *     two can merge -- which is what makes PHI2 = ce a legal PHI2 at all).
     *
     * WRITES ARE LATCHED ON THE CPU'S OWN EDGE. Every channel latch inside the
     * APU is `if (write)` on clk -- not gated by ce -- which is exactly right
     * here: this machine's write strobe is ONE CPU cycle wide, so the edge that
     * latches is the edge on which DO is the byte being written, and two
     * consecutive audio writes (three or four cycles apart, i.e. inside one
     * APU cycle) can never merge into one. MiSTer's write is a whole 21.477 MHz
     * CPU cycle wide, so it latches the same value on several edges; same
     * result, different shape.
     *
     * $0126..$0130 ($4016..$4020) fall inside machine.c's diverted window and
     * decode into the APU's control block too. Only $4017 is meaningful there
     * and it is gated by write_ce = write & phi2_ce, so a one-cycle write only
     * reaches it when it happens to land on a ce -- i.e. in practice it does
     * not, and no cartridge written for the emulator could use it anyway (the
     * emulator's apu_write has no case for $4017 at all). The frame counter
     * therefore runs in its reset default, the 4-step sequence, which is
     * exactly what the envelopes and the length counters need; its IRQ output
     * is unconnected and the emulator's APU raises no frame interrupt either.
     *
     * READS. A read of $0125 returns the NES status byte (DMC interrupt, frame
     * interrupt, and the length-count status of each channel), which is what a
     * cartridge polls to know a DPCM sample has finished; the emulator builds
     * the same byte for the length-count and DMC-interrupt bits (apu_read() in
     * apu/nes_apu.c) and always leaves the frame-interrupt bit 0 -- the one bit
     * that differs, in this core's favour. Every other address in the window
     * reads back `address >> 8` = $01, which is the emulator's model of the
     * open bus on the APU's side of the chip.
     * ------------------------------------------------------------------ */
    wire [ 7:0] apu_dout;   // the APU's status byte, driven by the instance below
    wire [ 7:0] apu_status = apu_dout;
    wire [ 7:0] apu_rdata  = (apu_reg == 5'h15) ? apu_status : 8'h01;

    /* ------------------------------------------------------------------ *
     * Data bus read substitution (combinational), then the one-6502-cycle
     * read register described above.
     * ------------------------------------------------------------------ */
    logic [7:0] di_next;

    always_comb begin
        unique case (AB)
            // No reset vector in this machine: machine.c does `pc = 0x200`
            // after reset6502(). Only aliased during the reset sequence, so
            // ordinary code reading $FFFC/$FFFD still sees RAM.
            16'hFFFC: di_next = res_seq ? 8'h00 : ram_rdata_a;
            16'hFFFD: di_next = res_seq ? 8'h02 : ram_rdata_a;

            // The vblank IRQ vector lives at $010E, not $FFFE. Only aliased
            // while we are actually raising an IRQ, so a BRK instruction still
            // vectors through the real $FFFE in RAM.
            16'hFFFE: di_next = irq_req ? irqvec_lo : ram_rdata_a;
            16'hFFFF: di_next = irq_req ? irqvec_hi : ram_rdata_a;

            // Controllers are read live; see the divergence note in the header.
            16'h0102: di_next = pad1;
            16'h0103: di_next = pad2;

            default:  di_next = apu_cs ? apu_rdata : ram_rdata_a;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst)
            DI <= 8'h00;
        else if (ss_cpu_we)
            // A restored state carries this byte: it is the opcode the stopped
            // core was about to dispatch. Requiring it is what makes the stop
            // point self-contained -- see the save-state section below.
            DI <= ss_cpu_word[63:56];
        else if (ss_freeze | ss_stop | pause_stop | pause_freeze)
            /* HOLD IT, AND FOR THE SAME REASON IN ALL FOUR CASES: at a DECODE
             * cycle DI holds the PREFETCHED OPCODE OF THE NEXT INSTRUCTION --
             * the previous instruction's last cycle put that address on AB and
             * left the byte on the bus -- while AB itself already points at
             * this instruction's first operand. A DI that keeps following
             * `di_next` while the machine is stopped therefore replaces the
             * prefetched opcode with an operand byte, and the instruction after
             * the one being stopped for fetches garbage the moment the machine
             * runs again.
             *
             * That is not hypothetical: the pause was first written to stop at
             * this boundary WITHOUT holding DI, and tools/boot_sim.sh measured
             * the consequence exactly -- the machine stopped, the picture after
             * the resume was 258 of 1024 cartridge bytes wrong, and the
             * cartridge never stored again ($0A00 stayed at its held value).
             * The save-state path never saw it because a load resets the CPU and
             * restores PC and DI afterwards; a pause has no restore, so what it
             * freezes is what it resumes with. */
            DI <= DI;
        else
            DI <= di_next;   // one 6502 cycle of latency, i.e. every cycle now
    end

    /* ------------------------------------------------------------------ *
     * The CPU's frame
     *
     * machine.c gives the CPU exec6502(6400000/60) = 106,666 cycles per frame,
     * and the vblank interrupt is what tells the program that frame is over. So
     * the frame that matters to the machine is a count of 106,666 machine
     * clocks -- exactly the emulator's -- and it is kept here rather than being
     * read off the video.
     *
     * It cannot be read off the video. The raster is a whole number of pixels
     * of the 1.32 MHz pixel clock, and the CPU's frame is 106,666 machine
     * clocks, which is 106,666.67 of those pixels -- not a whole number.
     * Reading the frame off the raster would either move the CPU budget off the
     * emulator's number (and the golden comparison in sim/ off exactness with
     * it) or move the machine's own rate off 6.4 MHz. So the raster free-runs at
     * 22,000 pixels: 60.000000 Hz, 106,666.67 clocks, i.e. 0.67 of a clock
     * LONGER than the CPU's frame -- 6.3 ppm, and by construction.
     *
     * 22,000 is 64 picture lines plus 186 invisible ones, and it is exactly
     * 88 x 250, so EVERY line of the frame is the same length -- the slack line
     * the earlier 67-cycle layout needed is gone (mc64_video.sv's WHY 22,000).
     * The 318-uniform-line geometry before that -- 21,306 px = 60.077 Hz, 0.13%
     * fast -- was the nearest whole number of LINES, and that is what the panel
     * reported on its last locking build: a frame the scaler drops every ~13 s.
     *
     * THE PHASE, THOUGH, IS LOCKED, and this counter is what it locks to. The
     * 0.67-clocks-per-frame difference used to be left to accumulate, and what
     * it accumulated into is the d-pad-up bug (spec 17.6): the raster began its
     * picture sweep at the same moment the CPU began its frame -- i.e. exactly
     * while the cartridge was redrawing the framebuffer -- so the rows the sweep
     * read first were rows the cartridge had erased and not yet rewritten. They
     * are in RAM (the emulator and the RTL agree about them, byte for byte) and
     * missing from the picture. The sweep is now anchored to the OTHER end of
     * this frame: the video is handed `frame_cnt` and its picture ENDS on the
     * frame boundary, so what the raster reads is the frame the cartridge has
     * just finished drawing. mc64_video.sv's THE RASTER'S PHASE has the
     * mechanism; all this counter owes it is its value and its length, and the
     * length goes in as a PARAMETER OVERRIDE below so the two modules cannot
     * disagree about it.
     *
     * The counter starts when `reset` drops, which is when the machine is
     * released with the cartridge loaded -- the same point machine.c starts
     * counting from. During the hold it is pinned at zero, so the first frame is
     * a whole frame and the golden comparison sees the same frame boundaries the
     * emulator does from the very first one. `~reset` is also what tells the
     * video there is a CPU frame to lock to at all.
     * ------------------------------------------------------------------ */
    localparam int FRAME_CLOCKS = 106666;
    logic [16:0] frame_cnt;
    logic        vblank_tick;

    always_ff @(posedge clk) begin
        if (ss_load_done) begin
            // Where inside its frame the machine was. Without this a state
            // taken mid-frame would come back at cycle 0 and the cartridge
            // would see a whole extra frame's worth of budget before its next
            // vblank -- a load that changes the machine's timing, which is
            // the one thing a state must not do.
            frame_cnt   <= ss_aux_word[16:0];
            vblank_tick <= 1'b0;
        end
        else if (rst) begin
            frame_cnt   <= '0;
            vblank_tick <= 1'b0;
        end
        else if (ss_freeze) begin
            // Where the machine is inside its frame is part of the state, and
            // a load puts the counter back where the save left it (see the
            // save-state section at the end of this file).
            vblank_tick <= 1'b0;
        end
        else begin
            vblank_tick <= 1'b0;
            if (frame_cnt == (FRAME_CLOCKS - 1)) begin
                frame_cnt   <= '0;
                vblank_tick <= 1'b1;
            end
            else
                frame_cnt <= frame_cnt + 17'd1;
        end
    end

    /* ---- the pause, engaged on the frame boundary (see the port) ------
     *
     * ONE DECISION PER BOUNDARY, and it is the level that decides: engage while
     * the button's level is up, release while it is down. Both ends land on the
     * frame boundary, so the machine is stopped for a WHOLE NUMBER OF FRAMES.
     *
     * That is not tidiness. The counter above keeps running through a pause (the
     * panel needs its frames), so a release anywhere inside a frame would hand
     * the cartridge a partial frame -- a different cycle budget for that one
     * frame than the emulator's, i.e. a machine that resumes at a phase it was
     * never at. Releasing on the boundary costs up to one frame of latency
     * between the button and the resume, which is under 17 ms, and it keeps the
     * machine's frame phase exact: what was true of every frame before the pause
     * is true of every frame after it.
     *
     * THE RASTER IS NOT PHASE-LOCKED TO A PAUSED MACHINE (2026-10-16, the
     * panel's `unpauses to a black screen`). `pause_hold` is fed into the
     * video's `sync_en`, so for as long as the user's pause lasts the raster
     * free-runs on its own 250-uniform-line geometry instead of aligning its
     * picture to this counter -- see the note at the video instantiation, and
     * mc64_video.sv's THE FROZEN CASE, which is the same fault one write-up
     * earlier: a raster that is waiting for a frame window the machine cannot
     * supply produces no picture at all, and on the panel that is a BLACK
     * screen with the OS menu still drawn over it. The freeze itself does not
     * change -- the counter still runs, the framebuffer is static, and the
     * picture the panel holds is the completed frame it did before (the raster
     * reads the frame the cartridge FINISHED, and a pause engages at the
     * boundary where it finished it). What changes is that a pause can no
     * longer be waiting on a phase lock when it ends: the release re-arms the
     * lock through mc64_video's own `sync_en && !sync_en_r` edge, which is
     * also what a save-state transfer does.
     *
     * The menu/state override is the exception and stays immediate: a save-state
     * transfer needs the CPU running on to its next instruction boundary, and
     * the engine must not wait a frame for it. */
    always_ff @(posedge clk) begin
        if (rst)
            pause_hold <= 1'b0;
        else if (ss_sleep)
            pause_hold <= 1'b0;   /* released: the state engine needs the CPU */
        else if (frame_cnt == (FRAME_CLOCKS - 1))
            pause_hold <= pause_s2 & ~pause_muted;   /* a whole frame in, a whole
                                                      * frame out -- and never
                                                      * the frame a load just
                                                      * handed back (above) */
    end

    /* ---- the hold itself, on the next instruction boundary --------------
     *
     * `pause_stop` is the cycle RDY drops in (combinational, like the save
     * state's `ss_stop`, because a registered stop would land one cycle later,
     * after the dispatch), and `pause_freeze` is the hold that carries it for
     * the rest of the pause. Both are released the moment the request is gone --
     * there is nothing to unwind: the CPU is parked on a DECODE cycle with its
     * opcode in the instruction register, so the first cycle with RDY high
     * dispatches that opcode exactly as it would have without the pause.
     *
     * `~ss_sleep` and `~rst` keep the pause out of the state engine's way and
     * out of the machine's own reset, the same two exclusions the state stop
     * takes and for the same reasons.
     */
    always_ff @(posedge clk) begin
        if (rst || ss_sleep || !pause_hold)
            pause_freeze <= 1'b0;
        else if (pause_stop)
            pause_freeze <= 1'b1;
    end

    /* ------------------------------------------------------------------ *
     * Video
     * ------------------------------------------------------------------ */
    logic video_frame_end;
    // What the frame in progress is drawn with, and where the raster's blanking
    // is -- both for the state engine's deferred window (see that module).
    logic [7:0] video_cur_vram_bank, video_cur_pal_bank;
    logic       video_blank_slot;
    logic       video_ld_valid;
    logic [9:0] video_ld_idx;

    mc64_video #(
        .FRAME_CLOCKS   (FRAME_CLOCKS)   // single source of truth: see above
    ) video (
        .clk            (clk),
        .ce             (ce_pix),
        .reset          (vid_reset),   // NOT `reset`: see the POR above
        .frame_cnt      (frame_cnt),   // the CPU's frame, for the phase lock
        .sync_en        (~reset & ~ss_freeze & ~pause_freeze),   // a CPU frame to lock to
        /* THE PICTURE IS NOT BLANKED FOR A LOAD ANY MORE (2026-10-18). The
         * engine holds back the only bytes a picture is drawn from -- the 4 KB
         * bank and 768-byte palette bank this frame uses -- and writes them in
         * the raster's blanking slot, so a load shows nothing at all: the frame
         * already on the screen becomes the state's own frame a frame later.
         * `flushing` is that window being written, and it is only ever a
         * fallback hold now (a host still streaming, or an abandoned transfer).
         * A save never asserts it: it does not touch the RAM, so its frozen
         * picture is the truth. */
        .load_blank     (ss_flushing),
        /* The bank latch is held for the length of a transfer: the load is what
         * rewrites $0100/$0101, and the raster must keep drawing the pair the
         * copy was planned around (mc64_video's `bank_hold`). */
        .bank_hold      (ss_loading),
        /* `~pause_hold` is in there for the pause (2026-10-16, the panel's
         * `unpauses to a black screen`). See the note below the video: a
         * machine held by the user has no frame the raster may align to, which
         * is exactly the condition `sync_en` names, and leaving it high was the
         * one way this core could be left showing NOTHING while the OS menu
         * still drew over it. */
        .ram_addr       (vid_ram_addr),
        .ram_rdata      (ram_rdata_b),
        .vram_bank      (vram_bank),
        .pal_bank       (pal_bank),
        /* The DMC's sample fetch takes port B for one cycle at a time; the
         * video's frame store re-issues a read that lands on one (see its port
         * note), so it needs to know. */
        .dmc_take       (dmc_take),
        .pic_byte       (video_pic_byte),
        .cp_addr        (video_cp_addr),
        .cp_busy        (video_cp_busy),
        // the CPU's own store strobe: the frame store's write-through (THE SNOOP
        // in mc64_video) applies every framebuffer store to the back buffer, so
        // what the panel shows is the RAM at the boundary, not at 97,000 clocks.
        // APU writes are excluded (they are not RAM bytes -- see the port-A gate
        // above) and so is the state engine, which owns port A.
        .cpu_we         (cpu_write && !apu_cs && !ss_ram_own),
        .cpu_addr       (AB),
        .cpu_wdata      (DO),
        .video_rgb      (video_rgb),
        .video_de       (video_de),
        .video_hs       (video_hs),
        .video_vs       (video_vs),
        .frame_end      (video_frame_end),  // the raster's own rollover; see below
        .cur_vram_bank  (video_cur_vram_bank),
        .cur_pal_bank   (video_cur_pal_bank),
        .blank_slot     (video_blank_slot),
        .ld_valid_o     (video_ld_valid),
        .ld_idx_o       (video_ld_idx)
    );

    assign dbg_frame_end = vblank_tick;
`ifdef MC64_SIM
    // Simulation-only observation taps. Purely combinational reads of signals
    // that already exist, so they cannot perturb behaviour.
    assign dbg_ce          = 1'b1;   // every clk is one 6502 cycle now
    assign dbg_state       = cpu_state;
    assign dbg_instr_start = cpu_instr_start;
    assign dbg_int_entry   = cpu_int_entry;
    assign dbg_int_done    = cpu_int_done;
`endif

    /* ------------------------------------------------------------------ *
     * Vblank interrupt
     *
     * machine.c, at the end of display_machine():
     *     if ((status & FLAG_INTERRUPT) == 0) irq6502();
     * i.e. the request is NOT latched -- it is sampled once, at the frame
     * boundary, and only taken if I was clear at that instant. We reproduce
     * that: raise on vblank_tick if I is clear, then hold until the CPU accepts,
     * so a later CLI cannot make the still-asserted request fire spuriously
     * mid-frame.
     *
     * "Accepts" means the whole vector has been fetched, not just its low byte.
     * The entry sequence is BRK0..BRK3 followed by the JMP0/JMP1 pair, and the
     * vector is read at $FFFE during BRK3 and at $FFFF during JMP0. Holding the
     * request across BOTH reads is what keeps the $FFFF read substituted from
     * $010F too; dropping it at BRK3 left the high byte coming from plain RAM
     * and vectored the CPU to $00xx. cpu_int_done reports the end of that second
     * read, not the end of BRK3.
     * ------------------------------------------------------------------ */
    always_ff @(posedge clk) begin
        if (ss_load_done)
            // A vblank the machine had raised and not yet taken.
            irq_req <= ss_aux_word[18];
        else if (rst)
            irq_req <= 1'b0;
        else begin
            if (vblank_tick && !cpu_P[2])
                irq_req <= 1'b1;

            if (irq_req && cpu_int_done)
                irq_req <= 1'b0;
        end
    end

    /* ------------------------------------------------------------------ *
     * The APU itself, and its sample fetch
     * ------------------------------------------------------------------ */

    // The APU's clock enable. 2^24 * 1.7897727/6.4, rounded: 0.04 ppm. The
    // accumulator is 24 bits, so the carry out of the 25-bit sum is the enable:
    // the step must be scaled by 2^24 and not by anything else (with 2^25 the
    // APU would run at exactly half speed and every pitch would be an octave
    // flat -- which is what the audio harness measures).
    localparam [23:0] APU_CE_STEP = 24'd4691782;

    logic [23:0] apu_acc;
    logic        apu_ce;
    logic        apu_get;      // get_or_put

    always_ff @(posedge clk) begin
        if (rst) begin
            apu_acc <= 25'd0;
            apu_ce  <= 1'b0;
            apu_get <= 1'b0;
        end
        else if (ss_freeze | pause_freeze) begin
            // The 2A03's enable stops with the machine. A state's APU registers
            // are copied out (and written back) one word at a time over the
            // register bus, and an APU still ticking through that window would
            // leave the file holding two different moments.
            //
            // A PAUSE stops it for the same reason a state does: the user asked
            // for the machine's execution to stop, and the APU is part of the
            // machine. Everything it holds -- envelope levels, length counters,
            // the phase of the tone generators -- resumes on the cycle it
            // stopped, which is the property that makes a pause a pause rather
            // than a cut.
            apu_ce <= 1'b0;
        end
        else begin
            {apu_ce, apu_acc} <= apu_acc + APU_CE_STEP;
            if (apu_ce)
                apu_get <= ~apu_get;
        end
    end

    /* ---- the DMC's sample fetch -------------------------------------- *
     * Minicube64's one real divergence from the NES's APU, stated outright in
     * the emulator (apu/nes_apu.c, and spec s.6):
     *
     *     address &= 0xfff;
     *     address |= read6502(IO_AUDIO) << 8;    // IO_AUDIO == $0104
     *
     * The DMC's samples come from the 4 KB block the cartridge selects at
     * $0104 ("DPCM sample block"), not from the NES's hardwired $C000. The
     * APU's own DmaAddr is a NES address -- it is built as $C000 + A*64 + n --
     * so the fetch address is the emulator's own arithmetic over it:
     *
     *     address &= 0xfff;                      // the offset in the block
     *     address |= read6502(IO_AUDIO) << 8;    // the block sits at bank*$100
     *
     * Note OR of the bank at bit 8 and not a concatenation (a bank byte
     * concatenated above twelve bits would put bank 3's samples at $3000, not
     * $0300: the emulator's `|=` ORs into the top byte, it does not add a
     * fourth nibble).
     *
     * dmc_bank is the shadow of $0104, and it is exact: the loader's image
     * starts at $0200, so $0104 is only ever written by the CPU (and by the
     * clear that zeroes it -- spec 17.4 -- which the register's power-up zero
     * matches), and the emulator's read6502() sees the same byte.
     *
     * There is no DMA and no CPU stall on either side: the emulator fetches the
     * byte with a plain read6502(), and this core reads it out of the RAM's
     * port B in one spare cycle. Port B is the video's, but the video consumes
     * its byte on a `ce_pix` cycle, and takes it from the array's address
     * register as that stood on the cycle BEFORE (mc64_ram's port B read is
     * registered: one clock of latency). So every cycle except "the cycle
     * before a ce_pix" is free -- and the cycle immediately AFTER a ce_pix is
     * free by construction: ce_pix is one cycle in five (1.32 MHz against
     * 6.4 MHz, four to five cycles apart, never two), so it can never be the
     * one before the next. dmc_take is that cycle.
     *
     * The byte is then held until the APU has an aclk1_delayed cycle to take it
     * on -- DmaAck is sampled inside `if (aclk1_delayed)` in the DMC -- so there
     * is exactly one ack per fetch, the request line is never asked to pulse,
     * and the fetch cannot be re-issued while one is in flight. The wait costs
     * nothing: the DMC needs the next byte before its current one finishes
     * shifting out, which at the fastest rate (54 CPU cycles per bit) is 30 us
     * away, against the two APU cycles this can add.
     * -------------------------------------------------------------------- */
    logic        ce_pix_d;
    logic        dmc_pend;     // a fetch is owed, waiting for a free port-B cycle
    logic        dmc_taken;    // ... the fetch happened last cycle
    logic        dmc_ackpend;  // the byte is here and is waiting for a put cycle
    logic [11:0] dmc_off;
    logic [ 7:0] dmc_byte;

    assign dmc_take = dmc_pend && ce_pix_d && !rom_we;
    assign dmc_addr = (16'(dmc_bank) << 8) | {4'b0, dmc_off};

    wire        apu_dma_req;
    wire [15:0] apu_dma_addr;
    wire [15:0] apu_sample;
    wire [63:0] apu_ss_dout;

    // One aclk1_delayed cycle, and only one: aclk1_delayed = ce & ~get_or_put,
    // and dmc_ackpend drops on the same edge.
    wire apu_dma_ack = dmc_ackpend && apu_ce && !apu_get;

    // A fetch is in flight from the cycle it is issued (dmc_pend) through the
    // cycle its byte is taken (dmc_taken) to the cycle the APU accepts it
    // (dmc_ackpend). All three are needed: after the take, dmc_pend and
    // dmc_ackpend are BOTH zero for one cycle -- dmc_pend has just dropped and
    // dmc_ackpend does not rise until dmc_taken -- and the APU's request line is
    // still high in that cycle, because the byte it asked for has not arrived.
    // Without dmc_taken in this expression the same byte is fetched twice (and
    // the second ack advances the DMC's own address, so the sample plays every
    // other byte): see the fetch checks in sim/tb_apu.cpp.
    wire dmc_busy = dmc_pend | dmc_taken | dmc_ackpend;

    always_ff @(posedge clk) begin
        ce_pix_d  <= ce_pix;
        dmc_taken <= dmc_take;

        if (rst) begin
            dmc_pend    <= 1'b0;
            dmc_ackpend <= 1'b0;
            dmc_taken   <= 1'b0;
        end
        else begin
            if (dmc_take)
                dmc_pend <= 1'b0;

            if (apu_dma_req && !dmc_busy) begin
                dmc_pend <= 1'b1;
                dmc_off  <= apu_dma_addr[11:0];
            end

            if (dmc_taken) begin
                dmc_ackpend <= 1'b1;
                dmc_byte    <= ram_rdata_b;
            end

            if (apu_dma_ack)
                dmc_ackpend <= 1'b0;
        end
    end

    APU #(
        .SSREG_INDEX_TOP  (SSREG_INDEX_APU_TOP),
        .SSREG_INDEX_DMC1 (SSREG_INDEX_APU_DMC1),
        .SSREG_INDEX_DMC2 (SSREG_INDEX_APU_DMC2),
        .SSREG_INDEX_FCT  (SSREG_INDEX_APU_FCT)
    ) apu (
        .MMC5           (1'b0),        // the APU is the machine's, not a mapper's
        .clk            (clk),
        .PHI2           (apu_ce),      // see the phases note above
        .ce             (apu_ce),
        .reset          (rst),         // every release is a fresh APU
        .cold_reset     (rst),         // machine.c does apu_create+apu_reset per load
        .allow_us       (1'b0),        // as MiSTer ties it: NES-accurate muting
        .PAL            (1'b0),        // APU_BASEFREQ is the NTSC rate
        .ADDR           (apu_reg),
        .DIN            (DO),
        .RW             (~cpu_write),
        .CS             (apu_cs),
        .audio_channels (5'b11111),
        .DmaData        (dmc_byte),
        .DmaAck         (apu_dma_ack),
        .DmaReq         (apu_dma_req),
        .DmaAddr        (apu_dma_addr),
        .get_or_put     (apu_get),
        .DOUT           (apu_dout),
        .Sample         (apu_sample),
        .IRQ            (),            // the frame counter's own; unused (see above)
        .get_ce         (),
        .put_ce         (),
        // Save-states: the framework's register bus, which this file now drives
        // (see the save-state section at the end). The APU is the bus's first
        // consumer and it takes its reset values from its four registers, which
        // is what makes a restore work: a load writes them, then resets the
        // machine, and the APU comes back as the state left it.
        .SaveStateBus_Din  (ss_reg_din),
        .SaveStateBus_Adr  (ss_reg_adr),
        .SaveStateBus_wren (ss_reg_wren),
        .SaveStateBus_rst  (ss_reg_rst),
        .SaveStateBus_load (ss_loading),
        .SaveStateBus_Dout (apu_ss_dout)
    );

    /* ------------------------------------------------------------------ *
     * Audio
     *
     * The APU's mixer output (APUMixer's LUT sum) is a LEVEL, not a signed
     * sample: silence is 0 and a channel adds to it, which is what the NES's
     * own unipolar output is -- its analog stage's coupling capacitor is what
     * turns that into the square wave you hear. It is 17 bits at the top of a
     * loud mix (about $F76F), so it is halved here to fit the 16-bit signed
     * word the Pocket's I2S path takes, and that is the whole conversion: no
     * DC blocker and no offset, because the value the DAC receives is exactly
     * the NES's own signal level, and the AC coupling that removes its DC is
     * on the other side of the pin.
     *
     * Everything from here to the pads -- the anti-alias filter, the 48 kHz
     * resample and the I2S serializer -- is rtl/mc64_audio.sv in core_top,
     * which owns the audio clock.
     * ------------------------------------------------------------------ */
    assign audio_l = 16'({1'b0, apu_sample[15:1]});
    assign audio_r = audio_l;

    /* ================================================================== *
     * Save states
     *
     * A Minicube64 state is 66056 bytes: an 8-byte header, the 64 register
     * words the framework's bus carries, and the machine's 64 KB of RAM in
     * address order (rtl/mc64_savestates.sv writes and reads all of it). This
     * section is the machine's half of it -- which registers exist, and where
     * the machine is stopped so that a state can be taken at all.
     *
     * WHICH REGISTERS. Two of the 64 are the machine's own:
     *
     *   index 0  (CPU)  { DI, P, S, Y, X, A, PC-1 }. The 6502's architectural
     *                   state, and PC is saved as the address of the
     *                   instruction ABOUT to execute rather than the core's own
     *                   post-fetch PC, so a state file's PC is a real address.
     *   index 24 (TOP)  the frame counter, the reset-sequence flag, the pending
     *                   interrupt, the five bytes this file shadows out of RAM
     *                   ($0100/$0101/$0104/$010E/$010F) and the DMC's bank.
     *
     * The APU's four (indices 16..19) are already on the bus: it takes its
     * reset values from them, so a restore resets the machine and the APU comes
     * back as the state left it. Nothing else in the machine has a register:
     * the video's raster free-runs by design (its picture is read out of the
     * RAM the state carries, and the link may never stop), and the DMC's
     * one-byte fetch in flight, the APU's fractional CE phase and the CPU's
     * microcode state are deliberately not in the file -- see each one below.
     *
     * WHERE THE MACHINE IS STOPPED, and why exactly there. Both transfers stop
     * on the DECODE cycle that follows an opcode fetch -- the one cycle in which
     * the core holds the opcode in DI, has not yet dispatched it, and will do
     * nothing until it does. That is the same place the Pocket's NES core stops
     * (`cpu_Instrnew`), and it is why a state needs no microcode state:
     * resuming is a re-fetch, not a re-entry. The `!cpu_int_entry` term keeps
     * the stop out of an interrupt entry's own DECODE, whose PC is mid-sequence
     * (the interrupt is carried in the state as `irq_req` and taken at the next
     * boundary instead, which is exactly what the CPU's own RDY semantics
     * would do).
     *
     * A SAVE ALSO WAITS FOR THE FRAME BOUNDARY (2026-10-17, the panel's "the
     * first save works and the second does not, and the picture tears"). An
     * instruction boundary is a coherent CPU state at any moment, but it is not
     * a coherent PICTURE: at an arbitrary point the cartridge is part-way
     * through redrawing the framebuffer, so the 64 KB the state carries is half
     * last frame and half this one -- the state loads back perfectly and shows
     * a torn frame, and the machine resumes with a partly-elapsed frame budget.
     * So a save waits for `frame_cnt` to roll over (the vblank, where the
     * cartridge's drawing for the frame is finished and the next has not
     * begun), and stops on the first instruction boundary after it -- usually
     * the first instruction of the vblank interrupt handler. What the state then
     * holds is a WHOLE frame, its `frame_cnt` is a handful of clocks past zero,
     * and a load resumes the machine at the top of a frame.
     *
     * It is the same rule `pause_hold` already follows, for the same reason, and
     * it costs at most one frame of latency before the stop (16.7 ms against a
     * menu action), which is why the OS's own watchdog is nowhere near it. A
     * LOAD does not wait: its state carries `frame_cnt` itself, the raster is
     * re-armed by the transfer's own `sync_en` edge, and the host is waiting on
     * the answer.
     *
     * A LOAD ENDS BY RESETTING THE MACHINE. Every part of it that reads a
     * savestate register does so from its reset path (the APU's channels, the
     * 6502's register file), so the engine holds `rst` for 16 clocks at the end
     * of a load and then, on `load_done`, this section writes the two things a
     * reset cannot carry: the CPU's PC/A/X/Y/S/P -- which the core's reset
     * branch would otherwise pin to $0200 and A=X=Y=0, S=$FD -- and the
     * machine's counters and shadows above. The CPU re-fetches its opcode from
     * the restored PC and the machine is exactly where the state left it.
     * ================================================================== */

    // The two machine registers, in the framework's own primitive -- the same
    // `eReg_SavestateV` the APU is built out of (rtl/eReg_SavestateV.v), so
    // there is one definition of what a state register is in this tree.
    wire [63:0] ss_cpu_dout, ss_aux_dout, ss_aux_word;

    // { DI, P, S, Y, X, A, PC-1 }: DI is the opcode of the instruction about to
    // execute, kept because it is what the stopped core is holding (the
    // restore re-fetches it, so the field is informative rather than load-
    // bearing), and the flags are the P byte this core assembles.
    wire [63:0] ss_cpu_din = {DI, cpu_P, cpu_dbg_s, cpu_dbg_y, cpu_dbg_x,
                              cpu_dbg_a, cpu_dbg_pc - 16'd1};

    /* { THE HOLDS AS THEY WERE, irqvec_hi, irqvec_lo, dmc_bank, pal_bank,
     *   vram_bank, irq_req, res_seq, frame_cnt[16:0] } -- least significant
     * field first.
     *
     * THE TOP FIVE BITS ARE A DIAGNOSTIC, and they exist because a frozen
     * panel is otherwise unobservable from here (2026-10-17: `a load stops
     * until a ROM is selected`, reproduced in every ordering the harness has
     * and in none of them on the panel). They are the machine's own hold
     * signals, sampled the cycle BEFORE a state is asked for -- `!ss_sleep`,
     * i.e. the last moment at which the machine was doing what the user sees --
     * in this order from bit 63 down:
     *
     *     [63] pause_s2       the pause REQUEST (the Select button)
     *     [62] pause_hold     ... latched on a frame boundary
     *     [61] pause_freeze   ... and the CPU actually parked by it
     *     [60] ss_freeze      the state engine's own hold
     *     [59] pause_muted    a load handed the machine back
     *
     * NOTHING READS THEM BACK: a load takes [58:0] and ignores these, so the
     * field cannot change machine behaviour and an old state (which carries
     * zeroes) loads exactly as it always did. What it buys is that `save while
     * it is stuck` is a diagnosis -- the file says which latch is holding the
     * machine, which is the one thing a screenshot cannot. */
    logic [4:0] ss_hold_seen;      // the holds, one cycle before the request
    wire  [4:0] ss_hold_now  = {pause_s2, pause_hold, pause_freeze, ss_freeze,
                                pause_muted};

    always_ff @(posedge clk) begin
        if (!ss_sleep)
            ss_hold_seen <= ss_hold_now;
    end

    wire [63:0] ss_aux_din = {ss_hold_seen, irqvec_hi, irqvec_lo, dmc_bank,
                              pal_bank, vram_bank, irq_req, res_seq, frame_cnt};

    eReg_SavestateV #(.Adr(SSREG_INDEX_CPU_1), .def(64'h0)) ss_reg_cpu (
        .clk       (clk),
        .BUS_Din   (ss_reg_din),
        .BUS_Adr   (ss_reg_adr),
        .BUS_wren  (ss_reg_wren),
        .BUS_rst   (ss_reg_rst),
        .BUS_Dout  (ss_cpu_dout),
        .Din       (ss_cpu_din),
        .Dout      (ss_cpu_word)
    );

    eReg_SavestateV #(.Adr(SSREG_INDEX_TOP), .def(SSREG_DEFAULT_TOP)) ss_reg_top (
        .clk       (clk),
        .BUS_Din   (ss_reg_din),
        .BUS_Adr   (ss_reg_adr),
        .BUS_wren  (ss_reg_wren),
        .BUS_rst   (ss_reg_rst),
        .BUS_Dout  (ss_aux_dout),
        .Din       (ss_aux_din),
        .Dout      (ss_aux_word)
    );

    // The register bus's answer: the wired-or of every register whose index is
    // being read (the APU's four are in here through its own instance).
    assign ss_reg_dout = ss_cpu_dout | ss_aux_dout | apu_ss_dout;

    assign ss_ram_own  = ss_ram_rden | ss_ram_wren;

    // The CPU's write is the load's completion pulse itself: the engine raises
    // `load_done` in the cycle its reset has already been released and
    // `ss_freeze` is still up, so the registers are written one cycle before
    // the core can execute anything.
    assign ss_cpu_we = ss_load_done;

    /* ---- the engine itself ------------------------------------------- */
    mc64_savestates savestates (
        .clk        (clk),
        .reset      (reset),          // the platform's reset only: not `rst`, or the
                                      // engine would abort its own load reset
        .save_tog   (ss_save_tog),
        .load_tog   (ss_load_tog),
        .sleep      (ss_sleep),
        .paused     (ss_freeze),
        .abort_tog  (ss_abort_tog),
        .busy       (ss_busy),
        .loading    (ss_loading),
        .reset_ss   (ss_reset),

        // the picture, for a load: the pair the frame in progress is drawn
        // with, and the blanking slot its bytes may be rewritten in
        .vis_vram_bank (video_cur_vram_bank),
        .vis_pal_bank  (video_cur_pal_bank),
        .blank_slot    (video_blank_slot),
        .flushing      (ss_flushing),

        .save_data  (ss_save_data),
        .save_wr    (ss_save_wr),
        .save_full  (ss_save_full),
        .load_rd    (ss_load_rd),
        .load_data  (ss_load_data),
        .load_empty (ss_load_empty),

        .reg_adr    (ss_reg_adr),
        .reg_din    (ss_reg_din),
        .reg_wren   (ss_reg_wren),
        .reg_rst    (ss_reg_rst),
        .reg_dout   (ss_reg_dout),

        .ram_addr   (ss_ram_addr),
        .ram_rden   (ss_ram_rden),
        .ram_wren   (ss_ram_wren),
        .ram_wdata  (ss_ram_wdata),
        // The engine's read is the same registered word port A presents; the
        // CPU's byte is a mux of it (see mc64_ram).
        .ram_rdata  (ram_rdata_a_word),

        .load_done  (ss_load_done),
        .load_err   (ss_load_err)
    );

    /* ---- the stop itself --------------------------------------------- *
     *
     * The machine is stopped on the DECODE cycle that dispatches an
     * instruction -- `cpu_instr_start`, the first cycle of every instruction in
     * this core (the opcode is prefetched during the previous instruction's
     * last cycle, so DECODE is where DI holds it and PC is one past it; the
     * tap's note in cpu_6502.sv has the mechanism). RDY drops DURING that
     * cycle, so the dispatch does not happen and the core is left sitting
     * there: the opcode in DI, PC one past it, nothing in flight, and the
     * address a state records is PC-1 -- the opcode's own address, which is
     * what a load writes back so the instruction is re-entered from its first
     * cycle. Neither side ever sees a half-executed instruction.
     *
     * COMBINATIONAL, not registered: a stop that fell one cycle later would
     * land on the instruction's first execution state, by which time the
     * dispatch has happened and PC has moved into the instruction's own
     * operands. `ss_freeze` is only the hold for the rest of the transfer, so
     * that the cycle is not re-run the moment the engine touches the RAM port.
     *
     * `~rst` keeps the stop out of the machine's own reset: during reset this
     * core sits in FETCH with a DECODE after it, and a state taken there would
     * be a state of a machine that has not been released -- the case the
     * controller's watchdog exists to answer instead (a cartridge that never
     * starts has no instruction boundary, so the command is answered err).
     *
     * `~cpu_int_entry` keeps the stop out of an interrupt entry's DECODE, whose
     * PC is mid-sequence (the interrupt is carried in the state as `irq_req`
     * and taken at the next boundary instead, which is exactly what the CPU's
     * own RDY semantics would do).
     */
    /* ---- the save's frame boundary (see WHERE THE MACHINE IS STOPPED) --- *
     *
     * `ss_frame_armed` is set by the vblank rollover and cleared when the
     * transfer ends. It gates the stop for a SAVE only: `ss_loading` is the
     * engine's own "this one is a restore", and a restore must not wait a
     * frame for anything (its `frame_cnt` is about to be overwritten by the
     * file's own).
     *
     * The counter it waits on is `frame_cnt`, which this module keeps running
     * while a state is merely pending -- it is only held once `ss_freeze`
     * engages -- so the boundary always arrives.
     */
    logic ss_frame_armed;

    always_ff @(posedge clk) begin
        if (rst || !ss_sleep)
            ss_frame_armed <= 1'b0;
        else if (frame_cnt == (FRAME_CLOCKS - 1))
            ss_frame_armed <= 1'b1;   // the frame is drawn: stop at its top
    end

    assign ss_stop = ss_sleep & ~rst & cpu_instr_start & ~cpu_int_entry &
                     (ss_loading | ss_frame_armed);

    always_ff @(posedge clk) begin
        if (reset)
            ss_freeze <= 1'b0;
        else if (!ss_sleep)
            ss_freeze <= 1'b0;    // the state is done: the machine runs again
        else if (!ss_freeze && ss_stop)
            ss_freeze <= 1'b1;    // ... and it is held until then
    end


`ifdef MC64_SIM
    // Observation taps for the audio harness: the DMC's fetch, as it happened.
    assign dbg_apu_dma_addr = dmc_addr;
    assign dbg_apu_dma_byte = dmc_byte;
    assign dbg_apu_dma_ack  = apu_dma_ack;
    assign dbg_apu_ce       = apu_ce;
    assign dbg_apu_get      = apu_get;
`endif

endmodule
