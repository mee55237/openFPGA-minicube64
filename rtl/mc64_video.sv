/*
 * Minicube64 video -- the signal the Pocket's scaler sees.
 *
 * THE PICTURE OWNS THE FRAME ORIGIN. The first pixel of the first line is
 * framebuffer (0,0), the first line is line 0. There is no front porch on
 * either axis and no border: the scaler captures the active window from the
 * sync marker forward, so a picture parked in the middle of a mostly-DE-low
 * frame is a screenful of nothing with a tiny picture buried in it (which is
 * exactly what the old 401x266, picture-at-(168,101) signal was).
 *
 * THE LINE, 88 pixel cycles (x is the position within the line):
 *
 *     x  0..63   picture, DE high
 *     x 64..85   blank    (DE low)   -- 22 DE-low cycles: the trailing blank
 *     x 86       HS       (DE low)   -- APF rule: >=1 DE-low cycle after the
 *     x 87       gap      (DE low)      DE deassertion, and >=1 before the
 *                                       next line's DE assertion
 *
 * One HS per line, in the second-to-last cycle, with one blank cycle after it
 * and the whole 22-cycle trailing blank before it -- the position the
 * hardware-proven sibling core uses (its HS is the last blank cycle before the
 * next picture row; the picture owns the line origin).
 *
 * THE FRAME. 64 picture lines, then 186 blank ones:
 *
 *     frame = 250 lines * 88 cycles = 22,000 pixels
 *           = EXACTLY 60.000000 Hz at the PLL's 1.32 MHz
 *
 * EVERY LINE IS THE SAME LENGTH, and that is a hardware finding rather than a
 * preference. This file used to lay the frame out as 67-cycle lines and give
 * the odd 24 cycles (22,000 is not a multiple of 67) to one line as a slack
 * line: a 91-cycle line, 1.36x its neighbours, once per frame. The panel never
 * locked that shape -- pure noise on every cart -- while the geometry before
 * it, whose lines were all identical, had locked and reported a frame rate.
 * The sibling core that runs on this hardware keeps its longest line within
 * 1.8% of the rest. 22,000 = 88 x 250 exactly, so the frame needs no slack
 * line and none is given to it: the line is 21 cycles wider (67 -> 88) and the
 * frame 78 lines shorter (328 -> 250), and every one of the 250 lines is the
 * same length -- one constant period for a scaler counting our clock.
 *
 * The blank cycles are DE-low time, and they are also where the palette reload
 * lives, which is free. The 186 blank lines sit BEHIND the VS, outside the
 * 64-row window the VS/HS pair opens, so nothing the scaler captures changes.
 *
 * THE RASTER IS NOT THE CPU'S FRAME -- BUT ITS PHASE IS THE CPU'S, AND THAT IS
 * THE DIFFERENCE BETWEEN A PICTURE AND A HALF-DRAWN ONE.
 *
 * The two frames cannot be the same length and it would be wrong to make them
 * so. The CPU's frame is machine.c's exec6502(6400000/60) = 106,666 machine
 * clocks, counted by mc64_top's own counter (exactly the emulator's, which is
 * what the golden comparison in sim/ compares), and the vblank interrupt is
 * generated from it. This raster lays 22,000 of the PLL's 1.32 MHz pixels end to
 * end: 106,666.67 machine clocks at 160/33 machine clocks per pixel. So the two
 * frames are 0.67 of a clock apart, 6.3 ppm, by construction -- and locking them
 * exactly would mean a CPU budget that is not the emulator's, 106,666.67 not
 * being the emulator's number.
 *
 * WHAT MUST BE LOCKED IS WHERE THE PICTURE SITS INSIDE THE CPU'S FRAME, because
 * the picture is a live read of the framebuffer: one port-B read per pixel, the
 * byte used on the next pixel, no line store and no frame store. A cartridge
 * redraws its screen immediately after the vblank interrupt -- the start of the
 * CPU's frame -- so a raster that is also sweeping the screen at that moment
 * reads rows the cartridge has already erased and not yet rewritten. That is
 * not a theory and it is not subtle: with the two frames merely free-running
 * (both starting together at reset, drifting 0.67 of a clock per frame), the
 * `input` demo's d-pad indicators came out as follows -- Up MISSING, every other
 * button drawn -- in 30 frames of 30, while the bytes were in RAM the whole time
 * and the emulator and this core agreed about them byte for byte. The demo
 * erases and redraws its indicators in the order up, down, left, right, so the
 * first one drawn is the first one the sweep reaches: the panel's "the d-pad up
 * does nothing". (docs/minicube64-fpga-spec.md 17.6 is the full account;
 * tools/input_sim.sh is the check, and tools/input_check.py the box table.)
 *
 * SO THE PHASE IS LOCKED, AND ONLY THE PHASE. Two things, and both of them are
 * conditional on sync_en (a held machine has no CPU frame to lock to, and this
 * module then runs exactly as it always did -- which is what keeps the bring-up
 * picture, and tools/video_sim.sh's free-running geometry, unchanged):
 *
 *   1. THE PICTURE MAY NOT START UNTIL THE CPU'S FRAME IS NEARLY OVER. The last
 *      PICTURE_CLK clocks of the CPU's frame are a window (mc64_top hands this
 *      module `frame_cnt`, so the window is just a comparison); the frame's last
 *      line is held, blank, until that window is open, and only then does the
 *      raster roll into the picture. The picture therefore always RUNS FROM the
 *      end of the CPU's frame TOWARDS its boundary, so what it reads is the
 *      frame the cartridge has just finished drawing -- the same thing
 *      display_machine() renders in the emulator, one frame later, exactly.
 *
 *   2. WHAT IS LEFT OVER GOES INTO THE BLANKING, A PIXEL AT A TIME. The raster's
 *      own frame is 0.67 of a clock longer, and the wait above is quantised to
 *      whole lines, so the picture still arrives out of phase by up to a few
 *      hundred clocks. The blanking pays it back: a blank line that loses a
 *      pixel moves everything after it -- the next picture included -- 160/33 of
 *      a clock (4.85) earlier; one that gains one moves it later. Up to
 *      N_BLANK-1 = 185 such pixels a frame, one per line (the frame's last line
 *      is excluded: the VS and its 3-cycle gap to the HS live there, and moving
 *      the HS without moving the VS would break that gap).
 *
 * WHAT THAT COSTS ON THE LINK, WHICH IS THE THING THIS FILE EXISTS TO PROTECT.
 * The picture region is untouched: the 64 picture lines are EXACTLY 88 pixels,
 * every frame, with the same DE shape, the same HS positions and the same VS.
 * What moves is a blank line, by ONE pixel (1.1%), on the frames where the phase
 * has drifted far enough to need one -- about one frame in seven in the locked
 * steady state. That is the same order as the shape the panel already locks: the
 * hardware-proven sibling core keeps its longest line within 1.8% of the rest.
 * The frame period is the CPU's 106,666 clocks to within a pixel, i.e. 60.000000
 * Hz with ~5-clock (0.0045%) jitter rather than the 0.13% systematic error that
 * made the panel drop a frame every 13 s.
 *
 * THE PHASE LOCK IS NOT THE WHOLE ANSWER, AND THE REST IS THE FRAME STORE.
 * Locking the phase fixed the cartridge that redraws on vblank and finishes
 * early; it cannot fix one that is still drawing while the beam is over the
 * picture. That is not hypothetical: the panel's `Dead End Holiday Demo` redraws
 * its `FIND PRESENTS` banner top-down and reaches it about 9,400 clocks after
 * the picture starts, so the beam read the last two rows of the bar before the
 * cartridge had written them and the picture showed the room instead. THE FRAME
 * STORE below is the answer -- the picture is drawn from a 4 KB copy taken in
 * the blanking, so it is always a frame the cartridge had FINISHED, at the cost
 * of at most one raster frame of latency -- zero for a cartridge that has
 * finished drawing before the capture. See that note for the whole of it.
 *
 * {frame_end} below is the raster's own rollover, kept for the testbench.
 *
 * WHY 22,000 PIXELS -- THE REFRESH RATE IS A DELIVERABLE, AND THE FRAME LENGTH
 * IS THE FREE PARAMETER. The panel reported 60.077 Hz, 0.13% fast, and the
 * reason was that 318 uniform 67-cycle lines is the nearest whole number of
 * LINES, not the nearest whole number of PIXELS -- because the pixel clock was
 * 6.4/5 MHz, and 1.28 MHz / 60 = 21,333.33 pixels, so the best a whole number
 * of pixels could do was 21,333 = 60.0009 Hz (that is what this file did
 * between the first two hardware tests, and the PLL's own note still explains
 * why 1.28 was chosen).
 *
 * The frame rate is the pixel clock divided by the pixel count, and BOTH are
 * ours: what is fixed is that the machine is 6.4 MHz (the 6502 and the
 * emulator's frame budget) and that both clocks must be integer divisions of
 * one PLL VCO. So the thing to choose is the RATIO. 6.4/1.32 = 480/99 = 160/33
 * is small integers; 6.4/1.28 = 5 was too, and it is exactly what pinned the
 * frame to 21,333.33. With 1.32 MHz, 22,000 pixels is a round number and:
 *
 *     1,320,000 Hz / 22,000 px = 60.000000 Hz, exactly, by construction.
 *
 * 22,000 was picked because 1.32 MHz is what the VCO's 480 counter gives under
 * the 6.4 MHz machine (see mf_pll_mc64_0002.v: VCO 633.6 MHz = 74.25/15 x 128,
 * counter 99 for the machine and 480 for the pixel, all integers). And 1.32 is
 * not one candidate among many: every output is the VCO divided by ONE integer
 * C counter, of which the device has 512, so a VCO satisfying both exact 6.4
 * and exact 1.32 has to be 6.4k = 1.32m with k = 33n and m = 160n -- n = 1 is
 * 633.6 MHz, n = 2 needs a counter of 640 and is already out of range. The
 * frame is not *rounded* to 60 Hz; it is defined to be it.
 *
 * THE FRAME-OPENING WINDOW, AND WHY THE VS IS ON THE LAST LINE OF THE FRAME.
 *
 * The platform's video rules are a SEQUENCE, not two independent edges:
 *
 *     "A frame starts with a single cycle pulse on VS. Each line starts with a
 *      single cycle pulse on HS."   ...   "HS may not be immediately sent
 *      until 3 cycles after VS."
 *
 * so a frame opens
 *
 *     VS -> (>=3) -> HS -> (>=1) -> DE
 *
 * and therefore THE FRAME'S FIRST ACTIVE LINE IS THE ONE WHOSE HS COMES AFTER
 * THE VS -- and the platform captures from that pair: it takes the declared
 * 64x64 window from the line/frame start, i.e. from the hs/vs edge.
 *
 * This core used to put the VS in the gap of the picture's LAST line. That
 * line's own HS is three cycles later on the SAME line, so the first active
 * line the platform finds is the first BLANK line, and its 64-row window then
 * covers 254 blank lines: a perfectly synced link (HS and VS well formed, one
 * per line and frame, every gap rule satisfied -- every check in tools/ passed)
 * carrying no picture at all. That is the fault the panel reported: black
 * screen, "sync ok", and the OS screenshot hanging on a capture that never
 * completes. Nothing that decodes OUR counters can see it, which is why the
 * harness now decodes the window's.
 *
 * So the VS goes on the FRAME'S LAST LINE, at x = 83, three cycles before that
 * line's ordinary HS at x = 86, and the picture's first pixel one gap cycle
 * after it:
 *
 *     line 249   x 0..82 blank
 *                x 83   VS, single cycle   (DE low)
 *                x 84..85 blank
 *                x 86   HS                (DE low)
 *                x 87   gap               (DE low)
 *     line 0     x 0    picture, DE high  -- the frame's first active line
 *
 * i.e. VS -> 3 -> HS -> 1 -> DE, the documented opening, with the picture's
 * row 0 at the frame origin and all 64 picture rows inside the 64-row window.
 * The blank lines are then BEHIND the VS (trailing blank), which is where the
 * platform's own rules allow any amount of blanking and where every envelope
 * of the panel-proven sibling core puts it.
 *
 * READING ORDER: EVERY_PIXEL. One pixel per `ce`; `ce` is the pixel enable,
 * not the CPU clock (see mc64_top.sv). The framebuffer's port B is read on
 * `ce` and the byte is used on the next `ce`, which is one clock of RAM
 * latency absorbed in the pixel period.
 *
 * PALETTE. The emulator reads $0100/$0101 and the palette bank at the top of
 * display_machine(), once per frame, and displays that frame from it. This does
 * the same thing in the same place: the bank registers are latched at
 * (hc==0, vc==0) and the 768 palette bytes are reloaded during the LAST TWELVE
 * blank lines, i.e. immediately before the frame boundary and entirely inside
 * invisible time. Nothing is displayed while the palette is arriving, so the
 * sample point is the frame boundary -- the emulator's own -- and a frame shows
 * a half-updated palette only if the CARTRIDGE writes the palette RAM during
 * those twelve lines: the reload is a one-byte-per-pixel sweep, so a store that
 * arrives after the sweep has passed its entry is simply not in this frame.
 * Measured 2026-10-20 (docs/panel-video-stress.md): a 768-byte write whose tail
 * crossed the window left entries 252-255 at their previous values for exactly
 * one frame -- 64 pixels -- and the emulator cannot show it, because
 * display_machine() samples the palette once per frame, atomically. The rule
 * for cartridges is therefore "finish palette writes before the last twelve
 * lines"; `stress_all` does it straight after a vblank interrupt.
 *
 * THE PALETTE IS 256 ENTRIES AND THE PIXEL BYTE IS THE INDEX. This was wrong
 * here until 2026-10-10, and the mistake was systematic, which is why nothing
 * caught it: upstream display_machine() does
 *
 *     uint8_t byt    = vram[i & 0xfff];
 *     int     lookup = byt*3;
 *     MFB_RGB(palette[lookup], palette[lookup+1], palette[lookup+2]);
 *
 * i.e. the RAW byte selects the triple; the palette pointer is
 * `&memory[paletteblock*256]` (memory_map.md: "Select 256 byte block for colors
 * 0x00-0xFF") and the data is "256 * 3 bytes for R8G8B8"; and the framebuffer
 * is "64x64 pixel framebuffer with 8 bits per pixel". Indices 64..255 are
 * therefore reachable and normal. This module used to mask the byte with & 0x3f
 * and hold 64 triples -- a 64-colour console -- and sim/compare.py masked the
 * same way, so the RTL and every test agreed with each other while both
 * disagreed with the emulator: any pixel >= 64 drew the wrong colour, and on
 * the default palette (where upstream leaves entries 64..255 ZERO, because its
 * initialiser fills only the first 64 triples of a 256*3 array) this core drew
 * a colour where the emulator draws black.
 *
 * DEFAULT PALETTE. machine.c fills byte0 = b*85, byte1 = g*85, byte2 = r*85
 * with the loops ordered r(outer) g b(inner), then display_machine() passes
 * byte 0 as RED, so for colour index c the default palette has RED in the LOW
 * bits: R = (c & 3)*85, G = ((c>>2) & 3)*85, B = ((c>>4) & 3)*85. That looks
 * like an emulator quirk but it is the observed behaviour and is reproduced
 * faithfully rather than "corrected". Palettes loaded from RAM use normal R,G,B
 * byte order -- the .pal files confirm it (`hex 000000 ff0000 00ff00 0000ff` is
 * documented as red, green, blue).
 */
`default_nettype none
`timescale 1ns / 1ps

module mc64_video #(
    // ---- geometry, in pixel cycles ----
    parameter int H_ACT   = 64,             // picture width
    parameter int V_PIC   = 64,             // picture lines
    parameter int H_TOTAL = 88,             // picture + 22 blank + HS + gap
    parameter int N_BLANK = 186,            // invisible DE-low lines BEHIND the VS

    /* The CPU's frame, in machine clocks. mc64_top owns that counter and passes
     * its value in (see `frame_cnt` below); the default here is the same number
     * so this module stands alone in a testbench (sim/tb_video.cpp), and
     * mc64_top OVERRIDES it with its own localparam so the two cannot drift
     * apart silently. See THE RASTER'S PHASE, below. */
    parameter int FRAME_CLOCKS = 106666
) (
    input  logic        clk,        // machine clock
    input  logic        ce,         // pixel enable: one pulse per pixel period
    input  logic        reset,

    /* ---- the CPU's frame, for the phase lock ----
     *
     * frame_cnt is mc64_top's frame counter: it runs 0..FRAME_CLOCKS-1 and rolls
     * over at the CPU's frame boundary, which is also the vblank interrupt, i.e.
     * the moment a cartridge starts its next frame's drawing. sync_en means
     * "there is a CPU frame to lock to": mc64_top drives it as
     * `~reset & ~ss_freeze & ~pause_freeze`, so it is low while the machine is
     * held in reset, for as long as a save-state transfer has the counter
     * stopped, AND for as long as the USER's pause holds the machine (the
     * panel's `unpauses to a black screen`; a held machine has no frame to
     * align to, which is exactly what this input means).
     *
     * BOTH ARE IGNORED WHILE sync_en IS LOW, and that is load-bearing rather
     * than tidy: with the machine held there is no CPU frame, so the raster
     * free-runs on the exactly-250-uniform-line geometry tools/video_sim.sh
     * measures and sim/tb_boot.cpp expects of the held picture.
     *
     * THE FROZEN CASE IS WHY THOSE TERMS ARE IN THERE, and it is a panel fault,
     * not a tidiness preference. The user's pause is the same fault reached the
     * other way: `pause_hold` does NOT stop `frame_cnt`, but a machine that is
     * held has no frame the picture may be aligned to, and leaving the lock
     * engaged across it is a raster waiting on a machine that is not running --
     * the identical black screen, which on the panel reads as the picture never
     * coming back. mc64_top holds `frame_cnt` for the whole of a state
     * transfer, so with sync_en left high the window can never open again: the
     * phase lock measures the picture's start as far out of reach, drops `locked`
     * and parks on the frame's last line waiting for a window that will not come
     * until the transfer ends. The raster then stops producing frames -- HS and VS
     * keep their own cadence but no frame ever completes -- and the panel, whose
     * scaler is fed by this raster, loses the picture: BLACK SCREEN for the length
     * of the transfer, and a picture that comes back only when it does. Whether
     * it happened at all depended on WHERE in the CPU's frame the counter was
     * stopped (the window is the frame's last 27,307 clocks, so a freeze in the
     * other 74% parked the raster and one inside the window did not) -- the panel's
     * "saves do not load, and the screen goes black". Dropping sync_en for the
     * hold instead free-runs the raster: the frames keep coming at 60 Hz (nothing
     * else about them changes), the framebuffer is static, so what the panel shows
     * is the frozen frame it should -- and the release re-arms the lock, which is
     * the `sync_en && !sync_en_r` edge below. */
    input  logic [16:0] frame_cnt,
    input  logic        sync_en,

    /* ---- the visible window is being written RIGHT NOW --------------------
     * (2026-10-17 the panel's `restored picture is torn or garbled`; 2026-10-18
     * the panel's `while it loads any user would think it is corrupted`.)
     *
     * This used to be "a load is in flight", and the picture was blanked for
     * the whole transfer. The load now writes the bytes the raster is drawing
     * from (one 4 KB framebuffer bank and one 768-byte palette bank) only
     * inside `blank_slot`, so for a normal load this input is never high while
     * a picture is being scanned and the panel sees NOTHING of the transfer: the
     * frame that was already on the screen simply becomes the state's own frame
     * a frame later. That is the whole answer to `it takes forever to load and
     * it looks corrupted while it does` -- there is no blank, no tear, and
     * nothing to wait for.
     *
     * It is still a hold, and it is still here because a load CAN overrun: the
     * host may still be streaming the file (`mc64_savestates`' staging), in
     * which case the flush of the visible window can be late, and a transfer
     * the host abandons can leave the window half-written. In those cases the
     * hold engages at the frame boundary exactly as it always did -- the last
     * complete frame is kept, and the first picture after the hold is drawn
     * from the reloaded banks. A save does not assert this: it does not touch
     * the RAM, so its frozen picture is the truth. */
    input  logic        load_blank,

    // ---- RAM port B ----
    output logic [15:0] ram_addr,
    input  logic [ 7:0] ram_rdata,

    // ---- bank registers, sourced from RAM by the CPU side ----
    input  logic [ 7:0] vram_bank,  // $0100
    input  logic [ 7:0] pal_bank,   // $0101

    /* ---- the DMC is borrowing port B THIS CYCLE ---------------------------
     *
     * mc64_top's port-B mux gives the sample fetch priority over the video, so
     * on such a cycle `ram_rdata` is the DMC's byte and not the one this module
     * asked for. The scan-out used to take it anyway (one stray pixel, which is
     * the old behaviour and is not made worse here -- the picture no longer
     * reads port B at all once the store is full), and the frame store's copy
     * RE-ISSUES the read instead, so a capture can never take a sample byte for
     * a framebuffer byte. */
    input  logic        dmc_take,

    // ---- APF video ----
    output logic [23:0] video_rgb,
    output logic        video_de,
    output logic        video_hs,
    output logic        video_vs,

    // ---- frame strobe (in the pixel domain) ----
    output logic        frame_end,

    /* ---- what the frame in progress is being drawn WITH -------------------
     *
     * `cur_vram_bank`/`cur_pal_bank` are this module's own per-frame latches:
     * the pair the picture on the screen right now is drawn through, which is
     * not the pair the machine's shadows currently hold (the latch is taken
     * once a frame, at LD_FIRST, in the blanking). The save-state LOAD needs
     * exactly this pair and not $0100/$0101: it has to know which 4 KB of RAM
     * and which 768 palette bytes the raster will read before the next latch,
     * because those are the one part of the state it may not rewrite while the
     * picture is being scanned (see the `blank_slot` note below and
     * rtl/mc64_savestates.sv's deferred window).
     *
     * They were simulation-only taps before this; two consumers now, so the
     * sim harness takes them from mc64_top's own nets rather than from here. */
    output logic [ 7:0] cur_vram_bank,
    output logic [ 7:0] cur_pal_bank,

    /* ---- the byte each pixel was DRAWN from, and the address it lives at ---
     *
     * In the pixel cycle the colour is shifted out, these say what produced it:
     * the store's byte (once the store is full) or port B's (before that), and
     * the VRAM address that byte was taken from -- `vram_bank*4096 + i` either
     * way, because the store is filled from exactly those addresses. tb_mc64
     * records them for compare.py's picture check, which is why they exist: the
     * pin's colour has to be checkable against the byte it came from, and that
     * byte is no longer the one on the RAM bus (see THE FRAME STORE below).
     * Nothing in the core reads them. */
    output logic [ 7:0] pic_byte,

    /* ---- the raster's vertical blanking, as a slot a state write fits in ---
     *
     * High from the first blank line of the picture's frame to SLOT_LINES lines
     * into it: most of the blanking, and the one part of the frame in which the
     * 4 KB and 768 bytes the picture is drawn from may be rewritten without the
     * panel ever catching a half-old, half-new frame (the scan-out reads none of
     * them, and the palette reload and bank latch are still ahead). See the
     * localparam for where the two ends come from.
     *
     * A LEVEL and not a strobe, deliberately. The engine arrives here from a
     * copy of unknown length, so a one-cycle edge would be a coin toss between
     * "flushes now" and "waits a whole frame" -- and that frame is the extra
     * machine freeze the panel feels as `takes forever to load`. */
    output logic        blank_slot,

    /* ---- hold the per-frame bank latch ------------------------------------
     *
     * Asserted while a state is being restored. The machine's $0100/$0101 are
     * rewritten by the load itself, and the raster latches them once a frame;
     * letting that happen mid-load would switch the picture onto a bank the
     * copy has not finished writing (or off the bank whose bytes the copy is
     * deliberately leaving alone). So the pair the load started with is kept
     * in force until the transfer is over, and the first latch afterwards
     * picks up the state's own pair -- in the blanking, as every other one
     * does. The palette reload deliberately keeps running on the held pair:
     * it is what makes the state's palette visible on the first frame after
     * the load rather than a frame later. */
    input  logic        bank_hold
);

    localparam int V_TOTAL = V_PIC + N_BLANK;

    /* ---- the phase lock, in two numbers --------------------------------
     *
     * PICTURE_CLK is how long the picture takes on the MACHINE clock: V_PIC *
     * H_TOTAL pixels at 160/33 machine clocks per pixel, i.e.
     * 64 * 88 * 160/33 = 27,306.67, rounded to 27,307.
     *
     * PH_START is where the picture must BEGIN for it to END on the CPU's frame
     * boundary: FRAME_CLOCKS - PICTURE_CLK = 79,359. Everything the phase lock
     * does is this one number. `frame_cnt >= PH_START` is the window in which
     * the picture may run at all, and the difference between where the picture
     * actually started and PH_START is the error the blanking then absorbs. */
    localparam int PICTURE_CLK = 27307;
    localparam int PH_START    = FRAME_CLOCKS - PICTURE_CLK;

    /* The trim's budget: the blank lines that may lend the phase one pixel
     * each, one pixel per line. The frame's LAST line is not one of them -- the
     * VS and its 3-cycle gap to the HS live there, and shifting the HS without
     * shifting the VS would break that gap -- so the budget is N_BLANK - 1 = 185
     * pixels = 897 clocks, which is MORE than the coarse lock can overshoot by
     * (one whole line: 427 clocks). That is why one frame is always enough to
     * come back to zero. */
    localparam int TRIM_MAX = N_BLANK - 1;

    /* The trim's REACH, in clocks: what TRIM_MAX pixels are worth at 160/33
     * clocks each (4 clocks is the coarse conversion the measurement uses, so
     * this is the trim's own budget in its own units). Beyond it the lock is
     * not "recovered", it is re-TAKEN: the raster waits for the CPU's window
     * instead, which costs one longer frame and puts it back on the boundary. */
    localparam signed [17:0] LOCK_MAX = 18'sd740;   // 4 * (N_BLANK - 1) clocks

    /* ---- geometry, AT THE WIDTH THE COUNTERS HAVE -------------------------
     *
     * Every one of these is compared against `hc`/`vc`/`hcd`/`vcd`, which are 9
     * bits, so every one of them is declared 9 bits. Do NOT go back to writing
     * `H_TOTAL - 2`-style `int` arithmetic and part-selecting it at the use site
     * (`H_SYNC[8:0]`): a part-select of a 32-bit `int` localparam is what
     * Verilator 5.050 evaluates to ZERO, and this is not a subtle failure.
     * `h_last` came out 0, so every line lasted one pixel, so a whole frame was
     * 318 pixels with one 64-pixel DE burst in it and NO HS and NO VS at all --
     * the scaler would have had nothing to sync to, on every cart. It was found
     * by driving this module on its own (sim/tb_video.cpp, added for exactly
     * this): the pins said 320 pixels per frame, 0 HS, 0 VS. (The truncation
     * that actually caused it was a 1-bit `wire h_last`; the aliases below are
     * still right, and stop the next one.)
     */
    localparam [8:0]  H_ACT_W    = 9'(H_ACT);             // 64
    localparam [8:0]  V_PIC_W    = 9'(V_PIC);             // 64
    localparam [8:0]  V_TOTAL_W  = 9'(V_TOTAL);           // 250
    localparam [8:0]  H_LAST_W   = 9'(H_TOTAL - 1);       // 87: last cycle of a line
    localparam [8:0]  H_SYNC_W   = 9'(H_TOTAL - 2);       // 86: HS, on every line
    localparam [8:0]  V_LAST_W   = 9'(V_TOTAL - 1);       // 249: the frame's last line
    localparam [8:0]  H_VS_W     = 9'(H_TOTAL - 5);       // 83: VS, 3 cycles before the HS
    localparam [11:0] H_ACT_W12  = 12'(H_ACT);            // framebuffer index multiply

    // Palette reload: one byte per pixel period, whole bytes, in the last
    // twelve invisible lines. 12*H_TOTAL = 1056 >= 768 = 256 triples.
    localparam int LD_ROWS = 12;
    localparam [9:0] LD_BYTES = 10'd768;
    localparam [8:0] LD_FIRST_W = 9'(V_TOTAL - LD_ROWS);

    logic [8:0] hc, vc;           // position, 0..H_TOTAL-1 (every line) / 0..V_TOTAL-1
    logic [8:0] hcd, vcd;         // position one pixel earlier (pipeline delay)
    logic [9:0] pl;               // palette reload counter, 0..767

    logic [ 7:0] vram_bank_q, pal_bank_q;

    /* ---- THE PALETTE IS 16-BIT RGB565, NOT 24-BIT RGB888 ----------------
     *
     * This is where the picture's colour is generated, so this is where the
     * format lives: the reload below packs each entry as it arrives --
     * { R[7:3], G[7:2], B[7:3] } -- and the output stage left-aligns that into
     * the 24-bit word the APF's VIDEO port takes, with the unused low bits
     * ZERO. No RGB888 value exists anywhere in the render path, including the
     * default palette (default_rgb565() is computed in 565, not truncated from
     * 888). It costs 256 x 16 = 4,096 registers instead of 6,144 -- measured:
     * mc64_video is 6,294 dedicated logic registers and 0 block-memory bits in
     * the fit, so `pal` is ~98% of them and none of it is a RAM block.
     *
     * WHAT IS *NOT* 16-BIT: the palette the CARTRIDGE owns. Those are 768 bytes
     * inside the machine's 64 KB RAM -- "256 * 3 bytes for R8G8B8"
     * (memory_map.md), the same layout as a .pal data slot, part of every saved
     * state and of the frame-by-frame 64 KB comparison in tools/sim.sh. That
     * memory is emulated console state and stays 8 bits per channel; only this
     * cache and the pixels on the wire are 565.
     *
     * The consequence to know: the two low bits the pack drops are gone for
     * good at this point, so a cartridge that ramps a channel one step at a
     * time -- a plasma, a fade -- steps in units of 8 (red, blue) or 4 (green)
     * instead of 1. Pure primaries are unaffected (FF -> F8 left-aligned is
     * still "full"), which is why a wheel of saturated hues looks the same. */
    logic [15:0] pal [0:255];     // RGB565: R[15:11] G[10:5] B[4:0]
    logic [ 7:0] pal_r_s, pal_g_s;

    // The pair the frame in progress is drawn with, for the state engine's
    // deferred window (see the port notes).
    assign cur_vram_bank = vram_bank_q;
    assign cur_pal_bank  = pal_bank_q;

    /* ---- the blanking's write slot (see the `blank_slot` port note) -------
     * A decode of the line counter, not a counter of its own: the position IS
     * the statement, and a second counter would be one more thing that can
     * drift from it.
     *
     * IT IS MOST OF THE BLANKING, and that is the number that decides how long
     * a load takes: the engine reaches this state when its copy of everything
     * else has finished, which can be anywhere in the frame, and every frame it
     * spends waiting for a slot is a frame of extra machine freeze. Both ends are
     * hard rather than chosen:
     *
     *   V_PIC       the picture's own lines are the only ones that read the
     *               framebuffer: DE is high for vc < V_PIC and the scan-out
     *               fetches nothing else. From the first blank line on, the
     *               4 KB window is read by nobody.
     *   V_PIC+158   the reload, which is what reads the PALETTE window, starts
     *               at LD_FIRST_W = V_TOTAL - 12 = 238, and a flush is 608
     *               words x 8 bytes = 4,864 clocks = 11.4 lines. 226 is the
     *               last start that clears it; 222 (158 lines) leaves four
     *               lines of margin on top.
     *
     * 158 of the frame's 250 lines therefore mean a copy that lands anywhere in
     * the blanking flushes at once, instead of waiting out most of a frame. */
    localparam int SLOT_LINES = 158;
    localparam [8:0] SLOT_END_W = 9'(V_PIC + SLOT_LINES);
    assign blank_slot = (vc >= V_PIC_W) && (vc < SLOT_END_W);

    /* ---- THE FRAME STORE: the picture is a COMPLETED frame ----------------
     *
     * (2026-10-21, the panel's `Dead End Holiday Demo`: the `FIND PRESENTS`
     * banner's last two rows showed the room's background instead of the bar.)
     *
     * The picture used to be a live read of the framebuffer -- one port-B byte
     * per pixel, taken as the beam reached it. That is only right if the
     * cartridge has finished writing by the time the beam arrives, and the beam
     * arrives over the last 27,307 clocks of the CPU's 106,666-clock frame
     * (rows are 426.67 clocks apart). A cartridge that redraws on vblank is
     * usually done long before that, which is what the phase lock above bought;
     * one that is still drawing -- and this one was, it reaches the banner about
     * 9,400 clocks after the picture starts -- was caught mid-redraw, and the
     * rows the beam passed before the cartridge got to them came out as what was
     * there before: the room. The emulator cannot do that: exec6502() runs a
     * whole frame and display_machine() then renders a snapshot, so it only ever
     * shows a COMPLETED frame.
     *
     * This makes the hardware do the same thing, in the only way a raster that is
     * always sweeping can: it CAPTURES the framebuffer into a 4 KB store during
     * the blanking and draws the picture from the store. 4,096 reads at one per
     * clock, in the fifty blank lines before the palette reload (vc 188..237) --
     * after the previous frame's redraw and before the machine draws this one,
     * and outside the picture's own lines so the copy can never disturb the
     * scan-out. What the panel shows is therefore always a frame that was
     * finished in the RAM when it was captured.
     *
     * AND IT IS NOT A FRAME LATE FOR MOST CARTRIDGES. The capture runs at clocks
     * ~52,900..72,800 of the CPU's 106,666-clock frame and the picture is drawn
     * at 79,359..106,666 of THE SAME FRAME (the phase lock above puts the picture
     * at the frame's end), so what the panel shows is the current frame's
     * framebuffer as of the capture -- the emulator's own frame N, not N-1. The
     * sample point is the only thing that moved: a cartridge that has finished
     * drawing by ~72,800 shows exactly what display_machine() shows for that
     * frame, which is what sim/ compares and finds at offset 0.
     *
     * It is a frame late for a cartridge still drawing after the capture -- this
     * one redraws its banner at ~88,800 -- and nothing can avoid that: from the
     * moment the beam is over the picture there is no later moment in the frame
     * at which a finished version of what it is scanning can exist. The only
     * alternative is buffering a whole frame and painting after the boundary,
     * which is what the emulator does and would be a frame late for EVERY
     * cartridge, not just the late ones.
     *
     * WHY THOSE 50 LINES AND NOT MORE. The capture has to be as LATE as possible
     * before the picture (the earlier it runs, the more of the frame it misses),
     * and the reload owns port B from vc 238. 50 lines of 87..89 cycles is 4,350
     * at worst -- 4,098 reads, plus one re-issue per DMC borrow.
     *
     * TWO STORES, because a capture may never be shown half-written: the beam
     * reads one while the copy fills the other, and they are swapped only when
     * all 4,096 bytes have landed. A capture that runs out of window, or that a
     * state transfer or a held machine interrupts, does not swap at all -- the
     * panel keeps the frame it had, which is exactly what a held picture should
     * be.
     *
     * THE BANK: the capture reads the bank latched for the frame it is capturing
     * (`vram_bank_q`). A cartridge that changes $0100, or its palette bank,
     * between the capture and the reload gets one frame with the previous one --
     * the same one-frame lag this costs everywhere else, not a new fault.
     *
     * THE FALLBACK: `cap_ok` is 0 until the first capture completes and the picture
     * then reads port B exactly as it always did, so the first frame of a run (and
     * any run too short to fill the store) is byte-identical to the old path.
     */
    localparam [8:0]  CP_FIRST_W = 9'd188;                     // first capture line
    localparam [8:0]  CP_LAST_W  = 9'(V_TOTAL - LD_ROWS - 1);  // 237: the reload is 238..249
    localparam [12:0] CP_N       = 13'd4096;

    /* `ramstyle` the way mc64_ram and mc64_savestates do it: these are 4 KB
     * arrays with one write port and one read port and they MUST be M10K blocks,
     * not 32,768 flip-flops each. `no_rw_check` because the read and the write
     * never land on the same array in the same cycle (the beam reads the front
     * store while the copy fills the back one), so Quartus needs no bypass. */
    (* ramstyle = "M10K, no_rw_check" *) logic [7:0] sh_a [0:4095];
    (* ramstyle = "M10K, no_rw_check" *) logic [7:0] sh_b [0:4095];
    logic [7:0] pic_q;           // the store's byte for the pixel being emitted

    logic        cp_on;          // a capture is running
    logic        cp_prev;        // a read was issued last cycle
    logic        cp_prev_ok;     // ... into a cycle the DMC was not borrowing
    logic [12:0] cp_idx;         // next index to issue
    logic [12:0] cp_pidx;        // the index issued last cycle
    logic        cap_ok;         // a complete frame is in the store
    logic        cap_sel;        // 0: sh_a on screen, 1: sh_b

    // The byte the pixel being emitted was drawn from: the store once it is
    // full, port B before that (see THE FALLBACK).
    wire [7:0]  pix_byte  = cap_ok ? pic_q : ram_rdata;
    wire [15:0] cap_base  = {vram_bank_q[3:0], 12'b0};

    // This cycle's copy read landed, and was not taken by the DMC.
    wire cp_hold  = cp_on && (vc <= CP_LAST_W) && cp_prev && cp_prev_ok && !dmc_take;
    // ... and if it did not land, the index has to be issued again.
    wire cp_retry = cp_on && (vc <= CP_LAST_W) && cp_prev && !cp_hold;

    /* ---- the DEFAULT palette, as a FUNCTION of the pixel byte -------------
     *
     * Bank 0 is not "the bytes at 0": it is the emulator's generated default
     * palette, and this is its whole definition (the arithmetic below is
     * upstream's own initialiser, `default_palette`: entry c is
     * [b*85, g*85, r*85] with b=c&3, g=(c>>2)&3, r=(c>>4)&3, for c < 64, and
     * ZERO for 64..255 -- a pixel byte >= 64 is BLACK).
     *
     * IT USED TO BE A 256-ENTRY REGISTER ARRAY, WRITTEN IN ONE CLOCK. That is
     * what the panel caught: a 256 x 24-bit array updated from one always_ff
     * cycle, with three multipliers and a loop through the whole thing, is the
     * heaviest single-cycle write anywhere in this core, and it is a pure
     * function of the byte being looked up. Three ways of saying the same thing:
     * nothing needs it stored; the simulation can never see a fit's timing
     * fallout; and cube -- the one ROM in sim/roms that runs with pal_bank 0,
     * i.e. the only one that uses this palette at all (every other ROM supplies
     * its own in RAM) -- was the one cartridge reported wrong on the panel, as
     * a correct shape drawn through a wrong palette (its field byte is 0, which
     * entry 0 of the default palette makes BLACK, drawn red = entry 3's colour).
     *
     * So it is decoded per pixel instead, and there is no default array to
     * write: the RAM-palette path below is untouched and still fills `pal`.
     * The colours are identical, which is what tools/sim.sh's picture comparison
     * (29/29 raster frames against display_machine()'s decode) holds -- in 565,
     * because that is now what this core draws: compare.py packs the emulator's
     * decode before comparing, exactly as this module packs it here. */
    /* THE DEFAULT PALETTE IS 565 TOO, AND IS COMPUTED IN 565.
     *
     * It used to return the 24-bit RGB888 triple and leave the truncation to a
     * later stage. Now it returns the 16-bit word directly (top 5 bits of red,
     * top 6 of green, top 5 of blue) and `rgb565_to_wire()` at the output
     * left-aligns it, so nothing in this path is ever 888.
     *
     * The arithmetic is the emulator's, unchanged: entries 64..255 are ZERO (its
     * initialiser fills only the first 64 triples), a pixel byte >= 64 is BLACK,
     * and RED IS IN THE LOW BITS (machine.c fills byte0 = b*85, byte1 = g*85,
     * byte2 = r*85 with r outermost, and display_machine() passes byte 0 as RED).
     * Each channel is 0/85/170/255, whose 5-bit codes are 0/10/21/31 and 6-bit
     * codes 0/21/42/63 -- exactly what truncating those bytes to 565 gives, so
     * the picture is what the emulator's own colours would be on this link. */
    function automatic [15:0] default_rgb565(input [7:0] idx);
        logic [7:0] r8, g8, b8;
        begin
            r8 = (idx & 8'h03) * 8'd85;
            g8 = ((idx >> 2) & 8'h03) * 8'd85;
            b8 = ((idx >> 4) & 8'h03) * 8'd85;
            default_rgb565 = (idx >= 8'd64) ? 16'h0000
                                            : { r8[7:3], g8[7:2], b8[7:3] };
        end
    endfunction

    /* RGB565 -> the 24-bit word the APF's VIDEO port takes: LEFT-ALIGNED, with
     * the unused low bits zero. This is a wiring change, not a conversion -- it
     * is the only place the picture is 24 bits wide, and it exists because the
     * platform's DDR link is 24 bits per pixel (apf_top: [23:12] on one edge,
     * [11:0] on the other). The zero fill is deliberate: a bit-replication
     * expand (0x1F -> 0xFF) would fill those bits and destroy the property that
     * proves the format on the wire -- see sim/tb_boot.cpp's `link format`. */
    function automatic [23:0] rgb565_to_wire(input [15:0] c);
        rgb565_to_wire = { c[15:11], 3'b000,
                           c[10: 5], 2'b00,
                           c[ 4: 0], 3'b000 };
    endfunction
    logic [ 9:0] ld_idx_d;
    logic        ld_valid_d;

    /* ---- phase-lock state (see the header, THE RASTER'S PHASE) ----------
     *
     * locked   the picture's start is within the trim's reach of PH_START
     * sync_en_q/sync_en_r  the release's rising edge, which re-arms the search
     * vs_done  this frame's VS has been sent; while the frame's last line is
     *          held (waiting for the window) it must not be sent again
     * err_px   pixels of phase error, SPENT one per blank line as the blanking
     *          runs -- shortening a line by one pixel makes everything after it
     *          (including the next picture) start that much earlier
     * adj/adj_q  this line's one-pixel trim, and the same one line later, which
     *          is where the HS compare needs it (hcd lags hc by a pixel)
     */
    logic               locked;
    logic               sync_en_q, sync_en_r;
    logic               vs_done;
    logic signed [10:0] err_px;
    logic [1:0]         adj, adj_q;

    /* The load's picture hold (see the `load_blank` port note): `pic_hold`
     * blanks DE, and `rel_done` says the palette/bank reload has run since it
     * began, so the hold can end on the frame rollover rather than on the load
     * itself. */
    logic               pic_hold;
    logic               rel_done;

    localparam [1:0] ADJ_NONE  = 2'd0,
                     ADJ_SHORT = 2'd1,   // this line is one pixel shorter
                     ADJ_LONG  = 2'd2;   // this line is one pixel longer

    /* The line's own tail, moved by this line's trim. The HS travels with the
     * trim (a 87-pixel line's HS is at x=85, not 86) but the VS does not, which
     * is exactly why the VS's line is excluded from the budget above. */
    wire [8:0] h_last_eff = (adj == ADJ_SHORT) ? (H_LAST_W - 9'd1) :
                            (adj == ADJ_LONG)  ? (H_LAST_W + 9'd1) : H_LAST_W;
    wire [8:0] h_sync_eff = (adj_q == ADJ_SHORT) ? (H_SYNC_W - 9'd1) :
                            (adj_q == ADJ_LONG)  ? (H_SYNC_W + 9'd1) : H_SYNC_W;

    /* The window: the last PICTURE_CLK clocks of the CPU's frame, i.e. the time
     * in which the picture must run for it to end on the frame boundary. */
    wire window_open = (frame_cnt >= 17'(PH_START));

    /* The signed distance from PH_START to where this picture actually began,
     * in clocks. Positive means the picture began LATE (the raster is behind
     * the CPU's frame) and the blanking must give pixels back. */
    logic signed [17:0] d_clk;
    assign d_clk = $signed({1'b0, frame_cnt}) - $signed(18'(PH_START));

    // ---- picture area, current and one pixel earlier ----
    wire h_pic_d = (hcd < H_ACT_W);
    wire v_pic_d = (vcd < V_PIC_W);
    wire de_d    = h_pic_d & v_pic_d;
    wire in_pic  = (hc < H_ACT_W) & (vc < V_PIC_W);

    /* Every operand below is at the width of the thing it feeds. The `9'(...)`
     * casts above and the 16-bit operands here are the same discipline as the
     * `wire [8:0] h_last` note further down: the fit log carries a Warning
     * (10230) for every 32-bit value assigned to a narrow target, and a
     * truncation warning in this file is the exact fingerprint of the bug that
     * made a whole frame one pixel long. Zero of them is the point.
     */
    wire [11:0] pix_index = (vc[5:0] * H_ACT_W12) + {6'b0, hc[5:0]};
    wire [15:0] bank_base = {vram_bank_q[3:0], 12'b0};
    wire [15:0] pix_addr  = bank_base + {4'b0, pix_index};

    /* The byte the picture is drawn FROM, for the harness (see the port note):
     * the store's once it is full, port B's before that. It is the same register
     * the colour lookup consumes, so a picture record holds exactly the byte
     * that produced each pixel on the wire. The address of that pixel is the
     * port-B address, which mc64_top still drives from `pix_addr` during the
     * picture -- and which is the address the store was filled from. */
    assign pic_byte = pix_byte;

    /* THE RELOAD'S FIRST CYCLE IS NOT A FETCH.
     *
     * It is the cycle `pl` is cleared AND the bank pair is latched, and both of
     * those land on this same edge -- so a fetch presented there would compute
     * its address from the pair the latch is REPLACING. It did, and the cost was
     * a whole palette: the fetch opened at {pal_bank_q, 8'b0} + 0 with the old
     * bank, which on the first frame of a run (bank 0, before anything has been
     * latched) is address $0000, so the byte at zero page $00 was taken as the
     * RED of palette entry 0 and every later byte landed one place along.
     * `tilemap` shows it: its palette entry 0 is black in the emulator and came
     * out #100000 on the panel-side picture, because $00 is one of the
     * cartridge's own scratch bytes. The fetch starts on the next cycle, where
     * `pl` already reads 0 and the new pair is in force. */
    wire ld_start  = (hc == 9'd0 && vc == LD_FIRST_W);
    wire ld_row    = (vc >= LD_FIRST_W) && !ld_start;
    wire ld_en     = ld_row && (pl != LD_BYTES);
    wire palbase0  = (pal_bank_q == 8'h00);

    /* 9 BITS, explicitly, and it stays that way. This was `wire h_last` -- a
     * 1-bit net -- which silently truncated the line's last cycle to its low
     * bit, 0. Every line then ended after one pixel: a frame came out 318
     * pixels long carrying one 64-pixel DE burst, with no HS and no VS anywhere
     * in it. The scaler would have had nothing to lock to, on every cart, and
     * nothing else in this repo looks at these pins (tb_video.cpp does, and is
     * what caught it).
     *
     * EVERY LINE IS H_TOTAL LONG -- there is no slack line any more. The
     * 22,000-pixel frame used to carry its remainder on one blank line 24
     * cycles longer than the rest (67 vs 91, a 1.36x period once per frame);
     * the panel never locked that shape, and 22,000 = 88 x 250 exactly, so
     * nothing has to ride on one line. tb_video.cpp checks that every HS-to-HS
     * interval is H_TOTAL.
     */
    wire [8:0] h_last = h_last_eff;

    /* There is deliberately no end-of-line ("slot") word on this interface.
     * The platform's rule is "RGB pixel value should remain all zero when DE
     * is deasserted unless using the frame feature bits or end-of-line bits",
     * and 24'h0 is itself the default end-of-line command (Set Scaler Slot 0);
     * the word that used to be driven here (bit 13 set) is Set Scaler Slot 1
     * under the current documentation. See the RGB note in rtl/core_top.v. */

    always_ff @(posedge clk) begin
        if (reset) begin
            hc <= '0;  vc <= '0;  hcd <= '0;  vcd <= '0;  pl <= '0;
            ld_valid_d <= 1'b0;  ld_idx_d <= '0;
            pal_r_s <= '0;  pal_g_s <= '0;
            ram_addr <= '0;
            video_rgb <= 24'h0;
            video_de  <= 1'b0;
            video_hs  <= 1'b0;
            video_vs  <= 1'b0;
            frame_end <= 1'b0;
            vram_bank_q <= '0;  pal_bank_q <= '0;
            locked <= 1'b0;  sync_en_q <= 1'b0;  sync_en_r <= 1'b0;
            vs_done <= 1'b0;  err_px <= '0;  adj <= ADJ_NONE;  adj_q <= ADJ_NONE;
            pic_hold <= 1'b0;  rel_done <= 1'b0;
            cp_on <= 1'b0;  cp_prev <= 1'b0;  cp_prev_ok <= 1'b0;
            cp_idx <= 13'd0;  cp_pidx <= 13'd0;
            cap_ok <= 1'b0;  cap_sel <= 1'b0;
        end
        else if (ce) begin
            frame_end <= 1'b0;
            hcd <= hc;
            vcd <= vc;
            adj_q <= adj;
            sync_en_q <= sync_en;
            sync_en_r <= sync_en_q;

            /* ---- the phase lock -------------------------------------------
             *
             * The raster is not the CPU's frame and cannot be (see the header):
             * 22,000 pixels is 106,666.67 machine clocks and the CPU's frame is
             * 106,666. What CAN be locked is its PHASE, and here is the whole
             * mechanism: the picture must END on the CPU's frame boundary, so
             *     1. it must not START before the last PICTURE_CLK clocks of
             *        the CPU's frame have begun -- at the frame's end the raster
             *        waits (whole blank lines, nothing else changes) for the
             *        window to open, so the picture always runs from the end of
             *        the CPU's frame towards its boundary; and
             *     2. whatever is left over -- the 0.67 of a clock by which the
             *        raster's own frame is longer, and the line-granular
             *        overshoot of that wait -- is absorbed by the BLANKING, one
             *        pixel per blank line: a line that loses a pixel moves
             *        everything after it, the next picture included, 160/33 of
             *        a clock earlier.
             *
             * Both halves are conditional on sync_en, so a held machine (no CPU
             * frame at all) keeps the free-running geometry. */

            // The release re-arms the search: from here on there IS a CPU frame
            // to lock to, and where the raster happens to be is arbitrary.
            //
            // AND IT IS TAKEN AT THE FRAME'S START, not where the release
            // arrived (2026-10-17, the panel's `looks corrupted while it
            // loads`). Dropping `locked` mid-frame changes `adj` on the very
            // next line, i.e. the length of the lines of a picture that is
            // already half scanned -- the frame is cut short and the panel is
            // shown a picture with its bottom missing. A state transfer's
            // freeze is exactly that release, so the change belongs where every
            // other geometry decision is taken: one per line, at hc==0, and the
            // lock itself only at vc==0.
            if (sync_en && !sync_en_r && vc == 9'd0)
                locked <= 1'b0;

            /* ---- the load's picture hold (see the `load_blank` port) --------
             * Set for as long as the state is being restored, and released only
             * once the raster has re-read the loaded banks and palette AND has
             * rolled into a frame with them -- so the first picture after a
             * load is a whole one, drawn from the top through the state's own
             * banks, instead of the tag end of a frame that was already half
             * scanned when the last byte landed. */
            /* THE HOLD IS TAKEN ON A FRAME BOUNDARY, NOT WHERE THE ENGINE
             * HAPPENS TO BE (2026-10-17, the panel's `it looks corrupted while
             * it loads`). `load_blank` rises as soon as the state engine starts,
             * which is somewhere in the middle of a frame; taking the hold there
             * cuts the frame the scaler is being fed in half -- the top of it
             * scanned from the old framebuffer, then no DE at all -- and the
             * panel shows that as a torn picture for the length of the transfer.
             * The blank has to begin where a picture begins. The engine's RAM copy
             * is aligned to the same edge (`mc64_savestates`' ST_LOAD_ALIGN), so
             * nothing is rewritten before the hold is up and nothing is shown
             * until the frame after the copy.
             *
             * The picture the panel holds during the transfer is the frame it
             * had already completed, which is what a scaler does with DE low: no
             * half-scanned frame, no tear. */
            if (load_blank) begin
                rel_done <= 1'b0;          // never release mid-load
                if (vc == 9'd0)
                    pic_hold <= 1'b1;
            end
            else if (pic_hold && ld_en && !rel_done) begin
                rel_done <= 1'b1;      // this frame's reload is the loaded one
            end

            /* ONE DECISION PER LINE, TAKEN AT ITS START. `adj` has to be in
             * force for the line's whole length (the counter compares hc against
             * h_last_eff on every pixel), and err_px has to be spent exactly
             * once per line, so both belong at hc==0 -- not in the pixel path. */
            if (hc == 9'd0) begin
                if (vc == 9'd0) begin
                    /* The picture is starting: measure the phase and hand the
                     * blanking the pixels it owes. One pixel is 160/33 = 4.85
                     * clocks, so `>>> 2` (4 clocks) is the conversion, and it
                     * errs slightly LONG in the trim -- which is the safe
                     * direction: it makes the loop settle with the picture a
                     * hair early rather than a hair late. */
                    if (sync_en && locked) begin
                        if (d_clk > LOCK_MAX || d_clk < -LOCK_MAX)
                            locked <= 1'b0;   // out of reach: wait for the window
                        else
                            err_px <= 11'(d_clk >>> 2);
                    end
                    else
                        err_px <= '0;
                end
                else if (vc >= V_PIC_W && vc != V_LAST_W) begin
                    /* A blank line, and not the VS's: it may lend the phase one
                     * pixel. Behind the CPU's frame -> one pixel SHORT of 88,
                     * which brings everything after it (the next picture
                     * included) 4.85 clocks earlier. Ahead of it -> one pixel
                     * LONG. The `+/- 1` above biases the loop the same way, so
                     * the residual sits under a pixel instead of on it. */
                    if (err_px > 11'sd0) begin
                        adj <= ADJ_SHORT;
                        err_px <= err_px - 11'sd1;
                    end
                    else if (err_px < 11'sd0) begin
                        adj <= ADJ_LONG;
                        err_px <= err_px + 11'sd1;
                    end
                    else
                        adj <= ADJ_NONE;
                end
                else
                    adj <= ADJ_NONE;
            end

            /* ---- position counters ---- */
            if (hc == h_last) begin
                hc <= '0;
                if (vc == (V_TOTAL_W - 9'd1)) begin
                    /* The frame's end. With a CPU frame to lock to, the raster
                     * may not roll into the picture until the window is open:
                     * it sits on this last line (blank, DE low, and its VS
                     * already sent -- `vs_done`) for as many whole lines as it
                     * takes. In lock that is 0 lines, because the window is
                     * 27,307 clocks wide and the raster arrives within a few
                     * clocks of its opening. */
                    if (sync_en && !locked && !window_open) begin
                        vc <= vc;                  // hold: no roll, no frame_end
                    end
                    else begin
                        vc <= '0;
                        frame_end <= 1'b1;
                        vs_done <= 1'b0;
                        if (sync_en)
                            locked <= 1'b1;
                        /* A loaded frame is only handed to the panel at the
                         * rollover that follows its own palette/bank reload. */
                        if (pic_hold && rel_done)
                            pic_hold <= 1'b0;
                    end
                end
                else
                    vc <= vc + 9'd1;
            end
            else
                hc <= hc + 9'd1;

            /* ---- THE CAPTURE (see THE FRAME STORE above) -------------------
             *
             * Started on the first cycle of CP_FIRST's line and run to the last
             * cycle of CP_LAST's, one read per clock: present an index, retire
             * the previous one's data a cycle later. The store is filled only
             * while this runs, and the buffers are swapped only by the cycle
             * that retires index 4,095 -- so a capture that is cut short (out of
             * window, a transfer, a held machine) leaves the panel's frame
             * alone. */
            if (hc == 9'd0 && vc == CP_FIRST_W && !load_blank && !bank_hold) begin
                cp_on      <= 1'b1;
                cp_idx     <= 13'd0;
                cp_prev    <= 1'b0;
                cp_prev_ok <= 1'b0;
            end

            if (cp_on && (vc <= CP_LAST_W)) begin
                if (cp_retry) begin
                    /* The DMC took port B on the cycle this read was to land in:
                     * the byte on the bus is its sample, not the framebuffer, so
                     * the index is issued again rather than captured wrong. */
                    cp_idx  <= cp_pidx;
                    cp_prev <= 1'b0;
                end
                else begin
                    if (cp_prev && (cp_pidx == (CP_N - 13'd1))) begin
                        cp_on <= 1'b0;
                        /* ... and only hand it to the panel if nothing was
                         * writing the framebuffer while it was captured. */
                        if (!load_blank && !bank_hold) begin
                            cap_sel <= ~cap_sel;
                            cap_ok  <= 1'b1;
                        end
                    end
                    /* issue this cycle's read (the address itself is driven by
                     * the port mux below, from the same index) */
                    if (cp_idx < CP_N) begin
                        cp_pidx    <= cp_idx;
                        cp_prev_ok <= !dmc_take;
                        cp_prev    <= 1'b1;
                        cp_idx     <= cp_idx + 13'd1;
                    end
                end
            end
            else if (cp_on) begin
                /* The window closed before the store was full: keep the frame
                 * the panel already has, and try again next frame. */
                cp_on   <= 1'b0;
                cp_prev <= 1'b0;
            end

            /* the byte the last issue delivered, into the buffer NOT on screen */
            if (cp_hold) begin
                if (cap_sel) sh_a[cp_pidx[11:0]] <= ram_rdata;
                else         sh_b[cp_pidx[11:0]] <= ram_rdata;
            end

            /* ... and the byte for the pixel this cycle's colour will emit: the
             * store's, read with the same index and the same one-clock latency
             * as the port-B read it replaces, so nothing downstream moves. */
            pic_q <= cap_sel ? sh_b[pix_index] : sh_a[pix_index];

            /* ---- the bank pair, read once per frame as the emulator does ----
             *
             * display_machine() reads $0100/$0101 once and renders the whole
             * frame from that pair, so the pair has to be in force from the
             * picture's FIRST pixel. This latch used to sit at (hc==0, vc==0),
             * which is where the address register is loaded from it -- so the
             * load read the value from before the latch and the frame's first
             * pixel came from the PREVIOUS frame's vram bank. One pixel, but
             * the picture comparison in sim/ sees it (white, at (0,0), reading
             * $0000 instead of $2000 on the frame the boot cartridge switches
             * banks), and a cartridge that changes banks between frames would
             * show it as a stray pixel at the start of every line's first
             * column.
             *
             * WHERE EXACTLY, and why not the frame's last cycle. It sits at the
             * FIRST cycle of the reload's first line (vc 238), one cycle before
             * the reload presents its first address, and it is in force for the
             * whole of the next picture -- the picture only starts at vc 0.
             *
             * The frame's last cycle was the original position and it is one
             * line TOO LATE, because the reload BELOW reads `pal_bank_q` as the
             * base of the 768 bytes it fetches. A latch after the reload means
             * the reload fetches the bank latched a frame EARLIER, so the next
             * picture draws this frame's framebuffer through the previous
             * frame's palette -- and the first picture of a run (whose reload
             * happened while the bank was still whatever the reset left) draws
             * through a palette that was never fetched at all: a whole frame of
             * the wrong colours after any change of $0101, and a black one when
             * the new bank's bytes are not in the RAM yet. `text` sets $0101 =
             * 14 on its first frame and does exactly that; sim/compare.py's
             * picture check reports it, byte for byte, once the harness records
             * the bank this latch produced instead of a RAM proxy for it. */
            /* `bank_hold` is a state transfer in flight (see its port note):
             * the pair stays as it was until the load is over, so the raster
             * never draws a bank the copy has not finished writing. */
            if (hc == 9'd0 && vc == LD_FIRST_W && !bank_hold) begin
                vram_bank_q <= vram_bank;
                pal_bank_q  <= pal_bank;
            end

            /* ---- palette reload, in the last three invisible lines ----
             *
             * The first cycle of the first reload line is the bank latch above;
             * `pl` is cleared with it, and the first ld_en (and therefore the
             * first address) is one cycle later, so the fetch of the k-th
             * palette byte starts from the latch's own value. */
            if (hc == 9'd0 && vc == LD_FIRST_W)
                pl <= 10'd0;

            if (ld_en) begin
                ram_addr <= {pal_bank_q, 8'b0} + {8'b0, pl};
                pl       <= pl + 10'd1;
            end
            else if (cp_on && (vc <= CP_LAST_W) && (cp_idx < CP_N) && !cp_retry)
                /* the capture's read; it owns port B for its window, and it
                 * never overlaps the reload above or the picture below */
                ram_addr <= cap_base + {3'b0, cp_idx};
            else if (in_pic)
                ram_addr <= pix_addr;
            else
                ram_addr <= 16'h0000;

            ld_valid_d <= ld_en;
            ld_idx_d   <= pl;

            if (ld_valid_d && !palbase0) begin
                case (ld_idx_d % 3)
                    10'd0: pal_r_s <= ram_rdata;
                    10'd1: pal_g_s <= ram_rdata;
                    /* packed as the three bytes arrive: the reload has just
                     * read R, then G, then B out of the machine's 768-byte
                     * palette window, so this is the only moment all three
                     * exist at once. */
                    10'd2: pal[ld_idx_d / 3] <= { pal_r_s[7:3],
                                                  pal_g_s[7:2],
                                                  ram_rdata[7:3] };
                endcase
            end

            /* ---- the default palette is decoded, not stored ----
             *
             * The 256-entry single-cycle write that used to sit here is gone:
             * `default_rgb565()` above is the same palette as a function of the
             * pixel byte, and the output stage below selects it when the bank is
             * 0. The upstream behaviour it reproduces is unchanged: indices
             * 64..255 are ZERO and a pixel byte >= 64 is BLACK,
             * and display_machine() passes byte 0 as RED -- RED IN THE LOW BITS
             * is the emulator's own behaviour and is reproduced rather than
             * "corrected", which is exactly what the function's arithmetic
             * carries. */

            /* ---- output pipeline: de/hs/vs describe the delayed position ---- */
            video_de <= de_d && !pic_hold;   /* the load's hold: DE low, RGB 0 */
            video_hs <= (hcd == h_sync_eff);
            /* One VS per frame, and not one per held line while the frame's last
             * line sits waiting for the window. */
            video_vs <= (vcd == V_LAST_W) && (hcd == H_VS_W) && !vs_done;
            if (vcd == V_LAST_W && hcd == H_VS_W)
                vs_done <= 1'b1;

            if (de_d && !pic_hold)
                /* the RAW byte is the index; bank 0 is the generated default,
                 * any other bank is the 768 bytes the reload packed into `pal`
                 * -- both already RGB565, both left-aligned onto the wire here.
                 * `pix_byte` is the STORE's byte once it is full (see THE FRAME
                 * STORE) and port B's before that, so the colour always comes
                 * from the byte tb_mc64 records for that pixel. */
                video_rgb <= palbase0 ? rgb565_to_wire(default_rgb565(pix_byte))
                                      : rgb565_to_wire(pal[pix_byte]);
            else
                video_rgb <= 24'h0;   /* zero whenever DE is low -- the rule,
                                       * and the load's held picture with it */
        end
    end

endmodule
