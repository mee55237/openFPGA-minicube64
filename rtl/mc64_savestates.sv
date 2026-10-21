/*
 * mc64_savestates -- the state itself.
 *
 * WHAT A MINICUBE64 STATE IS. Spec section 11: the machine is a 6502, a flat
 * 64 KB of RAM, and registers carved into the fabric. Everything a program can
 * see lives in one of those two places, and the RAM holds the framebuffer, the
 * palette, the control registers ($0100/$0101), the stack and the code -- so a
 * state is:
 *
 *     word 0        { STATESIZE, header_count }        a format tag, see below
 *     words 1..64   64 register words                  the machine's registers
 *     bytes 520..   the 64 KB of RAM, in address order
 *
 * 66056 bytes on the wire, and it is the machine's own byte order: byte 0 of
 * the file is bit 7:0 of the word that reaches the host first, so a state file
 * is the RAM's address space with 520 bytes of registers in front of it. (The
 * Agg23 controller this borrows its APF handshake from byte-swaps each word,
 * because it wants MiSTer's `.ss0` files; nothing here reads a MiSTer file.)
 *
 * WHY 64 REGISTERS AND NOT FOUR. The MiSTer framework's state engine
 * (`rtl/savestates.vhd`, the file this replaces) copies a fixed 64 words off
 * its register bus, whether or not anything is listening on the other end, and
 * writes all 64 back on a load. Keeping that count is what lets the parts of
 * the machine that already speak the framework's register bus -- the APU, whose
 * four registers are `eReg_SavestateV` instances at indices 16..19
 * (`rtl/regs_savestates.sv`) -- take part without any adapter. The unused
 * indices read back zero and write into nothing.
 *
 * THE HEADER, and what the tag is for. The framework writes
 * `{STATESIZE, header_amount}` as the first word and, on a load, refuses the
 * state unless the upper half matches its own STATESIZE -- a cheap "is this
 * file mine, and is it the format this build writes" test. The NES framework
 * picks 331776 for that constant, which is not the size of anything it writes;
 * here it is the exact number of bytes the state's own two regions occupy
 * (64*8 + 65536 = 66048) so that a reader can at least tell where the number
 * comes from. `header_amount` is the framework's version counter, bumped once
 * per save in MiSTer; nothing on this machine consumes it, so it is written as
 * a constant and ignored on load.
 *
 * THE REGISTER BUS is the framework's, names and all, because the primitives
 * on the other end (`eReg_SavestateV`, in Verilog: rtl/eReg_SavestateV.v) are:
 *
 *   reg_adr/reg_dout   the address being read and the wired-or of every
 *                      register's answer -- the engine reads one word per
 *                      address, 0..63, and the machine ORs its primitives
 *   reg_din/reg_wren   a load writes each word back into the primitive whose
 *                      Adr matches, on the register's own clock
 *   reg_rst            the framework's "restore defaults" pulse: this engine
 *                      asserts it at the start of a load (and on a platform
 *                      reset), never at the reset a load ends with -- see the
 *                      note at its driver
 *
 * THE RAM PORT IS FOUR BYTES WIDE (2026-10-18), which is the whole reason this
 * engine is not the VHDL one. The framework's engine pokes its memory port once
 * every eight clocks (a settle counter for an SDRAM round trip it does not have
 * here) -- 72 ms for 64 KB. Port A of `mc64_ram` answers one clock after its
 * address is presented (it is the array's own register, clocked half a cycle
 * later -- see that file) and hands back a whole 32-bit word, so a 64-bit state
 * word costs TWO clocks and the 64 KB costs 16,384 of them: 2.6 ms. That, and
 * not the host's ceiling, is what a save costs. Port A was byte-wide until
 * 2026-10-18 (one byte a clock, 10.2 ms); widening the array to 32 bits with
 * byte enables is what took the freeze down by four.
 *
 * That last clause is only true because of rtl/mc64_state_store.sv (2026-10-18).
 * The engine produces a word every eight machine clocks and the host reads one
 * every ~22, so a 256-entry pipe between them made the SAVE cost the host's
 * read of 66 KB -- with the machine stopped, since this engine holds it for the
 * whole transfer. The store is the whole state's worth of words behind the
 * crossing, so the copy now runs to its end at the port's rate and the machine
 * is handed back while the host is still reading. See that file and
 * save_state_controller.sv's save-side note.
 *
 * THE FREEZE, and why the engine owns it. A state has to be taken somewhere the
 * machine can leave from and come back to, and "somewhere" is an instruction
 * boundary: `sleep` asks mc64_top to stop the machine, `paused` tells the
 * engine it has stopped, and neither a save nor a load starts before that.
 * The same rule makes the load exact: the machine is stopped when the registers
 * are written back, so nothing is mid-update, and the CPU resumes on the
 * restored PC with the restored flags. (This is the shape the Pocket's own NES
 * core uses -- its `corepause` latches on `cpu_Instrnew` -- and it is why a
 * state does not need to carry a microcode state.)
 *
 * WHICH BOUNDARY is not this module's business -- it asks for `sleep` and waits
 * for `paused` -- but it is worth knowing here, because it is what the state's
 * own contents depend on: a LOAD stops at the first boundary (mc64_top passes
 * `ss_loading` through `ss_stop`), while a SAVE additionally waits for the
 * frame's rollover, so the 64 KB it carries is one whole picture instead of the
 * top of the last frame and the bottom of this one. See WHERE THE MACHINE IS
 * STOPPED in rtl/mc64_top.sv.
 *
 * RESET. Only the request synchronisers reset with the machine; the sequencer
 * is a power-on-initialized block whose constants cannot go stale, and a
 * machine reset arriving mid-state aborts the transfer (`reset` below drops the
 * sequencer to IDLE and lets the host's handshake time out) rather than
 * pretending the state it half-wrote is complete. A load never resets the
 * machine: at an instruction boundary there is nothing to unwind.
 */
`default_nettype none
`timescale 1ns / 1ps

module mc64_savestates #(
    // One 64-bit word per register, exactly as the MiSTer framework copies
    // them; see the header for why the count is kept.
    parameter int REGS      = 64,
    parameter int RAM_BYTES = 65536
) (
    input  logic        clk,          // the machine clock (6.4 MHz)
    input  logic        reset,        // machine reset: aborts a state in flight

    // ---- requests, crossed in from the host's clock domain ----
    // Levels that toggle once per request; two flops and an edge detect below.
    input  logic        save_tog,
    input  logic        load_tog,
    // The host's end has given up on a transfer it was driving (its watchdog
    // expired: no word of the stream for 113 ms, or a save whose instruction
    // boundary never arrived). The engine must DROP the state and hand the
    // machine back: it is holding the machine frozen, and one timed-out command
    // would otherwise leave the panel stopped with a half-written state. It
    // reports nothing -- the controller has already answered the OS with err.
    input  logic        abort_tog,

    // ---- the freeze ----
    output logic        sleep,        // hold the machine at an instruction boundary
    input  logic        paused,       // ... and mc64_top reports it has
    output logic        busy,         // a state is in flight
    output logic        loading,      // ... specifically one being restored
    output logic        reset_ss,     // the machine's reset pulse inside a load

    /* ---- the picture, for a load (see THE VISIBLE WINDOW below) ----------
     *
     * `vis_vram_bank`/`vis_pal_bank` are the pair mc64_video is drawing the
     * frame in progress with (its own per-frame latches, held still for the
     * length of a transfer). `blank_slot` is high while the raster is in the
     * part of its vertical blanking a 4.8 KB write fits in. Together they are
     * the whole of the rule this engine follows about the picture: the bytes
     * those two banks cover are written ONLY in that slot. Everything else in
     * the 64 KB is written as the file arrives, because nothing is reading it
     * but a frozen CPU. */
    input  logic [ 7:0] vis_vram_bank,
    input  logic [ 7:0] vis_pal_bank,
    input  logic        blank_slot,
    // ... and this is what the video blanks on, instead of the whole transfer.
    output logic        flushing,

    // ---- the state stream, machine-domain ends of core_top's two FIFOs ----
    output logic [63:0] save_data,
    output logic        save_wr,
    input  logic        save_full,
    output logic        load_rd,
    input  logic [63:0] load_data,
    input  logic        load_empty,

    // ---- the register bus (the framework's words, the framework's shapes) ----
    output logic [ 9:0] reg_adr,
    output logic [63:0] reg_din,
    output logic        reg_wren,
    output logic        reg_rst,
    input  logic [63:0] reg_dout,

    /* ---- the machine's RAM: FOUR BYTES PER CLOCK (2026-10-18) ------------
     *
     * Port A of mc64_ram is a 32-bit word with byte enables now, and this engine
     * is why: a 64-bit state word used to cost eight clocks (one byte a cycle
     * through a byte-wide port) and now costs two, so the copy in either
     * direction -- and the load's window flush with it -- runs at four bytes a
     * clock. The CPU's own byte access is unaffected: it is the same port, given
     * to one of them at a time (see mc64_top's `ss_ram_own`), and a byte written
     * by the CPU is a byte enable on this same word.
     *
     * `ram_addr` is still the BYTE address; the array drops the low two bits.
     * Every address this module presents is word-aligned, so a 32-bit write
     * covers exactly the four bytes the state file wants at that point. */
    output logic [15:0] ram_addr,
    output logic        ram_rden,
    output logic        ram_wren,
    output logic [31:0] ram_wdata,
    input  logic [31:0] ram_rdata,

    output logic        load_done,    // one pulse when a state has been restored
    output logic        load_err      // ... or when it was refused (bad tag)
);

    /* ------------------------------------------------------------------ *
     * The state's shape
     * ------------------------------------------------------------------ */
    localparam int RAM_WORDS  = RAM_BYTES / 8;
    localparam int WORD0      = 1;                 // header word, then REGS words
    localparam int REG_FIRST  = WORD0;
    localparam int RAM_FIRST  = WORD0 + REGS;
    localparam int WORDS      = RAM_FIRST + RAM_WORDS;   // 8257 words = 66056 bytes

    // The tag the header carries and the load checks. See the header note.
    localparam [31:0] STATESIZE = 32'(REGS * 8 + RAM_BYTES);
    localparam [31:0] HEADER    = 32'd1;

    /* ------------------------------------------------------------------ *
     * The request crossing
     *
     * A one-clock pulse in clk_74a would be 13 ns against a 156 ns machine
     * clock and could be missed outright, so the request crosses as a LEVEL
     * that toggles -- the same shape core_top's controller pulses and the same
     * reason (two flops plus an XOR on the pair is both the synchroniser and
     * the edge detect).
     * ------------------------------------------------------------------ */
    logic [2:0] save_s = 3'd0;
    logic [2:0] load_s = 3'd0;
    logic [2:0] abort_s = 3'd0;

    always_ff @(posedge clk) begin
        if (reset) begin
            save_s <= 3'd0;
            load_s <= 3'd0;
            abort_s <= 3'd0;
        end else begin
            save_s <= {save_s[1:0], save_tog};
            load_s <= {load_s[1:0], load_tog};
            abort_s <= {abort_s[1:0], abort_tog};
        end
    end

    wire save_req = save_s[2] ^ save_s[1];
    wire load_req = load_s[2] ^ load_s[1];
    wire abort_req = abort_s[2] ^ abort_s[1];


    /* ------------------------------------------------------------------ *
     * The sequencer
     *
     * One counter each for "which word of the file" and "which HALF of the
     * RAM word in hand", and a buffer for the halves: a save gathers two 32-bit
     * halves and pushes them as one 64-bit word, a load pulls one word and
     * spills two halves. RAM words are read and written four bytes per clock
     * throughout -- the port's full width.
     * ------------------------------------------------------------------ */
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_WAIT_PAUSE,
        ST_SAVE_HEADER,
        ST_SAVE_REGS,
        ST_SAVE_RAM_READ,       // one half of a state word: present this
                                // half's address, take the one before it
        ST_SAVE_RAM_TAIL,       // the high half arrives, the word is pushed, the
                                // next word's low half is presented -- one clock
        ST_LOAD_HEADER,
        ST_LOAD_REGS,
        ST_LOAD_RAM_FAST,       // take the next word and classify it (see THE
                                // VISIBLE WINDOW below)
        ST_LOAD_RAM_WRITE,      // ... 8 bytes of an ordinary word, into RAM
        ST_LOAD_BUF_STORE,      // ... or a visible-window word, into win_buf
        ST_LOAD_WIN_WAIT,       // wait for the raster's blanking slot
        ST_LOAD_WIN_FETCH,      // ... one cycle for the buffer's read to land
        ST_LOAD_WIN_WRITE,      // ... and rewrite the visible window there
        ST_LOAD_RESET,
        ST_FINISH
    } state_t;

    state_t state = ST_IDLE;

    /* ------------------------------------------------------------------ *
     * THE VISIBLE WINDOW, and why a load is not one straight copy
     *
     * 2026-10-18, the panel: *saves work, but it takes forever to load, and
     * while it loads any user would think it is corrupted*. The old shape was
     * a copy that started on the picture's frame boundary and BLANKED the panel
     * for its length (plus the frame it took the scaler to get the loaded banks
     * back), because the raster reads the framebuffer continuously and a copy
     * under it is a picture that is half one state and half another.
     *
     * Both halves of that are avoidable, and the key is that the raster reads
     * almost none of the state: during a picture it reads one 4 KB framebuffer
     * bank (`{vis_vram_bank, 12'b0}`, the bank mc64_video latched for this
     * frame) and, in the blanking, 768 bytes of one palette bank
     * (`{vis_pal_bank, 8'b0}`). Those 4,864 bytes are the ONLY bytes a load may
     * not rewrite while the picture is being scanned. Everything else -- code,
     * zero page, stack, the other 15 banks -- is read by nothing but a CPU that
     * is frozen for the transfer.
     *
     * So the copy is in two phases:
     *
     *   ST_LOAD_RAM_FAST   the whole file, in order, from the command. Words in
     *                      those two windows are PARKED in `win_buf` instead of
     *                      written; everything else goes straight into RAM. This
     *                      is 2.4 ms of the 2.6 a whole copy costs, and it
     *                      starts immediately: no frame to wait for.
     *
     *   ST_LOAD_WIN_WRITE  `win_buf` (608 words = 4,864 bytes = 0.29 ms) is
     *                      written in the raster's `blank_slot`, where nothing
     *                      is being scanned and the palette reload and bank
     *                      latch are still ~150 lines away. The loaded picture
     *                      is therefore the next picture, complete, drawn from
     *                      the state's own bytes -- with no blank frame and
     *                      nothing the panel can catch half-done.
     *
     * `win_vram`/`win_pal` are LATCHED when the transfer starts, not read live:
     * the load is what rewrites $0100/$0101, and mc64_video holds its own pair
     * for exactly that reason (`bank_hold`), so the pair that was in force when
     * the copy began is the pair it protects.
     *
     * The buffer is 608 x 64 bits = 38 Kbit: four M10K blocks, against a fit
     * that is 71 of 308 blocks in. It is a RAM and not registers for the same
     * reason mc64_ram is (`ramstyle`), so Quartus infers it.
     * ------------------------------------------------------------------ */
    localparam int VIS_WORDS = 4096 / 8;                 // one framebuffer bank
    localparam int PAL_WORDS =  768 / 8;                 // one palette bank
    localparam int WIN_WORDS = VIS_WORDS + PAL_WORDS;    // 608 words

    /* ------------------------------------------------------------------ *
     * The load's word gate
     *
     * `load_rd` has to be high in the SAME cycle as the word it takes: the
     * FIFO advances its read pointer at the end of that cycle, so the word on
     * `load_data` during it is the word consumed, and the next cycle's head is
     * the following one. Asserting it a cycle late (as a register) makes every
     * word arrive one cycle after the sequencer looked at it -- so the
     * sequencer reads each word twice and the stream is shifted by one word.
     * That is a real failure, not a redundancy: the header used to be written
     * back as the first register and the CPU's own word landed one index late,
     * which left the restored PC at $0000.
     *
     * COMBINATIONAL, and gated on the empty flag in the same breath, so the
     * sequencer's `load_ok` is exactly "there is a word, and this cycle takes
     * it".
     * ------------------------------------------------------------------ */
    /* THE NEXT WORD IS TAKEN DURING THE HIGH HALF OF THE PREVIOUS ONE
     * (2026-10-18). The copy is four bytes a clock through the array's 32-bit
     * port A, so a state word costs two clocks and both are the copy: the high
     * half's cycle is also where the next word is taken off the stage. The
     * stage's read is combinational (`load_data` is `load_stage[rd_count]`), so
     * the word for the cycle after the high half is already in front of this
     * module while that half is being written: taking it HERE puts the copy at
     * the port's own rate, two clocks to the word, and the fetch state below is
     * left for the one case that still needs it -- a stage that has run dry (a
     * host still streaming), where the copy waits for the host and nothing is
     * lost.
     *
     * `load_last_w` keeps the fetch off the state's LAST word: there is no word
     * after it, and asking for one would consume a word of the host's file that
     * belongs to nothing. */
    wire load_last_w = (word_idx == 14'(WORDS - 1));
    /* The feed is the same trick as before -- the next word is taken in the
     * high half's cycle -- but it has one more condition: a word that belongs in
     * `win_buf` is not written from here, so it cannot be fed into
     * ST_LOAD_RAM_WRITE. Those are ~600 words of 8,257; each costs the two
     * cycles of ST_LOAD_RAM_FAST + ST_LOAD_BUF_STORE instead, which is noise
     * against the 16,000-odd word writes around it. */
    wire load_feed   = (state == ST_LOAD_RAM_WRITE) &&
                       half && !load_last_w && !win_wr_n;
    wire load_wants  = (state == ST_LOAD_HEADER) ||
                       (state == ST_LOAD_REGS)   ||
                       (state == ST_LOAD_RAM_FAST) ||
                       load_feed;
    wire load_ok    = load_wants && !load_empty;

    assign load_rd = load_ok;

    logic [13:0] word_idx;    // 0..WORDS-1: which word of the file
    /* WHICH HALF OF A 64-BIT WORD the memory port is on: 0 = the low four
     * bytes, 1 = the high four. A state word costs two clocks, one per half, in
     * the gather, the copy and the flush alike. */
    logic        half;
    logic [63:0] word_buf;

    /* ---- the visible window, as addresses (see THE VISIBLE WINDOW) -------- */
    // The banks the picture in progress is drawn with, latched when the
    // transfer starts: `$0100`/`$0101` are about to be overwritten by the very
    // state this copy is carrying.
    logic [ 7:0] win_vram, win_pal;
    (* ramstyle = "M10K, no_rw_check" *) logic [63:0] win_buf [0:WIN_WORDS-1];
    /* THE BUFFER'S READ IS REGISTERED, AND THAT IS NOT AN OPTIMISATION
     * (2026-10-18, found by the fit). The flush wants one byte of
     * `win_buf[flush_idx]` at a time, so the first version read the array
     * combinationally and indexed the byte out of the result. Quartus will not
     * build a RAM out of that -- "uninferred due to asynchronous read logic" --
     * and 608 x 64 bits is 38,912 flip-flops: the fit came back at 23,806 of
     * 18,480 ALMs (129%), with mc64_savestates alone holding 39,150 registers
     * and 13,851 ALUTs. One word is now fetched into `win_q` a cycle before its
     * bytes are used, which is the shape M10K wants.
     *
     * THE ADDRESS HAS TO BE A REGISTER, not arithmetic. The first attempt read
     * `win_buf[flush_idx + 1]` (to have the next word ready in the same cycle as
     * the current one's last byte) and Quartus STILL called it asynchronous
     * logic: the block's address port is fed by an adder, so there is no
     * address register for it to use -- the same rule one level down from the
     * read's own register. The index is now plain `flush_idx`, presented every
     * cycle whether it is wanted or not, and the extra cycle a word's fetch
     * costs is `ST_LOAD_WIN_FETCH`. */
    logic [63:0] win_q;
    logic [ 9:0] flush_idx;

    // Which visible window a RAM word belongs to. Word granularity, and that
    // is exact rather than convenient: both windows are multiples of 8 bytes,
    // so a 64-bit word is wholly inside or wholly outside one.
    wire [12:0] rword    = 13'(word_idx - 14'(RAM_FIRST));
    // The palette window is 768 bytes = 96 words from {win_pal, 8'b0}, i.e.
    // THREE 256-byte pages and not one -- which is the sort of thing that only
    // shows up as a picture drawn through a palette that is 2/3 old.
    wire [12:0] pal_first = {5'b0, win_pal, 5'b0};           // $YY00, in words
    wire [12:0] pal_end   = pal_first + 13'(PAL_WORDS);
    wire        in_vis   = (rword[12:9] == win_vram[3:0]);   // $X000..$XFFF
    wire        in_pal   = (rword >= pal_first) && (rword < pal_end);
    wire        win_wr   = in_vis | in_pal;
    wire [ 9:0] win_ix   = in_vis ? {1'b0, rword[8:0]}
                                  : 10'(VIS_WORDS) + 10'(rword - pal_first);
    // ... and the same question asked about the NEXT word, which is what the
    // feed above has to know before it takes it.
    wire [12:0] rword_n   = rword + 13'd1;
    wire        win_wr_n  = (rword_n[12:9] == win_vram[3:0]) |
                            ((rword_n >= pal_first) && (rword_n < pal_end));

    /* EVERY SLOT IS WALKED, AND THE PALETTE PART SKIPS WHAT ISN'T THERE.
     *
     * 2026-10-18, the panel: *a few big black horizontal lines on top of normal
     * execution*, and then loads that report an error. This is the fault.
     *
     * The palette window's ADDRESS is `$0101` x 256 -- the emulator's memory
     * map says it in as many words (*select 256 byte block for colors
     * 0x00-0xFF*) -- and the framebuffer window's is `$0100` x 4096 (*select 4kb
     * block for display 0x00-0x0F*). So the palette lies inside the framebuffer
     * window exactly when `$0101[7:4] == $0100[3:0]`, and that is what decides
     * where its words were parked: a palette word inside the framebuffer window
     * is parked in the FRAMEBUFFER's slot (in_vis wins above) and one outside
     * it in the palette's own 96.
     *
     * The first version decided which with `win_pal[3:0] != win_vram[3:0]` --
     * the LOW nibble, which is the wrong half of the byte and not a page test
     * at all. Nor is it rare: a game that puts both its banks at 1 (an entirely
     * ordinary choice) has `win_pal[3:0] == win_vram[3:0]`, so the flush was
     * told "the palette is inside the framebuffer window, skip it" -- for a
     * palette 768 bytes at $0100, which is nothing of the kind. Its words were
     * parked in the palette's own slots and NEVER WRITTEN BACK, so the state's
     * palette never reached RAM and the machine drew on through the previous
     * state's colours: stale entries that resolve to black, i.e. a few wide
     * black rows over a game that is otherwise running perfectly. The mirror
     * case -- palette really inside the window, low nibbles different -- flushed
     * stale slots over the palette region instead.
     *
     * So the decision is per word, from the same arithmetic the parking uses: a
     * slot in the palette's part whose word the framebuffer part already wrote
     * is SKIPPED (its bytes are in that slot's own copy, and writing this one
     * would put stale bytes over them). No heuristic, and nothing here has to
     * be kept in agreement with a later change to the parking rule.
     *
     * The walk is the full 608 slots either way -- a skipped one still spends
     * its eight byte cycles, which is 768 clk_74a at worst against the flush's
     * 12.8 lines of blanking. */
    wire         pal_slot   = (flush_idx >= 10'(VIS_WORDS));
    wire [12:0]  pal_slot_w = pal_first + 13'(flush_idx - 10'(VIS_WORDS));
    wire         slot_taken = pal_slot && (pal_slot_w[12:9] == win_vram[3:0]);
    wire [ 9:0]  flush_last = 10'(WIN_WORDS - 1);
    // ... and the HALF of the word it is writing RIGHT NOW: four bytes go in per
    // clock, so the high half's base address is the low half's plus four. (This
    // was a byte index when the port was byte-wide, and forgetting it there once
    // put all eight writes of a word on one address -- a picture whose every
    // word was its own most significant byte.)
    wire [15:0]  flush_addr = (pal_slot
        ? (16'({win_pal, 8'b0}) + 16'((flush_idx - 10'(VIS_WORDS)) * 8))
        : (16'({win_vram[3:0], 12'b0}) + 16'(flush_idx * 8)))
        + (half ? 16'd4 : 16'd0);
    logic        load_next;   // the next request is a load, not a save
    logic        err_q;

    // The reset that a load ends with, long enough for every part of the
    // machine to have re-read its register (the 6502's reset branch, the APU's,
    // the frame counter): 16 machine clocks is 2.5 us against the 7 cycles a
    // 2A03 needs and the 7 the tape-out 6502 needs.
    logic [4:0] reset_cnt;
    localparam logic [4:0] RESET_CLOCKS = 5'd15;

    // Word and HALF counters in one place: the file is walked as
    // `word_idx * 8 + half * 4` bytes of RAM once the registers are behind us.
    // Four bytes at a time, and both windows are multiples of eight bytes, so a
    // state word never straddles one.
    wire [15:0] ram_off  = 16'((word_idx - 14'(RAM_FIRST)) * 8 + (half ? 16'd4 : 16'd0));
    wire [ 9:0] reg_idx  = 10'(word_idx - 14'(REG_FIRST));

    always_ff @(posedge clk) begin
        // Defaults: every strobe in this block is one clock wide.
        save_wr   <= 1'b0;
        ram_rden  <= 1'b0;
        ram_wren  <= 1'b0;
        load_done <= 1'b0;

        /* The visible window's read, presented EVERY cycle -- unconditional on
         * purpose. See THE BUFFER'S READ IS REGISTERED: a registered address
         * and a registered output are what make `win_buf` a block RAM; gating
         * the read into the flush state would not change that, and the flush's
         * own fetch state is what keeps the timing right. */
        win_q <= win_buf[flush_idx];

        if (reset) begin
            state   <= ST_IDLE;
            sleep   <= 1'b0;
            busy    <= 1'b0;
            err_q   <= 1'b0;
        end
        else if (abort_req) begin
            /* The host gave up on a stream this engine was driving (or was
             * still waiting for words of). The machine is handed straight
             * back: nothing is undone -- a state that was half-written stays
             * half-written -- but the CPU runs again, which is the property
             * that matters (a frozen machine is indistinguishable from a dead
             * core, and the user's only way out would be to reload the
             * cartridge). `load_done` is NOT pulsed, so the machine is never
             * told a restore completed, and `load_err` is left alone: the
             * controller has already reported the failure to the OS. */
            state     <= ST_IDLE;
            sleep     <= 1'b0;
            busy      <= 1'b0;
            err_q     <= 1'b0;
            load_next <= 1'b0;
        end
        else case (state)

        /* ---- nothing in flight ---- */
        ST_IDLE: begin
            sleep   <= 1'b0;
            busy    <= 1'b0;
            load_next <= 1'b0;
            if (save_req) begin
                state   <= ST_WAIT_PAUSE;
                sleep   <= 1'b1;
                busy    <= 1'b1;
                word_idx <= 14'd0;
                half     <= 1'b0;
            end
            else if (load_req) begin
                state   <= ST_WAIT_PAUSE;
                sleep   <= 1'b1;
                busy    <= 1'b1;
                load_next <= 1'b1;
                err_q   <= 1'b0;
            end
        end

        /* ---- the engine's one precondition: a stopped machine ---- */
        ST_WAIT_PAUSE: begin
            if (paused) begin
                state <= load_next ? ST_LOAD_HEADER : ST_SAVE_HEADER;
                // The register bus is walked from zero on both paths, and the
                // first RAM byte is byte 0 of the first RAM word.
                reg_adr  <= 10'd0;
                half     <= 1'b0;
                // The pair the picture in progress is drawn with, taken now:
                // the copy about to run is what rewrites $0100/$0101, and the
                // video holds its own pair for the length of the transfer.
                if (load_next) begin
                    win_vram <= vis_vram_bank;
                    win_pal  <= vis_pal_bank;
                end
            end
        end

        /* ---------------- save ---------------- */
        ST_SAVE_HEADER: begin
            if (!save_full) begin
                save_data <= {STATESIZE, HEADER};
                save_wr   <= 1'b1;
                state     <= ST_SAVE_REGS;
                word_idx  <= 14'(REG_FIRST);
                reg_adr   <= 10'd0;
            end
        end

        ST_SAVE_REGS: begin
            if (!save_full) begin
                save_data <= reg_dout;
                save_wr   <= 1'b1;
                if (reg_idx == 10'(REGS - 1)) begin
                    state    <= ST_SAVE_RAM_READ;
                    word_idx <= 14'(RAM_FIRST);
                    half     <= 1'b0;
                end
                else begin
                    reg_adr  <= reg_adr + 10'd1;
                    word_idx <= word_idx + 14'd1;
                end
            end
        end

        /* FOUR BYTES A CLOCK (2026-10-18). Present this half's address and take
         * the half asked for one clock earlier -- the array's address register
         * is inside the block, so a word comes back the clock after the address
         * that asked for it, and presenting an address and taking its data in
         * the same step would hand back the PREVIOUS word. The `half` flag is
         * the pipeline: with it low this is the prologue (present the low half,
         * nothing to take yet); from then on it is high and every one of these
         * clocks takes the low half of the word whose high half is going out.
         *
         * A state word therefore costs TWO clocks, four bytes each: 16,384
         * clocks for the 64 KB, 2.6 ms, which is the RAM's own floor (spec
         * 21.4). It was eight clocks and 10.2 ms while port A was byte-wide. */
        ST_SAVE_RAM_READ: begin
            ram_addr <= ram_off;
            ram_rden <= 1'b1;
            if (half) begin
                word_buf[31:0] <= ram_rdata;   // the low half asked for last clock
                state          <= ST_SAVE_RAM_TAIL;
            end
            else
                half <= 1'b1;
        end

        /* THE HIGH HALF ARRIVES HERE, AND THE WORD IS PUSHED IN THE SAME CLOCK.
         *
         * The word is complete without a register to hold it: the low half is in
         * `word_buf` and the high half is the word the port has just returned,
         * so the push is a concatenation -- and the next word's low half can be
         * presented in the same clock, which is what keeps the port from idling
         * between words. Two clocks per word, four bytes per clock.
         *
         * A FULL FIFO retries here. `ram_addr` is re-presented with this word's
         * high half on it, so `ram_rdata` keeps holding that half: the same
         * state is simply repeated until the host has made room, and no word is
         * lost or duplicated. (The low half is already in `word_buf` and is not
         * re-read.) */
        ST_SAVE_RAM_TAIL: begin
            if (!save_full) begin
                save_data <= {ram_rdata, word_buf[31:0]};
                save_wr   <= 1'b1;
                ram_rden  <= 1'b1;
                if (word_idx == 14'(WORDS - 1)) begin
                    state <= ST_FINISH;
                end
                else begin
                    word_idx <= word_idx + 14'd1;
                    ram_addr <= ram_off + 16'd4;   // the next word's low half
                    state    <= ST_SAVE_RAM_READ;  // ... and it is in flight
                end
            end
            else begin
                // Hold this word's high half so the push above can be retried
                // from `ram_rdata` without touching the array again.
                ram_addr <= ram_off;
                ram_rden <= 1'b1;
            end
        end

        /* ---------------- load ---------------- */
        ST_LOAD_HEADER: begin
            if (load_ok) begin
                if (load_data[63:32] != STATESIZE) begin
                    // Not this build's file. The framework's engine drops the
                    // whole transfer here without touching the machine, and so
                    // does this one -- but it says so on `load_err`, which the
                    // APF handshake passes on to the OS.
                    err_q <= 1'b1;
                    state <= ST_FINISH;
                end
                else begin
                    word_idx <= 14'(REG_FIRST);
                    reg_adr  <= 10'd0;
                    state    <= ST_LOAD_REGS;
                end
            end
        end

        ST_LOAD_REGS: begin
            if (load_ok) begin
                if (reg_idx == 10'(REGS - 1)) begin
                    /* THE RAM COPY STARTS IMMEDIATELY (2026-10-18). It used to
                     * wait for the picture's frame boundary, because the whole
                     * copy went under the raster and the panel had to be blanked
                     * for it. Nothing here needs that any more: the only bytes
                     * a picture is drawn from are held back for the blanking
                     * (THE VISIBLE WINDOW), so the fastest safe copy is the one
                     * that starts now. */
                    state    <= ST_LOAD_RAM_FAST;
                    word_idx <= 14'(RAM_FIRST);
                    half     <= 1'b0;
                end
                else begin
                    reg_adr  <= reg_adr + 10'd1;
                    word_idx <= word_idx + 14'd1;
                end
            end
        end

        // Take the next word and decide where it goes. Ordinary RAM words go
        // through ST_LOAD_RAM_WRITE (two 32-bit writes, with the next word fed
        // in during the high half); the two visible windows go to win_buf and
        // are written later, in the raster's blanking.
        ST_LOAD_RAM_FAST: begin
            if (load_ok) begin
                word_buf <= load_data;
                half     <= 1'b0;
                state    <= win_wr ? ST_LOAD_BUF_STORE : ST_LOAD_RAM_WRITE;
            end
        end

        // One word in, FOUR BYTES OUT per clock -- the reverse of the save's
        // gather, with the half written in the cycle the address and data are
        // presented (port A latches them mid-cycle). Two clocks to the word.
        ST_LOAD_RAM_WRITE: begin
            ram_addr  <= ram_off;
            ram_wdata <= word_buf[half*32 +: 32];
            ram_wren  <= 1'b1;
            if (half) begin
                half <= 1'b0;
                if (word_idx == 14'(WORDS - 1))
                    // Every byte has a home: the ordinary ones are in RAM and
                    // the visible ones are parked. Hand the window over.
                    state <= ST_LOAD_WIN_WAIT;
                else if (load_ok) begin
                    // The next word is here and was taken this cycle
                    // (`load_ok` is `load_feed && !load_empty`): keep writing,
                    // there is no gap between the words. `word_buf` is
                    // non-blocking, so the half written above is still this
                    // word's own high half, and `ram_addr`/`ram_wdata` below
                    // are the low half of the word just taken.
                    word_buf <= load_data;
                    word_idx <= word_idx + 14'd1;
                end
                else begin
                    word_idx <= word_idx + 14'd1;
                    state    <= ST_LOAD_RAM_FAST;
                end
            end
            else
                half <= 1'b1;
        end

        // A word the picture is drawn from: parked, not written. Costs two
        // cycles (this and the fetch), which is the price of not having to
        // stop the machine for a frame to get the picture out of the way.
        ST_LOAD_BUF_STORE: begin
            win_buf[win_ix] <= word_buf;
            if (word_idx == 14'(WORDS - 1))
                state <= ST_LOAD_WIN_WAIT;
            else begin
                word_idx <= word_idx + 14'd1;
                state    <= ST_LOAD_RAM_FAST;
            end
        end

        /* ---- the visible window, in the raster's blanking ------------------
         *
         * Nothing is being scanned here, and the palette reload and the bank
         * latch are still ~150 lines away, so this is the one place a picture's
         * own bytes can change under the panel and not be caught. `flushing`
         * tells mc64_video exactly that, and it is what the picture hold keys
         * off now (a late flush is the only way a load can still blank).
         */
        ST_LOAD_WIN_WAIT: begin
            if (blank_slot) begin
                flush_idx <= 10'd0;
                half      <= 1'b0;
                state     <= ST_LOAD_WIN_FETCH;
            end
        end

        // One cycle for the block's own read to land in `win_q` (the array's
        // read is `win_q <= win_buf[flush_idx]`, above this case). A word costs
        // three cycles rather than two (fetch, low half, high half) -- 608 words
        // is 1,824 clk_core, about 4.3 of the 158 blank lines the flush is
        // given, and 4,864 bytes at four bytes a clock is the port's own floor.
        ST_LOAD_WIN_FETCH: begin
            state <= ST_LOAD_WIN_WRITE;
        end

        // The word being flushed is in `win_q`, not read out of the array in
        // this cycle: see THE BUFFER'S READ IS REGISTERED. A slot the frame
        // buffer's own part of the flush already covers is walked through
        // without a write (see EVERY SLOT IS WALKED).
        ST_LOAD_WIN_WRITE: begin
            if (!slot_taken) begin
                ram_addr  <= flush_addr;
                ram_wdata <= win_q[half*32 +: 32];
                ram_wren  <= 1'b1;
            end
            if (half) begin
                half <= 1'b0;
                if (flush_idx == flush_last) begin
                    // The last byte of the state is in. Everything the machine
                    // reads back through its own reset path -- the 6502's, the
                    // APU's -- is now in the register bus, so it is reset for
                    // a few clocks to pick the state up; the two things a
                    // reset cannot carry (the CPU's PC/registers and the
                    // machine's counters) are written by mc64_top on
                    // `load_done`, after this.
                    state     <= ST_LOAD_RESET;
                    reset_cnt <= 5'd0;
                end
                else begin
                    flush_idx <= flush_idx + 10'd1;
                    state     <= ST_LOAD_WIN_FETCH;
                end
            end
            else
                half <= 1'b1;
        end

        ST_LOAD_RESET: begin
            reset_ss  <= 1'b1;
            reset_cnt <= reset_cnt + 5'd1;
            if (reset_cnt == RESET_CLOCKS) begin
                reset_ss <= 1'b0;
                state    <= ST_FINISH;
            end
        end

        /* ---- done: hand the machine back ---- */
        ST_FINISH: begin
            if (!err_q)
                load_done <= load_next;
            state     <= ST_IDLE;
            sleep     <= 1'b0;
            busy      <= 1'b0;
        end

        default: state <= ST_IDLE;

        endcase
    end

    /* ------------------------------------------------------------------ *
     * Wireouts
     * ------------------------------------------------------------------ */
    /* ------------------------------------------------------------------ *
     * reg_rst: the framework's "restore defaults" pulse
     *
     * It is asserted BEFORE a load's own words arrive (the whole of
     * ST_LOAD_HEADER, which cannot collide with ST_LOAD_REGS' first write), so a
     * register the file does not cover falls back to its power-on value rather
     * than keeping whatever the previous state left in it. A platform reset does
     * the same thing, for the same reason.
     *
     * It is NOT the machine reset a load ends with. That reset is the moment the
     * APU's channels read their restored values back out of their state
     * registers, and the CPU's restored word is read one cycle after it: driving
     * these registers to their defaults there erases the state a load just
     * wrote -- the load would then set PC to $0000 and A/X/Y/S to zero, and the
     * machine would resume in the zero page instead of where it was stopped.
     * ------------------------------------------------------------------ */
    assign reg_rst  = reset | (state == ST_LOAD_HEADER);
    assign reg_din  = load_data;                         // the word in hand
    assign reg_wren = (state == ST_LOAD_REGS) && load_ok;


    assign load_err = err_q;
    assign loading  = busy & load_next;
    /* The one thing mc64_video blanks on now: the visible window being written.
     * For a normal load this is 4,864 cycles inside the blanking, i.e. never
     * while a picture is on the screen; see the port note there. */
    assign flushing = (state == ST_LOAD_WIN_WRITE);

endmodule
