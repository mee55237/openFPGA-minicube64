/*
 * save_state_controller -- the APF end of the save-state path.
 *
 * WHAT THE OPERATING SYSTEM ASKS FOR, AND WHAT IT SEES.
 *
 * A core that reports `savestate_supported` = 1 must answer two host commands
 * (rtl/core_bridge_cmd.v, commands 0x00A0 and 0x00A4):
 *
 *   0x00A0 "Savestate: Start/Query"  the OS reads back supported, the buffer's
 *          address and its size, then -- with host_20[0] = 1 -- asks for a
 *          state to be taken. It then READS `size` bytes from the buffer
 *          address and stores them itself.
 *
 *   0x00A4 "Savestate: Load/Query"   same query, then -- host_20[0] = 1 --
 *          asks for a state to be restored, having written the file's bytes
 *          into the buffer address first.
 *
 * So the core is not asked to own a file, a slot or an SD card: it is asked to
 * present `size` bytes at a byte address in the host's own memory, and to take
 * `size` bytes back from it. The address is this core's declaration
 * (`savestate_addr` = 0x40000000 in core_top, matching the NES core's own
 * choice), the size is mc64_savestates' 66056 bytes, and the bridge hands the
 * transfers over as reads and writes carrying `bridge_addr[31:28] == 4'h4` --
 * one 32-bit word per access, little-endian inside the word (the first byte of
 * the file is in bits [7:0], which is what bridge_endian_little = 1 means for
 * the cartridge path too).
 *
 * WHY THE FIFOS ARE OURS AND NOT dcfifo. See rtl/mc64_async_fifo.sv: the
 * APF framework's streamer modules are built on Altera's mixed-width dcfifo,
 * and this tree has never carried a megafunction it could replace with twenty
 * lines. One per direction, both 64 bits wide (the engine's word):
 *
 *   save: written in the machine domain by mc64_savestates, read in clk_74a to
 *         answer the host's reads. The engine makes a word every ~8 machine
 *         cycles; the host's ceiling is one 32-bit read every 88 clk_74a cycles
 *         (io_bridge_peripheral's own number), i.e. two reads -- one word --
 *         per ~22 machine cycles. THE ENGINE IS THE FASTER SIDE BY ~2.7x, and
 *         that is the whole reason `fifo_save` is only 256 entries of
 *         CROSSING: behind it is `mc64_state_store`, the whole state's worth
 *         of words, so the copy finishes at the RAM port's rate and the host
 *         reads afterwards instead of the machine waiting for it (2026-10-18;
 *         the panel's *saves take forever*).
 *
 *   load: written in clk_74a from the host's writes, read by the engine. Here
 *         the host is the slower side by the same ratio, so the FIFO absorbs
 *         the host's lead and the engine drains it. The engine is started by
 *         the COMMAND, not by the first word of the stream -- the buffer holds
 *         the whole blob, so nothing overruns however fast the host writes, and
 *         the machine is therefore not frozen for the host's own file work (see
 *         THE FREEZE STARTS AT THE COMMAND below).
 *
 * THE HALF-WORD BOOKKEEPING, and why a repeat access must not consume anything.
 * Both directions move 32-bit host words and 64-bit engine words, and the
 * half-select is the ADDRESS's bit 2 -- the file is walked in order, so the low
 * half of a word is at bit 2 clear and the high half at bit 2 set, and nothing
 * has to be counted to know which one an access wants. On the read side the low
 * half of a NEW word is answered straight from the FIFO's head, in the access's
 * own cycle (the answer is sampled in that cycle: core_top presents it
 * combinationally, and it must therefore already be the right half), and the
 * word is copied into a holding register and popped in that same cycle so the
 * high half -- and any repeat of either half -- is answered from the register
 * and consumes nothing. A host that re-reads a word (Agg23's controller guards
 * against exactly that) therefore cannot shift its own file. On the write side
 * the mirror image: the low half is kept, the high half closes the word and
 * pushes it, and a repeated write address is dropped rather than pushed twice.
 *
 * ERROR REPORTING. `busy`/`ok`/`err` are what the OS reads back as its result
 * code, so both directions carry a watchdog: if the machine never comes to a
 * stop (no cartridge released, so no instruction boundary ever arrives) the
 * command is answered `err` rather than left hanging forever. The engine's own
 * refusal to load a file whose header tag is not this build's raises
 * `load_err` the same way. A read that runs ahead of the engine, or a write the
 * FIFO could not take, is counted in `underrun`/`overrun`, which nothing on the
 * panel can display but a testbench can assert on -- and which a *silently*
 * short state would otherwise leave invisible, the failure mode this project
 * has already paid for once (the cartridge loader's dropped words).
 */
`default_nettype none
`timescale 1ns / 1ps

module save_state_controller #(
    // A command that has not produced or consumed anything in this many
    // clk_74a cycles is answered err: 2^23 / 74.25 MHz = 113 ms, against a
    // whole state's ~40 ms in either direction.
    parameter int WATCHDOG_BITS = 24,
    // THE LOAD'S STAGING BUFFER, in 64-bit words. It must hold the whole blob the
    // host is allowed to hand over, because the host CANNOT BE PACED: an APF
    // bridge write has no backpressure, so whatever the host outruns has to fit.
    // core_top passes `savestate_maxloadsize`/8 here, so the declaration of what
    // the OS may send and the buffer that has to take it cannot drift apart.
    // 8769 x 64 bits = 561 Kbit = 56 M10K blocks on the 5CEBA4 (which the fit is
    // 18% and 71/308 blocks into before this).
    parameter int STAGE_WORDS = 8769,
    /* The engine's format tag (its STATESIZE), so this module can tell the
     * host's two 32-bit halves apart on the first word of a stream. See THE
     * HOST'S HALF ORDER below. */
    parameter logic [31:0] TAG = 32'h0001_0200
) (
    // ---- the host side: the APF bridge, in clk_74a ----
    input  logic        clk_74a,
    input  logic        reset_74a,      // clk_74a-domain reset for the FIFOs
    input  logic        bridge_wr,
    input  logic        bridge_rd,
    input  logic [31:0] bridge_addr,
    input  logic [31:0] bridge_wr_data,
    output logic [31:0] ss_rd_data,     // combinational: valid in the read's own cycle

    // ---- the APF save-state handshake, in clk_74a ----
    input  logic        ss_start,
    output logic        ss_start_ack,
    output logic        ss_start_busy,
    output logic        ss_start_ok,
    output logic        ss_start_err,
    input  logic        ss_load,
    output logic        ss_load_ack,
    output logic        ss_load_busy,
    output logic        ss_load_ok,
    output logic        ss_load_err,

    // ---- the machine side, in clk_core ----
    input  logic        clk_core,
    input  logic        reset_core,
    output logic        save_tog,       // one toggle per request, crossed by the engine
    output logic        load_tog,
    output logic        abort_tog,      // "drop it": the machine must not be left frozen
    input  logic        ss_busy,        // the engine is mid-state (clk_core)
    input  logic        ss_load_done,   // one pulse: a state was restored (clk_core)
    input  logic        ss_engine_err,  // ... or refused (clk_core)

    // ---- the engine's ends of the two FIFOs (clk_core) ----
    // The save FIFO is written from that side and read from this one, so its
    // write strobe/data come in and its full flag goes out; the load FIFO is
    // the mirror image.
    input  logic [63:0] ss_save_data,   // the engine's word, to push
    input  logic        ss_save_wr,
    output logic        ss_save_full,   // ... and whether it fits
    input  logic        ss_load_rd,     // the engine taking the next word
    output logic [63:0] ss_load_data,   // ... the word it gets
    output logic        ss_load_empty,

    // ---- diagnostics, for the harness and the fit's timing report ----
    output logic        underrun,
    output logic        overrun
);

    /* ================================================================== *
     * The two crossings, one FIFO each
     * ================================================================== */
    wire [63:0] save_q;
    wire        save_empty;
    logic       save_rd;



    /* ---- flushing the FIFOs between sessions --------------------------
     *
     * A load that is refused (a file whose header tag is not this build's)
     * leaves the host's stream sitting in the load FIFO, and a save the OS
     * stopped reading leaves the engine's words in the save FIFO. Neither may
     * seed the next transfer, so each is emptied before a command starts one.
     *
     * The write pointer belongs to one clock domain and the read pointer to
     * the other, so a flush is one pointer reset in each: the request toggles
     * in clk_74a and the machine side edge-detects it, exactly like the two
     * state requests (see the header). A flush is only ever asked for with the
     * engine idle, so no half-consumed word can be thrown away by one.
     */
    // Only the SAVE side needs one now: the load side's stream boundary is the
    // word count (word 0 sets it to zero), which is a reset in the writer's own
    // domain and needs no crossing at all -- see the staging note below.
    logic        save_flush_tog = 1'b0;
    logic        save_flush_74a;                      // this domain's own pulse
    logic [2:0]  save_flush_s = 3'd0;

    always_ff @(posedge clk_core) begin
        save_flush_s <= {save_flush_s[1:0], save_flush_tog};
    end

    wire save_flush_core = save_flush_s[2] ^ save_flush_s[1];

    /* ---- the save side: the crossing, then a staging buffer -------------
     *
     * THE ENGINE MUST NOT WAIT FOR THE HOST (2026-10-18). It used to: the FIFO
     * below was the whole buffer, the engine produces a word every ~8 machine
     * clocks and the host reads one every ~22 (two 32-bit accesses at its
     * 88-cycle ceiling), so 256 entries filled within a few hundred words and
     * the rest of the state was copied at the HOST's pace -- with the machine
     * stopped, because a state is taken with the CPU held. The panel's *saves
     * work, but they take forever* was that 40 ms; the copy is 10.
     *
     * So the FIFO is now only the crossing (its 256 entries cover the few
     * cycles between the two clocks) and `mc64_state_store` behind it is the
     * buffer the whole state fits in, drained at a word per clk_74a cycle. The
     * engine never sees `save_full`, its copy runs at the RAM port's own rate,
     * and the machine is handed back while the host is still reading the words
     * out. See rtl/mc64_state_store.sv for why the depth could not simply go
     * into the FIFO (its read is combinational, and an M10K's is not).
     */
    wire [63:0] fifo_q;
    wire        fifo_empty;
    logic       fifo_rd;

    mc64_async_fifo #(.WIDTH(64), .AW(8)) fifo_save (
        .wclk   (clk_core),
        .wrst   (reset_core | save_flush_core),
        .wr     (ss_save_wr),
        .wdata  (ss_save_data),
        .wfull  (ss_save_full),
        .wcount (),
        .rclk   (clk_74a),
        .rrst   (reset_74a | save_flush_74a),
        .rd     (fifo_rd),
        .rdata  (fifo_q),
        .rempty (fifo_empty),
        .rcount ()
    );

    // The state is 8,257 words (header + registers + 64 KB); 8,192 of them fit
    // here and the last 65 ride out on the host's own reads (~1,400 machine
    // clocks, 1% of the copy). A power of two keeps the store's pointer wrap
    // a drop of the top bit.
    localparam int SAVE_STORE_WORDS = 8192;

    logic        save_store_full;

    /* THE DRAIN WAITS FOR THE FLUSH TO SETTLE (2026-10-18). `rempty` is this
     * side's view of a pointer pair, and the WRITE pointer reaches it through
     * two flops: in the first few clk_74a cycles after a flush its synchronised
     * copy still reads as the previous transfer's write pointer, so a drain that
     * started at once saw a non-empty FIFO and popped one of the OLD words into
     * the store -- before the engine's own stream had even begun. The host then
     * read that stale word as word 0 and every word after it was misaligned by
     * one, i.e. a save that reads back as zeros with the same shape as a
     * transfer that never happened.
     *
     * So the drain is held off for 31 clk_74a cycles after the flush -- far more
     * than the two flops need, and far less than the frame the engine waits for
     * before it can produce its first word. Nothing else uses the count. */
    logic [4:0] save_flush_cnt = 5'd0;

    assign fifo_rd = !fifo_empty && !save_store_full && (save_flush_cnt == 5'd0);

    /* `save_q`/`save_empty`/`save_rd` keep their names, so the host's whole
     * read path -- the address key, the held word, the underrun report -- is
     * unchanged; only what is behind them changed. `save_flush_74a` drops the
     * store with the FIFO, so a new save starts at its own word 0. */
    mc64_state_store #(.WORDS(SAVE_STORE_WORDS), .WIDTH(64)) store_save (
        .clk   (clk_74a),
        .reset (reset_74a | save_flush_74a),
        .wr    (fifo_rd),
        .wdata (fifo_q),
        .full  (save_store_full),
        .rd    (save_rd),
        .rdata (save_q),
        .empty (save_empty)
    );

    /* ---- the load side's staging buffer ---------------------------------
     *
     * THIS REPLACED A 256-ENTRY FIFO (2026-10-17), and why is the whole point of
     * the change. A load's data comes from the HOST, and the APF's bridge cannot
     * be paced for a write: there is no signal the core can hold to make the
     * host wait. So the core has to absorb whatever the host sends, and the
     * arithmetic here is brutal:
     *
     *   the blob is 66 KB (8257 words), a burst host can send one 32-bit word per
     *   bridge cycle, and the MACHINE can only take a 64-bit word every nine
     *   machine clocks -- 8 bytes, one per clock, through the RAM's byte-wide
     *   port A -- i.e. 5.7 MB/s. A memcpy into the window is five to fifty times
     *   faster than that.
     *
     * 2 KB of FIFO therefore overflowed within microseconds of the host starting
     * (measured, with an unpaced copy-in in sim/tb_savestate.cpp: `overrun 1`, the
     * command left at result code 1, the state half-applied), and the words the
     * overflow dropped are words the engine then waited for for ever: the
     * transfer never finished and the panel's OS reported what it saw, *the OS
     * lets you save and gives an error when you load*. The SAVE direction has
     * always been safe from this -- there the engine is the producer and the FIFO
     * is the throttle -- which is exactly why the two directions behaved
     * differently on the panel.
     *
     * So the load side stages the WHOLE blob (spec 21.4). The host may write as
     * fast as the bridge takes it; the engine drains at its own rate and the only
     * thing that can now fail is a blob larger than `STAGE_WORDS`, which is a
     * capacity statement (the same one `savestate_maxloadsize` makes to the OS to
     * begin with) rather than a race, and it is reported as an error rather than
     * dropping words silently.
     *
     * The two sides are described by COUNTS, not by entries: the host's words are
     * CONTIGUOUS from word 0 (it is a memcpy), so the writer keeps `wr_count` --
     * the number of whole words it has taken -- and the reader keeps its own
     * `rd_count`, and `ss_load_empty` is simply `rd_count == wr_count` (crossed).
     * That is what makes a new stream safe with no flush: word 0 of a stream sets
     * `wr_count` back to 0, so the reader can never see the previous transfer's
     * leftovers no matter what is still sitting in the buffer. */
    logic [63:0] load_stage [0:STAGE_WORDS-1];

    localparam int STAGE_AW = $clog2(STAGE_WORDS);   // 14 for 8769

    logic [STAGE_AW:0] wr_count = '0;      // words taken from the host (clk_74a)
    logic [STAGE_AW:0] wr_gray  = '0;
    logic [STAGE_AW:0] wr_gray_s1 = '0, wr_gray_s2 = '0;
    logic [STAGE_AW:0] rd_count = '0;      // words the engine has taken (clk_core)
    logic [STAGE_AW:0] wr_count_r = '0;    // ... and the host's count, this domain

    wire [STAGE_AW:0] wr_count_n = wr_count + 1'b1;

    /* The count itself is written in the host-side always block below (one
     * driver per register -- the fit rejects two), so what is left here is the
     * crossing and the reader's own end of it. */
    always_ff @(posedge clk_core) begin
        if (reset_core) begin
            wr_gray_s1 <= '0;
            wr_gray_s2 <= '0;
        end else begin
            wr_gray_s1 <= wr_gray;
            wr_gray_s2 <= wr_gray_s1;
        end
    end

    /* THE READER'S COUNT, AND WHY IT NEEDS NO CROSSING OF ITS OWN. It is the
     * writer's count that says where a stream starts: word 0 sets it back to
     * zero, and the reader's count -- which can never be ahead of it while a
     * transfer runs, because the reader stops at it -- is then GREATER than the
     * count it can see, which is the signal that a new stream has begun. So the
     * reset condition is expressed entirely in this domain: no extra synchroniser
     * and no race with the engine about which side notices first. */
    always_ff @(posedge clk_core) begin
        if (reset_core)
            rd_count <= '0;
        else if (rd_count > wr_count_r)
            rd_count <= '0;              // the host's count went backwards: new stream
        else if (ss_load_rd)
            rd_count <= rd_count + 1'b1;
    end

    /* Gray back to binary, one running XOR from the top down -- the same
     * conversion mc64_async_fifo does, because it is the same crossing. */
    always_comb begin
        wr_count_r[STAGE_AW] = wr_gray_s2[STAGE_AW];
        for (int i = STAGE_AW - 1; i >= 0; i--)
            wr_count_r[i] = wr_count_r[i+1] ^ wr_gray_s2[i];
    end

    assign ss_load_data  = load_stage[rd_count[STAGE_AW-1:0]];
    /* `>=` and not `==`: in the cycle or two after a new stream's word 0 the
     * reader's count is still ahead of the crossed one, and that window must read
     * as EMPTY (nothing is available) rather than as "there is a word" -- which
     * would hand the engine an entry from the previous transfer. */
    assign ss_load_empty = (rd_count >= wr_count_r);

    /* ================================================================== *
     * The host side: the buffer, in clk_74a
     * ================================================================== */

    // The state buffer is the 0x4 window; the address in 32-bit words is also
    // the duplicate guard's key.
    wire        rd_hit = bridge_rd && (bridge_addr[31:28] == 4'h4);
    wire        wr_hit = bridge_wr && (bridge_addr[31:28] == 4'h4);
    wire [25:0] acc_addr = bridge_addr[27:2];

    /* Which half of the 64-bit word a 32-bit access is asking for is in the
     * ADDRESS, not in a counter: the buffer is walked in order, so the low half
     * is at BYTE bit 2 clear and the high half at byte bit 2 set. (Byte bit 2,
     * not a bit of `acc_addr`: that is the address in 32-bit words, so its bit
     * 0 is the same thing -- writing it as `acc_addr[2]` reads byte bit 4 and
     * silently swaps every pair of halves.)
     *
     * THE KEY IS THE 64-BIT WORD, NOT THE HALF (2026-10-17). It used to be the
     * half's own address, updated on EVERY read -- so a host that read a word's
     * high half and then its low half again looked like the start of a NEW word:
     * the engine would be asked for the next word and a stale one would answer
     * the address, i.e. the file the host collected would be shifted, and a
     * state whose header is not at byte 0 is refused on the way back in exactly
     * as a foreign file is -- "saves do not load", with the transfer itself
     * having reported ok. Agg23's controller cannot shift this way, because its
     * guard compares the word address and so treats any repeat of either half as
     * a repeat; this is the same rule. Nothing else changes: a word is still
     * taken on its LOW half, held, and every access to it -- either half, any
     * number of times -- is answered from the hold and consumes nothing. */
    logic [63:0] rd_hold = 64'd0;  // the word in hand (both halves are read from it)
    wire         rd_high  = bridge_addr[2];
    // The 64-bit word's index in the buffer: the half's own bit (byte bit 2) is
    // dropped, so both halves of one word carry the same key.
    wire [24:0]  acc_word = acc_addr[25:1];
    wire         rd_new   = rd_hit && !rd_high && (acc_word != last_rd_word);
    logic [24:0] last_rd_word = 25'h1FF_FFFF;
    logic [31:0] load_lo = 32'd0;   // the low half of the word in hand

    logic        prev_start = 1'b0;
    logic        start_go   = 1'b0;   // a save request waiting for the engine
    logic        save_tog_q = 1'b0;
    logic        load_tog_q = 1'b0;
    logic        abort_tog_q = 1'b0;
    logic        load_data_seen = 1'b0;   // a word of THIS stream has arrived
    logic        underrun_q = 1'b0;
    logic        overrun_q  = 1'b0;
    logic        swapped    = 1'b0;       // this stream's halves are the host's way round
    /* ... decided on word 0 and held for the stream. A wire, not a variable
     * assigned in the block: the value is a function of this cycle's address,
     * the half already in hand and the count, and Verilator's BLKSEQ rule (which
     * this tree treats as an error) is right to object to the other spelling. */
    wire         swap_word  = (wr_count == '0)
                            ? ((load_lo == TAG) && (bridge_wr_data != TAG))
                            : swapped;

    assign save_tog = save_tog_q;
    assign load_tog = load_tog_q;
    assign abort_tog = abort_tog_q;
    assign underrun = underrun_q;
    assign overrun  = overrun_q;

    // ---- the engine's status, crossed back into clk_74a ----
    logic [2:0] busy_s = 3'd0;
    logic [2:0] done_s = 3'd0;
    logic [2:0] err_s  = 3'd0;

    always_ff @(posedge clk_74a) begin
        busy_s <= {busy_s[1:0], ss_busy};
        done_s <= {done_s[1:0], ss_load_done};
        err_s  <= {err_s[1:0],  ss_engine_err};
    end

    wire engine_busy   = busy_s[2];
    wire load_done_now = done_s[2] ^ done_s[1];   // the pulse, edge-detected
    // The engine's refusal is a LEVEL (its error flag stays set until the next
    // request), so it is edge-detected here the way the completion is, and it
    // answers the command that is in flight when it arrives.
    wire engine_err_now = err_s[2] ^ err_s[1];

    wire ss_load_rising = ss_load & ~ss_load_prev;

    /* THE FREEZE STARTS AT THE COMMAND, NOT AT THE FIRST WORD (2026-10-18).
     *
     * `stream_started` is set when a stream begins at the buffer's first word,
     * `load_cmd_pending` by command 0x00A4, and the engine is armed when BOTH
     * are in hand. IT USED TO ARM ON THE FIRST WORD ALONE, and that is what the
     * panel reported as *saves work but it takes forever to load, and while it
     * loads any user would think it is corrupted*:
     *
     *   the OS reads the state off the SD card and writes it into the window
     *   word by word, and only THEN commands the load. Arming on the first word
     *   froze the
     *   machine -- and blanked the picture -- at the START of that copy-in, so
     *   the panel sat on a held/blank screen for the whole of the host's SD
     *   read, which is hundreds of milliseconds at best and seconds when the
     *   card is slow. None of that time was this core working: the engine had
     *   already drained the words it had.
     *
     * The staging buffer is what makes waiting legal. It holds the whole blob
     * `savestate_maxloadsize` promises the OS, so nothing overruns however long
     * the host takes and however fast it writes; the engine is started by the
     * command, reads what is already there and finishes the state in the copy's
     * own time (~10 ms, the RAM port's floor). The machine keeps running -- and
     * the picture keeps being drawn -- for the whole of the host's file work, so
     * what a player sees is a short hold (about two frames) at the moment they
     * asked for the load.
     *
     * Both orderings still work, which is why this is an AND and not "the
     * command alone": a host that commands FIRST and streams afterwards has no
     * stream when the command lands, so the arm waits for word 0 and then fires
     * -- exactly as it used to. What is gone is only the case where the data
     * was already in and the command was the last thing to arrive, which is the
     * panel's own ordering and the one that was costing the seconds.
     *
     * A SECOND load in the same session works the same way: its file starts at
     * word 0 like every other stream, which sets `stream_started` again, and the
     * command arms it. Nothing is left over from the previous transfer to be
     * replayed, because a word anywhere but word 0 is never the start of one.
     */
    logic        ss_load_prev = 1'b0;
    logic        load_active = 1'b0;
    logic        load_cmd_pending = 1'b0;   // a 0x00A4 request, waiting for its data
    logic        stream_started   = 1'b0;   // word 0 of a stream has been written
    logic [WATCHDOG_BITS-1:0] start_wd = '0;

    /* THE LOAD'S OWN CLOCK, and the two windows it can fire in.
     *
     * A load transfer HOLDS THE MACHINE STOPPED for as long as it is in
     * flight -- the engine freezes the CPU before it touches a word of it -- so
     * a transfer that never finishes leaves the panel frozen with a
     * half-written state: the picture holds, nothing responds, and the user's
     * only way out is to reload the cartridge. The host is not obligated to
     * finish what it starts (it can time out, its stream can be truncated), so
     * this clock is what answers for it: while a transfer is in flight and no
     * word has arrived for a window, the command is answered err, the engine is
     * told to drop the state, and the machine is handed back running.
     *
     * The window depends on whether the transfer has got started, because the
     * two silences mean different things:
     *
     *   no word yet  -- 0.90 s. The host was asked for the state and is reading
     *                   it off the card; its own notes say it may pause between
     *                   slot transfers (SD read, RTC, menu work), and the
     *                   sibling core records a 300,000-cycle (4 ms) gap between
     *                   two parts of one load. Nothing about this core is
     *                   waiting on it except itself, so it gets a long leash.
     *
     *   words seen   -- 113 ms of silence. A stream that is moving stops for
     *                   microseconds, so a tenth of a second of quiet in the
     *                   middle of one is the host having gone away. 2^23 at
     *                   74.25 MHz, the same shape and order as the save
     *                   command's own watchdog above.
     *
     * Neither window is a timer anyone has to meet: both are only ever reached
     * by a transfer that has stopped permanently. */
     /* BOTH WINDOWS ARE NOW LONG (2026-10-17), because the panel showed what a
     * short one answers for: *the OS lets you save, and gives an error when you
     * load*. The state file is read off the SD card by the OS itself, and how
     * long that takes is the OS's business -- a one-second read is ordinary, and
     * a pause in the middle of the stream is what reading a file in chunks looks
     * like. The window is not there to police the host's speed; it is there so a
     * host that has GONE AWAY cannot leave the machine stopped for ever, and a
     * few seconds of patience costs nothing when the alternative is refusing a
     * file that was still arriving. (Agg23's controller, which runs save states
     * on this panel, has no window at all for this direction.) */
    localparam [29:0] LOAD_STALL_MAX = 30'd134217728; // 2^27 = 1.81 s of silence
    localparam [29:0] LOAD_FIRST_MAX = 30'd536870912; // 2^29 = 7.23 s before the first word

    logic [29:0] load_wd = 30'd0;

    /* WHERE A STREAM BEGINS: at the ADDRESS, not at "data is present".
     *
     * A transfer begins at the buffer's FIRST WORD -- the state's own start,
     * word address 0 -- and a word anywhere else in the window is the tail of a
     * stream that has already been consumed. The host is allowed to send MORE
     * than the state (that is what `savestate_maxloadsize` is for -- a file
     * written by an earlier build, or one the OS padded), so words keep arriving
     * after the engine has finished the state it wanted; those are dead weight,
     * and a rule that armed on any of them would start a transfer whose "header"
     * is the middle of a file -- refused, and reported as this build's error.
     *
     * So word 0 is where `stream_started` is set, and it is what a SECOND load
     * in the same session hangs on too: its file starts at word 0 like every
     * other one. (The count reset below travels with it: word 0 puts the reader
     * back at the beginning, so nothing the previous transfer left in the buffer
     * can be read as part of this state.) */
    wire        load_new_stream   = wr_hit && (bridge_addr[27:3] == 25'd0)
                                    && !bridge_addr[2];

    wire        load_stage_wr = wr_hit && bridge_addr[2] &&
                                (bridge_addr[27:3] == {2'b0, wr_count}) &&
                                (wr_count != STAGE_WORDS[STAGE_AW:0]);

    /* ... AND A LOAD WAITS FOR THE ENGINE THE SAME WAY A SAVE DOES
     * (2026-10-18). `load_tog_q` is the same kind of toggle the save side uses,
     * and the engine's edge detect is a one-cycle pulse it only looks at in
     * ST_IDLE -- so a load commanded while a SAVE was still in flight was
     * consumed and dropped, the transfer never started, and the watchdog
     * answered err on the load. That is one of the shapes of "it won't load":
     * the state was fine and the request never reached the engine. */
    wire load_arm = !load_active && load_cmd_pending && stream_started &&
                    !engine_busy;

    // A transfer is in flight if the engine is driving one or the host is
    // waiting on one. Both are watched: the OS streams before it commands (the
    // arm above waits for it), and a host that commands first waits for data
    // with this clock running against it.
    wire        load_inflight = load_active | ss_load_busy;
    wire [29:0] load_limit    = load_data_seen ? LOAD_STALL_MAX : LOAD_FIRST_MAX;
    /* THE ENGINE IS BUSY WITH A TRANSFER THIS COMMAND DID NOT ARM -- a SAVE that
     * was in flight when the load was commanded (the mirror of the save side's
     * note below). The same rule applies: a watchdog is a command's patience
     * with ITS OWN transfer, and `abort_tog` stops whatever the engine is doing.
     * So this command's clock does not run while the engine is another's, and
     * its abort cannot fire in that state. */
    wire        engine_on_another = engine_busy && !load_active;
    wire        load_stalled  = load_inflight && !engine_on_another &&
                                (load_wd >= load_limit);

    /* ANOTHER COMMAND WHILE ONE IS ALREADY RUNNING IS DROPPED (2026-10-19).
     *
     * The panel's rule, and it is the whole of the command arbitration now: a
     * save or load that arrives while a transfer is ALREADY RUNNING does not
     * start a second one. It is acknowledged (the host's handshake requires
     * that) and then it is answered, not performed -- silently, with no error,
     * because nothing went wrong: the machine is simply busy.
     *
     * Why not the two things this used to do instead:
     *
     *   QUEUE IT (`start_go`, and `load_cmd_pending` waiting for the engine).
     *   A second command then ran against a machine the first had just moved.
     *   A save queued behind a load takes the state the load installed -- a
     *   file the player did not ask for -- and a load queued behind a save
     *   rewinds to a state saved microseconds earlier, undoing whatever the
     *   player did while the menu was up. Neither is what the press meant.
     *
     *   ABORT THE OTHER ONE. A watchdog is the ENGINE's (`abort_tog`), not the
     *   command's, so a command timing out killed whatever the engine was on:
     *   the save's 113 ms ending a load with a 1.8 s leash, which is the
     *   half-applied state whose top row is one state's and bottom another's.
     *   Dropping at the door means no watchdog ever has another direction's
     *   transfer in front of it.
     *
     * WHAT THE DROPPED COMMAND IS ANSWERED WITH. Two cases, and the difference
     * is only whether there is a verdict to inherit:
     *
     *   the SAME direction is running -- a save while a save, a load while a
     *   load: nothing is written to its status at all, so the transfer already
     *   in flight answers for it (ok when it finishes). For a save that is
     *   also the right DATA: the host reads the buffer the running save is
     *   filling, i.e. a real state, not the empty buffer a second transfer
     *   would have had to fill. For a load there is nothing to read afterwards
     *   -- the machine is simply left as it is.
     *
     *   the OTHER direction is running: there is no verdict to inherit, so the
     *   command is answered ok at once (`busy` never rises). A load dropped
     *   for a save is exactly "nothing happened", which is what was asked for.
     *   A save dropped for a load reports ok without a save having run, so the
     *   host's reads that follow find no fresh words and the slot it stores is
     *   not a state -- the price of not erroring, and the one case where a
     *   dropped command is visible in the file rather than on the screen.
     *
     * `save_running` includes `engine_busy && !load_active` because a save is
     * still producing words AFTER its ok (ok means "the first word is there",
     * see below), and `ss_start_busy` alone would declare it finished while the
     * engine was still streaming and an abort would still cut it in half.
     * `ss_load_busy` is the load's mirror: it is up from the command until the
     * engine's verdict, which covers both host orderings.
     */
    wire        save_running  = ss_start_busy | (engine_busy && !load_active);
    wire        load_running  = ss_load_busy | load_active;
    wire        busy_drop     = save_running | load_running;

    /* A DROPPED SAVE MUST NOT KICK. The kick below fires on the command's EDGE
     * (`ss_start & ~prev_start`), not on the accept branch, and `busy_drop` can
     * be true while `engine_busy` is still zero -- a load that has been
     * commanded and is waiting for its data has `ss_load_busy` up and nothing in
     * the engine yet. Without this gate the dropped command would still toggle
     * `save_tog`, flush the save pipe and start a save nobody is watching: no
     * `ss_start_busy`, so no 113 ms clock and no ok/err of its own, which is
     * exactly the untracked transfer this rule exists to prevent. */
    wire        save_take     = (ss_start & ~prev_start) && !busy_drop;

    always_ff @(posedge clk_74a) begin

        /* ---- defaults: every strobe here is one clock wide ---- */            ss_start_ack   <= 1'b0;
            ss_load_ack    <= 1'b0;
            save_flush_74a <= 1'b0;

            if (save_flush_74a)
                save_flush_cnt <= 5'd31;          // the drain's settling window
            else if (save_flush_cnt != 5'd0)
                save_flush_cnt <= save_flush_cnt - 5'd1;

            ss_load_prev <= ss_load;
            prev_start   <= ss_start;

        if (reset_74a) begin
            rd_hold      <= 64'd0;
            last_rd_word <= 25'h1FF_FFFF;
            wr_count     <= '0;
            wr_gray      <= '0;
            load_cmd_pending <= 1'b0;
            stream_started   <= 1'b0;
            load_data_seen <= 1'b0;
            load_active  <= 1'b0;
            start_go     <= 1'b0;

            ss_start_busy <= 1'b0;
            ss_start_ok   <= 1'b0;
            ss_start_err  <= 1'b0;
            ss_load_busy  <= 1'b0;
            ss_load_ok    <= 1'b0;
            ss_load_err   <= 1'b0;
            underrun_q    <= 1'b0;
            overrun_q     <= 1'b0;
        end
        else begin

            /* ---------------------------------------------------------- *
             * The host reads a word of the state
             *
             * IN THE ADDRESS'S OWN CYCLE, because that is the cycle the answer
             * is sampled in: core_top presents it combinationally (its note:
             * "the state byte the host is being handed was made ready on the
             * previous access, so the answer is stable for the whole cycle it
             * is asked in"), which is what makes one access per bridge read
             * possible -- and what the harness's read timing requires.
             *
             * A NEW LOW HALF STARTS A WORD: the FIFO's head answers it, and the
             * word is copied into the holding register and popped in the same
             * cycle, so the high half (the next access, at bit 2 set) is
             * answered from the register. A repeat of either address is
             * answered from the register too, and consumes nothing -- a host
             * that reads a word twice must not shift its own file.
             * ---------------------------------------------------------- */
            if (rd_new) begin
                if (!save_empty) begin
                    // A word was taken: the address IS consumed. The high half
                    // (and a repeat) is answered from the register above.
                    last_rd_word <= acc_word;
                    rd_hold <= save_q;
                end
                else begin
                    // THE HOST RAN AHEAD OF THE ENGINE, and the address is NOT
                    // consumed. That is the difference between a file that is
                    // merely late and a file that is silently shifted: with the
                    // key still pointing at this word, the next read of it (or
                    // of its high half) is answered once the engine produces it,
                    // which is the property Agg23's controller gets by only
                    // popping when its FIFO has data. The engine is the faster
                    // side by a factor of two at the bridge's own 88-cycle
                    // ceiling (one word per ~11 machine clocks against two
                    // 32-bit accesses per 176 clk_74a), so this is a
                    // belt-and-braces path -- but a shifted file is exactly the
                    // failure that looks like "saves do not load".
                    underrun_q <= 1'b1;
                end
            end
            /* NOTHING to update for an access that is not a new word: the key
             * moves only when a word is actually taken above. Updating it here
             * as well is what made the half's own address a "word" key, and so
             * what let a re-read shift the file (see the note at the key). */

            /* ---------------------------------------------------------- *
             * A NEW STREAM BEGINS AT THE BUFFER'S FIRST WORD, and whatever the
             * previous transfer left behind is emptied HERE -- two cycles
             * before this stream's first word is pushed (the push is the high
             * half's, see the arming rule below), so the word the engine reads
             * as a header is this stream's first and not a tail of the last
             * one's. That is the one place leftovers are cleared: the host is
             * allowed to send more than the state, so a completed transfer can
             * leave up to `maxloadsize - size` words in the FIFO, and a second
             * load in the same session would otherwise be read from behind them.
             *
             * Not while a transfer is in flight: those words belong to the
             * stream the engine is reading, and a flush there would starve it
             * (which the transfer's own clock would then answer for).
             * ---------------------------------------------------------- */
            if (load_new_stream && !load_active) begin
                // A new stream starts at the buffer's first word. Its word count
                // goes back to zero HERE, which is the whole of what a flush is
                // in this design: nothing already in the buffer can be read as
                // part of the new state, because the reader only ever reads what
                // the count says has arrived.
                wr_count <= '0;
                wr_gray  <= '0;
                stream_started <= 1'b1;   // ... and a load command now has data
            end
            else if (load_stage_wr) begin
                // The count of whole words taken, and the gray copy of it that
                // the reader's domain sees. `wr_count_n` is derived from the
                // register, so the staged word above and this count always
                // describe the same index.
                wr_count <= wr_count_n;
                wr_gray  <= (wr_count_n >> 1) ^ wr_count_n;
            end


            /* ---------------------------------------------------------- *
             * The host writes a word of the state
             *
             * CONTIGUOUS FROM WORD 0, which is what a memcpy into the window
             * gives and all this needs: the low half is kept in `load_lo`, the
             * high half closes the word and stages it at the count's own index,
             * and the count advances one per staged word. A word the host writes
             * again is not staged twice (its index is behind the count), and a
             * word that arrives out of order is not staged at all -- the reader
             * only walks the count, so a gap in the host's stream shows up as
             * the engine waiting for a word that never comes, which the transfer
             * clock answers for.
             * ---------------------------------------------------------- */
            /* ---------------------------------------------------------- *
             * THE HOST'S HALF ORDER (2026-10-17)
             *
             * A stream is a sequence of 64-bit words and the host hands each of
             * them over as two 32-bit writes, but WHICH write is which half is
             * not this core's to assume -- and the panel says the OS does not
             * use the order this file does. A state saved on the panel, opened
             * on a PC (tools/sta_inspect.py), has the format tag FIRST and the
             * count second in every word: `00 02 01 00 | 01 00 00 00` for word 0,
             * where this file hands out `01 00 00 00 | 00 02 01 00`. Every RAM
             * word in that file is the machine's word with its two halves the
             * other way round (measured: 125 of the first 128 words match under
             * the swap, 80 under identity, the rest being words that are equal
             * either way). So the OS collects the state in one order and plays
             * it back in the other, and a load of its own save was refused by
             * the tag check -- "the OS lets you save and gives an error when you
             * load", with the save direction, which reads nothing back, perfect.
             *
             * The header is what makes this decidable, and it is decidable in
             * ONE cycle: word 0 carries the format TAG in one half and the
             * count in the other, so whichever half the tag arrives in says how
             * the whole stream is laid out. The tag is a constant of this
             * build (`TAG`, = the engine's STATESIZE), so this needs no
             * agreement with the host about anything but the number itself.
             *
             * Both orders are honoured, deliberately: a host that copies the
             * file back exactly as it stored it lands on one, and one that
             * undoes its own reversal lands on the other, and neither is refused
             * for the order alone. The engine's tag check still refuses a file
             * that is not this build's -- this decides the ASSEMBLY, not the
             * acceptance. */
            if (wr_hit) begin
                load_data_seen <= 1'b1;      // ... so the stream is under way
                if (bridge_addr[2] &&
                    (bridge_addr[27:3] == {2'b0, wr_count}) &&
                    (wr_count == STAGE_WORDS[STAGE_AW:0]))
                    // A blob bigger than the buffer. This is a capacity statement
                    // (the same one `savestate_maxloadsize` makes to the OS)
                    // and not a race: the words are not dropped silently.
                    overrun_q <= 1'b1;
                else if (bridge_addr[2] && (bridge_addr[27:3] == {2'b0, wr_count})) begin
                    // The second half of the word closes it, and the first is
                    // the one before it in the file -- already in hand.
                    //
                    // On word 0 the two halves are checked against the tag: the
                    // half the tag is in is the UPPER half of the word, whatever
                    // address it arrived at. Neither half matching (a foreign
                    // file) leaves the order as this build writes it, and the
                    // engine refuses the file on its own tag check.
                    // A stream that does not start at word 0 (a host that began
                    // mid-buffer) keeps whatever order the last stream decided
                    // (`swap_word` is `swapped` again for every later word).
                    swapped <= swap_word;
                    load_stage[wr_count[STAGE_AW-1:0]] <=
                        swap_word ? {load_lo, bridge_wr_data}     // the host's way round
                                  : {bridge_wr_data, load_lo};    // this file's way round
                end
                else if (!bridge_addr[2])
                    load_lo <= bridge_wr_data;
            end

            /* ---- the kick a save command defers to here ------------------
             *
             * Fires in the request's own cycle when the engine is idle, and in
             * the cycle it GOES idle when it was not. It lives outside the
             * command block below because that block is an if/else-if chain
             * armed on the request edge, and this has to be able to fire on a
             * later cycle while `ss_start_busy` is counting. */
            if ((save_take || start_go) && !engine_busy) begin
                start_go     <= 1'b0;
                save_tog_q   <= ~save_tog_q;
                // Empty the pipe first: a save the OS stopped reading halfway
                // leaves its words in the FIFO, and the first read of this save
                // must be the state's first word.
                save_flush_tog <= ~save_flush_tog;
                save_flush_74a <= 1'b1;

                // A fresh save starts at the buffer's first word: nothing held
                // from the previous one may answer the first read, and the first
                // address of the buffer counts as new whatever the last read of
                // the previous save was.
                rd_hold      <= 64'd0;
                last_rd_word <= 25'h1FF_FFFF;
            end

            /* ---------------------------------------------------------- *
             * Command 0x00A0: take a state
             * ---------------------------------------------------------- */
            /* A SAVE COMMANDED WHILE A TRANSFER IS STILL RUNNING WAITS FOR IT
             * (2026-10-18, the panel: *after a while saves get corrupted and
             * won't load* -- "maybe it was saving while saving?"). It was, and
             * it was two faults in one cycle:
             *
             *   the KICK is a toggle, and the engine's edge detect turns it into
             *   a ONE-CYCLE pulse that only ST_IDLE looks at (mc64_savestates).
             *   Issued while the engine was busy, it was consumed and dropped --
             *   the new save never ran at all, and its command sat there until
             *   the watchdog answered err. That is the "won't load" half: the
             *   file the host collected was whatever the buffer already held;
             *
             *   the FLUSH reset the FIFO's two pointers and the store UNDER the
             *   engine that was still writing them, so the tail of the old
             *   stream landed at index 0 of a buffer whose read pointer had just
             *   gone back to 0 as well. The words of the abandoned transfer and
             *   those of the new one then interleaved, and the file the host read
             *   began with a zero header -- "count 0, tag 0" -- which the engine
             *   refuses on the way back in, exactly as a foreign file is. That is
             *   the corrupted half.
             *
             * Both are fixed by the same rule: the flush and the kick happen when
             * the engine is IDLE, not when the command arrives. The acknowledgement
             * to the host is still immediate (nothing about the handshake moves),
             * and `ok` still means "the first word is there to read" -- which is
             * why the wait is also required by the ok test below: with the previous
             * transfer's words still sitting in the buffer, an ok taken before this
             * one's kick would be answering with the OLD stream, and the host would
             * read its own file starting from a previous save's tail.
             *
             * The wait is bounded by the engine's own copy: the staging store
             * holds the whole state, so a save never blocks on the host and the
             * engine is idle again within ~2.6 ms of any command. */
            if (ss_start & ~prev_start) begin
                ss_start_ack  <= 1'b1;
                if (busy_drop) begin
                    /* DROPPED: see ANOTHER COMMAND WHILE ONE IS ALREADY RUNNING
                     * IS DROPPED above. A save already running answers for
                     * this one (its status is left exactly as it is, so its
                     * ok/err and its 113 ms clock keep running); a load that is
                     * running cannot answer for a save, so this one is told ok
                     * at once and nothing is taken. */
                    if (!save_running) begin
                        ss_start_busy <= 1'b0;
                        ss_start_ok   <= 1'b1;
                        ss_start_err  <= 1'b0;
                    end
                end
                else begin
                    ss_start_busy <= 1'b1;
                    ss_start_ok   <= 1'b0;
                    ss_start_err  <= 1'b0;
                    start_wd      <= '0;
                    // Held if the engine is mid-transfer; the kick below fires
                    // the cycle it is not (and fires this same cycle when it
                    // never was).
                    start_go      <= engine_busy;
                end
            end
            else if (ss_start_busy) begin
                /* A LOAD IN FLIGHT IS NOT THIS COMMAND'S TO KILL (2026-10-19).
                 *
                 * The panel: *after a while the bottom half of the screen got
                 * darker and saves didn't load* -- the picture's LOWER ROWS in
                 * wrong colours with the machine still running, then loads that
                 * error. That is a state applied HALF-WAY, and a load has one
                 * half-way point anyone can see: the flush of the picture's
                 * window walks the framebuffer in address order, so a transfer
                 * cut short inside it leaves the top of the picture one state's
                 * and the bottom another's.
                 *
                 * What cut it short was THIS watchdog. It exists to answer err
                 * when a command's machine never reaches an instruction
                 * boundary, and it fires 2^23 clk_74a (113 ms) after the
                 * command -- but `abort_tog` is the ENGINE's, not the
                 * command's, so a save commanded while a load was still in
                 * flight killed the load. The load's own patience for the same
                 * host is 1.8 s, so the save's patience was sixteen times
                 * shorter than the transfer it was standing behind, and the
                 * abort landed wherever the engine had got to. The load then
                 * reported err with a state half applied -- and because the
                 * controller never learns of an abort, it sat "busy" until its
                 * own (long) window expired, which is why the panel sees the
                 * load hang and then fail rather than fail at once.
                 *
                 * The rule is the one 21.19 already established for the kick:
                 * this command waits for the engine, and a wait it asked for is
                 * not a thing to abort. While a load is in flight the save's
                 * clock is held at zero -- its 113 ms begins when the machine
                 * is actually its to take -- and no abort is raised. The load's
                 * own watchdogs bound that wait, so this cannot become a hang. */
                /* `load_active` AND NOT `ss_load_busy`, deliberately: a load
                 * that is merely still ARRIVING has not taken the engine, and
                 * holding this clock for it would be a deadlock the other way
                 * -- the engine could be finishing a save, with this command's
                 * watchdog held and the load's held too (its own note below
                 * masks it while the engine is on another's transfer). Only an
                 * ARMED load (on the engine) may hold this one back. */
                if (load_active) begin
                    start_wd <= '0;
                end
                // `ok` MEANS "THE FIRST WORD IS THERE TO READ", NOT "THE ENGINE
                // HAS STARTED". The OS stops polling the moment it reads ok and
                // then reads the buffer from its first address; if the FIFO is
                // still empty at that point the first read is answered from a
                // holding register that holds nothing, and because the read is
                // keyed by address the whole file SHIFTS by one 32-bit word --
                // a state whose header is at byte 4, refused on the way back in
                // exactly as a foreign file is, with the machine's own transfer
                // having worked perfectly. That is invisible to any harness that
                // polls before reading, which is why this is a level and not an
                // ordering: Agg23's controller reports ok on the engine's FIRST
                // request for the same reason (its `SAVE_BUSY` ends when
                // `ss_req` arrives, i.e. when a word is in the FIFO), and this
                // is the same promise made with an empty flag.
                //
                // It costs nothing when the engine is slower than the host
                // (the normal case): the first word lands a few machine cycles
                // after the engine starts, and the OS's own round trip to the
                // buffer is longer than that. If it never lands, the watchdog
                // above answers err, which is the honest report.
                else if (!start_go && engine_busy && !save_empty) begin
                    ss_start_busy <= 1'b0;
                    ss_start_ok   <= 1'b1;
                end
                else if (start_wd[WATCHDOG_BITS-1]) begin
                    // It never stopped: no instruction boundary arrived (no
                    // cartridge released, or a machine held in reset). Report
                    // the refusal AND tell the engine to drop it -- a request
                    // the engine answered by freezing the machine has to end
                    // with the machine running, or one timed-out command leaves
                    // the panel frozen with the state half-written.
                    ss_start_busy <= 1'b0;
                    ss_start_ok   <= 1'b0;
                    ss_start_err  <= 1'b1;
                    abort_tog_q   <= ~abort_tog_q;
                end
                else
                    start_wd <= start_wd + 1'b1;
            end

            /* ---------------------------------------------------------- *
             * Command 0x00A4: restore a state
             * ---------------------------------------------------------- */
            if (ss_load & ~ss_load_prev) begin
                ss_load_ack  <= 1'b1;
                if (busy_drop) begin
                    /* DROPPED (see THE COMMAND ABOVE). No stream is consumed
                     * and the engine is not touched: a load already running
                     * answers for this one, and a save that is running has no
                     * verdict a load can inherit, so this is told ok at once
                     * -- the machine stays exactly where it was. `ss_load_busy`
                     * is NOT written: if a load is in flight that is its own
                     * flag, and clearing it would strand its completion. */
                    if (!load_running) begin
                        ss_load_ok  <= 1'b1;
                        ss_load_err <= 1'b0;
                    end
                end
                else begin
                    ss_load_busy <= 1'b1;
                    ss_load_ok   <= 1'b0;
                    ss_load_err  <= 1'b0;
                    load_wd      <= '0;
                    /* `load_data_seen` is NOT cleared here. The host may stream the
                     * state BEFORE it commands the load (Agg23's ordering, and the
                     * one a state written from the OS menu produces), and clearing
                     * it would leave the watchdog switched off for exactly the
                     * transfer that then stops mid-stream. It means "this transfer
                     * has received a word", so it is cleared where a transfer ENDS
                     * (the engine's done/err below, and the leftovers branch just
                     * under this). */

                    /* NO FLUSH AT THE COMMAND. A transfer's leftovers are emptied
                     * where the transfer ENDS (below, on the engine's done and on
                     * its refusal), which is after the host has stopped writing and
                     * cannot be holding a stream the core is about to need. Emptying
                     * the FIFO here instead threw away a stream that had already
                     * arrived: the host streams the state and THEN commands it (the
                     * ordering Agg23's controller documents -- "data is already
                     * copied into ss manager"), so a command-time flush deletes the
                     * very words the transfer is for. */

                    /* THE COMMAND ARMS WHAT THE HOST HAS ALREADY SENT, and it does
                     * it HERE rather than leaving the arm to answer for a
                     * transfer that ran while the file was still arriving. That
                     * earlier shape -- the engine started by the first word, the
                     * command merely reporting its verdict -- is what made a load
                     * cost the host's whole SD read on the panel: the machine was
                     * frozen and the picture held from the first byte of the file
                     * until the command that came after the last one. `load_arm`
                     * (below) takes both halves of what names a transfer, so this
                     * simply records the command; if the stream is already in, the
                     * arm fires the same cycle and the engine starts at once. */
                    load_cmd_pending <= 1'b1;
                end        // ... the accepted command's block (not dropped)
            end
            else if (ss_load_busy) begin
                if (load_done_now) begin
                    ss_load_busy <= 1'b0;
                    ss_load_ok   <= 1'b1;
                    ss_load_err  <= 1'b0;     // the live lines are exclusive
                    load_data_seen <= 1'b0;   // the transfer is over
                end
                else if (engine_err_now) begin
                    ss_load_busy <= 1'b0;
                    ss_load_err  <= 1'b1;
                    ss_load_ok   <= 1'b0;
                    load_data_seen <= 1'b0;
                end
            end

            /* ---- the transfer's own clock (see its declaration) --------- */
            if (load_inflight && !engine_on_another)
                load_wd <= load_wd + 30'd1;
            else
                load_wd <= 30'd0;

            if (load_stage_wr)
                load_wd <= 30'd0;      // words are still arriving: not a stall

            if (load_stalled) begin
                // The host has gone away (or its stream was cut). Answer err,
                // drop the state, and HAND THE MACHINE BACK: it is stopped for
                // the whole transfer, and leaving it stopped is the failure this
                // clock exists for. The command is spent too, so a stray word of
                // a dead stream cannot arm a transfer nobody is waiting on. The
                // next load is a new stream and a new command, and both of them
                // are what the arm asks for -- so this failure is NOT carried
                // into the next one's verdict, which is its own.
                ss_load_busy   <= 1'b0;
                ss_load_err    <= 1'b1;
                ss_load_ok     <= 1'b0;
                abort_tog_q    <= ~abort_tog_q;
                load_active    <= 1'b0;
                load_cmd_pending <= 1'b0;
                stream_started   <= 1'b0;
                load_data_seen <= 1'b0;
                load_wd        <= 27'd0;
            end

            /* ---------------------------------------------------------- *
             * Arming the engine's load, and its completion
             * ---------------------------------------------------------- */
            if (load_arm) begin
                load_tog_q  <= ~load_tog_q;
                load_active <= 1'b1;
                // Both halves of the request are spent by the arm; the next
                // transfer needs its own stream (word 0) and its own command.
                load_cmd_pending <= 1'b0;
                stream_started   <= 1'b0;
            end

            if (load_done_now) begin
                // The stream is spent. A host that wrote past the state's size
                // leaves words in the buffer, and they must not be able to start
                // a load of their own -- which is what the count does: the next
                // stream's word 0 puts it back to zero.
                load_active <= 1'b0;
            end

            if (engine_err_now) begin
                // Refused (the header tag was not this build's): the transfer is
                // over, and nothing left in the buffer may start another one --
                // the next load is its own stream and its own command.
                load_active <= 1'b0;
            end
        end
    end

    // The answer for the access in front of us right now: the low half of a new
    // word comes straight out of the FIFO (its head, which is popped at the end
    // of this cycle), and everything else -- the high half, and any repeat --
    // comes out of the holding register.
    /* ------------------------------------------------------------------ *
     * The save side's pop, which is the mirror of the engine's load gate.
     *
     * The pop is asserted in the SAME cycle as the head it takes, because
     * that is the FIFO's contract: `rd` advanced the read pointer at the edge
     * that ends the cycle, and the caller must already have sampled `rdata` in
     * that cycle. Registering the strobe instead would delay the pop by one
     * cycle -- harmless here only because the host's reads are 97 cycles
     * apart, i.e. it would be a bug waiting for a faster host.
     * ------------------------------------------------------------------ */
    assign save_rd = rd_new && !save_empty;

    assign ss_rd_data = rd_high ? rd_hold[63:32]
                     : rd_new  ? save_q[31:0]
                     :           rd_hold[31:0];

endmodule
