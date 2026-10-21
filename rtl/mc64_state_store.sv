/*
 * mc64_state_store -- the save direction's staging buffer.
 *
 * WHY THIS EXISTS (2026-10-18). The panel's report was *saves work, but they
 * take forever*; the load side had already been fixed the same day by giving
 * it a buffer the whole blob fits in (save_state_controller's `load_stage`).
 * The save direction had no such buffer: the engine pushed a word every ~8
 * machine clocks and the host read one every ~22 (its own ceiling is one
 * 32-bit access per 88 clk_74a, i.e. two per 64-bit word), so the 256-entry
 * cross-domain FIFO filled after a few hundred words and from then on THE
 * MACHINE WAITED FOR THE HOST -- the state is taken with the CPU stopped, so
 * the freezes lasted the host's whole read of 66 KB (~40 ms), twice what the
 * copy itself costs.
 *
 * With this store between them the engine's stream has somewhere to land:
 * `fifo_save`'s 256 entries cover the crossing's few cycles, and this array
 * takes the rest at a word per clk_74a cycle -- 74 MHz against the engine's
 * ~1.25 MHz word rate. The engine therefore never sees a full pipe, the copy
 * finishes at the RAM port's own rate, and the machine is handed back while
 * the host is still reading the words out. What the host reads does not
 * change: the same address-keyed walk, the same holding of a word for its two
 * halves, the same file.
 *
 * WHY A SECOND BUFFER AND NOT A DEEPER `fifo_save`. `mc64_async_fifo` hands
 * its head out COMBINATIONALLY from the read pointer (`assign rdata =
 * mem[rbin]`). That is what the load direction needs -- the engine samples a
 * word in the cycle it asks for it -- but it is also why that FIFO cannot be
 * made deep: an M10K's data comes out of the block's own ADDRESS REGISTER, so
 * a combinational read is inferrable only while the whole array fits in
 * registers (256 entries). This store is the other shape: it is one clock
 * domain (clk_74a), the array's read is registered, and a one-word show-ahead
 * register gives the consumer the same "the word is there NOW" contract the
 * FIFO has. So the crossing stays one FIFO wide and the depth is a RAM.
 *
 * THE SHOW-AHEAD. A word is fetched into `head_q` (two clocks: present the
 * address, then capture what the block's output register returns) and `empty`
 * is `!head_v`; `rdata` is `head_q`, valid for the whole cycle it is popped
 * in. The consumer's accesses are at least two clocks apart (one word is taken
 * by the low half, the high half is answered from the controller's holding
 * register), so the fetch is always finished before the next word is asked
 * for. If it is not -- a host faster than 74 MHz -- `empty` says so and the
 * controller reports an underrun and does not consume the address, exactly as
 * it already does against the FIFO.
 *
 * TWO ENTRIES ARE RESERVED: the one in `head_q` and the one whose address is
 * in flight are no longer in the array (their pointers have moved), so `full`
 * is `used >= WORDS - 2`. The save state is 8,257 words against the 8,192
 * here, so the last 65 words ride out on the host's own reads -- about 1,400
 * machine clocks, 1% of the copy.
 *
 * RESET drops the whole store. It is driven by the controller's save flush,
 * which fires when a save command starts: the previous save's words are spent
 * by then (the host has read what it wants of them), and the new state must
 * start at word 0.
 */
`default_nettype none
`timescale 1ns / 1ps

module mc64_state_store #(
    parameter int WORDS = 8192,
    parameter int WIDTH = 64
) (
    input  logic             clk,
    input  logic             reset,        // drop everything: a new save starts

    input  logic             wr,
    input  logic [WIDTH-1:0] wdata,
    output logic             full,

    input  logic             rd,
    output logic [WIDTH-1:0] rdata,
    output logic             empty
);

    localparam int AW = $clog2(WORDS);

    /* One write port and one registered read port, so this is a simple
     * dual-port M10K. `no_rw_check` because the two pointers collide only while
     * the store is empty (the read is already gated by `used != 0` on the
     * pointer side; the block's own output in that cycle is never looked at). */
    (* ramstyle = "M10K, no_rw_check" *) logic [WIDTH-1:0] mem [0:WORDS-1];

    logic [AW:0] wptr    = '0;   // where the next word is written
    logic [AW:0] fptr    = '0;   // where the next word is fetched from
    logic [WIDTH-1:0] mem_q;     // the block's registered read
    logic [WIDTH-1:0] head_q;    // the word the consumer's next read takes
    logic             head_v = 1'b0;
    logic             addr_v = 1'b0;   // an address was presented last cycle

    // Words in the array that have not been fetched yet. The ones in `head_q`
    // and in flight are past `fptr`, so they are not in this count.
    wire [AW:0] used = wptr - fptr;

    assign full  = (used >= (AW + 1)'(WORDS - 2));
    assign empty = ~head_v;
    assign rdata = head_q;

    always_ff @(posedge clk) begin
        if (reset) begin
            wptr   <= '0;
            fptr   <= '0;
            head_v <= 1'b0;
            addr_v <= 1'b0;
        end else begin
            if (wr && !full) begin
                mem[wptr[AW-1:0]] <= wdata;
                wptr <= wptr + 1'b1;
            end

            /* The block's own address register: presented every cycle, and the
             * data for the address presented last cycle is in `mem_q` now. */
            mem_q <= mem[fptr[AW-1:0]];

            if (!head_v) begin
                if (addr_v) begin
                    head_q <= mem_q;      // the fetch's second clock
                    head_v <= 1'b1;
                    addr_v <= 1'b0;
                end else if (used != 0) begin
                    fptr   <= fptr + 1'b1;  // ... and the first: present it
                    addr_v <= 1'b1;
                end
            end else if (rd) begin
                head_v <= 1'b0;             // the consumer took it; refetch
            end
        end
    end

endmodule
