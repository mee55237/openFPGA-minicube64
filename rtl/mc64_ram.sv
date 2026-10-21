/*
 * Minicube64 main memory.
 *
 * 64 KB, flat, no banking -- see docs/minicube64-fpga-spec.md section 3.
 *
 * TWO CLOCKS, one per port, both 6.4 MHz:
 *
 *   Port A : the 6502, clocked by clk_a -- the SAME FREQUENCY as the CPU but a
 *            half cycle later (180 degrees). The address is latched on clk_a's
 *            rising edge, and the read comes back REGISTERED AT THAT SAME EDGE
 *            -- the latch IS the array's own address register, which is the
 *            only form Quartus will turn into an M10K (see the long note at the
 *            port A logic below; it is Error 276003 otherwise).
 *
 *            PORT A IS FOUR BYTES WIDE (2026-10-18). It presents a 32-bit word
 *            with byte enables, and the two things that use it ask for different
 *            amounts of it:
 *
 *              * the CPU still reads and writes ONE byte, at the lane its own
 *                address selects (`a_be = 1 << AB[1:0]`, and the read is muxed
 *                out of the registered word with the address's low bits);
 *              * the STATE ENGINE takes the whole word -- four bytes a cycle,
 *                which is what makes a 64-bit state word two clocks instead of
 *                eight, and a save or a load about four times shorter.
 *
 *            Both are the same port, and only one of them is ever driving it:
 *            a transfer holds the CPU stopped (see `ss_ram_own` in mc64_top),
 *            so the engine has the port to itself.
 *
 *            FOUR BYTE-WIDE BANKS, NOT ONE 32-BIT ARRAY. The whole point of the
 *            wide port is that it costs no memory: 4 x 16,384 x 8 is the same
 *            524,288 bits, and 16 blocks of 1,024 words per bank is still 64
 *            M10K blocks. What it costs, and why it is shaped this way, is the
 *            fit: a 16,384 x 32 array with byte enables on a true dual port is
 *            NOT something Quartus will infer from RTL, and it says so in the
 *            least helpful way available --
 *
 *              Error (10028): Can't resolve multiple constant drivers for net
 *                             "mem[0][31]" at mc64_ram.sv(158)
 *
 *            which is not a driver conflict in the code (Verilator simulates it
 *            happily) but the array falling back to 524,288 registers with two
 *            write processes and two clocks. Both a mixed-width port and a
 *            symmetric byte-enabled wide port were tried; both stopped the fit
 *            the same way. Four byte-wide banks are the shape that DID fit
 *            (the 65,536 x 8 array this replaces inferred as "True Dual Port ;
 *            524288 bits ; 64 M10K", 03:01 fit.rpt): every write below is a
 *            whole 8-bit word with a plain enable, one write statement per bank
 *            per port, which is exactly what that array did.
 *
 *            That is the whole reason the machine can run on a single 6.4 MHz
 *            clock. The CPU puts AB out on the rising edge of its clock; half a
 *            cycle later clk_a's edge latches it (stable, so no same-edge race)
 *            and the byte is presented for the rest of the CPU cycle, in time
 *            for mc64_top to register it into DI at the cycle boundary -- which
 *            is exactly arlet's contract, "valid data (DI) expected on the
 *            cycle AFTER valid address".
 *
 *            The difference from a flow-through read (latch AB externally, then
 *            `assign a_rdata = mem[a_addr_q]`) is a couple of nanoseconds of
 *            M10K clock-to-out against tens of nanoseconds of margin: the
 *            address still latches at the same edge and the byte is still there
 *            for the same half cycle. It is NOT a change in when the CPU sees
 *            data, and it is the difference between a fit and no fit.
 *
 *   Port B : video scan-out (framebuffer and palette fetches), or the cartridge
 *            loader while the machine is held in reset. These two never happen
 *            at the same time, so they share safely. Port B's read is
 *            registered too -- one clock of latency -- because the video
 *            generator consumes it a whole pixel period later (five clock
 *            cycles), so the latency is free there. Its DATA is one byte in and
 *            one byte out, and it writes one bank. The low address bits travel
 *            with the data through the same register -- otherwise a byte would
 *            come back selected by an address one cycle newer than the one that
 *            asked for it.
 *
 * Vector aliasing (reset vector -> $0200, IRQ vector -> $010E) and the APU /
 * controller read substitution are done in mc64_top, not here.
 */
`default_nettype none
`timescale 1ns / 1ps

module mc64_ram (
    // ---- Port A: the 6502 (one byte) or the state engine (four) ------------
    input  logic        clk_a,
    input  logic [15:0] a_addr,
    input  logic        a_we,
    input  logic [ 3:0] a_be,          // which of the four bytes this write covers
    input  logic [31:0] a_wdata,
    output logic [ 7:0] a_rdata,       // the CPU's view: the byte a_addr selects
    output logic [31:0] a_rdata_word,  // the engine's: the whole word

    // ---- Port B: video scan-out, or cartridge load ---- (one byte)
    input  logic        clk_b,
    input  logic [15:0] b_addr,
    input  logic        b_we,
    input  logic [ 7:0] b_wdata,
    output logic [ 7:0] b_rdata
`ifdef MC64_SIM
    ,
    // Simulation-only back door, used by the Verilator harness to dump and
    // compare the machine state against the golden C model. Absent from any
    // synthesis build (Quartus never defines MC64_SIM).
    input  logic [15:0] dbg_addr,
    output logic [ 7:0] dbg_data
`endif
);

    /*
     * no_rw_check -- read-during-write is a genuine don't-care here, and saying
     * so is what lets Quartus infer M10K at all. Without it the fitter reports
     * "unsupported read-during-write behavior", builds the array out of
     * registers instead, and dies with Error 276003 (524,288 registers are more
     * than the device's 18,480 ALMs can hold).
     *
     * Why it is a don't-care:
     *
     *   Port A: a cycle is either a read or a write, never both, so the only
     *   case is a write cycle whose read address is the address being written.
     *   The CPU does not consume the data bus on a write cycle -- arlet's core
     *   latches DI for reads -- so the byte the array presents there is
     *   invisible. (Quartus resolves the pair as "new data" anyway: the fit at
     *   03:01 reports read_during_write_mode "New data" for both ports.)
     *
     *   Port B: video scan-out reading a byte the CPU writes in the same cycle.
     *   M10K leaves mixed-port read-during-write undefined; the worst case is
     *   one pixel at the frame edge carrying the previous byte. It cannot affect
     *   anything the testbench compares -- sim/ decodes the picture from RAM,
     *   not from the live pixel stream.
     */
    /* ONE BANK PER BYTE OF THE WORD. 4 x 16,384 x 8 = 524,288 bits, 64 M10K
     * blocks, the same memory the byte-wide array was -- see the header for why
     * this shape and not one 32-bit array. Bank n holds byte n of every word, so
     * a 32-bit access is four whole 8-bit accesses at the same word address, and
     * the CPU's byte access is one of them. */
    (* ramstyle = "M10K, no_rw_check" *) logic [7:0] bank0 [0:16383];
    (* ramstyle = "M10K, no_rw_check" *) logic [7:0] bank1 [0:16383];
    (* ramstyle = "M10K, no_rw_check" *) logic [7:0] bank2 [0:16383];
    (* ramstyle = "M10K, no_rw_check" *) logic [7:0] bank3 [0:16383];

    /* ---- Port A: the address register IS the array's ----------------------
     *
     * `a_word_q <= bank...[a_addr]` and not "latch AB externally, then read the
     * array combinationally". The second form is what this module used to do,
     * and it is what killed the fit: Quartus reported
     *
     *   Info (276007): RAM logic "core_top:ic|mc64_top:machine|mc64_ram:ram|mem"
     *                  is uninferred due to asynchronous read logic  (line 84)
     *   Error (276003): Cannot convert all sets of registers into RAM
     *                   megafunctions ...
     *
     * An M10K's output can only come out of the block's own address register, so
     * a read whose address comes from a register OUTSIDE the array is not
     * inferrable at all; the array becomes 524,288 registers and the device has
     * 18,480 ALMs. Moving the register inside the block (one line, same clock,
     * same edge) is the fix, and it is the shape the pre-retime RTL fitted:
     *
     *   mc64_ram:ram|altsyncram:mem_rtl_0|ALTSYNCRAM ; M10K block ;
     *   True Dual Port ; 65536 x 8 ; 524288 bits ; 64 M10K   (03:01 fit.rpt)
     *
     * `ramstyle = "M10K, no_rw_check"` on the arrays is the other half of it --
     * without no_rw_check the fitter rejects the read-during-write behavior.
     *
     * ONE WRITE STATEMENT PER BANK, with the enable outside the array: a lane
     * the CPU is not writing, or a word a transfer is not copying, simply does
     * not write that bank. That is the form the byte-wide array fitted in, and
     * it is the reason the four banks are separate arrays and not lanes of one.
     */
    logic [31:0] a_word_q;
    logic [ 1:0] a_lane_q;

    always_ff @(posedge clk_a) begin
        if (a_we) begin
            if (a_be[0]) bank0[a_addr[15:2]] <= a_wdata[ 7: 0];
            if (a_be[1]) bank1[a_addr[15:2]] <= a_wdata[15: 8];
            if (a_be[2]) bank2[a_addr[15:2]] <= a_wdata[23:16];
            if (a_be[3]) bank3[a_addr[15:2]] <= a_wdata[31:24];
        end
        a_word_q <= {bank3[a_addr[15:2]], bank2[a_addr[15:2]],
                     bank1[a_addr[15:2]], bank0[a_addr[15:2]]};
        a_lane_q <= a_addr[1:0];      // the lane THIS read was for
    end

    assign a_rdata_word = a_word_q;
    assign a_rdata      = a_word_q[a_lane_q*8 +: 8];

    // ---- Port B: registered read, one clock of latency, one byte in/out ----
    logic [31:0] b_word_q;
    logic [ 1:0] b_lane_q;

    always_ff @(posedge clk_b) begin
        if (b_we) begin
            if (b_addr[1:0] == 2'd0) bank0[b_addr[15:2]] <= b_wdata;
            if (b_addr[1:0] == 2'd1) bank1[b_addr[15:2]] <= b_wdata;
            if (b_addr[1:0] == 2'd2) bank2[b_addr[15:2]] <= b_wdata;
            if (b_addr[1:0] == 2'd3) bank3[b_addr[15:2]] <= b_wdata;
        end
        b_word_q <= {bank3[b_addr[15:2]], bank2[b_addr[15:2]],
                     bank1[b_addr[15:2]], bank0[b_addr[15:2]]};
        b_lane_q <= b_addr[1:0];
    end

    assign b_rdata = b_word_q[b_lane_q*8 +: 8];

`ifdef MC64_SIM
    assign dbg_data = (dbg_addr[1:0] == 2'd0) ? bank0[dbg_addr[15:2]] :
                      (dbg_addr[1:0] == 2'd1) ? bank1[dbg_addr[15:2]] :
                      (dbg_addr[1:0] == 2'd2) ? bank2[dbg_addr[15:2]] :
                                                bank3[dbg_addr[15:2]];
`endif

endmodule
