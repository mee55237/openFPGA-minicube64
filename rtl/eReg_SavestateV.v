/*
 * eReg_SavestateV -- one save-state register, in Verilog.
 *
 * This is the same primitive as the `eReg_SavestateV` entity in
 * rtl/bus_savestates.vhd (the MiSTer framework file the APU was ported from),
 * and it is written in Verilog rather than used from the VHDL for one reason:
 * rtl/apu.sv instantiates it, and Verilator -- the tool every pre-fit check in
 * this project runs (tools/lint.sh, tools/sim.sh, tools/boot_sim.sh) -- cannot
 * elaborate VHDL. Adding the VHDL file to rtl/ap_core.qsf and this one to the
 * sims instead would build the fit and the simulations from two definitions of
 * one primitive, which is exactly the split tools/lint.sh's third pass exists
 * to prevent. One definition, in both builds.
 *
 * Behaviour, matching the entity line for line:
 *
 *   stored   holds `def` until a savestate bus write selects this register.
 *            A savestate *load* is what BUS_rst means here: the framework
 *            pulses it while loading a state so that registers the state does
 *            not cover fall back to their power-on values. Nothing drives it
 *            yet (the framework is not in the fit), so the register simply
 *            holds `def` -- which is the value the APU's own reset path reads
 *            back into its state (see rtl/apu.sv, e.g. SS_FCT's 15'h7FFF).
 *
 *   BUS_Dout is the read side: this register's slice of the 64-bit wired-or the
 *             framework sums. Zero when it is not addressed.
 *
 * Port order is the entity's, and the parameters are the entity's generics in
 * the entity's order, because rtl/apu.sv overrides them positionally:
 *
 *   eReg_SavestateV #(SSREG_INDEX_TOP, SSREG_DEFAULT_APU_TOP) ...
 *
 * If the framework is ported in full, either drop rtl/bus_savestates.vhd from
 * the tree or drop this file -- do not put both in one build.
 *
 * Quartus parses this file as Verilog-2001 (VERILOG_FILE in rtl/ap_core.qsf),
 * so it stays Verilog-2001: no `logic`, no `always_ff`, no unsized literals.
 */
`timescale 1ns / 1ps

module eReg_SavestateV #(
    parameter [ 9:0] Adr = 10'd0,
    parameter [63:0] def = 64'd0
) (
    input  wire        clk,
    input  wire [63:0] BUS_Din,
    input  wire [ 9:0] BUS_Adr,
    input  wire        BUS_wren,
    input  wire        BUS_rst,
    output wire [63:0] BUS_Dout,
    input  wire [63:0] Din,
    output wire [63:0] Dout
);

    reg [63:0] stored = def;

    always @(posedge clk) begin
        if (BUS_rst)
            stored <= def;
        else if (BUS_wren && BUS_Adr == Adr)
            stored <= BUS_Din;
    end

    assign Dout = stored;

    assign BUS_Dout = (BUS_Adr == Adr) ? Din : 64'd0;

endmodule
