`ifdef __openlane__
    `ifdef VERILATOR
        // openlane using Verilator for linting
        `define OPENLANE_LINT 1
    `endif
`endif

`ifdef CONFIG_TT
    // Tiny Tapeout config

    // use async reset ff's for ASIC flow
    `define USE_ASYNC_RESET 1

    `ifdef SYNTHESIS
        `define NO_TIMESCALE 1
    `elsif OPENLANE_LINT
        // in this synth-like context, skip timescales too
        `define NO_TIMESCALE 1
    `endif
`else
    // default config

    // use sync reset ff's for FPGA, etc. flow
    `undef USE_ASYNC_RESET

    // Use timescales by default, but do not force them off: a build that cannot
    // parse rtl/timescale.vh's macro form defines NO_TIMESCALE itself, and an
    // `undef here would throw that define away. Quartus is that build -- it reads
    // a `timescale directive before expanding macros and stops with Error 10096
    // ("incorrect use of predefined text macro timescale") on `TS_UNIT, which
    // costs it rtl/cpu_6502.sv and rtl/alu_6502.sv entirely. ap_core.qsf sets it.
`endif
