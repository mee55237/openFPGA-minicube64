# ---------------------------------------------------------------------------
# Minicube64 core constraints.
#
# rtl/apf_constraints.sdc is Analogue's and says not to edit it.  It already
# declares clk_74a, clk_74b and bridge_spiclk and calls derive_pll_clocks, so
# every PLL output is a clock object before this file is read.  That is why
# nothing here re-declares a clock: a second create_clock on the same pin is a
# duplicate-clock error in TimeQuest, not a stricter constraint.
#
# What is left to say is how the domains relate, and only the fitter can be
# told that:
#
#   * the APF's SPI clock is an input and is unrelated to the 74 MHz fabric
#     clock -- its phase is set by the Pocket, not by this core;
#   * clk_74a, clk_74b and bridge_spiclk each get their own group.  The APF's
#     own file says clk_74a and clk_74b are "not phase aligned, so treat these
#     domains as asynchronous" -- putting them in one group would ask the
#     fitter to time paths the framework has already declared to be unrelated.
#   * the PLL outputs stay in clk_74a's group.  Every machine clock is derived
#     from clk_74a by that PLL, so the two are related and the real paths
#     between them (the bridge into the machine, the machine's video into the
#     DDR pins) should be timed rather than waved through.  This is the
#     grouping VexiiRiscv ships, where it has been through a fit.
#
#     That group now contains FIVE related clocks, and three of the crossings#             between them are real single-half-cycle paths that must be timed, not
#             waved through:
#
#       clk_core (6.4 MHz)  -- the machine, and NOT a multiple of it
#       clk_ram  (6.4 MHz, +180) -- RAM port A.  The CPU launches AB/DO/WE on
#             clk_core, port A's address register latches them half a cycle
#             later, and the read data (registered at that same edge) is back
#             for the CPU inside the same clk_core cycle.  Both directions are
#             half-cycle paths, which is why the phase_shift3 value in
#             mf_pll_mc64_0002.v is a constraint and not a detail -- but at this
#             VCO it is 178.18 or 181.82 degrees rather than 180.000000, i.e.
#             0.79 ns from the ideal on a 156.25 ns period.  That is the phase
#             quantiser (the machine period is 99 VCO cycles, and half of 99 is
#             not a whole one), not a mistake: read the achieved value off the
#             fit report's PLL section.  The paths are safe because they are half
#             a cycle with tens of ns of margin either side, not because the
#             shift is bit-exact.
#       clk_pixel (1.32 MHz) -- the video link, clk_core * 33/160.  It is
#             deliberately NOT a whole division of the machine: 22,000 pixels of
#             1.32 MHz is exactly 60.000000 Hz, and no pixel count is exactly
#             60 Hz while a pixel is a whole number of 6.4 MHz clocks.  The
#             pixel edge (core_top's pix_tog) is sampled into clk_core by
#             pt_s1/pt_s2 and becomes `ce_pix` -- one crossing INTO the machine
#             domain -- while the pads are registered the other way: core_top's
#             pix_rgb/pix_de/pix_hs/pix_vs stage is clocked by THIS clock and
#             takes its data from clk_core, one crossing out of it.  Both are
#             timed.  The second one must stay on clk_pixel: the link is DDR, so
#             a pad that changes anywhere between the two launches puts one
#             pixel's high bits on the wire with the next pixel's low bits, and
#             tools/boot_sim.sh now asserts that every change of
#             {rgb,de,hs,vs,skip} coincides with a rising edge of this clock.
#             The three phase_shift values in mf_pll_mc64_0002.v (a quarter of
#             the pixel period, a quarter again for the capture clock, half of
#             the machine period for RAM port A) are constraints, not details,
#             and the 160/33 machine/pixel ratio is why that register is the one
#             crossing that had to be moved.
#
# If a later stage adds a clock that IS related to one of these (a second PLL
# output, a forwarded memory clock), put it in that clock's -group rather than
# in a new one, or the paths between them stop being timed.
# ---------------------------------------------------------------------------

# --- which clock is where ---------------------------------------------------
#
# The clock lists are built here, by PERIOD and not by name, because Quartus's
# name for a PLL output differs by version and by instance ("outclk[4]",
# "...|general[4].gpll~PLL_OUTPUT_COUNTER|divclk", ...) and because a get_clocks
# pattern that matches nothing makes the constraint that uses it fail the whole
# file -- a broken fit rather than a tighter one. Two lists come out of it:
#
#   machine_names -- clk_74a and the PLL outputs that are the machine's, which
#                    stay related (below), and
#   audio_names   -- the PLL output that is NOT: 633.6/52 = 82.07 ns, the audio
#                    mclk (12.184615 MHz), which mc64_audio runs on and which
#                    gets its own asynchronous group for the reason given under
#                    "the audio domain" at the end of this file.
#
# Both are glob-quoted, because a clock name may carry an output index in [].
set machine_names {}
set audio_names {}
foreach_in_collection c [get_clocks] {
    set n [get_clock_info -name $c]
    if {[string match "*altera_pll_i*" $n] || $n eq "clk_74a"} {
        set q [string map {{[} {\[} {]} {\]}} $n]
        set p [get_clock_info -period $c]
        if {$p > 70.0 && $p < 100.0} {
            lappend audio_names $q
        } else {
            lappend machine_names $q
        }
    }
}

set_clock_groups -asynchronous \
    -group [get_clocks { bridge_spiclk }] \
    -group [get_clocks { clk_74b }] \
    -group [get_clocks -nowarn $machine_names]

# --- the clk_74a <-> machine handshakes ------------------------------------
#
# The first fit reported this as the ONLY failing timing: clk_74a -6.963 ns and
# the machine domain -4.944 ns (TNS -209), while every domain's own Fmax passed
# with room to spare.  So no path inside the machine is too slow; what failed is
# the requirement the tool invents for the crossings.  (That fit ran the machine
# at 25.6 MHz with a /4 enable; it now runs at 6.4 MHz with no divide, so the
# internal slack only improved -- but the crossings are the same crossings.)
#
# The PLL's clocks are derived from clk_74a, so the two are related, and a
# related pair is asked to move data within a fraction of a 74.25 MHz cycle.
# No crossing in this design is a synchronous transfer -- they are all
# synchronised handshakes:
#
#   * the cartridge loader's FIFO crosses both ways as a synchronised
#     pointer pair (core_top, "THE HOST CANNOT BE PACED"): a word and its
#     address are written in clk_74a, and the gray-coded write pointer that
#     publishes them is only ever read through two flops in clk_core -- so the
#     data is already stable when the reader is told it is there, and only one
#     pointer bit changes per step, so a pointer caught mid-change reads as its
#     old or its new value. The empty/idle level comes back the same way. (This
#     replaced a four-phase handshake whose acknowledge round trip could not
#     keep up with the bridge -- see the cartridge section of core_top.v; the
#     crossing rule is unchanged, the protocol on top of it is not.)
#   * the controller vector is latched in clk_74a and marked with a toggle the
#     machine sees through two flops, then used one cycle after that;
#   * run_core is a level through two flops: asynchronous assert, synchronised
#     release, which is exactly what a reset wants;
#   * pll_locked_raw is the PLL's own `locked` output, which asserts
#     asynchronously after the VCO settles. It is sampled into clk_74a by
#     pll_locked_s1/s2 in core_top before anything reads it, and the sampled
#     level is what drives status_boot_done / status_setup_done (the OS's
#     Request Status answer) and the power-on counter. Those two lines below
#     used to cover this path while it was still a bare wire, which is how an
#     unsynchronised status reached the bridge and the Pocket refused the
#     handshake. The structure is what makes it correct now; the false path is
#     only what stops the tool inventing a single-cycle requirement for a
#     crossing that was never synchronous.
#
# Nothing crosses between these two clock sets except those handshakes, so the
# pairs are declared false here.  A false path removes the invented single-cycle
# requirement and the meaningless hold check on a crossing; what keeps the
# crossing correct is the structure above plus its protocol margins (a loaded
# word is held until the machine acknowledges, a pad vector is held for
# milliseconds), not a route bound.  This is also the one form this tool
# accepts: Quartus 25.1's set_max_delay takes no -datapath_only, so a bounded
# crossing is not available -- if a future crossing genuinely needs a route
# bound, constrain it by hand.  And if a future change adds a crossing that is
# neither a handshake nor a synchroniser, do NOT lean on these two lines: give
# it its own constraint.
set_false_path -from [get_clocks { clk_74a }] -to [get_clocks { *altera_pll_i* }]
set_false_path -from [get_clocks { *altera_pll_i* }] -to [get_clocks { clk_74a }]

# --- the audio domain: clk_audio (12.184615 MHz) and the machine -------------
#
# mc64_audio (rtl/mc64_audio.sv, instantiated in core_top) is the first thing in
# this design that runs on a second PLL output of its own. It is NOT in clk_74a's
# group with the rest: the two are related (the same VCO, so TimeQuest will time
# every crossing between them), but no transfer between them is synchronous, and
# the tightest launch/capture pair between a 6.4 MHz clock and a 12.18 MHz one is
# a VCO cycle or two apart -- a requirement no real path can meet, which is
# exactly the shape of the first fit's only failing timing (see above). So the
# clock goes in its own asynchronous group, which cuts the crossings and leaves
# every path INSIDE each domain timed.
#
# What keeps the crossings correct is structure, and it is the same structure the
# machine's other domains use:
#
#   * load_req (clk_audio -> clk_core) arrives through req_s, three flops, and
#     held/held_tog are latched on the code edge it produces, so the 16-bit level
#     is a register that has been stable for a full frame by the time the audio
#     domain reads it;
#   * held_tog (clk_core -> clk_audio) crosses the same way, through tog_s, and
#     the audio domain copies `held` on that edge only -- a two-toggle handshake,
#     so a word changing across the crossing cannot be read half-old/half-new;
#   * nothing else in mc64_audio crosses: audio_dac and audio_lrck are clk_audio
#     registers driving output pins, and clk_audio has no other interface.
#
# The audio clock is not in the group above: it is the one PLL output that is
# NOT the machine's, and the machine's own call has to leave it out (TimeQuest
# does not allow a clock in two groups -- deciding it is in "both" is an error,
# not a merge). Both sides are named from the lists built at the top of this
# file, not by repeating a pattern that would match it.
if {[llength $audio_names] > 0} {
    set_clock_groups -asynchronous \
        -group [get_clocks -nowarn $audio_names] \
        -group [get_clocks -nowarn $machine_names]
}

derive_clock_uncertainty
