// common.v — shared small modules for the WASM-4 Pocket core.
//
// 2026-09-05: trimmed to what the design actually instantiates. The
// bram_block_dp / bram_block_dp_nonstd modules were referenced by
// nothing anywhere in the project (RTL, sim, or scripts) and are gone.
// synch_2 (an edge-pulse sync variant) was also unreferenced and is gone;
// io_pad_controller.v / io_bridge_peripheral.v instantiate synch_3 with
// 3, 4, or 5 connections (rise/fall may float). Recover any of them from
// git history (git show HEAD:VexRiscv/common.v) if ever needed.

//
// 3-stage synchronizer
//
module synch_3 #(parameter WIDTH = 1) (
   input  wire [WIDTH-1:0] i,     // input signal
   output reg  [WIDTH-1:0] o,     // synchronized output
   input  wire             clk,   // clock to synchronize on
   output wire             rise,   // one-cycle rising edge pulse
   output wire             fall    // one-cycle falling edge pulse
);

reg [WIDTH-1:0] stage_1;
reg [WIDTH-1:0] stage_2;
reg [WIDTH-1:0] stage_3;

assign rise = (WIDTH == 1) ? (o & ~stage_3) : 1'b0;
assign fall = (WIDTH == 1) ? (~o & stage_3) : 1'b0;
always @(posedge clk) 
   {stage_3, o, stage_2, stage_1} <= {o, stage_2, stage_1, i};
   
endmodule