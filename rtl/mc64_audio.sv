/*
 * Minicube64's audio output stage: the APU's mixer level to the Pocket's I2S.
 *
 * THE PINS. The Pocket's scaler takes three audio signals from the core --
 * audio_mclk (the master clock), audio_lrck (the frame clock) and audio_dac
 * (the serial data) -- and this module produces the last two from clk_audio.
 * The amount of data per frame is two 16-bit words, left then right, MSB first,
 * one bit per SCLK falling edge with SCLK = mclk/4 and LRCK = mclk/256, which
 * is the conventional I2S shape and the one the hardware-proven sibling core
 * (C:/Code/Wasm4/VexiiRiscv) drives on this panel: it changes its data line on
 * the falling edge of mclk/4, reloads its shift register on the last bit of
 * each 32-bit slot, and leaves the remaining 16 bits of every slot zero.
 *
 * audio_mclk is core_top's clk_audio, which at this VCO is 12.184615 MHz
 * (633.6 MHz / 52) rather than 12.288, so LRCK comes out 47.596 kHz rather
 * than 48.000 -- 0.84% slow, i.e. every sound 0.84% flat, 14 cents, which is
 * the one open item this leaves (spec s.6/s.19: an exact 12.288 MHz needs a
 * second PLL with a fractional VCO, as agg23's openfpga-NES does, and the
 * video PLL's integer outputs cannot make it). No part of the interface breaks
 * at 0.84%: the scaler is given both clocks by the core, so it takes the frames
 * at the rate they arrive.
 *
 * WHY THERE IS A FILTER, AND WHERE IT GOES. The APU is a level generator: its
 * mixer output steps at up to 894 kHz (the square channels' timers) and the
 * steps are what a 48 kHz resample would fold back into the audible band as
 * fizz. So the level is low-passed FIRST, in the machine's own domain and at
 * the machine's own rate, and only then resampled: a one-pole filter
 * y += (x - y) >> LPF_SHIFT, i.e. a 16.0 kHz corner at 6.4 MHz with a unity DC
 * gain, -5 dB at 24 kHz and -30 dB by 500 kHz. That is a gentle filter -- this
 * is where NES audio's genuine character lives, and the sharpest harmonics are
 * 30 dB down rather than gone -- but it takes the fold-back from "square waves
 * aliasing into whistles" to a soft top end, and it costs one adder.
 *
 * THE FILTER KEEPS ITS FRACTIONS. The obvious implementation, `y += (x - y)
 * >> 6` on a 16-bit y, is broken in a way that is invisible until you listen:
 * the error term truncates, so it goes to zero while the output is still up to
 * 63 counts away -- and for any input under 64 counts the term is zero from
 * the first cycle, i.e. the filter outputs silence. A quiet tail, a fade, or
 * a low-volume channel would simply disappear. So the state is held in 16.6
 * fixed point (the level scaled by 2^LPF_SHIFT) and rounded at each step: the
 * filter's own steps are 1/64 of an LSB, the DC gain is exactly 1, and the
 * smallest real input still converges. The output is rounded on the way out,
 * so a settled filter returns the level it was given, to the count.
 *
 * The filter also makes the crossing safe, which is why it is worth doing in
 * clk and not in clk_audio: its output changes by a fraction of an LSB per
 * clk_audio cycle, so no single sampled value is ever far off even if the
 * sample lands while the byte is settling.
 *
 * THE RESAMPLE. One value per LRCK frame, moved with a two-toggle handshake
 * rather than a bare synchroniser, because a 16-bit word crossing domains while
 * it changes can be read as a mix of old and new bits -- and a mix of 0x07FF
 * and 0x0800 is 4095 counts, audible. The handshake removes the question: the
 * request is issued, the machine domain latches the filtered level into `held`
 * and flips `held_tog` on the same edge, and the audio domain copies `held`
 * when it sees that toggle arrive. By then `held` has been stable for at least
 * two clk_audio cycles and stays stable for the rest of the frame, so the copy
 * is a settled word, not a race.
 *
 * WHEN THE REQUEST GOES, and why it is exactly there. Each slot's word is
 * reloaded at one point in the frame (div = 0x7F for the second slot, div =
 * 0xFF for the first), so a request issued in the FIRST half of a frame
 * reaches one reload and misses the other, and the frame's two slots would
 * carry two different samples -- a 21 us left/right skew on the wire. The
 * request goes in the second half instead (div = 0xC0, 63 clk_audio cycles --
 * 5.2 us -- before the boundary), where it is ahead of both reloads of the
 * frame about to start: the first slot is reloaded 63 cycles later and the
 * second 191, and the crossing itself takes about six, so both carry the same
 * word and one frame carries exactly one sample. Nothing is sampled twice and
 * nothing is dropped.
 *
 * Sample time is therefore one point per frame, 5.2 us early -- a constant
 * offset, and the same trade the sibling core makes with its own FIFO.
 *
 * RESET. Only the machine-domain side is reset. The audio-domain registers come
 * up at zero (their declared values), which is silence: the first frame is
 * loaded 21 us after the clocks start, and until then the DAC shifts out
 * zeroes -- which is also what an unconfigured APU's level is.
 */
`default_nettype none
`timescale 1ns / 1ps

module mc64_audio #(
    // One-pole low-pass, as a shift of the error term. 6 -> 16.0 kHz at 6.4 MHz.
    parameter int LPF_SHIFT = 6
) (
    // ---- the machine's side: the APU's level, in its own clock domain ----
    input  logic        clk,
    input  logic        reset,
    input  logic [15:0] level,      // 0 == silence; see mc64_top's audio note

    // ---- the Pocket's side ----
    input  logic        clk_audio,  // audio_mclk: 12.184615 MHz (nominal 12.288)
    output logic        audio_lrck, // clk_audio / 256
    output logic        audio_dac   // serial data, MSB first
);

    /* ------------------------------------------------------------------ *
     * The filter (clk domain), in 16.LPF_SHIFT fixed point
     * ------------------------------------------------------------------ */
    localparam signed [22:0] FILT_RND = 23'sd1 << (LPF_SHIFT - 1);   // rounded >>>

    wire  signed [22:0] filt_tgt = $signed({7'b0, level}) <<< LPF_SHIFT;
    logic signed [22:0] filt_q   = 23'sd0;

    always_ff @(posedge clk) begin
        if (reset)
            filt_q <= 23'sd0;
        else
            filt_q <= filt_q + ((filt_tgt - filt_q + FILT_RND) >>> LPF_SHIFT);
    end

    wire [15:0] filt = 16'((filt_q + FILT_RND) >>> LPF_SHIFT);

    /* ------------------------------------------------------------------ *
     * The request (clk_audio domain)
     *
     * div is the whole clock plan: div[1] is SCLK (mclk/4), div[7] is LRCK
     * (mclk/256), div[6:2] is the bit position inside the current slot, and the
     * wrap of div[7:0] is the frame boundary. The request goes at 0xC0, in the
     * second half -- 63 clocks before next frame's first-slot reload (0xFF) and
     * 191 before the second-slot reload after it (0x7F of the following frame),
     * so one request serves both slots of the frame about to start. See the
     * note above.
     * ------------------------------------------------------------------ */
    logic [ 7:0] div = 8'd0;
    logic        load_req = 1'b0;

    always_ff @(posedge clk_audio) begin
        div <= div + 8'd1;
        if (div == 8'hC0)
            load_req <= ~load_req;
    end

    assign audio_lrck = div[7];

    /* ---- request -> machine domain: latch the level, answer with a toggle ---- */
    logic [2:0] req_s = 3'd0;
    logic [15:0] held = 16'd0;
    logic        held_tog = 1'b0;

    always_ff @(posedge clk) req_s <= {req_s[1:0], load_req};

    always_ff @(posedge clk) begin
        if (req_s[2] ^ req_s[1]) begin
            held     <= 16'(filt);
            held_tog <= ~held_tog;
        end
    end

    /* ---- answer -> audio domain: take the settled word ---- */
    logic [2:0] tog_s = 3'd0;
    logic [15:0] sample = 16'd0;

    always_ff @(posedge clk_audio) tog_s <= {tog_s[1:0], held_tog};

    always_ff @(posedge clk_audio)
        if (tog_s[2] ^ tog_s[1])
            sample <= held;

    /* ------------------------------------------------------------------ *
     * The serializer
     *
     * SCLK falls where div[1:0] == 2'b11, and that edge is the one the data
     * line moves on (the conventional I2S placement -- the receiver samples on
     * the rising edge, in the middle of the bit). The shift register is reloaded
     * on the last bit of a slot, so the MSB of the new word goes out one SCLK
     * after LRCK changes, which is the one-bit delay I2S specifies. The upper
     * 16 bits of each 32-bit slot shift out zeroes.
     * ------------------------------------------------------------------ */
    logic [15:0] shift = 16'd0;

    always_ff @(posedge clk_audio) begin
        if (div[1:0] == 2'b11) begin
            audio_dac <= shift[15];
            if (div[6:2] == 5'd31)
                shift <= sample;      // the reload: 16 data bits then 16 zeroes
            else
                shift <= {shift[14:0], 1'b0};
        end
    end

endmodule
