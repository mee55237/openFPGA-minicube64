# openFPGA-minicube64
Minicube64 for the Analogue Pocket

Minicube64 is a fantasy console based on 6502, with the intention of helping people learn how games were made for older systems such as Atari 26OO, NES or C64, but without many of the difficult hardware restrictions.

It runs 6.4mhz, with 64kb of ram and a simple 64x64 pixel framebuffer with 64 colors.

As it is based on 6502, Minicube64 follows the same rules, directives and instructions that are common to those systems. 

https://aeriform.itch.io/minicube64
https://github.com/aeriform-io/minicube64 

Note that Arlet 6502 does not support common illegal instructions.

ROMS/carts: Unlike the official Minicube64, this cannot bundle the Loopy Assembler. This repo does have pre-assembled roms for convenience, and if a rom is in .s then just run it in the official emulator to get the .bin. Default file extensions supported by data.json are .bin and .mc64.   

AI use: Yes this is ai, but NOT a slop core, this uses mostly human modules (Arlet 6502 by Arlet Ottens and Anders and apu by Ludvig Strigeus). I wrote this myself. I hand tested all test roms, everything should work; if not submit issues or pull requests please.  For those curious, I used DeepSeek V4.1 Flash max reasoning in freebuff. 

Credits: ΛERIF°RM and MonstersGoBoom for Minicube64 itself and docs. Arlet Ottens and Anders for the 6502 cpu. [Agg23](https://github.com/agg23/openfpga-NES), [Ludvig Strigeus](https://github.com/strigeus/fpganes), [@sorgelig](https://github.com/sorgelig), [@greyrogue](https://github.com/greyrogue), [@Kitrinx](https://github.com/Kitrinx), [@paulb-nl](https://github.com/paulb-nl), and many more.

Licenses: I am not a lawyer, but some components of this used GPLv3 and that is why this is GPLv3

