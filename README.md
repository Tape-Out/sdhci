# sdhci

SD host controller for the SD bus in one-bit mode, with the register layout of the Linux `sdhci` driver.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. Card initialisation commands and single-block PIO reads and writes work in simulation against a behavioural card model.

Register offsets and bits follow `drivers/mmc/host/sdhci.h` in Linux. This is not a claim of conformance to the SD Association's host controller specification, and an implementation of the SD specifications may require a license from the SD Association, SD Group, SD-3C, LLC or other third parties.

| Offset | Register | Implemented |
| :-- | :-- | :-- |
| 0x04 | block size, block count | 512 bytes, one block |
| 0x08 | argument | 32 bits |
| 0x0C | transfer mode, command | the read direction; writing the word sends the command |
| 0x10–0x1C | response | 48-bit responses in the first word, 136-bit responses without their CRC byte over all four |
| 0x20 | buffer data port | PIO, four bytes per word, lowest byte first |
| 0x24 | present state | command and data inhibit, read and write active, buffer read and write enable, DAT0 and CMD levels |
| 0x28 | host control, power control | one-bit bus only |
| 0x2C | clock control, timeout control, software reset | the reset bits clear themselves in one cycle |
| 0x30–0x38 | interrupt status, status enable, signal enable | command and transfer complete, buffer read and write ready, and the command and data errors below |
| 0x40 | capabilities | 3.3 V |
| 0xFC | host controller version | 3.00 |

- Only aligned 32-bit accesses are supported; a Linux platform driver has to supply 32-bit register accessors.
- Commands and 48-bit responses carry CRC7; data blocks carry CRC16. A response is checked for its CRC and index only when the command asks for it, and always for its end bit.
- A command with a busy response completes when the card releases DAT0.
- After a block write the controller reads the card's CRC status token and waits for the card to release DAT0.
- Errors: command timeout, CRC, end bit and index; data timeout, CRC and end bit. A status bit is only set when its status enable bit is set.
- The SD clock is the system clock divided by 2 × (divider + 1); SDHCI's divided and programmable clock modes are not implemented.

Not implemented yet:

- four-bit and eight-bit buses, high speed and UHS modes with voltage switching and tuning;
- DMA and ADMA, multi-block transfers, auto CMD12 and CMD23;
- SDIO, card detection, eMMC;
- 8-bit and 16-bit accesses at unaligned offsets.

These registers keep the Linux layout so a driver can write them, but this version ignores what is written: block size and block count (fixed at one 512-byte block), the block count enable in the transfer mode, the data timeout counter (the controller times out on its own clock count), the four-bit bus width bit, and bus power and voltage select. They are listed in `ip.yaml` under `test.unused`.

## References

See [NOTICE](NOTICE).

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
