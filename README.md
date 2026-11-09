<!-- SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later -->
<!-- SPDX-FileCopyrightText: 2026 Red Wire Technologies -->

# gr4-rwt

GNU Radio 4 out-of-tree module for **RWT radios**, the [Red Wire Technologies](https://redwiretechnologies.us) SDRs such as the Oxygen: source and sink blocks equivalent to the ones in [gr-rwt](https://github.com/redwiretechnologies/gr-rwt), written for GR4's C++23 block API.

The project was scaffolded with [gr4_modtool](https://github.com/gnuradio/gnuradio4-modtool) (`newmod`, `newblock --spec`), and `gr4_modtool check` passes. The radio code is shared with SoapyRWT, the SoapySDR module for the same radio: gr-rwt's packed sample format and FPGA registers, on libiio v1.

## Blocks

| GR4 block (registry key) | Ports | gr-rwt counterpart |
|---|---|---|
| `gr::rwt::RWTSource<T>` | `out` | `rwt_source` / `rwt_source_s16` with one of `ch1_en`, `ch2_en` |
| `gr::rwt::RWTDualSource<T>` | `out#0` (RX1), `out#1` (RX2) | `rwt_source` with both channels |
| `gr::rwt::RWTSink<T>` | `in` | `rwt_sink` with one channel |
| `gr::rwt::RWTDualSink<T>` | `in#0` (TX1), `in#1` (TX2) | `rwt_sink` with both channels |

`T` is `std::complex<float>`, scaled the way gr-rwt scales it (RX ÷2048, TX ×32768), or `std::int16_t`, which carries raw interleaved I,Q. The int16 type is GR4's convention (GR4's SoapySource uses it too); gr-rwt's `rwt_source_s16` used separate I and Q ports instead. Registry keys use GR4's type names: `gr::rwt::RWTSource<complex<float32>>`, `gr::rwt::RWTSink<int16>`, and so on.

```cpp
#include <gnuradio-4.0/rwt/RWTSource.hpp>
#include <gnuradio-4.0/rwt/RWTSink.hpp>

auto& rx = graph.emplaceBlock<gr::rwt::RWTSource<std::complex<float>>>({{"sample_rate", 1e6f}, {"frequency", 915e6}});
auto& tx = graph.emplaceBlock<gr::rwt::RWTSink<std::complex<float>>>({{"sample_rate", 1e6f}, {"frequency", 916e6}, {"attenuation1", 30.0}});
graph.connect<"out", "in">(rx, tx);
```

## Modes

| | `mode: local` (default when `uri` is empty) | `mode: remote` |
|---|---|---|
| Runs | on the RWT radio itself, like gr-rwt | on a host, against an RWT radio with the `griio` personality (`uri: ip:<host>`) |
| FPGA | `default` personality, loaded with `fpgaloader`; registers mapped from `/dev/mem` (root) | — |
| Samples | gr-rwt's packed 64-bit words with in-band tags | standard AD9361 layout |
| RX timestamps, PPS, overflow tags | yes | no |
| Timed TX bursts | yes | no (`tx_eob` still sends a partial buffer at once) |
| RX below 600 kS/s | FPGA CIC decimator ×10…10⁵ (down to 6 S/s) | AD9361 limits apply |

## Settings

Names and defaults follow gr-rwt's GRC blocks. RF and FPGA settings can be changed while the graph runs: the source applies changes on its I/O thread, the sink in `work()`. Device, channel and streaming settings (`uri`, `mode`, `channel`, `buffer_size`, …), and the sink's `use_tags`/`escape`, take effect at the next start.

| gr-rwt | GR4 setting | Notes |
|---|---|---|
| `freq` | `frequency` | RX/TX frequency, through the RWT Hardware Manager's `connection_control` (see below); shared by both channels |
| `samplerate` | `sample_rate` | float, Hz. RX uses the CIC below 600 kS/s (local mode). RX and TX share the AD9361 clock. |
| `bandwidth` | `bandwidth` | |
| `rfport1`, `rfport2` | `rf_port1`, `rf_port2` | RX `A_BALANCED`…, TX `A`/`B` |
| `gain_mode`, `manual_gain` | `gain_mode`, `gain` | applied to the channels in use |
| `attenuation1/2` (sink) | `attenuation1`, `attenuation2` | dB, 0–89.75 in 0.25 dB steps |
| `quadrature`, `rfdc`, `bbdc` | same | AD9361 tracking loops |
| `filter`, `auto_filter` | same | `auto_filter` uses libad9361 |
| `extra` | `extra` | `k=v,...` with gr-rwt keys (`rx_gain1`, `tx_gain`, …) or AD9361 attribute names |
| `escape`, `use_tags` | same | |
| `correction_enable`, `bypass_enable`, `decimation_arbitrary` | same (source) | CIC |
| `use_ext_pps`, `time_now`, `time_next_pps` | same (source) | time strings are `"secs,nsecs"` |
| `base_addr`, `force_reload`, `buffer_size` | same | `base_addr: 0` skips the FPGA registers (emulation) |
| — | `uri`, `mode`, `personality`, `load_fpga`, `check_blkid` | device selection, as in SoapyRWT |
| — | `api_url`, `api_transceiver` | the RWT Hardware Manager (below): empty `api_url` picks it from the radio, `none` turns it off; `api_transceiver` N selects `TransceiverN`: `0` (Transceiver0) on an Oxygen, `1` for Transceiver1 on radios that have one |
| — | `channel` | single-channel blocks: 0 = channel 1, 1 = channel 2 |
| — | `num_buffers`, `max_chunk_size`, `timeout_us`, `n_samples_max` (source), `max_queued_chunks`, `mute_on_stop` (sink) | streaming |

gr-rwt's `ctrl` message port is replaced by GR4's own mechanism, where settings can be changed through messages and tags. On gnuradio4-core before 9fda77c, a tag whose key matches a setting (`sample_rate`, `frequency`) updates a block that has not had that setting set explicitly (GR4's auto-update, as for GR4's SoapySink). From 9fda77c on, a setting follows a tag only when the application couples them, e.g. `block.settings().setTagCouplings({{"frequency", "gr:frequency"}})`.

## RWT Hardware Manager

RWT radios run the **RWT Hardware Manager**, the service that manages the radio's hardware, with a REST API (FastAPI, `http://<radio>/docs`). The blocks use its `connection_control` endpoints, which configure a whole RF path rather than the AD9361 alone:

| When | Request |
|---|---|
| source start, and each `frequency` change | `PUT /api/connection_control/rx_freq/{transceiver}/{path}/{Hz}` for each RX path the block uses (RX0, RX1) |
| source start, after the frequency | `PUT /api/connection_control/receive_enable/{transceiver}/{path}/true` |
| source stop, failed start, or destruction | `PUT .../receive_enable/{transceiver}/{path}/false` for the paths it switched on |
| sink start, and each `frequency` change | `PUT /api/connection_control/tx_freq/{transceiver}/{path}/{Hz}` for each TX path (TX0, TX1) |
| sink start, after the frequency | `PUT /api/connection_control/transmit_enable/{transceiver}/{path}/true` |
| sink stop, failed start, or destruction | `PUT .../transmit_enable/{transceiver}/{path}/false` for the paths it switched on |

`transceiver` is `Transceiver<api_transceiver>`: `Transceiver0` on an Oxygen (the default, `api_transceiver: 0`), `Transceiver1` with `api_transceiver: 1` on radios that have a second transceiver. The setting is an integer so that gr-studio's unquoted value loads. The path is the block's channel: a single-channel block uses `channel` (0 → RX0/TX0, 1 → RX1/TX1), a dual block both. The frequency is sent in Hz.

`api_url` empty means `http://127.0.0.1` in local mode (the graph runs on the radio), `http://<host>` for `uri: ip:<host>`, and no RWT Hardware Manager for anything else (the emulator, USB). `none` turns it off; any other value is used as the base URL. Without the RWT Hardware Manager, or when a frequency request fails, the block sets the AD9361 LO through libiio as before and says so on stderr. A `receive_enable` or `transmit_enable` that fails is reported on stderr, the block keeps running, and that path is not switched off later. The setting names keep the `api_` prefix so that saved graphs still load. With `GR4_RWT_TRACE=1` each request and its reply are printed.

Needs [cpp-httplib](https://github.com/yhirose/cpp-httplib) (header-only).

## Tags

Tag keys follow gnuradio4-core's `domain:name` scheme (from c145ebc): GNU Radio's own keys are `gr:` keys, and the keys only these blocks define are in the `rwt:` domain. On an older core the `gr:` keys are written as that core spells them, without the domain.

**RX (published by the source, local mode).** The source uses GR4's standard tag keys wherever one exists, rather than gr-rwt's `time`/`pps`/`overflow` keys.

| FPGA event | GR4 tag on the sample it precedes |
|---|---|
| time | `gr:trigger_name: "RWT_TIME"`, `gr:trigger_time` [ns] = sample index ÷ `sample_rate`, `gr:trigger_offset: 0`, `rwt:sample_index` (uint64) |
| PPS | `rwt:pps`, plus `gr:trigger_name: "RWT_PPS"` and `gr:trigger_time` while the time is known |
| overflow | `gr:rx_overflow: true`, `rwt:overflow`. Trigger times resume at the next FPGA time tag. |

The first sample, and the first after a `sample_rate` or `frequency` change, also carries `gr:sample_rate` and `gr:frequency`. `gr:trigger_time` is UTC only if the FPGA counter was set to UTC, for example with `time_next_pps`.

**TX (read by the sink, local mode):** GNU Radio's `gr:tx_*` burst keys and `rwt:tx_sample`, turned into the FPGA's in-band markers. The bare spellings gr-rwt uses (`tx_sob`, `tx_eob`, `tx_time`, `tx_sample`) are read too.

| Tag | Effect |
|---|---|
| `gr:tx_sob: true` | start of burst |
| `gr:tx_time` | hold the tagged sample until this time: an integer is nanoseconds, a float is seconds (gr-rwt used a `(secs, nsecs)` pair, which GR4's PMT has no type for) |
| `rwt:tx_sample` | hold the tagged sample until this FPGA DAC sample index |
| `gr:tx_eob: true` | the tagged sample ends the burst, and the burst is sent at once |

## Building

Requirements:

* GNU Radio 4 (`gnuradio4-core`), developed against 4.0.0-RC3 (core `6c917fa`); also tested with meta-sdr's `6b40a37`, with `83d15cc` (Oct 22, 2026), with `86457be` (Oct 28, 2026) and gnuradio4-blocks `7c9d119`, and with `2d3481c` (Nov 2, 2026) and gnuradio4-blocks `46b99b5` (Nov 3, 2026), the first with `domain:name` tag keys. The tests use gnuradio4-blocks' testing blocks under `gr::blocks::testing`; the `gr::testing` alias was removed in gnuradio4-blocks `a9cc695`. This needs GCC 14+ (GCC 15 recommended) or Clang 20+.
* **libiio ≥ 1.0** ([v1.0.0](https://github.com/analogdevicesinc/libiio/releases/tag/v1.0.0)). Build it with `WITH_EMU_BACKEND=ON` to run the emulator tests.
* libad9361, optional but recommended for low rates. Released versions are built on libiio 0.x and are skipped with a warning; use the libiio v1 port, branch [`staging/libiio1-support`](https://github.com/analogdevicesinc/libad9361-iio/tree/staging/libiio1-support). Set with `-DENABLE_LIBAD9361=AUTO|ON|OFF`.
* [cpp-httplib](https://github.com/yhirose/cpp-httplib), header-only, for the RWT Hardware Manager.
* Boost.UT, for the tests.

```sh
cmake -B build -G Ninja -DCMAKE_PREFIX_PATH=<gnuradio4 prefix>
cmake --build build
ctest --test-dir build
cmake --install build --prefix <gnuradio4 prefix>   # the block library goes to lib/gnuradio-4/plugins
```

When the tests are installed and run somewhere else, for example as a Yocto ptest package, these cache variables set the paths compiled into them:

| Variable | Default |
|---|---|
| `GR4_RWT_TEST_DATA_DIR` | `blocks/rwt/test/data` in the source tree |
| `GR4_RWT_TEST_EXAMPLE_FILE` | `examples/rwt_repeater.grc` in the source tree |
| `GR4_RWT_TEST_PLUGIN_DIR` | the block library's build directory |
| `GR4_RUNGRAPH` | `rungraph` found next to GR4 |
| `GR4_RWT_TEST_EXAMPLE` | `ON`; `OFF` skips `qa_Example` |

`FindLibIIO` ignores a libiio 0.x `libiio.pc`, so the v1 headers are found when a v0 install exists next to v1.

The Yocto layer `meta-gr4-rwt` builds the module with meta-sdr's GNU Radio 4 recipes.

Installed into the GR4 prefix, the block library sits in GR4's plugin directory, so GR4's tools load it without extra flags. Other projects use `find_package(gr4_rwt)` and link `gr4_rwt::blocks_rwt_headers`.

## Diagnostics

A block that failed to start says so on stderr (`<name>: start failed: ...`), and so does every later settings change sent to it: `settings change not applied, the block holds no radio (...)`. With `GR4_RWT_TRACE=1` in the environment of the process running the graph (rungraph, or gnuradio4-control-plane for gr-studio), each block also prints every settings change it applies, with what the AD9361 reads back:

```
tx: TX settings applied: attenuation1=20, frequency=434500000; AD9361 reads back out_altvoltage1_TX_LO_frequency=434500000, out_voltage0_hardwaregain=-20.00
```

A setting the radio cannot take never stops the graph by itself. GR4 turns an error a block reports into a scheduler error, which ends the whole run. gnuradio4-control-plane does not notice, so gr-studio then waits on "Connecting to live source..." forever. Instead:

* at start, the block fails its start with the reason on stderr, e.g. `start failed: TX sample_rate 1e+09 S/s is outside the AD9361's 520834 .. 61440000 S/s`;
* while running, the change is refused (`settings change refused: ...; keeping 2500000`), the setting goes back to the value in force, and the graph keeps running.

The AD9361 takes 520 834 to 61 440 000 S/s with libad9361 (2 083 334 S/s without it). In local mode the source's FPGA CIC goes down to 6 S/s.

GR4 answers a Set on the `settings` endpoint (gr-studio's `immediate` mode) with no reply at all, for every block, so gnuradio4-control-plane reports a timeout even though the change was applied. The `staged_settings` endpoint (gr-studio's default `staged` mode) replies.

## Example

`examples/rwt_repeater.grc` is an RX→TX repeater for an RWT radio:

```sh
rungraph --graph examples/rwt_repeater.grc --plugin-dir build/blocks/rwt
```

## Tests

`ctest` runs seven suites. None of them needs hardware.

| Suite | Covers |
|---|---|
| `qa_RWTSource` | source blocks in real GR4 graphs: FPGA time/overflow/PPS tags → GR4 tags at the right samples, a tag split across DMA blocks, escaped data, dual channel int16, remote mode RX2, error on a missing channel |
| `qa_RWTSink` | sink blocks: exact TX words for `tx_sob`+`tx_time`…`tx_eob` bursts, `tx_sample`, escaping, a burst across 1-sample chunks, dual-port marker merging, `use_tags: false`, remote layout |
| `qa_RWTRadio` | wire format against gr-rwt's packing expressions, CIC rate planning, time conversions, every AD9361 control, the `extra` aliases, and the missing-`fpgaloader` error |
| `qa_HardwareManager` | the RWT Hardware Manager against a mock of its HTTP server: `Transceiver0`/`Transceiver1` from `api_transceiver`; `rx_freq`/`tx_freq` at start and on a live retune, for the paths in use; `receive_enable`/`transmit_enable` on at start (after the frequency) and off once at stop, and a refused path never switched off; the libiio fallback when the RWT Hardware Manager fails or is unreachable; `api_url` resolution |
| `qa_Registry` | the compiled library registers all 4 blocks × 2 types; create by name with settings |
| `qa_LiveSettings` | settings changed on running blocks the way gnuradio4-control-plane (gr-studio) sends them: `staged_settings` and `settings` messages through the scheduler reach the source and the sink, under GR4's Simple single- and multi-threaded schedulers, also for a sink with no input; an out-of-range `sample_rate` is refused without stopping the graph; an RX retune leaves the sink's TX frequency alone |
| `qa_Example` | runs `examples/rwt_repeater.grc` with GR4's `rungraph` and checks every RX sample reaches TX |

The radio is libiio v1's emulation backend with libiio's FMCOMMS2 description (`blocks/rwt/test/data/fmcomms2.xml`), the same AD9361 device chain as the RWT radios. The emulator serves RX blocks from a file and records TX blocks, so the tests check the exact bytes. All suites also pass under AddressSanitizer with leak detection and under ThreadSanitizer, and 25 repeated runs show no flakes. GCC's UndefinedBehaviorSanitizer cannot compile GR4's compile-time port names (`"out#0"`) at all, so UBSan needs Clang.

## Status and assumptions

Tested against the emulator only, **not yet run on an RWT radio**. Local mode's FPGA registers and `fpgaloader` cannot be emulated. These points should be confirmed on hardware; they are the same open questions as in SoapyRWT:

1. **Sample index rate with the CIC.** FPGA times are converted with the user-facing RX rate (after the CIC), and TX `tx_time` with the TX rate.
2. **`EOB` placement.** The EOB marker precedes the word that holds the burst's last sample. `SOB` and `HOLD` start a word of their own, padding a half-filled single-channel word with a zero sample.
3. **Timestamps after an overflow** are withheld until the FPGA sends a new time tag.
4. **The `griio` layout** is assumed to be the standard AD9361 one.
5. **The shared AD9361 clock.** Source and sink each open the radio. Setting one block's rate moves the other's, and only the source accounts for its own CIC ratio.

## gr4_modtool notes

Building the scaffold against GR4 RC3 hit three modtool issues, worth reporting upstream:

* `cmake/Dependencies.cmake` calls `find_package(GnuRadioBlockLib)` inside a function. The Python interpreter core's macros find there goes out of scope, so the block-registration generator ran without one and configure failed with an empty error. Fixed here by passing `Python3_EXECUTABLE` to the parent scope.
* Exported targets had no `EXPORT_NAME`, so the build tree's `gr4_rwt::blocks_rwt_headers` was installed as `gr4_rwt::gr4_rwt_blocks_rwt_headers`. Fixed here.
* The docs list a `filter` archetype; the code calls it `sync`.

## gr-studio and newer cores

Since core `7f43a66` the graph loader refuses a block parameter that is not a writable setting. For an ordinary block (not `Resampling<>`/`Stride<>`) the read-only ones are `unique_name`, `input_chunk_size`, `output_chunk_size` and `stride`. gr-studio (up to at least `fbb3ac2`) writes all four into every block's `parameters`, so every graph it submits to such a core fails with `declares no setting named 'unique_name'` (or one of the others). This is not specific to these blocks. Either patch gr-studio to leave them out, or use a core before `7f43a66`, such as meta-sdr's `6b40a37`.

## License

gr4-rwt is © 2026 Red Wire Technologies and dual licensed (`LICENSE`):

* **Used with Red Wire Technologies hardware** (such as the Oxygen): the terms of the Apache License 2.0, limited to that use (`LICENSE.RWT-Hardware`, with the Apache text in `LICENSE.Apache-2.0`). "Used with" means running on the hardware, or controlling or exchanging data with it, so remote mode on a host counts.
* **Any other use:** GPL-3.0-or-later (`LICENSE.GPL-3.0`). This option is always available, also on RWT hardware.

SPDX: `LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later`.

The wire format and register handling are a reimplementation of the protocol gr-rwt (Red Wire Technologies) uses with the RWT radios' FPGA; no gr-rwt code is included. Two files keep their own licenses: `blocks/rwt/test/data/fmcomms2.xml`, from libiio v1.0.0's test resources (GPL-2.0-or-later, libiio's license for its tests; used only by the tests, unmodified), and `cmake/Modules/FindLibAD9361.cmake` (BSD-3-Clause, as its header states).

Every file carries an `SPDX-License-Identifier` and `SPDX-FileCopyrightText` header, or, where it cannot carry a comment (`CMakePresets.json`, `fmcomms2.xml`), a `.license` file next to it, as [REUSE](https://reuse.software) does. The scaffold was generated by gr4_modtool (MIT).
