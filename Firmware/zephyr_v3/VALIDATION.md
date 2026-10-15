# Standalone VS Code project verification

Verified on **2026-10-15**, Windows, with the existing nRF Connect SDK **3.3.1**, toolchain
**936afb6332** / Zephyr SDK 0.17.0, ARM GCC 12.2.0. Zephyr reports **4.3.99**.
West reports **1.5.0**; CMake reports **4.2.1**. These results apply to the installed
SDK, without upgrading or modifying it.

## Direct builds

All eight applications passed pristine builds directly from their own source
folders into their own `build` directories. All three nRF5340 `hci_ipc` network
images also passed: **11 images total**, with no compiler errors or warnings.
The standard `west build --sysbuild` entry point was used with the SDK environment,
board selection, source directory and build directory. An audit loop scheduled
the independent builds; it did not stage sources or supply feature/configuration
fragments. Compiler caches remain inside each build through the application's
own CMake setup.

From an NCS 3.3.1 toolchain terminal with `ZEPHYR_BASE=C:/ncs/v3.3.1/zephyr`, the
individual command is:

```text
west build --sysbuild -p always -b <board-target> -s <application-folder> -d <application-folder>/build
```

Use the board targets in [README.md](README.md). The completed CLI logs are saved
locally at `<application-folder>/build-audit/west-build.log`, alongside the exit
status in `exit-code.txt`. These generated audit files and build outputs are
ignored by Git. The temporary read-only artifact audit passed with no findings;
its local result is `bt832a/build-audit/artifact-audit.json`.

| Application | Flash used (bytes) | RAM used (bytes) | Result |
| --- | ---: | ---: | --- |
| bt832a | 129140 | 23108 | Pass |
| bt832a_min | 121744 | 21316 | Pass |
| bt52840 | 223292 | 54396 | Pass |
| bt52840_min | 172964 | 46140 | Pass |
| bt52840_full | 253228 | 64948 | Pass |
| bt5340 | 159676 | 54348 | Pass |
| bt5340_min | 142064 | 48324 | Pass |
| bt5340_full | 199184 | 71924 | Pass |

Each nRF5340 `hci_ipc` network image uses **142280 bytes flash / 38080 bytes RAM**.
All images fit their board's configured flash and RAM regions. These figures
match the previous 2026-09-04 validation table exactly.
The nRF52810 default leaves **1468 bytes** outside its statically allocated RAM;
this is not a measurement of runtime stack high-water marks.

## Warnings and environment

The only build warnings are the existing SDK Partition Manager deprecations:

- Sysbuild Kconfig: `Deprecated symbol PARTITION_MANAGER is enabled.`
- Application Kconfig: `Deprecated symbol PARTITION_MANAGER_ENABLED is enabled.`
  This also occurs for each nRF5340 network image.
- Nordic sysbuild CMake: `SB_CONFIG_PARTITION_MANAGER` is deprecated; the SDK
  recommends migrating boards to devicetree-based partitioning.

These are configuration warnings, not compiler warnings. The flash layout has
not been migrated or given a bootloader during this review.

Initial attempts in the restricted execution environment failed before
compilation because Windows refused to launch ARM GCC with an illegal system
DLL relocation error. The installed compiler and all builds succeeded outside
that restriction. The completed logs above contain the successful builds.

## Checks performed

- Generated CMake caches point at each independent application, its own
  `prj.conf` and its own `app.overlay`; no hidden extra fragments were used.
- Sources, headers and custom bindings in all eight copies match each other byte
  for byte. The firmware sources were reviewed and remain unchanged by this audit.
- Correct board selections are accepted and incorrect board selections are
  rejected by all eight board guards.
- nRF52810 builds retain 192 KiB flash / 24 KiB RAM, non-DMA SPI, MTU 64,
  no nRF52832-emulation definitions and the ADC-only feature restrictions.
- nRF52840 has no audio/microphone; expanded features appear only in the intended
  copies. Minimal copies exclude acquisition/sensor/scanning support.
- nRF52810 J-Link metadata contains exactly one device selection:
  [`nRF52810_xxAA`](https://www.segger.com/supported-devices/nordic-semi/nrf52/nrf52810_xxaa/).
  The inherited nRF52832 pyOCD runner is excluded. This SDK-specific hook is
  covered by the runner audit and should be rechecked when upgrading NCS.
- Every application has an ELF, HEX and valid runner metadata. Referenced flash
  artifacts and BIN files exist for all 11 images. nRF5340 sysbuild flash order
  includes `hci_ipc` followed by the application; nRF52 has one flash domain.
- nRF5340 network images disable console, serial and logging and select the RC
  low-frequency clock, as specified by the local network-core configuration.
- Generated compile commands include exactly the application modules selected
  by each preset; disabled features do not compile into the application.
- The workspace lists all eight applications using portable workspace-relative
  paths, and each copy contains its local hardware/protocol documentation.

## Corrections from this review

- Fixed three broken workspace folder paths and their application registrations:
  `bt840`, `bt840_min`, `bt840_full` now reference the existing `bt52840` folders.
- Corrected the top-level `bt52832a` folder references to the existing `bt832a`
  and `bt832a_min` folders, and aligned nRF52840 documentation/configuration labels.
- Removed instructions pointing to the absent shared `zephyr v3` tree and its
  `tests/verify_vscode.py` audit. The previous C/Python test suites are not in this
  checkout and were **not rerun**; their historical pass counts are not current
  verification results. Regular development uses the normal Build and Flash
  actions described in README.md.

## Verification boundaries

These results validate the normal build entry point and generated IDE/runner
inputs. **The VS Code UI was not driven interactively, no probe was accessed,
and nothing was flashed.** Probe routing, device protection, electrical wiring,
ADC clocks, physical sampling, sustained throughput, reconnect behavior on
hardware and runtime stack margin still require bench testing.

Start with a default or minimal copy. Do not flash `bt52840_full` onto a carrier
whose ADC clock net is already driven externally: this preset enables the
P1.05 clock output at boot. Confirm all expanded wiring against the hardware.
If moving a copy, do not reuse its old build cache; build afresh at the new
short, space-free source path.
