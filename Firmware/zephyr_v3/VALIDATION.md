# Standalone VS Code project verification

Verified on **2026-09-04**, Windows, with nRF Connect SDK **3.3.1**, toolchain
**936afb6332** / Zephyr SDK 0.17.0, ARM GCC 12.2.0. Zephyr reports **4.3.99**.
The installed nRF Connect extension's settings schema was checked against
version **2026.8.2347**.

## Direct builds

All eight applications were built directly from their own source folders into
their own `build` directories. The standard sysbuild entry point was used with
only the SDK environment, board selection, source directory and build directory.
There was **no build.ps1, source staging, EXTRA_CONF_FILE, EXTRA_DTC_OVERLAY_FILE,
custom feature argument or SDK modification**. Compiler caches are redirected
inside each build by the application's own CMake setup.

| Application | Flash used (bytes) | RAM used (bytes) | Result |
| --- | ---: | ---: | --- |
| bt832a | 129140 | 23108 | Pass |
| bt832a_min | 121744 | 21316 | Pass |
| bt840 | 223292 | 54396 | Pass |
| bt840_min | 172964 | 46140 | Pass |
| bt840_full | 253228 | 64948 | Pass |
| bt5340 | 159676 | 54348 | Pass |
| bt5340_min | 142064 | 48324 | Pass |
| bt5340_full | 199184 | 71924 | Pass |

Each nRF5340 `hci_ipc` network image uses **142280 bytes flash / 38080 bytes RAM**.
All images remain inside the reviewed memory budgets from the shared project.
The nRF52810 default leaves **1468 bytes** outside its statically allocated RAM;
this is not a measurement of runtime stack high-water marks.

The SDK emits its existing Partition Manager deprecation warning. No application
compiler errors were observed. The flash layout has not been migrated or given
a bootloader as part of this packaging change.

## Checks performed

- Generated CMake caches point at each independent application, its own
  `prj.conf` and its own `app.overlay`; no hidden extra fragments were used.
- Sources, headers and custom bindings in all eight copies match the previously
  tested shared application byte for byte. Only packaging/configuration and
  the nRF52810 runner setup differ; runtime behavior has not been expanded.
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
  artifacts exist. nRF5340 sysbuild flash order includes `hci_ipc` followed by
  the application; nRF52 has one flash domain.
- The workspace lists all eight applications using portable workspace-relative
  paths, and each copy contains its local hardware/protocol documentation.
- Shared protocol/transport C tests: **2/2 pass**. Python tests: **15/15 pass**.
  The direct-build audit also passes with Python optimization enabled (`-O`).

The optional read-only audit is `../zephyr v3/tests/verify_vscode.py`. It checks
the generated builds and budgets; **it is not needed to build or flash in VS
Code**, and it does not access devices. Regular development uses the extension's
Build and Flash actions described in README.md.

## Verification boundaries

These results validate the normal build entry point and generated IDE/runner
inputs. **The VS Code UI was not driven interactively, no probe was accessed,
and nothing was flashed.** Probe routing, device protection, electrical wiring,
ADC clocks, physical sampling, sustained throughput, reconnect behavior on
hardware and runtime stack margin still require bench testing.

Start with a default or minimal copy. Do not flash `bt840_full` onto a carrier
whose ADC clock net is already driven externally: this preset enables the
P1.05 clock output at boot. Confirm all expanded wiring against the hardware.
If moving a copy, do not reuse its old build cache; build afresh at the new
short, space-free source path.
