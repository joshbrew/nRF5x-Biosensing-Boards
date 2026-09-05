# Fanstel Zephyr v3 — open directly in nRF Connect for VS Code

These are **eight independent application copies**, one for every tested board
and preset. Each contains its own C sources, headers, Kconfig, `prj.conf`,
`app.overlay`, bindings and sysbuild setup. No build helper, source staging,
external source tree, extra configuration flags or custom board installation is
required. The SDK's board target supplies the chip support; `app.overlay`
supplies the Fanstel carrier wiring.

Use **nRF Connect SDK v3.3.1** with its matching toolchain (tested installation:
`936afb6332`, Zephyr SDK 0.17.0). The firmware revision is v3; this is not a
Zephyr 3.x compatibility claim. This SDK reports Zephyr 4.3.99.

## Pick a project

Start with the default project for your module. The `_min` copies are BLE and
command/LED baselines with acquisition disabled. The `_full` copies include the
optional wiring and need a hardware check before flashing.

| Application folder | Board target to select | Included features |
| --- | --- | --- |
| `bt52832a` | `nrf52dk/nrf52810` | One ADS131M08, BLE, minimal RTT logs; ADC-only sensor profile |
| `bt52832a_min` | `nrf52dk/nrf52810` | BLE/command/LED baseline; no ADC acquisition |
| `bt52840` | `nrf52840dk/nrf52840` | ADC, I2C sensors, BLE, beacon scanning, RTT shell; **no audio** |
| `bt52840_min` | `nrf52840dk/nrf52840` | BLE/command/LED baseline |
| `bt52840_full` | `nrf52840dk/nrf52840` | BT840 default plus second ADC, USB and 4 MHz ADC clock; **no audio** |
| `bt5340` | `nrf5340dk/nrf5340/cpuapp` | ADC, I2C sensors, BLE, beacon scanning, RTT shell |
| `bt5340_min` | `nrf5340dk/nrf5340/cpuapp` | BLE/command/LED baseline |
| `bt5340_full` | `nrf5340dk/nrf5340/cpuapp` | BT5340 default plus second ADC, USB, I2S test tone, PDM microphone and RP2040 UART |

BT5340 here means the existing Fanstel BT40/nRF5340 project. BT840 is nRF52840;
BT832A follows the existing project's nRF52810 target, despite its historical
`nrf52832A` directory name. The nRF52810 has **192 KiB flash and 24 KiB RAM**.
Its configuration remains size-optimized and uses the smaller framed BLE MTU.

## VS Code workflow — no terminal commands

1. Open **Fanstel-v3.code-workspace**, or open one application folder directly
   using File > Open Folder. You can also use nRF Connect's **Open an existing
   application** and select the folder containing `CMakeLists.txt` and `prj.conf`.
2. In nRF Connect's Applications view, select the desired application and
   **Add Build Configuration**. Select **SDK v3.3.1**, its matching toolchain,
   and the exact board target from the table.
3. Keep **sysbuild enabled** and use the default short build-directory name
   `build`. Leave extra CMake arguments, extra Kconfig files and extra devicetree
   overlays empty. Do not select a non-secure `/ns` or network-core target.
4. Select **Build Configuration / Build**. All configuration and wiring are
   picked up automatically from that application's `prj.conf` and `app.overlay`.
5. After verifying wiring, power and the intended SWD debug-probe connection,
   select **Flash on the top-level build configuration**. On nRF5340 this includes
   both the application and the matching `hci_ipc` network-core image. Do not
   flash only the application domain or choose an unrelated network firmware.

This follows Nordic's documented [build workflow](https://docs.nordicsemi.com/r/bundle/nrf-connect-vscode/page/get_started/build_app_ncs.html/how-to-build-an-application?contentId=biGm8MMyYkhFtpvtAIa1pQ)
and [flash workflow](https://docs.nordicsemi.com/r/bundle/nrf-connect-vscode/page/get_started/quick_debug.html/how-to-flash-an-application?contentId=ez7Ifp_Bz_bo~CrwGtbvew).
The workspace only registers the applications and recommends the Nordic
extension pack; it does not install an SDK, pick a probe, run custom tasks or
override your global VS Code settings. Build configuration choices are stored
normally by the extension after you select them.

If the folder does not appear automatically, use **Open an existing application**
on that specific folder. Select a short `build` directory and start a pristine
build when changing SDK or configuration. The small nRF52810 configuration
should retain its size optimization; do not turn on debug-oriented optimization
or extra logging without rechecking its memory usage.

## Safe defaults and hardware limits

- Default applications do not generate an ADC clock and leave optional second
  ADC, USB, audio, microphone and auxiliary UART off. Sampling and scanning start
  stopped. Missing sensors do not prevent BLE access.
- **`bt52840_full` drives P1.05 with a 4 MHz clock at boot.** Do not flash it if an
  external oscillator drives the same net. ADS rate labels assume 8.192 MHz, so
  actual rates with this clock are multiplied by 4 / 8.192. For USB/second ADC
  with an external clock, change `CONFIG_BIO_ADC_CLOCK=n` in that copy's `prj.conf`
  and disable/remove the PWM clock section in `app.overlay` before rebuilding.
- Expanded second-ADC and audio wiring follows the documented choices from
  conflicting legacy files. Check the local `HARDWARE.md` before using it.
- No TLC5940, old FFT/assets, bootloader/OTA upgrade path or complete legacy
  command compatibility has been added. USB identifiers are placeholders and
  BLE is unpaired/unencrypted for controlled bench testing.
- A successful build does not validate electrical wiring, runtime stack margin
  or sensor/radio throughput. Flashing replaces the existing firmware; do not
  select erase/recover unless you intend to lose existing device data.

## Editing, copying and directory names

Edit the chosen application's **own** `src/`, `include/`, `prj.conf` and
`app.overlay`. Board and preset settings are flattened into those two config
files; there are no hidden feature-selection arguments. The local `board.cmake`
rejects an incorrect board rather than reusing incompatible carrier wiring.
The CMake source list still excludes disabled modules from the binary.

These are deliberately independent copies. Changes do **not** synchronize to
other applications or to the earlier shared `zephyr v3/app`. Keep fixes aligned
manually if you intend to maintain multiple copies. Do not edit generated files
under `build`; pristine builds replace them.

The new sibling is named **`zephyr_v3`**, with an underscore, because the Windows
Zephyr toolchain does not reliably handle spaces in source/configuration paths.
The earlier **`zephyr v3`** folder and its CLI workflow remain intact. Do not open
its generated `.zephyr-v3-build` staging directories as your source projects.

You can copy any application folder by itself to another **short, space-free
path** and open it in VS Code. Exclude its generated `build` directory, or create
a new build directory after moving: CMake caches contain absolute paths. No file
from the parent workspace or a sibling application is needed to compile it.

See each application's local hardware/protocol documentation and
[VALIDATION.md](VALIDATION.md) for verification scope and results.
