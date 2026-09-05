# bt5340_min — standalone nRF Connect application

Fanstel BT40 / nRF5340: ADS131M08, I2C sensors, BLE, beacon scanning and RTT shell. Sysbuild supplies the matching network-core firmware.

This is a BLE/command/LED baseline. ADC acquisition and I2C sensor/scanning support are disabled.

## Open, build and flash in VS Code

1. Open **this folder** with File > Open Folder, or use nRF Connect > Open an existing application and choose this folder (the one containing CMakeLists.txt and prj.conf).
2. Add Build Configuration. Select **nRF Connect SDK v3.3.1**, its matching toolchain, and board **`nrf5340dk/nrf5340/cpuapp`**. Keep **sysbuild enabled** and the build directory `build`.
3. Leave extra CMake arguments, extra configuration files and extra devicetree overlays empty. The local `prj.conf` and `app.overlay` already contain this preset.
4. Select Build. After checking the carrier wiring and connecting the intended module through an SWD debug probe, select the **top-level build configuration's Flash** action. This programs both the application and hci_ipc network-core domains; do not flash only the application image.

These copies do not need build.ps1, the original source tree, or any files from sibling applications. You can copy this folder alone to another **short, space-free path**. Do not copy its generated build directory; create a fresh build at the new location. Changing SDK, board, or configuration may require a pristine build. The board guard deliberately rejects other targets.

Edit this copy's `src/`, `include/`, `prj.conf` and `app.overlay` directly. Changes are not synchronized to other copies. Kconfig and CMake still exclude unsupported drivers from the linked image; unused C source files do not consume device memory. All sampling/scanning and audio capture/output start off (the expanded BT840 clock output is the exception).

No device has been flashed or hardware-tested as part of build verification. Images have no MCUboot/DFU upgrade path, BLE is unpaired/unencrypted for bench use, and USB IDs are placeholders. Flashing replaces the current firmware. Avoid erase/recover unless you intend to lose existing device data.

See the local [HARDWARE.md](HARDWARE.md) and [PROTOCOL.md](PROTOCOL.md) before use. The nRF52810 has 192 KiB flash and 24 KiB RAM; its framed BLE stream is required for larger ADC packets.
