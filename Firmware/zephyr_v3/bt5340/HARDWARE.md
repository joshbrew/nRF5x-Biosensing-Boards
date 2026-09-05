> Standalone copy: **bt5340** (default). Enabled optional features: none. The tables below cover all carriers; only this copy's `prj.conf` and `app.overlay` are built. In an expanded copy the optional wiring is already enabled.

# Carrier wiring and bring-up assumptions

Pin names below are **Nordic GPIO numbers**, not Fanstel module pad numbers.
Assignments were carried from the existing executable firmware/overlays; they
are not independently verified against a schematic or physical board.
SDK development-kit LEDs/buttons/UART/QSPI defaults are disabled or remapped.
The nRF52810 profile removes the SDK's nRF52832-emulation compile definitions
so the real chip does not execute the emulated DK's NFC/UICR setup.

## Base peripherals

| Signal | BT5340 / BT40 | BT840 | BT832A / nRF52810 |
| --- | --- | --- | --- |
| SPI0 SCK | P0.23 | P0.15 | P0.29 |
| SPI0 MOSI | P0.29 | P0.14 | P0.25 |
| SPI0 MISO | P0.20 | P1.02 | P0.28 |
| ADS0 CS, active low | P0.22 | P1.01 | P0.05 |
| ADS0 DRDY, active low | P0.15 | P0.06 | P0.04 |
| ADS0 RESET, active low | P0.12 | P0.08 | P0.07 |
| I2C1 SDA / SCL | P1.02 / P1.03 | P0.26 / P0.27 | Not included |
| Status LED, active high | P1.06 | P0.25 | P0.13 |

The nRF52810 uses non-DMA SPI (`nordic,nrf-spi`), not SPIM. Larger chips use
SPIM. SPI mode 1, 8 MHz SCK, 30-byte / 24-bit-word ADS131M08 frames are used.

I2C runs at 400 kHz with external pull-ups assumed. Fixed addresses: MAX30102
0x57, MPU6050 0x68, QMC5883L 0x0d, BME/BMP280 0x76. Change the overlay's
`bio_bme` address to 0x77 if needed. The other addresses are in `src/sensors.c`.
The four sensors are polled: MPU 100 Hz, MAX red/IR 100 SPS, QMC 100 Hz, BME
approximately 1 Hz. Presence of all four is not required to use an ADC.

## Optional wiring — verify before selecting features

| Feature | Wiring chosen |
| --- | --- |
| BT5340 second ADC | Shared SPI0; CS P0.30, DRDY P0.11, RESET P0.09 |
| BT840 second ADC | Shared SPI0; CS P1.04, DRDY P1.08, RESET P0.29 |
| BT840 `adc-clock` | PWM0 output 0 on P1.05, **4 MHz** |
| BT5340 `audio` | I2S0 SCK P1.11, LRCK P0.08, SDOUT P1.14 |
| BT5340 `dmic` | PDM0 CLK P1.08, DIN P0.13 |
| BT5340 `aux-uart` | UART2 TX P1.15, RX P1.13; 115200, 8N1 |
| USB on BT5340/BT840 | Native USB D+/D-/VBUS hardware, not an arbitrary UART bridge |

Second-ADC wiring is disabled by default, including its additional chip-select
GPIO. The BT5340 old overlay used CS P0.25, but `main.cpp` used P0.30. The
BT840 old overlay used CS P1.15, while `main.cpp` used P1.04; its reset comments
also disagreed. The new optional overlays follow `main.cpp`. This is a documented
choice for bring-up, **not a resolution of the schematic discrepancy**.

The old BT5340 audio comments disagreed on SDIN/SDOUT P0.09/P1.14. This project
follows the executable overlay's SDOUT P1.14. Only output is configured.
The old TLC5940 used GSCLK P1.07, XLAT P1.08, BLANK P1.04, SIN P1.05,
SCLK P1.06. XLAT conflicts with DMIC CLK and SCLK with the status LED, so TLC
must not be added unchanged alongside those features.

## ADC clock and sample rates

The ADS requires a real CLKIN signal; SPI SCK does not replace it. Default
register settings assume **8.192 MHz CLKIN**, nominal 250 SPS, gain 32 on all
eight channels. Defaults are written and checked after reset. A valid 0x28xx
ID is required before the driver reports ready. Register changes require that
ADC's acquisition to be stopped. Output CRC is checked before publishing data.
See the [TI ADS131M08 datasheet](https://www.ti.com/lit/ds/symlink/ads131m08.pdf)
for clock/rate relationships, frame format and register meanings.

The old BT840 draft attempted an 8.192 MHz PWM workaround. Nordic's PWM
COUNTERTOP has a minimum of 3, so a two-tick/8 MHz setting is invalid even if
the driver accepts it. See the [Nordic nRF52840 product specification](https://docs-be.nordicsemi.com/bundle/nRF52840_PS_v1.8/raw/resource/enus/nRF52840_PS_v1.8.pdf), PWM chapter.
The optional `adc-clock` uses four 16 MHz counter ticks for **4 MHz at 50% duty**
and requires P1.05 wired to CLKIN. Do not enable it if an oscillator is already
driving the same net. Actual sample rates then scale by 4/8.192:

| Rate command | 8.192 MHz external clock | 4 MHz PWM clock |
| --- | ---: | ---: |
| 250 | 250 SPS | 122.0703125 SPS |
| 500 | 500 SPS | 244.140625 SPS |
| 1000 | 1000 SPS | 488.28125 SPS |
| 2000 | 2000 SPS | 976.5625 SPS |

Clock availability/routing on each carrier still needs measurement. The
system's low-frequency clock uses the internal RC source for initial bring-up,
avoiding an assumed external 32.768 kHz crystal. This does not eliminate the
module's high-frequency crystal or the ADC's own clock requirements.

## Bench order

1. Verify chip marking, power rails, SWD and clock sources; keep electrodes or
   external analog sources disconnected during initial electrical checks.
2. Program the correct core images only after reviewing flash layout and pins.
3. Check BLE/RTT and LED with `minimal`, then the default profile.
4. Confirm ADC ID, clock frequency, DRDY timing, CRC acceptance, gain and known
   test input. `ads_missed` counts coalesced IRQs plus SPI/CRC failures, not every
   possible physical sample loss.
5. Validate each I2C sensor separately. Compare scaling and packet decoding.
6. Add USB/second ADC/audio/microphone/UART individually before combinations.
7. Stress disconnect/reconnect, host backpressure, simultaneous streams and
   runtime stacks. Especially check the nRF52810's tight RAM/stack budget.
