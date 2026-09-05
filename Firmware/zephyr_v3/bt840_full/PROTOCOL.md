# Transport and commands

UUIDs use the full suffix `-b0ba-8bad-f00d-deadbeef0000`.

| UUID first group | Purpose |
| --- | --- |
| `0000cafe` | Primary service |
| `0002cafe` | Legacy ADS0 notification |
| `0003cafe` | Legacy MAX30102 notification |
| `0004cafe` | Legacy MPU6050 notification |
| `0005cafe` | Legacy ADS1 notification, when enabled |
| `0006cafe` | Legacy BME/BMP280 notification |
| `0008cafe` | Legacy QMC5883L notification |
| `0009cafe` | Limited legacy binary control write |
| `000acafe` | Legacy iBeacon report, when enabled |
| `000bcafe` | New UTF-8 command write |
| `000ccafe` | Last text reply read / framed reply notification |
| `000dcafe` | New framed sensor notification stream |

There is no advertising name assumption on the client beyond discovery; service
UUID is shared between profiles. Sensor characteristics absent from a build
are omitted, except ADS0's legacy characteristic remains in the minimal build.
Only one BLE connection is supported. No pairing, encryption or authorization
is enforced in these development profiles.

## Commands

Text commands are documented in README. BLE writes contain one complete command,
1..127 bytes, no embedded NUL; prepare/offset writes are unsupported. A `bio`
prefix is only used for the RTT shell. Numbers accept C base-0 notation: decimal,
`0x` hexadecimal, and leading-zero octal; do not pad decimal numbers with zeros.
USB accepts CR/LF-delimited commands and discards overlong lines through the
next delimiter. The RP2040 UART's received lines are published as data, not
executed as firmware commands. `uart TEXT` sends a newline-terminated line.

Commands are queued; a successful BLE write acknowledges queue acceptance, not
successful hardware execution. Read/subscribe to the reply characteristic for
`ok`, status/help text, or a negative errno. Audio/microphone starts acknowledge
the request; later hardware errors appear in logs. Queues are bounded; clients
should send one command at a time and wait for its reply. Keep BLE commands
within the negotiated ATT payload (MTU minus 3).

USB commands now carry the session token from the DTR-open interval in which
the completed line was accepted. Closing DTR, resetting the bus or removing
VBUS invalidates pending commands/replies and clears the application's partial
RX line and TX ring. BLE similarly invalidates pending work and resets the last
readable reply to its initial help hint. Duplicate DTR-open events do not create
a new session. Sampling does **not** stop automatically on disconnect, and a
command already executing is not cancelled. Bytes already handed to a transport
driver are outside the application-ring reset; recreate the host decoder after
reconnect. Hardware reconnect/driver-buffer behavior still needs bench testing.

## Status and diagnostics

`status` reports `built`, `ready`, and `running` capability masks. Bits 2..10
follow the stream IDs below; bit 11 is speaker output and bit 12 is USB.

- `built`: functionality included in this firmware.
- `ready`: sensor probe succeeded, or the relevant interface driver initialized.
  Audio/DMIC readiness does not prove a speaker/microphone is physically attached.
- `running`: ADC/I2C sampling enabled, speaker/DMIC driver started, beacon scanning
  active, UART bridge listening, or USB DTR open, as appropriate.
- `ble`: current BLE connection state, independent of beacon scanning.

The legacy `ads_mask`, `ads_missed` and `ble_drop` summary fields remain. Other
older status fields (`sensors_mask`, `usb`, `audio`, `dmic`, `uart`) are replaced
by the capability masks. Status is a best-effort snapshot across workers, not
an atomic measurement of the entire system or proof of successful samples.

`stats` returns cumulative unsigned 32-bit counters, reset on reboot and wrapping
at 2^32. They are diagnostic events and must not be added as a sample-loss total:

| Field | Meaning |
| --- | --- |
| `ads_irq` | DRDY events coalesced while that ADC already had pending work |
| `ads_spi` | SPI transfer errors, including setup and register operations |
| `ads_crc` | Conversion frames rejected by output CRC |
| `ble_drop` | BLE queue full, oversized legacy notification, or notification failure |
| `usb_tx_drop` | Whole frames rejected because the application TX ring was full |
| `usb_rx_drop` | Invalid/overlong lines or complete lines rejected by the command queue |
| `uart_tx_drop` | Outgoing RP2040 lines rejected because the TX ring was full |
| `uart_rx_drop` | Invalid/overlong RP2040 lines or a full incoming line queue |
| `cmd_drop` | Complete commands rejected by the shared command queue |
| `stale_cmd` | Queued commands discarded because their originating session ended |

`ads_missed` is the sum of the three ADC diagnostic counters; because SPI setup
errors are included, it is not a count of lost physical samples. Output skipped
while no host is connected is intentional and is not a TX-ring overflow. USB
queue rejection increments both its RX counter and `cmd_drop`; those overlap.
The full `stats` reply fits the existing 244-byte packet even at maximum counters.

Supported legacy binary controls are at least 3 bytes:

- Byte 0: 1=ADS, 2=BME, 3=MAX, 4=MPU, 5=QMC, 6=audio, 7=DMIC.
- Byte 1: 1=start, 2=stop.
- Byte 2: ADC index 0/1 for ADS; ignored for other listed devices.
- Beacon scan: `[9,3,0]` starts; `[9,4,0]` stops.

Other old tuning/configuration opcodes are not implemented; use text ADC
register/gain/rate commands. This is deliberately partial legacy compatibility.

## New frame format

Each BLE notification starts with this eight-byte header:

| Offset | Length | Meaning |
| --- | ---: | --- |
| 0 | 1 | Magic `0xb3` |
| 1 | 1 | Sensor/type ID |
| 2 | 2 | Packet sequence, unsigned little-endian, wraps at 65536 |
| 4 | 2 | Payload fragment offset, unsigned little-endian |
| 6 | 2 | Total unfragmented payload length, unsigned little-endian, 1..244 |
| 8 | variable | Payload fragment |

IDs: 0=text reply, 2=ADS0, 3=ADS1, 4=MPU, 5=MAX, 6=BME, 7=QMC,
8=PCM, 9=iBeacon, 10=RP2040 UART. These IDs are **not** the legacy UUID numbers.
Reassemble by connection, characteristic, type and sequence. A new offset-zero
fragment starts a packet; discard partial data on a gap or disconnect. Sequence
numbers are global per transport, not per sensor. Reply and sensor notifications
use separate characteristics and may interleave.

At default ATT MTU 23, each notification holds 12 payload bytes after the
header. Request a larger MTU where possible. **BT832A/nRF52810 caps MTU at 64**
to fit 24 KiB RAM: its 227-byte ADC packet must use the framed stream. Legacy
ADC notifications cannot fit and are dropped/counted if subscribed. Do not
subscribe to that legacy characteristic on BT832A. Framing does not guarantee
enough radio throughput at every ADC rate; begin at 250 SPS and measure loss.

BT5340/BT840 support MTU 247. Legacy data notifications are sent unchanged only
if the entire packet fits MTU minus 3; they are never silently truncated. To
avoid duplicate traffic, subscribe to either legacy data or the framed stream.

USB CDC sends complete frames with offset zero; serial reads can split or
combine them arbitrarily. Read eight bytes, validate, then read `total` bytes.
No extra transport CRC is included. USB must not share this stream with log
text. DTR gates output; a full TX buffer drops new whole frames. BLE queue
pressure drops whole packets or the remainder of a failed fragmented packet.
Counters/sequences expose some losses, but this is not lossless recording.

`tools/frames.py` includes independent BLE reassembly and USB parsing helpers,
without requiring a specific BLE/serial library. Recreate them on reconnect.

## Sensor payloads

| ID | Payload |
| --- | --- |
| ADS0/1 | 227 bytes: nine samples, each eight signed big-endian 24-bit channel values followed by one 8-bit DRDY-edge counter; final CR LF |
| MPU | 243 bytes: packet counter; 20 samples of accel XYZ then gyro XYZ, signed BE16; latest two-byte BE16 temperature |
| MAX | 195 bytes: packet counter; 32 red/IR pairs, each value three bytes BE with 18 valid bits; latest two-byte temperature registers |
| BME/BMP | Chip ID, packet counter, three samples; each has temperature, pressure, and BME humidity as applicable; each quantity is Zephyr `sensor_value` as signed LE32 `val1`, signed LE32 `val2` |
| QMC | 243 bytes: packet counter; 40 samples of signed LE16 XYZ; latest two-byte temperature registers |
| PCM | Mono signed LE16 at requested 16 kHz, typically 160 bytes / 5 ms; framed stream only |
| iBeacon | 28 bytes: raw six-byte Zephyr address, signed RSSI byte, 16-byte UUID, two-byte BE major, two-byte BE minor, signed TX-power byte |
| UART | Raw received line without CR/LF; framed stream only |

BME is 74 bytes (chip ID 0x60); BMP is 50 bytes (no humidity). `sensor_value`
represents `val1 + val2 / 1,000,000`; units are degrees C, kPa, and percent RH.
These public API values replace the old code's dependency on private driver
layout. Test old client scaling explicitly. MAX temperature may initially be
zero until its asynchronous temperature conversion completes.

Defaults: ADS gain32, nominal250SPS with8.192MHz CLKIN; MPU +/-4g and +/-500dps,
100Hz; MAX red/IR100SPS; QMC8G/100Hz. Polling and transport backpressure can lose
samples; CRC-valid ADC frames alone do not prove timing continuity.
