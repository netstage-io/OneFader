# OneFader

OneFader is an open-source hardware device that connects a physical slide potentiometer to a single sACN (E1.31) DMX channel over Ethernet. Built around the Waveshare ESP32-S3-ETH, it is designed for lighting professionals and enthusiasts who need a simple, reliable, standalone fader node — no computer, no driver software, no console required.

Output is configurable as 8-bit (single channel) or 16-bit (coarse/fine across two consecutive channels), making it compatible with any sACN-capable lighting console or software.

---

## Flashing

1. Connect OneFader to your computer via USB-C
2. Open **[onefader.netstage.io](https://onefader.netstage.io)** in Chrome or Edge
3. Click **Check for Latest Firmware** then **Flash to Device**

The tool handles everything over USB directly in the browser — no software installation needed.

---

## Default Passwords

| Purpose | Username | Password |
|---|---|---|
| Web UI (when auth enabled) | `admin` | `netstage` |
| OTA (Arduino IDE) | — | `netstage` |

Web authentication is **disabled by default**. When enabled, the browser will prompt for credentials on first visit and cache them for the session. Both passwords are configurable from the System Actions card in the web interface.

---

## Web Interface

Once running, connect the device to your network and navigate to its IP address in any browser to access the full configuration interface. The IP is shown on the USB tool if you are unsure.

### Device Settings

- **Stream Name** — the sACN source name shown on your lighting console. Changing this triggers a reboot.
- **Universe** — sACN universe number, 1–63999
- **DMX Start Address** — the channel the fader controls (1–512)
- **16-bit Mode** — uses two consecutive channels (start address = coarse, start address +1 = fine), giving 65,536 steps of resolution instead of 256
- **sACN Priority** — 0–200, default 100
- **Unicast Mode** — send directly to a target IP instead of multicast; useful for networks where multicast is restricted or unavailable

### Fader Control

- **Live Display** — shows the current fader value in real time with a visual bar
- **Simulation Mode** — replaces the physical fader with an on-screen slider (0–100%), useful for testing without hardware connected. Simulation mode is never saved — the device always boots in hardware mode.
- **Fader Invert** — reverses the fader direction so fully up = 0 and fully down = full
- **Test Mode** — automatically sweeps the output 0→100→0 in a continuous loop, useful for checking fixture response

### Network Settings

- **DHCP / Static IP** — toggle between dynamic and fixed addressing. The current mode and gateway are shown in the network status panel.
- **Static IP, Subnet, Gateway, DNS** — fully configurable when using a static address

### Input Tuning

These settings control how the raw ADC signal from the fader is processed before being sent as sACN. All values are saved to the device and persist across reboots.

- **EMA Smoothing** — Exponential Moving Average filter applied on top of the moving average. Reduces noise on slow movements. The alpha value (1–100) controls the strength — 1 is maximum smoothing, 100 effectively disables it. Default: enabled, alpha 15.
- **Spike Rejection** — discards single-sample ADC readings that jump more than the set threshold in one step. Prevents random electrical glitches from reaching the output. Uses a confirmation window — a jump must be consistent across several consecutive samples to be accepted as a real move, so snapping the fader quickly still works. Default: 200 counts. Set to 0 to disable.
- **Deadband** — minimum change in ADC counts required before the output updates. Prevents the output flickering when the fader is held still due to ADC noise. Default: 30 counts.

#### Fader Calibration

The **Calibrate Fader** button opens a dedicated calibration page. Calibration teaches the device the actual physical travel range of your specific potentiometer so the output reliably reaches 0% and 100%.

1. Click **Calibrate Fader**
2. Move the fader slowly all the way to both ends of its travel
3. The page shows live min/max ADC values and a progress bar as you move
4. Click **Save Calibration** when done

A 2-minute countdown is shown on the calibration page. If you navigate away without saving or cancelling, calibration auto-cancels when the timer expires. The saved calibration values (ADC min/max) are shown on the Input Tuning card for reference.

### System Actions

- **Web Authentication** — enable or disable HTTP Basic Auth on the web interface. When enabled, set a custom username and password below the toggle. Default: disabled.
- **OTA Password** — the password used when uploading firmware via Arduino IDE OTA. Default: `netstage`.
- **Firmware Update** — upload a new firmware `.bin` directly via the browser without touching USB. Use the **app-only** `.bin` (`Sketch.ino.bin`) — not the merged binary.
- **Reboot** — restart the device without losing any saved settings
- **Factory Reset** — clears all saved settings and returns to defaults. The device will reboot after reset.

---

## USB Tool

The [OneFader tool](https://onefader.netstage.io) at the same address also connects via USB-C serial for:

- Viewing device status (firmware version, stream name, IP, universe)
- Setting a static IP without needing network access first
- Reboot and factory reset
- Live serial log streaming
- Flashing firmware via ESP Web Tools

Requires Chrome or Edge. The tool communicates at 115200 baud over the native USB-C CDC port.

---

## Hardware & Connections

### What You Need

- Waveshare ESP32-S3-ETH board (with POE Module)
- Slide potentiometer (10kΩ recommended)
- Ethernet cable
- USB-C cable (for initial flashing)
- PoE-capable Ethernet switch or injector (802.3af)

### Wiring the Fader

| Potentiometer Pin | ESP32-S3-ETH |
|---|---|
| Top (VCC) | 3.3V |
| Wiper (signal) | IO1 |
| Bottom (GND) | GND |

Any slide potentiometer between 1kΩ and 100kΩ will work. 10kΩ is ideal. The wiper is the centre pin — if the fader reads backwards, either swap VCC and GND or enable **Fader Invert** in the web interface.

### Ethernet

Connect a standard RJ45 Ethernet cable to the board's Ethernet port. The device uses DHCP by default — your network will assign an IP automatically on first boot.

### Power

The board is powered via Power over Ethernet (PoE, 802.3af) through the RJ45 port — no separate power supply needed. A USB-C cable is only required for the initial firmware flash.

### Board Overview

| Feature | Detail |
|---|---|
| MCU | ESP32-S3 |
| Ethernet | W5500 (SPI, hardware TCP/IP) |
| Fader Input | IO1 (12-bit ADC) |
| USB | Native USB-C (CDC serial) |
| Connectivity | Ethernet only — no WiFi |

---

## Arduino IDE Board Settings

If building from source, the following board settings are required in Arduino IDE:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB Mode | USB-OTG (TinyUSB) |
| USB CDC On Boot | Enabled |
| Upload Mode | USB-OTG CDC (TinyUSB) |

For subsequent uploads after the first flash, use OTA via the network port — no USB cable needed.

---

## License

MIT © 2026 [netstage.io](https://netstage.io)
