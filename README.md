# DEEED Open-Source Flight Computer

The DEEED Flight Computer is an easy-to-build, open-source flight computer / altimeter for model rockets.
It is designed around commonly available breakout boards, so it is simple to assemble and use. I designed it as a good first PCB project for myself and for every hobbyist.

The programming side of the project was mostly done by Claude AI, as I can't really program myself – this accelerated that part enormously. I recommend you write your own code or improve the existing code and design if you can or want to. I hope you have a great experience!

> ⚠️ **Safety notice**
> This is a hobby project, provided **without any warranty** (see [LICENSE](LICENSE)). It is **not** a certified recovery system. In the original setup the ejection charge is fired by the rocket motor itself; the igniter output is an additional function. Follow the laws and safety codes of your rocketry association, and **always test the igniter output without any igniter connected**.

I Build my own fully custom and 3D printed model Rocket around this Flightcomputer. The Rocket can be found on Makerworld. I also uploaded some of my Flight on YouTube ELG_Space https://www.youtube.com/@ELG_Space

## Technical overview

| Function | Part |
|---|---|
| MCU, IMU and Bluetooth recovery beacon | Seeed XIAO nRF52840 **Sense** |
| Barometer | BMP280 (six-pin breakout board version) |
| Stable 5 V supply | Pololu 4941 |
| Data logging | Adafruit 4682 SD card breakout board |
| Buzzer | Standard 5 V buzzer (found in most electronics starter kits) |
| Switching (buzzer and pyro channel) | 2× N-channel MOSFET IRLZ44N |

The board is 24 mm wide and 106 mm tall, so it fits into a 35mm airframe.

### Pinout

| Pin | Function |
|---|---|
| D0 | Manual override button (to GND) for ground tests |
| D1 | Buzzer (via MOSFET) |
| D2 | Igniter / pyro channel (via MOSFET) |
| D4 / D5 | I2C to the BMP280 (address 0x76) |
| D6 | BMP280 supply (3.3 V from a GPIO, so the firmware can power-cycle the sensor) |
| D7 | SD card chip select (SCK/MISO/MOSI on D8/D9/D10) |
| D3 | free |

Recommended for each MOSFET: **330 Ω** in series with the gate and a **10 kΩ** pulldown from gate to GND, so the output can never switch while the MCU is booting.

## Code overview

The current software is **DEEEDSoft V0.2**. It can:

- detect the flight phases automatically: **PAD → BOOST → COAST → APOGEE → DESCENT → LANDED**
- detect apogee using a median-filtered barometric altitude, a vertical-speed window, a plausibility check and a 15 s backup timer
- fire the igniter output at apogee
- record flight data into RAM during the flight (2000 records, 3 s pre-launch buffer; 50 Hz up to apogee, 5 Hz during descent) and write it after landing to the SD card as `flightNNN.csv` plus a summary `flightNNN.txt`
- help you find the rocket: buzzer and a Bluetooth beacon named **"Rocket-Recovery"** after landing
- show its status on the board's RGB LED (green = IMU, barometer and SD OK; red = something is missing)
- keep running if the barometer misbehaves: it uses its own software I2C with timeouts, powers the BMP280 from D6 and automatically power-cycles it after repeated read errors; a hardware watchdog covers everything else

### Serial commands (115200 baud)

| Key | Action |
|---|---|
| `l` | list files on the SD card |
| `d` | print the last written file (read flight data without removing the card) |
| `w` | write the data buffer to the card now |
| `r` | re-initialise the SD card |
| `f` | show flight summary |
| `m` | show RAM usage |
| `s` | status (state, records, SD, BMP280 errors/restarts) |

### Ground test

Pull D0 briefly to GND to step to the next state (PAD → BOOST → … → LANDED). Do this **without any igniter connected**.

## Build and flash

Requires [PlatformIO](https://platformio.org/).

## Repository layout

| Folder | Part |
|---|---|
| Software/DEEEDSoft 0.2/ | firmware (PlatformIO project: src/, lib/, platformio.ini) |
| Hardware/ | KiCad project (schematic and PCB) |
| Example Flightdata/ | recorded flight data |

The firmware lives in `Software/DEEEDSoft 0.2/`. Open that folder in VS Code / PlatformIO to build and flash it.

```
src/main.cpp           main loop, serial console, setup
lib/Sensors/           IMU, BMP280, software I2C
lib/StateMachine/      flight phases and detection logic
lib/Actuators/         igniter and buzzer
lib/DataLogger/        RAM buffer, CSV and summary files on SD
hardware/              KiCad project (schematic and PCB)
```


All detection thresholds are named constants at the top of `lib/StateMachine/StateMachine.cpp`.

## Known limitations

- Tuned for small model rockets and tested on only a few flights.
- Barometric altitude can be disturbed by airflow or ejection pressure; hence the plausibility check and the backup timer.
- During a BMP280 outage the last valid altitude is held.

## Contributing

Issues and pull requests are welcome – especially flight data and improvements to the apogee detection.

## License

[GPL-3.0](LICENSE)
