# Build Guide (Overview)
Follow along: [A Self Balancing Robot Journey | Building SABR](https://youtu.be/h8H8bgWLwpI)

## 1. Print Parts

Use STL files in `hardware/cad/stl/`

## 2. Assemble Frame

* Mount motors
* Attach wheels
* Secure electronics

## 3. PCB

* Order from `hardware/pcb/gerbers/`
* Solder components

## 4. Wiring

Follow `docs/wiring.md`

## 5. Upload Firmware

Flash `firmware/sabr.ino` to ESP32

## 6. First Test

* Hold robot upright
* Power on
* Adjust PID if unstable
