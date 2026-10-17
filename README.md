# x1xcp_serial – XCP over the serial port (USB) for Arduino AVR boards

Adds measurement (dynamic DAQ) and calibration of global variables to an Arduino firmware over
its USB serial port, for XCP-Ray. `Serial.print()` keeps working: XCP packets travel as frames
between the text, XCP-Ray shows the text in the console of the ECU and sends typed lines back.
The A2L file is generated from the ELF file after every build: addresses and data types from the
debug information, the selection, units and scaling from a short variable list.

Tested: Arduino Nano clone (ATmega328P, 16 MHz, CH340, Optiboot), PlatformIO (atmelavr 5.3,
avr-gcc 7.3), 115200 baud: 14 signals in three rasters (10 ms, 100 ms, 500 ms) for 30 s without
loss, calibration, console in both directions. Other AVR boards should work (Uno, Mega 2560),
but are untested.

Part of the free kits of XCP-Ray, a measurement and calibration tool. The library is MIT
licensed; XCP-Ray itself and its A2L generator `x1-elf2a2l` are free to use, but not open source.

## Installation

- **Arduino IDE:** Sketch > Include Library > Add .ZIP Library… with the zip of this repository
  (Code > Download ZIP on GitHub, or a release). Then File > Examples > x1xcp_serial > Basic.
- **PlatformIO:** in `platformio.ini`: `lib_deps = https://github.com/flmueller/x1xcp-serial.git`

## Contents

| | |
|---|---|
| `src/` | the library: `x1xcp_serial.h` (API), the portable XCP slave of XCP-Ray, `x1xcp_config.h` (sizes) |
| `examples/Basic/` | a sketch extended with XCP (the lines marked "XCP") |
| `extras/a2l/variables.example.json` | template of the variable list for the A2L file |
| `extras/scripts/x1_a2l.py` | PlatformIO post-build step: the A2L file after every build |

## Steps

1. **Library:** install it (above) and add `#include <x1xcp_serial.h>` (see the example and step 4).

2. **A2L file:** XCP-Ray needs an A2L file with the addresses of the variables; `x1-elf2a2l`
   makes it from the ELF file of the build and a short variable list (step 3). `x1-elf2a2l` is
   in the `bin/` folder of XCP-Ray (Linux and Windows). Make the A2L again after every change of
   the firmware: the addresses move.

   **Arduino IDE:** Sketch > Export Compiled Binary writes the ELF file into the sketch folder,
   `build/<board>/<sketch>.ino.elf`. Then, in the sketch folder:

   ```sh
   x1-elf2a2l -v variables.json -o <sketch>.a2l build/<board>/<sketch>.ino.elf
   ```

   If `x1-elf2a2l` reports a variable without debug information (always for AVR boards: the
   Arduino IDE builds them with LTO, which drops it), give the variable its type in the list,
   e.g. `"type": "uint16_t"`, `"float"` or `"bool"`; the address then comes from the symbol
   table. Curves need the `"type"` of the values and, inside `"curve"`, of the axis; maps need
   the debug information (PlatformIO).

   Optional, AVR boards with full debug information: switch LTO off with a file
   `platform.local.txt` next to the `platform.txt` of the AVR core
   (`~/.arduino15/packages/arduino/hardware/avr/<version>/`, on Windows
   `%LOCALAPPDATA%\Arduino15\packages\arduino\hardware\avr\<version>\`) that repeats the
   four flag lines of `platform.txt` without the LTO options, e.g. for core 1.8.8:

   ```
   compiler.c.flags=-c -g -Os {compiler.warning_flags} -std=gnu11 -ffunction-sections -fdata-sections -MMD
   compiler.c.elf.flags={compiler.warning_flags} -Os -g -Wl,--gc-sections
   compiler.S.flags=-c -g -x assembler-with-cpp -MMD
   compiler.cpp.flags=-c -g -Os {compiler.warning_flags} -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD
   ```

   Restart the IDE. Then no `"type"` is needed and maps work too. It applies to all sketches of
   that core, the firmware gets about 2 KB larger, and after an update of the core the file has
   to be made again in the new version folder.

   **PlatformIO:** copy `extras/scripts/x1_a2l.py` into `scripts/` of the project, `x1-elf2a2l`
   into `tools/` (or set `X1_ELF2A2L`), and add to `platformio.ini`:

   ```ini
   ; A2L: debug information of the variables (avr-gcc 7.3 loses it with LTO)
   build_unflags = -flto
   build_flags = -g
   extra_scripts = post:scripts/x1_a2l.py
   ; optional, these are the defaults:
   custom_x1_variables = a2l/variables.json
   custom_x1_a2l = <project folder name>.a2l
   ```

   Every `pio run` then writes the A2L file.

3. **Variable list:** copy `extras/a2l/variables.example.json` to `variables.json` (Arduino IDE: next to the sketch; PlatformIO: `a2l/variables.json`) and list the
   variables to measure (`"event"`) and to calibrate (`"parameter": true`), with units, scaling
   and texts. `"baudrate"` and `"events"` must match the firmware (step 4).

   **Characteristic curves** (y over x, edited as a graph in XCP-Ray): two global arrays of the
   same length, the axis points (ascending) and the values, e.g.
   `volatile float gain_x[8]; volatile float gain[8];` and in the list

   ```json
   { "name": "gain", "unit": "%", "min": 0, "max": 100,
     "curve": { "axis": "gain_x", "input": "rpm", "unit": "1/min", "min": 0, "max": 8000 } }
   ```

   `"input"` names the measurement on the axis (XCP-Ray marks the operating point); `"factor"`
   and `"offset"` scale the values, inside `"curve"` the axis. The firmware interpolates the
   curve itself (linearly between the points, as ECUs do). In XCP-Ray both the values and the
   axis points can be changed (the axis points stay ascending).

   **Characteristic maps** (z over x and y, edited as a heat map or table): a two-dimensional
   array `values[y][x]` and one ascending axis array for x and y each, e.g.
   `volatile float spark_x[8]; volatile float spark_y[6]; volatile float spark[6][8];` (y first,
   as in C: one row per y point) and in the list

   ```json
   { "name": "spark", "unit": "deg", "min": -10, "max": 50,
     "map": { "x": { "axis": "spark_x", "input": "rpm", "unit": "1/min" },
              "y": { "axis": "spark_y", "input": "load", "unit": "%" } } }
   ```

   The axis objects take the same keys as `"curve"`. Curves and maps may share an axis array.
   The firmware interpolates the map itself (bilinear). In XCP-Ray the values can be changed; the
   axis points of a map in the editor of a curve that shares its axis.

4. **Firmware:**

   ```cpp
   #include <x1xcp_serial.h>

   static const x1xcp::Event kEvents[] = {{"10ms", 10, false}, {"100ms", 100, true}};

   void setup() {
       Serial.begin(115200);
       x1xcp::Config xcp;
       xcp.events = kEvents;
       xcp.eventCount = 2;
       x1xcp::begin(Serial, xcp);
   }

   void loop() {
       x1xcp::poll();                  // in every loop(): commands, events with autoTrigger
       ...                             // compute event 0, then:
       x1xcp::trigger(0);
   }
   ```

   - `poll()` answers the commands; a `loop()` that takes long delays the answers (the tool
     waits up to its timeouts, `"timeoutsMs"`, 500 ms by default).
   - Events with `autoTrigger` (third value `true`) are triggered by `poll()` every `cycleMs`;
     `trigger()` samples exactly when the values are consistent.
   - Call `poll()`, `trigger()` and `Serial.print()` **only from `loop()`**, not from interrupts:
     a frame must not be interrupted by other output. Do not send zero bytes (`Serial.write(0)`):
     they belong to the frames (a stray one costs at most one frame).
   - Lines typed into the console of the tool arrive through `x1xcp::console()` (a `Stream`:
     `available()`, `read()`); do not read `Serial` directly, it would take the XCP commands.
   - Variables must be **global** (or `static` at file scope) and should be **`volatile`**.
     Parameters must not be `const` (those can lie in flash). Program memory (`PROGMEM`) can be
     measured (read only) at its flash address.
   - `x1xcp::stats()` gives the connection state and counters.

5. **Build and flash:** `pio run -t upload` (the tool must not hold the port: Disconnect in
   XCP-Ray releases it). Then in XCP-Ray: Devices > Add ECU, the A2L, transport
   "XCP on serial", the port (e.g. `/dev/ttyUSB0`; the baud rate comes from the A2L). View >
   Console shows the text of the ECU.

## Technical data

- Frames: `0x00 COBS(packet, CRC16) 0x00`, CRC-16/CCITT-FALSE; text in between. The tool
  recovers from any position in the stream (port opened while the ECU sends).
- MAX_CTO 64, MAX_DTO 64; dynamic DAQ with at most 4 DAQ lists, 12 ODTs and 48 ODT entries
  (`X1_XCPSLAVE_MAX_*` in `src/x1xcp_config.h`); 2-byte timestamp in ms; up to 8 events.
- RAM: about 500 bytes for the library (DAQ tables, receive buffer, console input), plus the
  64-byte buffers of `Serial`. The test firmware uses 1095 of 2048 bytes.
- Data rate at 115200 baud: about 11 KB/s. XCP-Ray estimates the load of the line before a
  measurement and refuses it above the limit (80 % by default). Example: 10 signals of 2–4 bytes
  every 10 ms ≈ 20 %. 250000, 500000 or 1000000 baud are exact on a 16 MHz AVR.
- Opening the port resets many Nanos (DTR). XCP-Ray waits 2 s after opening (reset delay in
  the ECU settings; 0 for boards without reset).
- Addresses: as in the ELF file, data memory at `0x800000` + RAM address.

## License

MIT, see [LICENSE](LICENSE). Version 1.0.0.
