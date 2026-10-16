#pragma once

// x1xcp_serial: XCP over the serial port for Arduino (XCP-Ray, REQ-FW-011), e.g. an Arduino Nano
// on its USB serial adapter. Measurement (dynamic DAQ) and calibration of global variables;
// Serial.print() keeps working: XCP packets travel as frames between the text (COBS + CRC16, see
// serial_frame.hpp), the console of XCP-Ray shows the text and sends typed lines back.
//
//   static const x1xcp::Event kEvents[] = {{"10ms", 10, true}, {"100ms", 100, true}};
//
//   void setup() {
//       Serial.begin(115200);                 // the baud rate of the A2L (XCP_ON_SxI)
//       x1xcp::Config xcp;
//       xcp.events = kEvents;
//       xcp.eventCount = 2;
//       x1xcp::begin(Serial, xcp);
//   }
//   void loop() {
//       x1xcp::poll();                        // often: commands, events with autoTrigger
//       ...                                   // compute; Serial.print() as before
//       x1xcp::trigger(0);                    // or: sample event 0 right after computing
//   }
//
// Call poll(), trigger() and Serial.print() only from loop() (not from interrupts): a frame must
// not be interrupted by other output on the line.

#include <Arduino.h>

#include <stdint.h>

namespace x1xcp {

struct Event {
    const char* name;      ///< as in the A2L
    uint16_t cycleMs;      ///< nominal cycle (GET_DAQ_EVENT_INFO), 0 = not cyclic
    bool autoTrigger;      ///< triggered by poll() every cycleMs instead of by trigger()
};

struct Config {
    const Event* events = nullptr; ///< DAQ event channels 0..eventCount-1, same order as in the A2L
    uint8_t eventCount = 0;
};

/// Starts the XCP slave on `serial` (opened with begin(baud) before). False on a wrong config.
bool begin(Stream& serial, const Config& config);

/// Receives and answers commands, triggers the events with autoTrigger. Call it in every loop();
/// the longer a loop() takes, the later the answers (the tool waits up to its timeouts).
void poll();

/// Event `event` occurred: samples its running DAQ lists now (timestamp millis()).
void trigger(uint8_t event);
void trigger(uint8_t event, uint32_t timestampMs);

/// Text typed into the console of the tool (available()/read()); writing to it prints like Serial.
Stream& console();

struct Stats {
    bool connected = false;  ///< a master is connected
    bool daqRunning = false;
    uint32_t commands = 0;   ///< XCP commands processed
    uint32_t dtos = 0;       ///< DAQ packets sent
    uint32_t overloads = 0;  ///< DAQ packets not sent (the master is told)
    uint32_t badFrames = 0;  ///< received frames with a wrong CRC (or text with a zero byte)
};
Stats stats();

} // namespace x1xcp
