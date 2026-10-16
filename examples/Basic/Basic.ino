// An Arduino sketch extended with XCP over the serial port (x1xcp_serial): the lines marked
// "XCP" are all it takes. Serial.print() keeps working; the text appears in the console of
// XCP-Ray, and lines typed there arrive through x1xcp::console().

#include <Arduino.h>
#include <x1xcp_serial.h> // XCP

// global, volatile: fixed address, always in memory (measured and calibrated by address)
volatile uint16_t counter;          // measurement
volatile float temperature;         // measurement
volatile uint16_t blink_ms = 500;   // parameter
volatile bool led_on;               // measurement

static const x1xcp::Event kEvents[] = {{"10ms", 10, false}, {"100ms", 100, true}}; // XCP: as in the A2L

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200); // the baud rate of the A2L ("baudrate" in a2l/variables.json)
    Serial.println(F("example: started"));
    x1xcp::Config xcp; // XCP
    xcp.events = kEvents;
    xcp.eventCount = 2;
    x1xcp::begin(Serial, xcp);
}

void loop()
{
    x1xcp::poll(); // XCP: commands, event "100ms" (autoTrigger)

    static uint32_t next = millis();
    const uint32_t now = millis();
    if (now - next >= 10) {
        next += 10;
        ++counter;
        temperature = 20.0f + 0.1f * (analogRead(A0) - 512) / 10.0f;
        led_on = (now / (blink_ms > 0 ? blink_ms : 1)) % 2 == 0;
        digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
        x1xcp::trigger(0); // XCP: event "10ms", right after computing its values
    }

    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 2000) {
        lastPrint = now;
        Serial.print(F("counter "));
        Serial.println(counter);
    }

    // a line typed into the console of the tool
    while (x1xcp::console().available() > 0) {
        const char c = static_cast<char>(x1xcp::console().read());
        if (c == '?') {
            Serial.println(F("example: '?' received"));
        }
    }
}
