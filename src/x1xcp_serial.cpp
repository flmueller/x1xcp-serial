// x1xcp_serial: XCP over the serial port for Arduino (XCP-Ray, REQ-FW-011), see x1xcp_serial.h.

#include "x1xcp_serial.h"

#include "x1/xcp/serial_frame.hpp"
#include "x1/xcpslave/xcp_slave.hpp"

#if defined(__AVR__)
#include <avr/pgmspace.h>
#elif defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
// arduino-pico (RP2040, RP2350): the end of the RAM and the image in flash, from its linker script
extern "C" char __StackTop[], __flash_binary_start[], __flash_binary_end[];
#define X1XCP_ARM_CHECKED 1
#elif defined(ARDUINO_ARCH_STM32)
// STM32duino: start of the variables, end of the RAM, vector table and initial data in flash
extern "C" char _sdata[], _edata[], _sidata[], _estack[];
extern "C" const uint32_t g_pfnVectors[];
#define X1XCP_ARM_CHECKED 1
#endif

#include <string.h>

namespace x1xcp {

namespace {

using x1::xcp::serial::FrameReceiver;
using x1::xcp::serial::encodeFrame;
using x1::xcp::serial::maxFrameSize;
using x1::xcpslave::EventChannel;
using x1::xcpslave::IDtoTransmitter;
using x1::xcpslave::IMemory;
using x1::xcpslave::SlaveConfig;
using x1::xcpslave::XcpSlave;

constexpr uint8_t kMaxEvents = 8;
constexpr uint8_t kInputSize = 32; // text from the console not read yet

/// XCP addresses = addresses of the ELF file. AVR: data memory at 0x800000 + RAM address
/// (read/write), program memory at its flash address (read only, e.g. PROGMEM tables).
/// RP2040/RP2350 and STM32: the RAM (read/write) and the firmware image in flash (read only, e.g.
/// const tables); any other address is refused instead of faulting the CPU.
class Memory final : public IMemory {
public:
#if defined(X1XCP_ARM_CHECKED)
    static bool inside(uint32_t address, size_t size, uintptr_t begin, uintptr_t end)
    {
        return address >= begin && address <= end && size <= end - address;
    }
    static bool inRam(uint32_t address, size_t size)
    {
#if defined(ARDUINO_ARCH_STM32)
        return inside(address, size, reinterpret_cast<uintptr_t>(_sdata), reinterpret_cast<uintptr_t>(_estack));
#else
        return inside(address, size, 0x20000000u, reinterpret_cast<uintptr_t>(__StackTop));
#endif
    }
    static bool inFlash(uint32_t address, size_t size)
    {
#if defined(ARDUINO_ARCH_STM32)
        return inside(address, size, reinterpret_cast<uintptr_t>(g_pfnVectors),
                      reinterpret_cast<uintptr_t>(_sidata) + static_cast<uintptr_t>(_edata - _sdata));
#else
        return inside(address, size, reinterpret_cast<uintptr_t>(__flash_binary_start),
                      reinterpret_cast<uintptr_t>(__flash_binary_end));
#endif
    }
#endif

    bool read(uint32_t address, uint8_t* out, size_t size) override
    {
#if defined(__AVR__)
        if (address >= kData && address - kData + size <= static_cast<uint32_t>(RAMEND) + 1) {
            memcpy(out, reinterpret_cast<const void*>(static_cast<uintptr_t>(address - kData)), size);
            return true;
        }
        if (address + size <= static_cast<uint32_t>(FLASHEND) + 1) {
            for (size_t i = 0; i < size; ++i) {
#if FLASHEND > 0xFFFF
                out[i] = pgm_read_byte_far(address + i);
#else
                out[i] = pgm_read_byte(static_cast<uint16_t>(address + i));
#endif
            }
            return true;
        }
        return false;
#else
#if defined(X1XCP_ARM_CHECKED)
        if (!inRam(address, size) && !inFlash(address, size)) {
            return false;
        }
#endif
        memcpy(out, reinterpret_cast<const void*>(static_cast<uintptr_t>(address)), size);
        return true;
#endif
    }
    bool write(uint32_t address, const uint8_t* data, size_t size) override
    {
#if defined(__AVR__)
        if (address < kData || address - kData < 0x100 || address - kData + size > static_cast<uint32_t>(RAMEND) + 1) {
            return false; // registers and I/O (below 0x100) are not calibrated
        }
        memcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(address - kData)), data, size);
#else
#if defined(X1XCP_ARM_CHECKED)
        if (!inRam(address, size)) {
            return false;
        }
#endif
        memcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(address)), data, size);
#endif
        return true;
    }

private:
    static constexpr uint32_t kData = 0x800000; // avr-gcc: data memory in the ELF address space
};

/// Packets as frames on the serial port.
class Link final : public IDtoTransmitter {
public:
    /// One write per frame: USB serial ports (Pico, STM32) send a packet per write call, byte by
    /// byte would limit them to a few KB/s. AVR: byte by byte into the buffer of Serial (saves the
    /// RAM of a frame).
    void send(const uint8_t* packet, size_t size)
    {
#if defined(__AVR__)
        encodeFrame(packet, size, [this](uint8_t b) { port->write(b); });
#else
        size_t n = 0;
        encodeFrame(packet, size, [this, &n](uint8_t b) {
            if (n < sizeof frame_) {
                frame_[n++] = b;
            }
        });
        port->write(frame_, n);
#endif
    }
    bool transmitDto(const uint8_t* packet, size_t size) override
    {
        send(packet, size);
        ++dtos;
        return true;
    }

    Stream* port = nullptr;
    uint32_t dtos = 0;

#if !defined(__AVR__)
private:
    uint8_t frame_[maxFrameSize(XcpSlave::kMaxCto > XcpSlave::kMaxDto ? XcpSlave::kMaxCto : XcpSlave::kMaxDto)];
#endif
};

/// Text typed into the tool's console (ring buffer); writes go to the port.
class Console final : public Stream {
public:
    int available() override { return static_cast<uint8_t>(head_ - tail_); }
    int read() override { return head_ == tail_ ? -1 : buffer_[tail_++ % kInputSize]; }
    int peek() override { return head_ == tail_ ? -1 : buffer_[tail_ % kInputSize]; }
    size_t write(uint8_t b) override { return port != nullptr && b != 0 ? port->write(b) : 0; }
    using Print::write;

    void push(uint8_t b)
    {
        if (static_cast<uint8_t>(head_ - tail_) < kInputSize) {
            buffer_[head_++ % kInputSize] = b;
        }
    }

    Stream* port = nullptr;

private:
    uint8_t buffer_[kInputSize];
    uint8_t head_ = 0;
    uint8_t tail_ = 0;
};
static_assert((kInputSize & (kInputSize - 1)) == 0 && kInputSize <= 128, "ring indexes wrap at 256");

struct State {
    Config config;
    EventChannel channels[kMaxEvents];
    uint32_t last[kMaxEvents] = {}; ///< nominal time of the last automatic trigger
    SlaveConfig slaveConfig;
    Memory memory;
    Link link;
    Console console;
    uint8_t rx[maxFrameSize(XcpSlave::kMaxCto)];
    FrameReceiver receiver{rx, sizeof rx};
    XcpSlave* slave = nullptr;
};

State& state()
{
    static State s;
    return s;
}

/// The XCP time unit code of a cycle in ms (6 = 1 ms, 7 = 10 ms, 8 = 100 ms).
void cycleOf(uint16_t ms, EventChannel& c)
{
    c.cycle = static_cast<uint8_t>(ms <= 255 ? ms : ms / 10 <= 255 ? ms / 10 : ms / 100);
    c.timeUnit = static_cast<uint8_t>(ms <= 255 ? 6 : ms / 10 <= 255 ? 7 : 8);
}

} // namespace

bool begin(Stream& serial, const Config& config)
{
    State& s = state();
    if (s.slave != nullptr || config.eventCount > kMaxEvents) {
        return false;
    }
    s.config = config;
    const uint32_t now = millis();
    for (uint8_t e = 0; e < config.eventCount; ++e) {
        s.channels[e].name = config.events[e].name;
        cycleOf(config.events[e].cycleMs, s.channels[e]);
        s.last[e] = now;
    }
    SlaveConfig& c = s.slaveConfig;
    c.maxCto = XcpSlave::kMaxCto;
    c.maxDto = XcpSlave::kMaxDto;
    c.driverVersion = 0x10;
    c.events = s.channels;
    c.eventCount = config.eventCount;
    c.timestampSize = 2; // 1 ms, wraps after 65 s (the tool extends it)
    c.timestampUnit = 6;
    c.timestampTicks = 1;
    c.maxOdtEntrySize = 8;
    s.link.port = &serial;
    s.console.port = &serial;
    static XcpSlave slave(s.memory, s.slaveConfig);
    s.slave = &slave;
    return true;
}

void poll()
{
    State& s = state();
    if (s.slave == nullptr) {
        return;
    }
    const uint32_t now = millis();
    s.slave->setClock(now);
    // a little at a time, so that a long text input does not hold up the events
    for (uint8_t n = 0; n < 64 && s.link.port->available() > 0; ++n) {
        const int b = s.link.port->read();
        if (b < 0) {
            break;
        }
        s.receiver.feed(
            static_cast<uint8_t>(b),
            [&s](const uint8_t* packet, size_t size) {
                uint8_t response[XcpSlave::kMaxCto];
                const size_t length = s.slave->process(packet, size, response);
                if (length > 0) {
                    s.link.send(response, length);
                }
            },
            [&s](const uint8_t* text, size_t size) {
                for (size_t i = 0; i < size; ++i) {
                    s.console.push(text[i]);
                }
            });
    }
    for (uint8_t e = 0; e < s.config.eventCount; ++e) {
        const Event& ev = s.config.events[e];
        if (!ev.autoTrigger || ev.cycleMs == 0 || now - s.last[e] < ev.cycleMs) {
            continue;
        }
        s.last[e] += ev.cycleMs; // the nominal time: exactly one cycle apart
        if (now - s.last[e] >= ev.cycleMs) {
            s.last[e] = now; // far behind (long loop): no catch-up burst
        }
        trigger(e, s.last[e]);
    }
}

void trigger(uint8_t event)
{
    trigger(event, millis());
}

void trigger(uint8_t event, uint32_t timestampMs)
{
    State& s = state();
    if (s.slave == nullptr || event >= s.config.eventCount || !s.slave->daqRunning()) {
        return;
    }
    s.slave->triggerEvent(event, timestampMs, s.link);
}

Stream& console()
{
    return state().console;
}

Stats stats()
{
    State& s = state();
    Stats st;
    if (s.slave == nullptr) {
        return st;
    }
    st.connected = s.slave->connected();
    st.daqRunning = s.slave->daqRunning();
    st.commands = s.slave->commandCount();
    st.overloads = s.slave->overloadCount();
    st.dtos = s.link.dtos;
    st.badFrames = s.receiver.invalid();
    return st;
}

} // namespace x1xcp
