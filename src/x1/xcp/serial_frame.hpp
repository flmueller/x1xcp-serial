#pragma once

// XCP over a serial line together with text output (REQ-BUS-007, ADR-023): every XCP packet is
// sent as a frame
//
//     0x00  COBS(packet, CRC16 low, CRC16 high)  0x00
//
// COBS removes all zero bytes from the frame, so 0x00 only occurs as a delimiter; text (print
// output of the firmware, input typed into the console) never contains 0x00 and is sent as it is
// between the frames. CRC16 = CRC-16/CCITT-FALSE (polynomial 0x1021, initial value 0xFFFF) of
// the packet.
//
// The receiver needs no synchronisation: the zero bytes divide the stream into segments that are
// alternately text and frames. A segment taken for a frame that does not decode (bad COBS
// structure, CRC, too long) was text: it is passed on as text and the zero byte that ended it
// starts the next frame. So a receiver that starts in the middle of a frame, or a zero byte in
// the text, costs at most that frame.
//
// Freestanding (no heap, no standard library besides <stdint.h>/<stddef.h>): the same code runs
// in the XCP tool and in the firmware (e.g. an ATmega328P).

#include <stddef.h>
#include <stdint.h>

namespace x1 {
namespace xcp {
namespace serial {

constexpr uint8_t kDelimiter = 0x00;
constexpr size_t kCrcSize = 2;

inline uint16_t crc16(const uint8_t* data, size_t size, uint16_t crc = 0xFFFF)
{
    for (size_t i = 0; i < size; ++i) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
        for (int bit = 0; bit < 8; ++bit) {
            crc = static_cast<uint16_t>((crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1);
        }
    }
    return crc;
}

/// Largest size of an encoded frame (with both delimiters) for a packet of `size` bytes.
constexpr size_t maxFrameSize(size_t size)
{
    return size + kCrcSize + (size + kCrcSize) / 254 + 1 + 2;
}

/// Writes the frame of `packet` byte by byte through `put(uint8_t)`; needs no buffer.
template <class Put>
void encodeFrame(const uint8_t* packet, size_t size, Put&& put)
{
    const uint16_t crc = crc16(packet, size);
    const size_t total = size + kCrcSize;
    auto at = [&](size_t i) -> uint8_t {
        return i < size ? packet[i] : static_cast<uint8_t>(i == size ? crc & 0xFF : crc >> 8);
    };
    put(kDelimiter);
    size_t i = 0;
    for (;;) {
        size_t len = 0;
        while (i + len < total && len < 254 && at(i + len) != 0) {
            ++len;
        }
        put(static_cast<uint8_t>(len + 1));
        for (size_t k = 0; k < len; ++k) {
            put(at(i + k));
        }
        i += len;
        if (i == total) {
            break;
        }
        if (len == 254) {
            continue; // a full block without zero
        }
        ++i; // the zero that ended the block
        if (i == total) {
            put(1); // the data ended with a zero: an empty last block
            break;
        }
    }
    put(kDelimiter);
}

/// Calls fn(uint8_t) for every decoded byte of a COBS segment (without delimiters); false if the
/// segment is not valid COBS (fn may have been called for a part of it).
template <class Fn>
bool forEachDecoded(const uint8_t* data, size_t size, Fn&& fn)
{
    size_t in = 0;
    while (in < size) {
        const uint8_t code = data[in++];
        if (code == 0 || in + code - 1 > size) {
            return false;
        }
        for (uint8_t k = 1; k < code; ++k) {
            const uint8_t b = data[in++];
            if (b == 0) {
                return false;
            }
            fn(b);
        }
        if (code != 0xFF && in < size) {
            fn(static_cast<uint8_t>(0));
        }
    }
    return true;
}

/// Checks a COBS segment without changing it: valid COBS whose last two bytes are the CRC16 of
/// the bytes before (at least one). Returns the packet size, 0 if it is no frame.
inline size_t checkFrame(const uint8_t* data, size_t size)
{
    size_t count = 0;
    uint16_t crc = 0xFFFF;
    uint8_t last[2] = {0, 0}; // the last two bytes are the CRC, not part of it
    const bool cobs = forEachDecoded(data, size, [&](uint8_t b) {
        if (count >= 2) {
            crc = crc16(&last[0], 1, crc);
        }
        last[0] = last[1];
        last[1] = b;
        ++count;
    });
    if (!cobs || count <= kCrcSize) {
        return 0;
    }
    return crc == static_cast<uint16_t>(last[0] | (last[1] << 8)) ? count - kCrcSize : 0;
}

/// Decodes a COBS segment in place (the decoded data is never longer); returns its size.
inline size_t decodeCobs(uint8_t* data, size_t size)
{
    size_t out = 0;
    forEachDecoded(data, size, [&](uint8_t b) { data[out++] = b; });
    return out;
}

/// Splits a received byte stream into packets and text. The buffer holds one encoded frame; its
/// capacity limits the packet size (maxFrameSize(packet) - 2 bytes are enough).
class FrameReceiver {
public:
    FrameReceiver(uint8_t* buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {}

    /// Processes one byte: complete packets go to onPacket(const uint8_t*, size_t), text bytes to
    /// onText(const uint8_t*, size_t) (one call per byte or per segment taken for text).
    template <class OnPacket, class OnText>
    void feed(uint8_t byte, OnPacket&& onPacket, OnText&& onText)
    {
        if (!inFrame_) {
            if (byte == kDelimiter) {
                inFrame_ = true;
                length_ = 0;
            } else {
                onText(&byte, 1);
            }
            return;
        }
        if (byte != kDelimiter) {
            if (length_ < capacity_) {
                buffer_[length_++] = byte;
                return;
            }
            // longer than any frame: it was text, and the next zero starts a frame
            ++invalid_;
            onText(static_cast<const uint8_t*>(buffer_), length_);
            onText(&byte, 1);
            length_ = 0;
            inFrame_ = false;
            return;
        }
        if (length_ == 0) {
            return; // "0x00 0x00": the first zero ended text, this one starts the frame
        }
        const size_t size = checkFrame(buffer_, length_);
        if (size == 0) {
            // not a frame: text, and this zero starts the next frame
            ++invalid_;
            onText(static_cast<const uint8_t*>(buffer_), length_);
            length_ = 0;
            return;
        }
        decodeCobs(buffer_, length_);
        ++packets_;
        inFrame_ = false;
        length_ = 0;
        onPacket(static_cast<const uint8_t*>(buffer_), size);
    }

    /// No byte for a while (host): an unfinished segment is text (a frame is sent at once).
    template <class OnText>
    void timeout(OnText&& onText)
    {
        if (inFrame_ && length_ > 0) {
            ++invalid_;
            onText(static_cast<const uint8_t*>(buffer_), length_);
        }
        inFrame_ = false;
        length_ = 0;
    }

    bool inFrame() const { return inFrame_; }
    uint32_t packets() const { return packets_; }
    uint32_t invalid() const { return invalid_; } ///< segments that were no frame (CRC, length, COBS)

private:
    uint8_t* buffer_;
    size_t capacity_;
    size_t length_ = 0;
    bool inFrame_ = false;
    uint32_t packets_ = 0;
    uint32_t invalid_ = 0;
};

} // namespace serial
} // namespace xcp
} // namespace x1
