// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
//
// Wire.hpp - wire format used by the Red Wire Technologies "default"
// FPGA personality on the Oxygen SDR.
//
// This is a hardware-independent reimplementation of the framing logic in
// gr-rwt (lib/base_device_source.cc and lib/rwt_sink_impl.cc):
//
//  * The DMA stream is a sequence of little-endian 64-bit words.
//  * Single channel: one word carries two complex samples, packed MSB first:
//      bits 63:48 = I0, 47:32 = Q0, 31:16 = I1, 15:0 = Q1
//  * Dual channel: one word carries one complex sample of each channel:
//      bits 63:48 = I(ch1), 47:32 = Q(ch1), 31:16 = I(ch0), 15:0 = Q(ch0)
//  * When tags are enabled an "escape" word (default 0xAAAAAAAAAAAAAAAA)
//    introduces an out-of-band word:
//      escape, 0                -> a literal data word equal to the escape
//      escape, [meta:8|val:56]  -> a tag. meta bit 7 set means another tag
//                                  word follows directly (still escaped).
//
// Tag types (gr-rwt lib/common_tag_types.h):
//   RX: TIME (sample index of the next sample), PPS, OVERFLOW
//   TX: HOLD (wait until DAC sample index), SOB, EOB

#ifndef GNURADIO_RWT_DETAIL_WIRE_HPP
#define GNURADIO_RWT_DETAIL_WIRE_HPP

#include <cstddef>
#include <cstdint>

namespace gr::rwt::detail {

inline constexpr std::uint64_t DEFAULT_ESCAPE = 0xAAAAAAAAAAAAAAAAULL;
inline constexpr std::uint64_t TAG_VALUE_MASK = (std::uint64_t(1) << 56) - 1;

enum TagType : std::uint8_t
{
    TAG_NONE     = 0x00,
    TAG_TIME     = 0x01,
    TAG_PPS      = 0x02,
    TAG_OVERFLOW = 0x03,
    TAG_HOLD     = 0x04,
    TAG_SOB      = 0x05,
    TAG_EOB      = 0x06,
};

//! Builds the word that follows an escape to carry a tag.
constexpr std::uint64_t makeTagWord(const std::uint8_t type, const std::uint64_t value, const bool more = false)
{
    return (std::uint64_t(type & 0x7f) << 56) | (more ? (std::uint64_t(0x80) << 56) : 0) | (value & TAG_VALUE_MASK);
}

/*!
 * Stateful decoder for the RX word stream. Feed words one at a time.
 * Escape state survives buffer boundaries, matching gr-rwt.
 */
class StreamDecoder
{
public:
    enum Kind
    {
        CONSUMED, //!< word was an escape prefix; nothing to deliver
        DATA,     //!< data word in 'data'
        TAG,      //!< tag in 'tagType'/'tagValue'
    };

    explicit StreamDecoder(const std::uint64_t escape = DEFAULT_ESCAPE, const bool useEscape = true):
        _escape(escape), _useEscape(useEscape), _inEscape(false) {}

    void configure(const std::uint64_t escape, const bool useEscape)
    {
        _escape = escape;
        _useEscape = useEscape;
        _inEscape = false;
    }

    void reset(void) { _inEscape = false; }

    Kind feed(const std::uint64_t word, std::uint64_t &data, std::uint8_t &tagType, std::uint64_t &tagValue)
    {
        if (_inEscape)
        {
            if (word == 0)
            {
                data = _escape;
                _inEscape = false;
                return DATA;
            }
            const std::uint8_t meta = std::uint8_t(word >> 56);
            tagType = meta & 0x7f;
            tagValue = word & TAG_VALUE_MASK;
            if ((meta & 0x80) == 0) _inEscape = false;
            return TAG;
        }
        if (_useEscape and word == _escape)
        {
            _inEscape = true;
            return CONSUMED;
        }
        data = word;
        return DATA;
    }

private:
    std::uint64_t _escape;
    bool _useEscape;
    bool _inEscape;
};

//! Single channel: one word -> two complex samples (I0,Q0,I1,Q1).
constexpr void unpackSingle(const std::uint64_t w, std::int16_t out[4])
{
    out[0] = std::int16_t(std::uint16_t(w >> 48));
    out[1] = std::int16_t(std::uint16_t(w >> 32));
    out[2] = std::int16_t(std::uint16_t(w >> 16));
    out[3] = std::int16_t(std::uint16_t(w >> 0));
}

constexpr std::uint64_t packSingle(const std::int16_t i0, const std::int16_t q0, const std::int16_t i1, const std::int16_t q1)
{
    return (std::uint64_t(std::uint16_t(i0)) << 48) | (std::uint64_t(std::uint16_t(q0)) << 32) |
           (std::uint64_t(std::uint16_t(i1)) << 16) | (std::uint64_t(std::uint16_t(q1)) << 0);
}

//! Dual channel: one word -> one complex sample for each of ch0 and ch1.
constexpr void unpackDual(const std::uint64_t w, std::int16_t ch0[2], std::int16_t ch1[2])
{
    ch1[0] = std::int16_t(std::uint16_t(w >> 48));
    ch1[1] = std::int16_t(std::uint16_t(w >> 32));
    ch0[0] = std::int16_t(std::uint16_t(w >> 16));
    ch0[1] = std::int16_t(std::uint16_t(w >> 0));
}

constexpr std::uint64_t packDual(const std::int16_t i0, const std::int16_t q0, const std::int16_t i1, const std::int16_t q1)
{
    return (std::uint64_t(std::uint16_t(i1)) << 48) | (std::uint64_t(std::uint16_t(q1)) << 32) |
           (std::uint64_t(std::uint16_t(i0)) << 16) | (std::uint64_t(std::uint16_t(q0)) << 0);
}

} // namespace gr::rwt::detail

#endif // GNURADIO_RWT_DETAIL_WIRE_HPP
