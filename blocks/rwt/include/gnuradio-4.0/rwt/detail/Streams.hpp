// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_DETAIL_STREAMS_HPP
#define GNURADIO_RWT_DETAIL_STREAMS_HPP

// RX and TX sample streams of an RWT radio.
//
// Local mode carries gr-rwt's packed format (see Wire.hpp): RX in-band tags are
// returned as events at the sample offset they precede, TX events are encoded
// before the word holding the sample they belong to. Remote mode carries the
// standard AD9361 layout and no tags. Samples are int16 I,Q pairs, interleaved
// per channel.

#include <gnuradio-4.0/rwt/detail/Iio.hpp>
#include <gnuradio-4.0/rwt/detail/RWTRadio.hpp>
#include <gnuradio-4.0/rwt/detail/Wire.hpp>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <span>
#include <vector>

#include <endian.h>

namespace gr::rwt::detail {

struct StreamEvent {
    std::size_t   offset = 0UZ; ///< sample index within the chunk (RX) or the write call (TX)
    std::uint8_t  type   = TAG_NONE;
    std::uint64_t value  = 0U;
};

struct StreamConfig {
    Mode                     mode = Mode::Local;
    std::vector<std::size_t> channels{0UZ}; ///< device channels, in port order
    std::size_t              samplesPerBlock = 32768UZ;
    std::size_t              numBlocks       = 4UZ;
    std::uint64_t            escape          = DEFAULT_ESCAPE;
    bool                     useTags         = true;
};

namespace stream_detail {
inline void positions(const std::vector<std::size_t>& chans, std::ptrdiff_t& ch0, std::ptrdiff_t& ch1) {
    ch0 = ch1 = -1;
    for (std::size_t k = 0; k < chans.size(); ++k) {
        if (chans[k] == 0UZ) {
            ch0 = static_cast<std::ptrdiff_t>(k);
        }
        if (chans[k] == 1UZ) {
            ch1 = static_cast<std::ptrdiff_t>(k);
        }
    }
}
inline StreamConfig evenBlocks(StreamConfig cfg) {
    // a packed word holds two single-channel samples: keep blocks an even number of frames long
    if (cfg.samplesPerBlock % 2UZ != 0UZ) {
        ++cfg.samplesPerBlock;
    }
    cfg.numBlocks = std::max(cfg.numBlocks, 2UZ);
    return cfg;
}
} // namespace stream_detail

/***********************************************************************
 * RX
 **********************************************************************/
class RxStream {
    StreamConfig   _cfg;
    BlockRing      _ring;
    std::size_t    _cur  = 0UZ;
    bool           _held = false; // _cur is dequeued and being consumed
    bool           _dual;
    std::ptrdiff_t _ch0 = -1;
    std::ptrdiff_t _ch1 = -1;

    // packed (local) state
    StreamDecoder        _decoder;
    const std::uint64_t* _raw    = nullptr;
    std::size_t          _rawLen = 0UZ;
    std::size_t          _rawIdx = 0UZ;
    bool                 _haveCarry = false;
    std::int16_t         _carry[2]{};

    // standard (remote) state
    std::size_t _frameCount = 0UZ;
    std::size_t _frameIdx   = 0UZ;

    int refill(long timeoutUs) {
        if (_held) { // hand the finished block back to the hardware
            _held   = false;
            _raw    = nullptr;
            _rawLen = _rawIdx = 0UZ;
            _frameCount = _frameIdx = 0UZ;
            if (const int r = _ring.enqueue(_cur); r < 0) {
                return r;
            }
            _cur = (_cur + 1UZ) % _ring.numBlocks();
        }
        if (const int r = _ring.dequeue(_cur, timeoutUs); r < 0) {
            return r;
        }
        _held = true;
        std::uint8_t* p = _ring.start(_cur);
        if (p == nullptr) {
            return -EFAULT;
        }
        if (_cfg.mode == Mode::Local) {
            _raw    = reinterpret_cast<const std::uint64_t*>(p);
            _rawLen = _ring.blockBytes() / sizeof(std::uint64_t);
            _rawIdx = 0UZ;
        } else {
            _frameCount = _ring.blockBytes() / _ring.sampleSize();
            _frameIdx   = 0UZ;
        }
        return 0;
    }

    int readPacked(std::span<std::int16_t* const> iq, std::size_t maxSamples, std::vector<StreamEvent>& events, long timeoutUs) {
        std::size_t produced = 0UZ;
        auto        put      = [&iq](std::size_t k, std::size_t pos, std::int16_t i, std::int16_t q) {
            iq[k][2UZ * pos]       = i;
            iq[k][2UZ * pos + 1UZ] = q;
        };
        while (produced < maxSamples) {
            if (_haveCarry) {
                put(0, produced++, _carry[0], _carry[1]);
                _haveCarry = false;
                continue;
            }
            if (_rawIdx >= _rawLen) {
                if (produced > 0UZ) { // return what we have rather than wait a second time
                    break;
                }
                if (const int r = refill(timeoutUs); r < 0) {
                    return r;
                }
                continue;
            }
            std::uint64_t data = 0U;
            std::uint64_t tagValue = 0U;
            std::uint8_t  tagType  = 0U;
            const auto    kind     = _decoder.feed(le64toh(_raw[_rawIdx++]), data, tagType, tagValue);
            if (kind == StreamDecoder::CONSUMED) {
                continue;
            }
            if (kind == StreamDecoder::TAG) {
                events.push_back({produced, tagType, tagValue});
                continue;
            }
            if (_dual) {
                std::int16_t c0[2];
                std::int16_t c1[2];
                unpackDual(data, c0, c1);
                put(static_cast<std::size_t>(_ch0), produced, c0[0], c0[1]);
                put(static_cast<std::size_t>(_ch1), produced, c1[0], c1[1]);
                ++produced;
            } else {
                std::int16_t s[4];
                unpackSingle(data, s);
                put(0, produced++, s[0], s[1]);
                if (produced < maxSamples) {
                    put(0, produced++, s[2], s[3]);
                } else {
                    _carry[0]  = s[2];
                    _carry[1]  = s[3];
                    _haveCarry = true;
                }
            }
        }
        return static_cast<int>(produced);
    }

    int readStandard(std::span<std::int16_t* const> iq, std::size_t maxSamples, long timeoutUs) {
        if (_frameIdx >= _frameCount) {
            if (const int r = refill(timeoutUs); r < 0) {
                return r;
            }
        }
        const std::size_t n    = std::min(maxSamples, _frameCount - _frameIdx);
        const std::size_t step = _ring.sampleSize();
        for (std::size_t k = 0; k < _cfg.channels.size(); ++k) {
            const std::uint8_t* pi = _ring.first(_cur, _ring.chanI(k));
            const std::uint8_t* pq = _ring.first(_cur, _ring.chanQ(k));
            if (pi == nullptr || pq == nullptr) {
                return -EFAULT;
            }
            pi += _frameIdx * step;
            pq += _frameIdx * step;
            for (std::size_t j = 0; j < n; ++j) {
                std::uint16_t i;
                std::uint16_t q;
                std::memcpy(&i, pi + j * step, sizeof(i));
                std::memcpy(&q, pq + j * step, sizeof(q));
                iq[k][2UZ * j]       = static_cast<std::int16_t>(le16toh(i));
                iq[k][2UZ * j + 1UZ] = static_cast<std::int16_t>(le16toh(q));
            }
        }
        _frameIdx += n;
        return static_cast<int>(n);
    }

public:
    RxStream(iio_device* dev, const StreamConfig& cfg) : _cfg(stream_detail::evenBlocks(cfg)), _ring(dev, cfg.channels, false), _dual(cfg.channels.size() == 2UZ), _decoder(cfg.escape, cfg.useTags) { stream_detail::positions(_cfg.channels, _ch0, _ch1); }
    ~RxStream() { deactivate(); }

    /// opens and starts the stream with every block queued, as ADI's examples do; 0 or a negative errno
    int activate() {
        if (_ring.isOpen()) {
            return 0;
        }
        int r = _ring.open(_cfg.samplesPerBlock, _cfg.numBlocks);
        for (std::size_t i = 0; r == 0 && i < _ring.numBlocks(); ++i) {
            r = _ring.enqueue(i);
        }
        if (r == 0) {
            r = _ring.start();
        }
        if (r < 0) {
            _ring.close();
            return r;
        }
        _cur = 0UZ;
        _held = false;
        _decoder.reset();
        _raw    = nullptr;
        _rawLen = _rawIdx = 0UZ;
        _haveCarry = false;
        _frameCount = _frameIdx = 0UZ;
        return 0;
    }

    void deactivate() noexcept {
        _ring.close();
        _held = false;
        _raw  = nullptr;
        _rawLen = _rawIdx = 0UZ;
        _frameCount = _frameIdx = 0UZ;
    }

    [[nodiscard]] bool active() const noexcept { return _ring.isOpen(); }

    /// changes the escape word / tag decoding of a running stream (gr-rwt "escape")
    void setTagConfig(std::uint64_t escape, bool useTags) {
        _cfg.escape  = escape;
        _cfg.useTags = useTags;
        _decoder.configure(escape, useTags);
    }

    /// reads up to maxSamples into iq[k][0 .. 2n) and appends in-band events; returns n,
    /// -ETIMEDOUT when nothing arrived within timeoutUs, or another negative errno
    int read(std::span<std::int16_t* const> iq, std::size_t maxSamples, std::vector<StreamEvent>& events, long timeoutUs) {
        if (!_ring.isOpen()) {
            return -EBADF;
        }
        if (maxSamples == 0UZ) {
            return 0;
        }
        return _cfg.mode == Mode::Local ? readPacked(iq, maxSamples, events, timeoutUs) : readStandard(iq, maxSamples, timeoutUs);
    }
};

/***********************************************************************
 * TX
 **********************************************************************/
class TxStream {
    StreamConfig   _cfg;
    BlockRing      _ring;
    std::size_t    _cur       = 0UZ;
    bool           _haveBlock = false;
    bool           _dual;
    std::ptrdiff_t _ch0 = -1;
    std::ptrdiff_t _ch1 = -1;

    std::uint64_t* _data    = nullptr; // packed (local)
    std::size_t    _dataLen = 0UZ;
    std::size_t    _dataIdx = 0UZ;
    bool           _haveCarry = false;
    std::int16_t   _carry[2]{};

    std::size_t _frameCount = 0UZ; // standard (remote)
    std::size_t _frameIdx   = 0UZ;

    const std::atomic<bool>* _stop = nullptr;

    // waits for the next block to come back from the hardware, then maps it
    int acquire() {
        for (;;) {
            const int r = _ring.dequeue(_cur, 100'000L);
            if (r == -ETIMEDOUT) {
                if (_stop != nullptr && _stop->load(std::memory_order_acquire)) {
                    return -ECANCELED;
                }
                continue; // a far-future HOLD keeps a block in the FPGA for as long as it takes
            }
            if (r < 0) {
                _haveBlock = false;
                return r;
            }
            break;
        }
        std::uint8_t* p = _ring.start(_cur);
        if (p == nullptr) {
            _haveBlock = false;
            return -EFAULT;
        }
        _data       = reinterpret_cast<std::uint64_t*>(p);
        _dataLen    = _ring.blockBytes() / sizeof(std::uint64_t);
        _dataIdx    = 0UZ;
        _frameCount = _ring.blockBytes() / _ring.sampleSize();
        _frameIdx   = 0UZ;
        _haveBlock  = true;
        return 0;
    }

    int submit(std::size_t bytesUsed) {
        int r = _ring.enqueue(_cur, bytesUsed);
        if (r == 0) {
            r = _ring.start(); // starts with the first block, so the DMA never runs dry before data exists
        }
        if (r < 0) {
            return r;
        }
        _haveBlock = false;
        _cur       = (_cur + 1UZ) % _ring.numBlocks();
        return acquire();
    }

    int pushWord(std::uint64_t word) {
        _data[_dataIdx++] = htole64(word);
        return _dataIdx >= _dataLen ? submit(0UZ) : 0;
    }
    int pushTag(std::uint8_t type, std::uint64_t value) {
        if (!_cfg.useTags) {
            return 0;
        }
        if (const int r = pushWord(_cfg.escape); r < 0) {
            return r;
        }
        return pushWord(makeTagWord(type, value));
    }
    int pushData(std::uint64_t word) {
        if (const int r = pushWord(word); r < 0) {
            return r;
        }
        return _cfg.useTags && word == _cfg.escape ? pushWord(0U) : 0; // escaped literal
    }
    int padCarry() {
        if (!_haveCarry) {
            return 0;
        }
        _haveCarry = false;
        return pushData(packSingle(_carry[0], _carry[1], 0, 0));
    }

    int writePacked(std::span<const std::int16_t* const> iq, std::size_t n, std::span<const StreamEvent> events) {
        std::size_t ev = 0UZ;
        for (std::size_t s = 0; s < n; ++s) {
            bool eob = false;
            for (; ev < events.size() && events[ev].offset <= s; ++ev) {
                if (events[ev].type == TAG_EOB) {
                    eob = true;
                    continue;
                }
                // SOB and HOLD start a word of their own, so no earlier sample is held back with them
                if (int r = padCarry(); r < 0) {
                    return r;
                }
                if (int r = pushTag(events[ev].type, events[ev].value); r < 0) {
                    return r;
                }
            }
            int r = 0;
            if (_dual) {
                const std::int16_t* p0 = iq[static_cast<std::size_t>(_ch0)] + 2UZ * s;
                const std::int16_t* p1 = iq[static_cast<std::size_t>(_ch1)] + 2UZ * s;
                if (eob) {
                    r = pushTag(TAG_EOB, 0U); // EOB precedes the word holding the burst's last sample
                }
                if (r == 0) {
                    r = pushData(packDual(p0[0], p0[1], p1[0], p1[1]));
                }
            } else {
                const std::int16_t* p = iq[0] + 2UZ * s;
                if (!_haveCarry && !eob) {
                    _carry[0]  = p[0];
                    _carry[1]  = p[1];
                    _haveCarry = true;
                    continue;
                }
                if (eob) {
                    r = pushTag(TAG_EOB, 0U);
                }
                if (r == 0) {
                    r = _haveCarry ? pushData(packSingle(_carry[0], _carry[1], p[0], p[1])) : pushData(packSingle(p[0], p[1], 0, 0));
                    _haveCarry = false;
                }
            }
            if (r == 0 && eob) {
                r = flush(); // the end of a burst goes out now, not when the block fills
            }
            if (r < 0) {
                return r;
            }
        }
        // an EOB past the last sample (e.g. on an empty write) closes the burst: it precedes
        // the word holding a carried sample, which flush() pads out
        for (; ev < events.size(); ++ev) {
            if (events[ev].type != TAG_EOB) {
                continue;
            }
            int r = pushTag(TAG_EOB, 0U);
            if (r == 0) {
                r = flush();
            }
            if (r < 0) {
                return r;
            }
        }
        return static_cast<int>(n);
    }

    int writeStandard(std::span<const std::int16_t* const> iq, std::size_t n, std::span<const StreamEvent> events) {
        constexpr std::size_t npos    = static_cast<std::size_t>(-1);
        const auto            nextEob = [&events](std::size_t from) {
            for (const auto& e : events) {
                if (e.type == TAG_EOB && e.offset >= from) {
                    return e.offset;
                }
            }
            return npos;
        };
        const std::size_t step = _ring.sampleSize();
        std::size_t       done = 0UZ;
        while (done < n) {
            const std::size_t eobAt = nextEob(done);
            const std::size_t limit = eobAt == npos ? n : std::min(n, eobAt + 1UZ);
            const std::size_t m     = std::min(limit - done, _frameCount - _frameIdx);
            for (std::size_t k = 0; k < _cfg.channels.size(); ++k) {
                std::uint8_t* pi = _ring.first(_cur, _ring.chanI(k));
                std::uint8_t* pq = _ring.first(_cur, _ring.chanQ(k));
                if (pi == nullptr || pq == nullptr) {
                    return -EFAULT;
                }
                pi += _frameIdx * step;
                pq += _frameIdx * step;
                for (std::size_t j = 0; j < m; ++j) {
                    const std::uint16_t li = htole16(static_cast<std::uint16_t>(iq[k][2UZ * (done + j)]));
                    const std::uint16_t lq = htole16(static_cast<std::uint16_t>(iq[k][2UZ * (done + j) + 1UZ]));
                    std::memcpy(pi + j * step, &li, sizeof(li));
                    std::memcpy(pq + j * step, &lq, sizeof(lq));
                }
            }
            _frameIdx += m;
            done += m;
            int r = 0;
            if (_frameIdx >= _frameCount) {
                r = submit(0UZ);
            } else if (eobAt != npos && done == eobAt + 1UZ) {
                r = flush(); // the burst ends here: send the partial block now
            }
            if (r < 0) {
                return r;
            }
        }
        if (nextEob(n) != npos) {
            if (const int r = flush(); r < 0) {
                return r;
            }
        }
        return static_cast<int>(n);
    }

public:
    TxStream(iio_device* dev, const StreamConfig& cfg) : _cfg(stream_detail::evenBlocks(cfg)), _ring(dev, cfg.channels, true), _dual(cfg.channels.size() == 2UZ) { stream_detail::positions(_cfg.channels, _ch0, _ch1); }
    ~TxStream() { deactivate(0L); }

    /// a flag that makes a wait for a free block give up with -ECANCELED
    void setStopFlag(const std::atomic<bool>* stop) noexcept { _stop = stop; }

    int activate() {
        if (_ring.isOpen()) {
            return 0;
        }
        if (const int r = _ring.open(_cfg.samplesPerBlock, _cfg.numBlocks); r < 0) {
            return r;
        }
        _cur       = 0UZ;
        _haveCarry = false;
        if (const int r = acquire(); r < 0) {
            _ring.close();
            return r;
        }
        return 0;
    }

    /// waits up to drainTimeoutUs for queued blocks to go out, then closes the stream
    void deactivate(long drainTimeoutUs) noexcept {
        if (_ring.isOpen()) {
            for (std::size_t i = 0; i < _ring.numBlocks(); ++i) {
                if (_ring.queued(i) && _ring.dequeue(i, drainTimeoutUs) == -ETIMEDOUT) {
                    break; // e.g. a block held by a far-future HOLD tag
                }
            }
            _ring.close();
        }
        _haveBlock = false;
        _data      = nullptr;
        _dataLen = _dataIdx = 0UZ;
        _frameCount = _frameIdx = 0UZ;
        _haveCarry  = false;
    }

    [[nodiscard]] bool active() const noexcept { return _ring.isOpen(); }

    /// sends whatever is buffered, including a partly filled block
    int flush() {
        if (!_ring.isOpen() || !_haveBlock) {
            return 0;
        }
        if (_cfg.mode == Mode::Local) {
            if (const int r = padCarry(); r < 0) {
                return r;
            }
            return _dataIdx == 0UZ ? 0 : submit(_dataIdx * sizeof(std::uint64_t));
        }
        return _frameIdx == 0UZ ? 0 : submit(_frameIdx * _ring.sampleSize());
    }

    /// writes n samples from iq[k][0 .. 2n); events (sorted by offset) are encoded before
    /// the samples they belong to; returns n or a negative errno
    int write(std::span<const std::int16_t* const> iq, std::size_t n, std::span<const StreamEvent> events) {
        if (!_ring.isOpen() || !_haveBlock) {
            return -EBADF;
        }
        return _cfg.mode == Mode::Local ? writePacked(iq, n, events) : writeStandard(iq, n, events);
    }
};

} // namespace gr::rwt::detail

#endif // GNURADIO_RWT_DETAIL_STREAMS_HPP
