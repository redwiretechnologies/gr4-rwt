// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_DETAIL_IIO_HPP
#define GNURADIO_RWT_DETAIL_IIO_HPP

// libiio v1 plumbing for the AD9361 on the RWT Oxygen: contexts, attribute
// access by the sysfs-style names gr-rwt uses, and a ring of buffer blocks.

#include <iio/iio.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <locale>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace gr::rwt::detail {

inline constexpr const char* kPhyName = "ad9361-phy";
inline constexpr const char* kRxName  = "cf-ad9361-lpc";
inline constexpr const char* kTxName  = "cf-ad9361-dds-core-lpc";

[[nodiscard]] inline std::string iioError(int err) {
    char buf[128];
    iio_strerror(err < 0 ? -err : err, buf, sizeof(buf));
    return buf;
}

[[nodiscard]] inline double parseDouble(const std::string& s) {
    std::istringstream ss(s);
    ss.imbue(std::locale::classic());
    double d = 0.0;
    ss >> d;
    return d;
}

/// owning libiio context
class Context {
    iio_context* _ctx = nullptr;

public:
    Context() = default;
    explicit Context(const std::string& uri) {
        iio_context* ctx = iio_create_context(nullptr, uri.empty() ? nullptr : uri.c_str());
        if (const int err = iio_err(ctx); err != 0) {
            throw std::runtime_error("could not create libiio context '" + uri + "': " + iioError(err));
        }
        _ctx = ctx;
    }
    ~Context() {
        if (_ctx != nullptr) {
            iio_context_destroy(_ctx);
        }
    }
    Context(const Context&)            = delete;
    Context& operator=(const Context&) = delete;

    [[nodiscard]] iio_context* get() const noexcept { return _ctx; }

    [[nodiscard]] iio_device* findDevice(const char* name) const {
        iio_device* dev = iio_context_find_device(_ctx, name);
        if (dev == nullptr) {
            throw std::runtime_error(std::string("IIO device '") + name + "' not found");
        }
        return dev;
    }
};

/// AD9361 phy attributes, addressed by the sysfs file names gr-rwt uses
/// ("in_voltage0_hardwaregain", "out_altvoltage0_RX_LO_frequency", ...)
class Ad9361Phy {
    iio_device*                                     _phy = nullptr;
    mutable std::mutex                              _mutex;
    mutable std::map<std::string, const iio_attr*> _cache;

public:
    Ad9361Phy() = default;
    explicit Ad9361Phy(iio_device* phy) : _phy(phy) {}

    /// points at another phy device (or none); forgets the attribute lookups
    void reset(iio_device* phy = nullptr) {
        std::lock_guard lock(_mutex);
        _phy = phy;
        _cache.clear();
    }

    [[nodiscard]] iio_device* device() const noexcept { return _phy; }

    // libiio v1 dropped iio_device_identify_filename(). Channel attributes are
    // matched by filename, preferring voltage0 for attributes the driver shares
    // between the channels of one type (the channel libad9361 uses), then
    // device and debug attributes by name.
    [[nodiscard]] const iio_attr* find(const std::string& filename) const {
        std::lock_guard lock(_mutex);
        if (auto it = _cache.find(filename); it != _cache.end()) {
            return it->second;
        }
        const iio_attr*    found = nullptr;
        const unsigned int nc    = iio_device_get_channels_count(_phy);
        for (unsigned int pass = 0; pass < 2U && found == nullptr; ++pass) {
            for (unsigned int c = 0; c < nc && found == nullptr; ++c) {
                const iio_channel* chn   = iio_device_get_channel(_phy, c);
                const bool         isCh0 = std::strcmp(iio_channel_get_id(chn), "voltage0") == 0;
                if ((pass == 0U) != isCh0) {
                    continue;
                }
                const unsigned int na = iio_channel_get_attrs_count(chn);
                for (unsigned int a = 0; a < na; ++a) {
                    const iio_attr* attr = iio_channel_get_attr(chn, a);
                    const char*     fn   = attr != nullptr ? iio_attr_get_filename(attr) : nullptr;
                    if (fn != nullptr && filename == fn) {
                        found = attr;
                        break;
                    }
                }
            }
        }
        if (found == nullptr) {
            found = iio_device_find_attr(_phy, filename.c_str());
        }
        if (found == nullptr) {
            found = iio_device_find_debug_attr(_phy, filename.c_str());
        }
        _cache[filename] = found;
        return found;
    }

    /// returns an empty string on success, otherwise the reason
    [[nodiscard]] std::string write(const std::string& filename, const std::string& value) const {
        const iio_attr* attr = find(filename);
        if (attr == nullptr) {
            return "no AD9361 attribute " + filename;
        }
        if (const ssize_t r = iio_attr_write_string(attr, value.c_str()); r < 0) {
            return "writing " + filename + "=" + value + " failed: " + iioError(static_cast<int>(r));
        }
        return {};
    }

    [[nodiscard]] std::optional<std::string> read(const std::string& filename) const {
        const iio_attr* attr = find(filename);
        if (attr == nullptr) {
            return std::nullopt;
        }
        char          buf[1024];
        const ssize_t r = iio_attr_read_raw(attr, buf, sizeof(buf) - 1U);
        if (r < 0) {
            return std::nullopt;
        }
        buf[r]        = '\0';
        std::string v = buf;
        while (!v.empty() && (v.back() == '\n' || v.back() == ' ')) {
            v.pop_back();
        }
        return v;
    }

    [[nodiscard]] std::optional<double> readDouble(const std::string& filename) const {
        auto v = read(filename);
        return v ? std::optional<double>(parseDouble(*v)) : std::nullopt;
    }

    /// loads an AD9361 .ftr file holding RX and TX coefficients (gr-rwt load_filter_file())
    [[nodiscard]] std::string loadFilterFile(const std::string& path) const;
};

/// A channel mask, an open buffer stream and a ring of blocks handed back and
/// forth with the kernel (or iiod). Block i is either owned by us or queued.
class BlockRing {
    iio_device*               _dev = nullptr;
    std::vector<iio_channel*> _iioChans; // I,Q pairs of every device channel used
    iio_channels_mask*        _mask   = nullptr;
    iio_buffer_stream*        _stream = nullptr;
    std::vector<iio_block*>   _blocks;
    std::vector<bool>         _queued;
    std::size_t               _sampleSize = 0UZ;
    std::size_t               _blockBytes = 0UZ;
    bool                      _started    = false;

public:
    BlockRing(iio_device* dev, const std::vector<std::size_t>& channels, bool output) : _dev(dev) {
        for (const std::size_t ch : channels) {
            const std::string i  = "voltage" + std::to_string(2UZ * ch);
            const std::string q  = "voltage" + std::to_string(2UZ * ch + 1UZ);
            iio_channel*      ci = iio_device_find_channel(_dev, i.c_str(), output);
            iio_channel*      cq = iio_device_find_channel(_dev, q.c_str(), output);
            if (ci == nullptr || cq == nullptr) {
                throw std::runtime_error("IIO channels for channel " + std::to_string(ch) + " not found");
            }
            _iioChans.push_back(ci);
            _iioChans.push_back(cq);
        }
        _mask = iio_create_channels_mask(iio_device_get_channels_count(_dev));
        if (_mask == nullptr || iio_err(_mask) != 0) {
            _mask = nullptr;
            throw std::runtime_error("unable to create IIO channel mask");
        }
        for (auto* c : _iioChans) {
            iio_channel_enable(c, _mask);
        }
    }

    ~BlockRing() {
        close();
        if (_mask != nullptr) {
            iio_channels_mask_destroy(_mask);
        }
    }

    BlockRing(const BlockRing&)            = delete;
    BlockRing& operator=(const BlockRing&) = delete;

    /// opens the buffer stream and creates the blocks (does not start it); returns 0 or a negative errno
    int open(std::size_t samplesPerBlock, std::size_t numBlocks) {
        if (_stream != nullptr) {
            return 0;
        }
        iio_buffer* buf = iio_device_get_buffer(_dev, 0);
        if (buf == nullptr) {
            return -ENOENT;
        }
        const ssize_t sampleSize = iio_device_get_sample_size(_dev, _mask);
        if (sampleSize <= 0) {
            return sampleSize < 0 ? static_cast<int>(sampleSize) : -EINVAL;
        }
        _sampleSize = static_cast<std::size_t>(sampleSize);
        _blockBytes = samplesPerBlock * _sampleSize;

        iio_buffer_stream* stream = iio_buffer_open(buf, _mask);
        if (const int err = iio_err(stream); err != 0) {
            return err;
        }
        _stream = stream;
        for (std::size_t i = 0; i < numBlocks; ++i) {
            iio_block* b = iio_buffer_stream_create_block(_stream, _blockBytes);
            if (const int err = iio_err(b); err != 0) {
                close();
                return err;
            }
            _blocks.push_back(b);
        }
        _queued.assign(_blocks.size(), false);
        _started = false;
        return 0;
    }

    void close() {
        if (_stream == nullptr) {
            return;
        }
        // cancel first so blocks the hardware still owns come back at once (an RX
        // block at a few S/s through the CIC could otherwise take hours)
        iio_buffer_stream_cancel(_stream);
        for (std::size_t i = 0; i < _blocks.size(); ++i) {
            if (_queued[i]) {
                std::ignore = iio_block_dequeue(_blocks[i], false);
            }
            iio_block_destroy(_blocks[i]);
        }
        _blocks.clear();
        _queued.clear();
        if (_started) {
            iio_buffer_stream_stop(_stream);
        }
        iio_buffer_close(_stream);
        _stream  = nullptr;
        _started = false;
    }

    [[nodiscard]] bool isOpen() const noexcept { return _stream != nullptr; }

    int start() {
        if (_started) {
            return 0;
        }
        const int r = iio_buffer_stream_start(_stream);
        if (r == 0) {
            _started = true;
        }
        return r;
    }

    /// hands block i to the hardware; bytesUsed 0 means the whole block
    int enqueue(std::size_t i, std::size_t bytesUsed = 0UZ) {
        const int r = iio_block_enqueue(_blocks[i], bytesUsed, false);
        if (r == 0) {
            _queued[i] = true;
        }
        return r;
    }

    /// takes block i back: timeoutUs < 0 waits forever, 0 polls once; -ETIMEDOUT when not ready
    int dequeue(std::size_t i, long timeoutUs) {
        if (!_queued[i]) {
            return 0;
        }
        int r;
        if (timeoutUs < 0) {
            r = iio_block_dequeue(_blocks[i], false);
        } else {
            // a non-blocking dequeue returns -EBUSY on every backend while the block is
            // in flight; poll it so callers can bound how long they wait
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
            for (;;) {
                r = iio_block_dequeue(_blocks[i], true);
                if (r != -EBUSY) {
                    break;
                }
                const auto now = std::chrono::steady_clock::now();
                if (now >= deadline) {
                    return -ETIMEDOUT;
                }
                std::this_thread::sleep_for(std::min(std::chrono::duration_cast<std::chrono::microseconds>(deadline - now), std::chrono::microseconds(200)));
            }
        }
        if (r == 0) {
            _queued[i] = false;
        }
        return r;
    }

    [[nodiscard]] bool        queued(std::size_t i) const noexcept { return _queued[i]; }
    [[nodiscard]] std::size_t numBlocks() const noexcept { return _blocks.size(); }
    [[nodiscard]] std::size_t blockBytes() const noexcept { return _blockBytes; }
    [[nodiscard]] std::size_t sampleSize() const noexcept { return _sampleSize; }

    [[nodiscard]] std::uint8_t* start(std::size_t i) const noexcept {
        void* p = iio_block_start(_blocks[i]);
        return iio_err(p) != 0 ? nullptr : static_cast<std::uint8_t*>(p);
    }

    [[nodiscard]] std::uint8_t* first(std::size_t i, const iio_channel* chn) const noexcept {
        void* p = iio_block_first(_blocks[i], chn);
        return iio_err(p) != 0 ? nullptr : static_cast<std::uint8_t*>(p);
    }

    [[nodiscard]] const iio_channel* chanI(std::size_t k) const noexcept { return _iioChans[2UZ * k]; }
    [[nodiscard]] const iio_channel* chanQ(std::size_t k) const noexcept { return _iioChans[2UZ * k + 1UZ]; }
};

inline std::string Ad9361Phy::loadFilterFile(const std::string& path) const {
    const iio_attr* fir = iio_device_find_attr(_phy, "filter_fir_config");
    if (fir == nullptr) {
        return "the AD9361 driver has no filter_fir_config attribute";
    }
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) {
        return "cannot open filter file " + path;
    }
    std::string content;
    char        chunk[4096];
    std::size_t n;
    while ((n = std::fread(chunk, 1, sizeof(chunk), f)) > 0) {
        content.append(chunk, n);
    }
    std::fclose(f);

    // the file must hold RX and TX coefficients ("rx,tx" columns), as gr-rwt checks
    std::istringstream lines(content);
    std::string        line;
    bool               hasBoth = false;
    while (std::getline(lines, line)) {
        if (!line.empty() && (line[0] == '-' || (line[0] >= '0' && line[0] <= '9'))) {
            hasBoth = line.find(',') != std::string::npos;
            break;
        }
    }
    if (!hasBoth) {
        return "filter file " + path + " must contain RX and TX coefficients";
    }
    if (iio_attr_write_raw(fir, content.data(), content.size()) <= 0) {
        return "writing filter_fir_config failed";
    }
    return {};
}

} // namespace gr::rwt::detail

#endif // GNURADIO_RWT_DETAIL_IIO_HPP
