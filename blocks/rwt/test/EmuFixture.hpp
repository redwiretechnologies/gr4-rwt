// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GR4_RWT_TEST_EMU_FIXTURE_HPP
#define GR4_RWT_TEST_EMU_FIXTURE_HPP

// libiio v1's emulation backend ("emu:") with an emulated AD9361 (libiio's
// FMCOMMS2 test description). The emu backend serves RX blocks from
// cf-ad9361-lpc_buf0.bin, looping at its end, and writes each dequeued TX block
// to cf-ad9361-dds-core-lpc_buf0.bin.

#include <iio/iio.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <unistd.h>

#ifndef GR4_RWT_TEST_DATA_DIR
#error "GR4_RWT_TEST_DATA_DIR must name the directory holding fmcomms2.xml"
#endif

namespace gr::rwt::test {

class EmuFixture {
    std::string _dir;
    std::string _uri;

public:
    static bool available() { return iio_has_backend(nullptr, "emu"); }

    EmuFixture() {
        const char*       tmp  = std::getenv("TMPDIR");
        std::string       tmpl = std::string(tmp != nullptr ? tmp : "/tmp") + "/gr4-rwt-emu-XXXXXX";
        std::vector<char> buf(tmpl.begin(), tmpl.end());
        buf.push_back('\0');
        if (::mkdtemp(buf.data()) == nullptr) {
            throw std::runtime_error("mkdtemp failed");
        }
        _dir = buf.data();
        std::ifstream in(std::string(GR4_RWT_TEST_DATA_DIR) + "/fmcomms2.xml", std::ios::binary);
        if (!in) {
            throw std::runtime_error("fmcomms2.xml not found in " GR4_RWT_TEST_DATA_DIR);
        }
        std::ofstream out(_dir + "/fmcomms2.xml", std::ios::binary);
        out << in.rdbuf();
        _uri = "emu:" + _dir + "/fmcomms2.xml";
        setRxData(std::vector<std::uint8_t>(64, 0));
    }

    ~EmuFixture() {
        if (DIR* d = ::opendir(_dir.c_str())) {
            while (dirent* e = ::readdir(d)) {
                const std::string n = e->d_name;
                if (n != "." && n != "..") {
                    ::unlink((_dir + "/" + n).c_str());
                }
            }
            ::closedir(d);
        }
        ::rmdir(_dir.c_str());
    }

    EmuFixture(const EmuFixture&)            = delete;
    EmuFixture& operator=(const EmuFixture&) = delete;

    [[nodiscard]] const std::string& uri() const { return _uri; }

    void setRxData(const std::vector<std::uint8_t>& bytes) const {
        std::ofstream out(_dir + "/cf-ad9361-lpc_buf0.bin", std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }

    [[nodiscard]] std::vector<std::uint8_t> txData() const {
        std::ifstream in(_dir + "/cf-ad9361-dds-core-lpc_buf0.bin", std::ios::binary);
        return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    }

    [[nodiscard]] static std::vector<std::uint8_t> words(const std::vector<std::uint64_t>& w) {
        std::vector<std::uint8_t> out(w.size() * 8U);
        for (std::size_t i = 0; i < w.size(); ++i) {
            for (std::size_t b = 0; b < 8U; ++b) {
                out[i * 8U + b] = static_cast<std::uint8_t>(w[i] >> (8U * b)); // little endian
            }
        }
        return out;
    }

    [[nodiscard]] static std::vector<std::uint64_t> unwords(const std::vector<std::uint8_t>& c) {
        std::vector<std::uint64_t> out;
        for (std::size_t i = 0; i + 8U <= c.size(); i += 8U) {
            std::uint64_t w = 0U;
            for (std::size_t b = 0; b < 8U; ++b) {
                w |= static_cast<std::uint64_t>(c[i + b]) << (8U * b);
            }
            out.push_back(w);
        }
        return out;
    }

    [[nodiscard]] static std::vector<std::uint8_t> int16s(const std::vector<std::int16_t>& v) {
        std::vector<std::uint8_t> out;
        for (const std::int16_t x : v) {
            out.push_back(static_cast<std::uint8_t>(static_cast<std::uint16_t>(x)));
            out.push_back(static_cast<std::uint8_t>(static_cast<std::uint16_t>(x) >> 8U));
        }
        return out;
    }
};

} // namespace gr::rwt::test

#endif // GR4_RWT_TEST_EMU_FIXTURE_HPP
