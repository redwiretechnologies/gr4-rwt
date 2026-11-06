// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_DETAIL_REGISTERS_HPP
#define GNURADIO_RWT_DETAIL_REGISTERS_HPP

// FPGA user registers of the RWT Oxygen "default" personality, and the
// fpgaloader tool that switches personalities.
//
// Register map and semantics follow gr-rwt (lib/common_registers.h,
// lib/cic_filter_fpga.h, lib/time_helper.cc). The register window is
// memory-mapped from /dev/mem, so this works only on the Oxygen itself and
// needs root (or CAP_SYS_RAWIO).

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace gr::rwt::detail {

// register blocks
inline constexpr std::uint32_t REG_BLK_USER = 0;
inline constexpr std::uint32_t REG_BLK_ADC  = 1;
inline constexpr std::uint32_t REG_BLK_DAC  = 2;

// ADC / DAC blocks
inline constexpr std::uint32_t REG_ADC_CTL        = 0;
inline constexpr std::uint32_t REG_ADC_ESCAPE_MSB = 1;
inline constexpr std::uint32_t REG_ADC_ESCAPE_LSB = 2;
inline constexpr std::uint32_t REG_DAC_CTL        = 0;
inline constexpr std::uint32_t REG_DAC_ESCAPE_MSB = 1;
inline constexpr std::uint32_t REG_DAC_ESCAPE_LSB = 2;

// "default" personality user block
inline constexpr std::uint32_t REG_USER_BLKID                = 0;
inline constexpr std::uint32_t REG_DEFAULT_CTL               = 1;
inline constexpr std::uint32_t MASK_CTL_WHICH_PPS            = 0x0001;
inline constexpr std::uint32_t MASK_CTL_OVERFLOW_ENA         = 0x0002;
inline constexpr std::uint32_t MASK_CTL_PPS_ENA              = 0x0004;
inline constexpr std::uint32_t MASK_CTL_HOLD_ENA             = 0x0008;
inline constexpr std::uint32_t MASK_CTL_SAMPLE_IDX           = 0x0010;
inline constexpr std::uint32_t REG_DEFAULT_STATUS            = 2;
inline constexpr std::uint32_t MASK_STATUS_UNDERFLOW         = 0x0001;
inline constexpr std::uint32_t MASK_STATUS_OVERFLOW          = 0x0002;
inline constexpr std::uint32_t REG_SET_SAMPLE_IDX_MSB        = 3;
inline constexpr std::uint32_t REG_SET_SAMPLE_IDX_LSB        = 4;
inline constexpr std::uint32_t REG_GET_SAMPLE_IDX_ADC_MSB    = 5;
inline constexpr std::uint32_t REG_GET_SAMPLE_IDX_ADC_LSB    = 6;
inline constexpr std::uint32_t REG_GET_SAMPLE_IDX_DAC_MSB    = 7;
inline constexpr std::uint32_t REG_GET_SAMPLE_IDX_DAC_LSB    = 8;

// CIC decimator (in the user block)
inline constexpr std::uint32_t REG_CIC_RESET               = 12;
inline constexpr std::uint32_t REG_CIC_DECIMATION_RAT      = 13;
inline constexpr std::uint32_t REG_CIC_DECIMATION_STAGE_EN = 14;
inline constexpr std::uint32_t REG_CIC_CORRECTION_EN       = 15;
inline constexpr std::uint32_t REG_CIC_CORRECTION_COEFF_A  = 16;
inline constexpr std::uint32_t REG_CIC_CORRECTION_COEFF_B  = 17;
inline constexpr std::uint32_t REG_CIC_FILTER_ID           = 18;

// expected identity of the "default" personality's user block; gr-rwt's
// source requires version 2, its sink accepts 1..2
inline constexpr std::uint16_t DEFAULT_BLK_ID      = 0x0002;
inline constexpr std::uint16_t DEFAULT_BLK_VER_MIN = 0x0001;
inline constexpr std::uint16_t DEFAULT_BLK_VER_MAX = 0x0002;
inline constexpr std::uint16_t RX_BLK_VER_MIN      = 0x0002;

inline constexpr std::uint64_t DEFAULT_REG_BASE_ADDR = 0x9D000000ULL;

inline constexpr int MAX_CIC_DECIMATION_POW = 5; // the CIC decimates by 10^0 .. 10^5

// CIC stage-enable code for a decimation of 10^pow (gr-rwt CIC_DEC_1eN)
constexpr std::uint32_t cicStageForPow(int pow) noexcept {
    constexpr std::uint32_t codes[] = {0, 1, 2, 3, 6, 7};
    return codes[pow < 0 ? 0 : (pow > MAX_CIC_DECIMATION_POW ? MAX_CIC_DECIMATION_POW : pow)];
}

// Q14 gain correction for a decimation of 10^pow (gr-rwt CIC_COEFF_1eN)
constexpr std::uint32_t cicCoeffForPow(int pow) noexcept {
    constexpr std::uint32_t coeffs[] = {0x4000, 0x4333, 0x4666, 0x4999, 0x4CCC, 0x50A3}; // 1.00 .. 1.26
    return coeffs[pow < 0 ? 0 : (pow > MAX_CIC_DECIMATION_POW ? MAX_CIC_DECIMATION_POW : pow)];
}

/// memory-mapped register window of the FPGA user logic
class Registers {
    static constexpr std::size_t   kMapSize = 8192UZ;
    static constexpr std::uint64_t kMapMask = kMapSize - 1U;

    int                     _fd      = -1;
    void*                   _mapBase = MAP_FAILED;
    volatile std::uint8_t*  _regs    = nullptr;

    static constexpr std::uint32_t offsetOf(std::uint32_t blk, std::uint32_t addr) noexcept { return ((blk & 0x3U) << 11U) | ((addr & 0xffU) << 2U); }

public:
    explicit Registers(std::uint64_t baseAddr) {
        _fd = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (_fd < 0) {
            throw std::runtime_error(std::string("could not open /dev/mem (") + std::strerror(errno) + "); local mode runs on the Oxygen as root");
        }
        _mapBase = ::mmap(nullptr, kMapSize, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, static_cast<off_t>(baseAddr & ~kMapMask));
        if (_mapBase == MAP_FAILED) {
            const int err = errno;
            ::close(_fd);
            throw std::runtime_error(std::string("mmap() of the FPGA registers failed (") + std::strerror(err) + ")");
        }
        _regs = static_cast<volatile std::uint8_t*>(_mapBase) + (baseAddr & kMapMask);
    }

    ~Registers() {
        if (_mapBase != MAP_FAILED) {
            ::munmap(_mapBase, kMapSize);
        }
        if (_fd >= 0) {
            ::close(_fd);
        }
    }

    Registers(const Registers&)            = delete;
    Registers& operator=(const Registers&) = delete;

    void write(std::uint32_t blk, std::uint32_t addr, std::uint32_t value) noexcept { *reinterpret_cast<volatile std::uint32_t*>(_regs + offsetOf(blk, addr)) = value; }

    [[nodiscard]] std::uint32_t read(std::uint32_t blk, std::uint32_t addr) const noexcept { return *reinterpret_cast<volatile const std::uint32_t*>(_regs + offsetOf(blk, addr)); }

    // MSB first: some registers latch on the LSB write
    void writeU64(std::uint32_t blk, std::uint32_t msb, std::uint32_t lsb, std::uint64_t value) noexcept {
        write(blk, msb, static_cast<std::uint32_t>(value >> 32U));
        write(blk, lsb, static_cast<std::uint32_t>(value));
    }

    // re-reads the MSB until it is stable, so a carry between the two reads is not missed
    [[nodiscard]] std::uint64_t readU64(std::uint32_t blk, std::uint32_t msbAddr, std::uint32_t lsbAddr) const noexcept {
        std::uint32_t msb = read(blk, msbAddr);
        std::uint32_t prev;
        std::uint32_t lsb;
        do {
            prev = msb;
            lsb  = read(blk, lsbAddr);
            msb  = read(blk, msbAddr);
        } while (msb != prev);
        return (static_cast<std::uint64_t>(msb) << 32U) | lsb;
    }

    void writeMask(std::uint32_t blk, std::uint32_t addr, std::uint32_t mask, bool set) noexcept {
        std::uint32_t v = read(blk, addr);
        write(blk, addr, set ? (v | mask) : (v & ~mask));
    }
};

/// runs `fpgaloader switch <personality> [-f]`, as gr-rwt does before using the radio; returns the exit status
inline int loadPersonality(const std::string& personality, bool force, std::string& error) {
    if (personality.empty() || personality.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-.") != std::string::npos) {
        error = "invalid FPGA personality name '" + personality + "'";
        return -1;
    }
    std::vector<std::string> args = {"fpgaloader", "switch", personality};
    if (force) {
        args.emplace_back("-f");
    }
    std::vector<char*> argv;
    for (auto& a : args) {
        argv.push_back(a.data());
    }
    argv.push_back(nullptr);

    pid_t     pid = 0;
    const int r   = ::posix_spawnp(&pid, "fpgaloader", nullptr, nullptr, argv.data(), environ);
    if (r != 0) {
        error = std::string("could not run fpgaloader: ") + std::strerror(r);
        return -1;
    }
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) {
            error = std::string("waitpid(fpgaloader) failed: ") + std::strerror(errno);
            return -1;
        }
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        error = "fpgaloader switch " + personality + " failed (status " + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1) + ")";
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }
    return 0;
}

} // namespace gr::rwt::detail

#endif // GNURADIO_RWT_DETAIL_REGISTERS_HPP
