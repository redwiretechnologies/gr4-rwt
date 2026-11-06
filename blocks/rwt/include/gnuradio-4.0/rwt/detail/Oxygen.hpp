// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_DETAIL_OXYGEN_HPP
#define GNURADIO_RWT_DETAIL_OXYGEN_HPP

// The RWT Oxygen as one device: its libiio context and AD9361, and in local
// mode the FPGA "default" personality (tags, timing, CIC decimator), set up
// the way gr-rwt's base_device_common, time_helper and cic_filter_fpga do.

#include <gnuradio-4.0/rwt/detail/Iio.hpp>
#include <gnuradio-4.0/rwt/detail/Registers.hpp>
#include <gnuradio-4.0/rwt/detail/Wire.hpp>

#ifdef GR4_RWT_HAS_AD9361
#include <ad9361.h>
#endif

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gr::rwt::detail {

enum class Direction { RX, TX };

/// how the sample stream is laid out
enum class Mode {
    Local,  ///< on the Oxygen, "default" FPGA personality: packed 64-bit words with in-band tags (gr-rwt)
    Remote, ///< "griio" personality over libiio (iiod): standard AD9361 layout, no tags
};

inline constexpr double kMaxBasebandRate = 61'440'000.0;
#ifdef GR4_RWT_HAS_AD9361
inline constexpr double kMinBasebandRate = 520'834.0;   // 25e6/48: libad9361 loads the FIR
inline constexpr double kMinRxRateNoCic  = 600'000.0;   // gr-rwt MIN_SAMPLE_RATE
#else
inline constexpr double kMinBasebandRate = 2'083'334.0; // 25e6/12 without a FIR
inline constexpr double kMinRxRateNoCic  = 2'083'334.0;
#endif
inline constexpr double kTxAttenuationMax = 89.75;

struct OxygenConfig {
    std::string   uri;                 ///< libiio URI; empty means "local:"
    Mode          mode        = Mode::Local;
    std::string   personality = "default";
    bool          loadFpga    = true;
    bool          forceReload = false;
    std::uint64_t baseAddr    = DEFAULT_REG_BASE_ADDR; ///< 0 = no FPGA register access (emulation and tests)
    bool          checkBlkId  = true;
};

/// gr-rwt configuration keys that name AD9361 attributes (base_device_source/sink aliases)
inline const std::multimap<std::string, std::string>& grRwtAliases() {
    static const std::multimap<std::string, std::string> aliases = {
        {"rx_freq", "out_altvoltage0_RX_LO_frequency"},
        {"tx_freq", "out_altvoltage1_TX_LO_frequency"},
        {"rx_bw", "in_voltage_rf_bandwidth"},
        {"tx_bw", "out_voltage_rf_bandwidth"},
        {"quadrature", "in_voltage_quadrature_tracking_en"},
        {"rfdc", "in_voltage_rf_dc_offset_tracking_en"},
        {"bbdc", "in_voltage_bb_dc_offset_tracking_en"},
        {"rx_gain1_mode", "in_voltage0_gain_control_mode"},
        {"rx_gain1", "in_voltage0_hardwaregain"},
        {"rx_gain2_mode", "in_voltage1_gain_control_mode"},
        {"rx_gain2", "in_voltage1_hardwaregain"},
        {"rx_rfport1", "in_voltage0_rf_port_select"},
        {"rx_rfport2", "in_voltage1_rf_port_select"},
        {"tx_rfport1", "out_voltage0_rf_port_select"},
        {"tx_rfport2", "out_voltage1_rf_port_select"},
        {"tx_gain1", "out_voltage0_hardwaregain"},
        {"tx_gain2", "out_voltage1_hardwaregain"},
        {"tx_gain", "out_voltage0_hardwaregain"},
        {"tx_gain", "out_voltage1_hardwaregain"},
    };
    return aliases;
}

class Oxygen {
    OxygenConfig               _cfg;
    std::unique_ptr<Context>   _ctx;
    Ad9361Phy                  _phy;
    iio_device*                _rxDev = nullptr;
    iio_device*                _txDev = nullptr;
    std::size_t                _numRxChannels = 1UZ;
    std::size_t                _numTxChannels = 1UZ;
    std::unique_ptr<Registers> _regs;
    std::uint16_t              _blkId  = 0;
    std::uint16_t              _blkVer = 0;

    static const char* dirPrefix(Direction d) noexcept { return d == Direction::RX ? "in_voltage" : "out_voltage"; }

public:
    using Logger = std::function<void(const std::string&)>;
    Logger warn = [](const std::string&) {};

    Oxygen() = default;
    Oxygen(const Oxygen&)            = delete;
    Oxygen& operator=(const Oxygen&) = delete;

    /// opens the radio; throws std::runtime_error with the reason on failure
    void open(const OxygenConfig& cfg) {
        _cfg = cfg;
        if (_cfg.mode == Mode::Local && _cfg.loadFpga) {
            // gr-rwt loads the personality before touching the radio; fpgaloader leaves an
            // already-loaded image alone unless forced
            std::string err;
            if (loadPersonality(_cfg.personality, _cfg.forceReload, err) != 0) {
                throw std::runtime_error(err);
            }
        }
        _ctx   = std::make_unique<Context>(_cfg.uri.empty() ? std::string("local:") : _cfg.uri);
        _phy.reset(_ctx->findDevice(kPhyName));
        _rxDev = _ctx->findDevice(kRxName);
        _txDev = _ctx->findDevice(kTxName);
        _numRxChannels = iio_device_find_channel(_rxDev, "voltage2", false) != nullptr && iio_device_find_channel(_rxDev, "voltage3", false) != nullptr ? 2UZ : 1UZ;
        _numTxChannels = iio_device_find_channel(_txDev, "voltage2", true) != nullptr && iio_device_find_channel(_txDev, "voltage3", true) != nullptr ? 2UZ : 1UZ;

        if (_cfg.mode == Mode::Local && _cfg.baseAddr != 0U) {
            _regs = std::make_unique<Registers>(_cfg.baseAddr);
            const std::uint32_t blk = _regs->read(REG_BLK_USER, REG_USER_BLKID);
            _blkId                  = static_cast<std::uint16_t>(blk >> 16U);
            _blkVer                 = static_cast<std::uint16_t>(blk & 0xffffU);
            const bool ok           = _blkId == DEFAULT_BLK_ID && _blkVer >= DEFAULT_BLK_VER_MIN && _blkVer <= DEFAULT_BLK_VER_MAX;
            if (!ok && _cfg.checkBlkId) {
                throw std::runtime_error(std::format("FPGA user block id 0x{:04x} ver {} is not the 'default' personality (expected 0x{:04x} ver {}-{}); set check_blkid=false to override", _blkId, _blkVer, DEFAULT_BLK_ID, DEFAULT_BLK_VER_MIN, DEFAULT_BLK_VER_MAX));
            }
        }
    }

    void close() noexcept {
        _regs.reset();
        _phy.reset();
        _rxDev = _txDev = nullptr;
        _ctx.reset();
    }

    [[nodiscard]] bool               isOpen() const noexcept { return _ctx != nullptr; }
    [[nodiscard]] const OxygenConfig& config() const noexcept { return _cfg; }
    [[nodiscard]] Mode               mode() const noexcept { return _cfg.mode; }
    [[nodiscard]] bool               hasRegisters() const noexcept { return _regs != nullptr; }
    [[nodiscard]] Registers*         registers() const noexcept { return _regs.get(); }
    [[nodiscard]] const Ad9361Phy&   phy() const noexcept { return _phy; }
    [[nodiscard]] iio_device*        streamDevice(Direction d) const noexcept { return d == Direction::RX ? _rxDev : _txDev; }
    [[nodiscard]] std::size_t        numChannels(Direction d) const noexcept { return d == Direction::RX ? _numRxChannels : _numTxChannels; }
    [[nodiscard]] std::uint16_t      blockVersion() const noexcept { return _blkVer; }

    /// writes an AD9361 attribute and reports a failure through warn
    bool set(const std::string& filename, const std::string& value) const {
        if (const std::string err = _phy.write(filename, value); !err.empty()) {
            warn(err);
            return false;
        }
        return true;
    }
    bool setNumber(const std::string& filename, double value) const { return set(filename, std::format("{}", static_cast<long long>(std::llround(value)))); }

    // ---- RF controls ---------------------------------------------------------
    void setLo(Direction d, double hz) const { set(d == Direction::RX ? "out_altvoltage0_RX_LO_frequency" : "out_altvoltage1_TX_LO_frequency", std::format("{}", static_cast<long long>(std::llround(hz)))); }
    void powerLo(Direction d, bool on) const { set(d == Direction::RX ? "out_altvoltage0_RX_LO_powerdown" : "out_altvoltage1_TX_LO_powerdown", on ? "0" : "1"); }
    void setBandwidth(Direction d, double hz) const { setNumber(std::string(dirPrefix(d)) + "_rf_bandwidth", hz); }
    void setPort(Direction d, std::size_t ch, const std::string& port) const { set(std::format("{}{}_rf_port_select", dirPrefix(d), ch), port); }
    void setRxGainMode(std::size_t ch, const std::string& mode) const { set(std::format("in_voltage{}_gain_control_mode", ch), mode); }
    void setRxGain(std::size_t ch, double dB) const { set(std::format("in_voltage{}_hardwaregain", ch), std::format("{}", static_cast<long long>(std::llround(dB)))); }
    /// TX output power as attenuation in dB (0 .. 89.75, 0.25 dB steps); gr-rwt's tx_gain is its negative
    void setTxAttenuation(std::size_t ch, double dB) const {
        const double att = std::clamp(std::round(dB * 4.0) / 4.0, 0.0, kTxAttenuationMax);
        set(std::format("out_voltage{}_hardwaregain", ch), std::format("{:.2f}", -att));
    }
    void setRxTracking(bool quadrature, bool rfdc, bool bbdc) const {
        set("in_voltage_quadrature_tracking_en", quadrature ? "1" : "0");
        set("in_voltage_rf_dc_offset_tracking_en", rfdc ? "1" : "0");
        set("in_voltage_bb_dc_offset_tracking_en", bbdc ? "1" : "0");
    }

    /// AD9361 baseband rate; with autoFilter (and libad9361) a matching FIR is loaded,
    /// as gr-rwt's auto_filter does. RX and TX share the clock tree, so this moves both.
    void setBasebandRate(Direction d, double rate, bool autoFilter) const {
#ifdef GR4_RWT_HAS_AD9361
        if (autoFilter) {
            if (const int r = ad9361_set_bb_rate(_phy.device(), static_cast<unsigned long>(std::llround(rate))); r == 0) {
                return;
            } else {
                warn(std::format("ad9361_set_bb_rate({}) failed ({}); writing the rate directly", rate, r));
            }
        }
#else
        (void)autoFilter;
#endif
        setNumber(std::string(dirPrefix(d)) + "_sampling_frequency", rate);
    }
    [[nodiscard]] double basebandRate(Direction d) const { return _phy.readDouble(std::string(dirPrefix(d)) + "_sampling_frequency").value_or(0.0); }

    [[nodiscard]] std::string loadFilter(const std::string& path) const { return _phy.loadFilterFile(path); }

    /// gr-rwt "extra" string: comma-separated key=value pairs; keys are gr-rwt aliases or AD9361 attribute names
    void applyExtra(const std::string& extra) const {
        std::size_t pos = 0;
        while (pos < extra.size()) {
            std::size_t end = extra.find(',', pos);
            if (end == std::string::npos) {
                end = extra.size();
            }
            const std::string item = extra.substr(pos, end - pos);
            pos                    = end + 1U;
            const std::size_t eq   = item.find('=');
            if (item.empty() || eq == std::string::npos) {
                continue;
            }
            const std::string key   = item.substr(0, eq);
            const std::string value = item.substr(eq + 1U);
            const auto        range = grRwtAliases().equal_range(key);
            if (range.first == range.second) {
                set(key, value);
            } else {
                for (auto it = range.first; it != range.second; ++it) {
                    set(it->second, value);
                }
            }
        }
    }

    // ---- FPGA ("default" personality), local mode only -------------------------
    /// escape word and tag enables for one direction (gr-rwt setup_tags() + time_helper::setup())
    void setTagConfig(Direction d, std::uint64_t escape, bool useTags) const {
        if (!_regs) {
            return;
        }
        const std::uint32_t blk = d == Direction::RX ? REG_BLK_ADC : REG_BLK_DAC;
        _regs->write(blk, REG_ADC_ESCAPE_MSB, static_cast<std::uint32_t>(escape >> 32U));
        _regs->write(blk, REG_ADC_ESCAPE_LSB, static_cast<std::uint32_t>(escape));
        _regs->write(blk, REG_ADC_CTL, useTags ? 1U : 0U);
        _regs->writeMask(REG_BLK_USER, REG_DEFAULT_CTL, MASK_CTL_OVERFLOW_ENA | MASK_CTL_HOLD_ENA | MASK_CTL_PPS_ENA, useTags);
    }

    void setExternalPps(bool external) const {
        if (_regs) {
            _regs->writeMask(REG_BLK_USER, REG_DEFAULT_CTL, MASK_CTL_WHICH_PPS, external);
        }
    }

    /// sets the FPGA sample counter now or at the next PPS (gr-rwt sample_idx_now / sample_idx_next_pps)
    void setSampleIndex(std::uint64_t index, bool atNextPps) const {
        if (_regs) {
            _regs->writeMask(REG_BLK_USER, REG_DEFAULT_CTL, MASK_CTL_SAMPLE_IDX, atNextPps);
            _regs->writeU64(REG_BLK_USER, REG_SET_SAMPLE_IDX_MSB, REG_SET_SAMPLE_IDX_LSB, index);
        }
    }

    [[nodiscard]] std::uint64_t sampleIndex(Direction d) const {
        if (!_regs) {
            return 0U;
        }
        return d == Direction::RX ? _regs->readU64(REG_BLK_USER, REG_GET_SAMPLE_IDX_ADC_MSB, REG_GET_SAMPLE_IDX_ADC_LSB) : _regs->readU64(REG_BLK_USER, REG_GET_SAMPLE_IDX_DAC_MSB, REG_GET_SAMPLE_IDX_DAC_LSB);
    }

    [[nodiscard]] std::uint32_t status() const { return _regs ? _regs->read(REG_BLK_USER, REG_DEFAULT_STATUS) : 0U; }

    /// CIC decimator parameters (gr-rwt cic_filter_fpga::write_cic_params(), then reset())
    void applyCic(int pow, bool correction, bool bypass, std::uint32_t arbitrary, std::uint32_t filterId = 0U) const {
        if (!_regs) {
            return;
        }
        const std::uint32_t coeff = cicCoeffForPow(pow);
        _regs->write(REG_BLK_USER, REG_CIC_FILTER_ID, filterId);
        _regs->write(REG_BLK_USER, REG_CIC_DECIMATION_STAGE_EN, cicStageForPow(pow));
        _regs->write(REG_BLK_USER, REG_CIC_CORRECTION_COEFF_A, coeff);
        _regs->write(REG_BLK_USER, REG_CIC_CORRECTION_COEFF_B, coeff);
        _regs->write(REG_BLK_USER, REG_CIC_CORRECTION_EN, correction ? 1U : 0U);
        _regs->write(REG_BLK_USER, REG_CIC_DECIMATION_RAT, arbitrary);
        _regs->write(REG_BLK_USER, REG_CIC_RESET, 0xFFFFU);
        if (!bypass) {
            _regs->write(REG_BLK_USER, REG_CIC_RESET, 0x0000U);
        }
    }
};

/// splits an RX rate into an AD9361 rate and a CIC decimation of 10^pow, as gr-rwt's
/// set_samplerate() does: below the AD9361 floor the AD9361 runs 10^pow times faster
struct RxRatePlan {
    double adRate = 0.0;
    int    pow    = 0;
    bool   valid  = false;
};

[[nodiscard]] inline RxRatePlan planRxRate(double rate, bool useCic) noexcept {
    RxRatePlan p{rate, 0, false};
    if (useCic) {
        while (p.adRate < kMinRxRateNoCic && p.pow < MAX_CIC_DECIMATION_POW) {
            p.adRate *= 10.0;
            ++p.pow;
        }
    }
    p.valid = p.adRate >= (useCic ? kMinRxRateNoCic : kMinBasebandRate) && p.adRate <= kMaxBasebandRate;
    return p;
}

} // namespace gr::rwt::detail

#endif // GNURADIO_RWT_DETAIL_OXYGEN_HPP
