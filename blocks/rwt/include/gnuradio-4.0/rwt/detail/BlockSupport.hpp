// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_DETAIL_BLOCK_SUPPORT_HPP
#define GNURADIO_RWT_DETAIL_BLOCK_SUPPORT_HPP

// Pieces shared by RWTSource and RWTSink: sample conversion, settings parsing
// and tag keys.

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/rwt/detail/Oxygen.hpp>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace gr::rwt {

/// keys of the tags the blocks publish (RX) and act on (TX)
///
/// GNU Radio 4 tag keys are domain:name (gnuradio4-core c145ebc). GNU Radio's own keys are gr:..., and
/// the blocks write them through the tag:: constants (gr:sample_rate, gr:trigger_time, ...). The keys
/// only these blocks define are in the rwt: domain.
namespace tag_key {
inline constexpr std::string_view kSampleIndex = "rwt:sample_index"; ///< RX: FPGA sample index of the tagged sample (uint64)
inline constexpr std::string_view kPps         = "rwt:pps";          ///< RX: FPGA PPS tag value (uint64)
inline constexpr std::string_view kOverflow    = "rwt:overflow";     ///< RX: FPGA overflow tag value (uint64)
inline constexpr std::string_view kTxSample    = "rwt:tx_sample";    ///< TX: start at this FPGA DAC sample index (uint64)
// TX bursts: GNU Radio's gr:tx_sob / gr:tx_eob / gr:tx_time. The sink also reads the bare spellings gr-rwt
// and GNU Radio 4 before c145ebc used, and tx_sample for rwt:tx_sample.
inline constexpr std::string_view kTxTime      = "gr:tx_time";       ///< TX: start at this time; integer ns or floating-point s
inline constexpr std::string_view kTxSob       = "gr:tx_sob";        ///< TX: start of burst
inline constexpr std::string_view kTxEob       = "gr:tx_eob";        ///< TX: the tagged sample ends the burst
inline constexpr std::string_view kTriggerTime = "RWT_TIME";         ///< RX trigger_name of FPGA timestamps
inline constexpr std::string_view kTriggerPps  = "RWT_PPS";          ///< RX trigger_name of FPGA PPS events

/// the key a block reads `key` under: itself, then its name without the domain
[[nodiscard]] constexpr std::string_view legacyName(std::string_view key) noexcept {
    const auto colon = key.find(':');
    return colon == std::string_view::npos ? key : key.substr(colon + 1UZ);
}
} // namespace tag_key

namespace detail {

/// the key tag::put() writes a GNU Radio tag constant under: key() (gr:name) on gnuradio4-core from c145ebc,
/// shortKey() (the bare name) before it
template<typename TTag>
[[nodiscard]] constexpr std::string_view writtenKey(const TTag& t) noexcept {
    if constexpr (requires { t.shortKey(); }) {
        return std::string_view(t.shortKey());
    } else {
        return std::string_view(t.key());
    }
}

/// samples per port item: std::int16_t ports carry interleaved I,Q
template<typename T>
inline constexpr std::size_t kItemsPerSample = std::is_same_v<T, std::int16_t> ? 2UZ : 1UZ;

// RX: 12-bit samples LSB aligned (full scale 2048); TX: MSB aligned (full scale 32768), as in gr-rwt
inline constexpr float kRxFullScale = 2048.0f;
inline constexpr float kTxFullScale = 32768.0f;

template<typename T>
inline void fromIq(const std::int16_t* iq, std::size_t nSamples, T* out) noexcept {
    if constexpr (std::is_same_v<T, std::complex<float>>) {
        constexpr float scale = 1.0f / kRxFullScale;
        for (std::size_t n = 0; n < nSamples; ++n) {
            out[n] = {static_cast<float>(iq[2UZ * n]) * scale, static_cast<float>(iq[2UZ * n + 1UZ]) * scale};
        }
    } else {
        std::copy_n(iq, 2UZ * nSamples, out);
    }
}

[[nodiscard]] inline std::int16_t saturate16(float v) noexcept {
    const float r = std::nearbyint(v);
    return r >= 32767.0f ? std::int16_t{32767} : (r <= -32768.0f ? std::int16_t{-32768} : static_cast<std::int16_t>(r));
}

template<typename T>
inline void toIq(const T* in, std::size_t nSamples, std::int16_t* iq) noexcept {
    if constexpr (std::is_same_v<T, std::complex<float>>) {
        for (std::size_t n = 0; n < nSamples; ++n) {
            iq[2UZ * n]       = saturate16(in[n].real() * kTxFullScale);
            iq[2UZ * n + 1UZ] = saturate16(in[n].imag() * kTxFullScale);
        }
    } else {
        std::copy_n(in, 2UZ * nSamples, iq);
    }
}

[[nodiscard]] inline Mode parseMode(const std::string& mode, const std::string& uri) {
    if (mode == "local") {
        return Mode::Local;
    }
    if (mode == "remote") {
        return Mode::Remote;
    }
    if (!mode.empty()) {
        throw std::invalid_argument("mode must be '', 'local' or 'remote', not '" + mode + "'");
    }
    return uri.empty() || uri == "local:" ? Mode::Local : Mode::Remote;
}

/// "secs,nsecs" (gr-rwt time_now / time_next_pps) to an FPGA sample index at `rate`
[[nodiscard]] inline std::optional<std::uint64_t> parseTime(const std::string& value, double rate) {
    std::uint64_t secs  = 0U;
    std::uint64_t nsecs = 0U;
    if (rate <= 0.0 || std::sscanf(value.c_str(), "%" SCNu64 ",%" SCNu64, &secs, &nsecs) < 1) {
        return std::nullopt;
    }
    const auto r = static_cast<std::uint64_t>(std::llround(rate));
    return secs * r + (nsecs * r) / 1'000'000'000ULL;
}

/// nanoseconds to an FPGA sample index at `rate`, without overflowing for large times
[[nodiscard]] inline std::uint64_t nsToSamples(std::uint64_t ns, double rate) noexcept {
    const auto r = static_cast<std::uint64_t>(std::llround(rate));
    return (ns / 1'000'000'000ULL) * r + ((ns % 1'000'000'000ULL) * r) / 1'000'000'000ULL;
}

[[nodiscard]] inline std::uint64_t samplesToNs(std::uint64_t index, double rate) noexcept {
    if (rate <= 0.0) {
        return 0U;
    }
    const auto r = static_cast<std::uint64_t>(std::llround(rate));
    return (index / r) * 1'000'000'000ULL + ((index % r) * 1'000'000'000ULL) / r;
}

/// a tag value as a number, whatever arithmetic type it was published with
[[nodiscard]] inline std::optional<double> asNumber(const pmt::Value& v) {
    if (const auto* p = v.get_if<double>()) return *p;
    if (const auto* p = v.get_if<float>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::uint64_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::int64_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::uint32_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::int32_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::uint16_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::int16_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::uint8_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<std::int8_t>()) return static_cast<double>(*p);
    if (const auto* p = v.get_if<bool>()) return *p ? 1.0 : 0.0;
    return std::nullopt;
}

/// an integer tag value without going through double (sample indices exceed 2^53)
[[nodiscard]] inline std::optional<std::uint64_t> asUnsigned(const pmt::Value& v) {
    if (const auto* p = v.get_if<std::uint64_t>()) return *p;
    if (const auto* p = v.get_if<std::int64_t>()) return *p < 0 ? std::optional<std::uint64_t>{} : static_cast<std::uint64_t>(*p);
    if (auto d = asNumber(v); d && *d >= 0.0) return static_cast<std::uint64_t>(std::llround(*d));
    return std::nullopt;
}

[[nodiscard]] inline bool asTrue(const pmt::Value& v) {
    const auto n = asNumber(v);
    return n && *n != 0.0;
}

/// GR4_RWT_TRACE set (and not "0"): the blocks print each settings change they apply, with the
/// AD9361's read-back, to stderr
[[nodiscard]] inline bool traceEnabled() noexcept {
    static const bool enabled = [] {
        const char* v = std::getenv("GR4_RWT_TRACE");
        return v != nullptr && *v != '\0' && std::string_view(v) != "0";
    }();
    return enabled;
}

/// a settings value for a log line
[[nodiscard]] inline std::string describeValue(const pmt::Value& v) {
    if (v.is_string()) {
        return std::format("'{}'", v.value_or(std::string{}));
    }
    if (const auto* b = v.get_if<bool>()) {
        return *b ? "true" : "false";
    }
    if (const auto n = asNumber(v)) {
        return std::format("{}", *n);
    }
    return "(value)";
}

/// "key=value, ..." of a settings map, in key order
[[nodiscard]] inline std::string describeSettings(const property_map& m) {
    std::vector<std::string> items;
    for (const auto& [k, v] : m) {
        items.push_back(std::format("{}={}", std::string_view(k.data(), k.size()), describeValue(v)));
    }
    std::ranges::sort(items);
    std::string out;
    for (const auto& i : items) {
        out += out.empty() ? i : ", " + i;
    }
    return out;
}

} // namespace detail
} // namespace gr::rwt

#endif // GNURADIO_RWT_DETAIL_BLOCK_SUPPORT_HPP
