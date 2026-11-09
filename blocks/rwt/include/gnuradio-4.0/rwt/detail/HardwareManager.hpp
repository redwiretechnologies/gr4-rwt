// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_DETAIL_HARDWARE_MANAGER_HPP
#define GNURADIO_RWT_DETAIL_HARDWARE_MANAGER_HPP

// The RWT Hardware Manager: the service on an RWT radio that manages its hardware, with a REST API
// (FastAPI, http://<radio>/docs). Its connection_control endpoints configure a whole RF path, not
// just the AD9361: setting a frequency there also sets the filters along the path, and RX and TX
// paths are switched on and off there.
//
//   PUT /api/connection_control/rx_freq/{transceiver}/{rx_path}/{freq}
//   PUT /api/connection_control/tx_freq/{transceiver}/{tx_path}/{freq}
//   PUT /api/connection_control/receive_enable/{transceiver}/{rx_path}/{enable}
//   PUT /api/connection_control/transmit_enable/{transceiver}/{tx_path}/{enable}
//
// transceiver is a name, "Transceiver<N>" (Transceiver0 on an Oxygen; radios with two have
// Transceiver1 too), the path an integer (0 = RX0/TX0, 1 = RX1/TX1),
// freq an integer in Hz, enable "true" or "false".

#include <gnuradio-4.0/rwt/detail/RWTRadio.hpp>

#include <httplib.h>

#include <cctype>
#include <cmath>
#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace gr::rwt::detail {

/// the RWT Hardware Manager's base URL for a block's api_url setting: "none"/"off" turns it off;
/// empty picks it from the radio the block opens (this radio in local mode, the uri's host for
/// "ip:<host>"); anything else is used as given. An empty result means it is not used.
[[nodiscard]] inline std::string resolveHardwareManagerUrl(const std::string& apiUrl, Mode mode, const std::string& uri) {
    if (apiUrl == "none" || apiUrl == "off") {
        return {};
    }
    if (!apiUrl.empty()) {
        return apiUrl;
    }
    if (uri.empty() || uri == "local:") {
        return mode == Mode::Local ? std::string("http://127.0.0.1") : std::string{};
    }
    if (uri.starts_with("ip:")) {
        std::string host = uri.substr(3);
        if (const auto colon = host.find(':'); colon != std::string::npos && host.find(']') == std::string::npos) {
            host.resize(colon); // an iiod port is not the RWT Hardware Manager's port
        }
        return host.empty() ? std::string{} : "http://" + host;
    }
    return {}; // usb:, emu:, ... have no RWT Hardware Manager
}

/// whether a 2xx reply body reports a failure: a top-level "status" of false or a string naming an error
[[nodiscard]] inline bool replyReportsFailure(std::string_view body) {
    const auto key = body.find("\"status\"");
    if (key == std::string_view::npos) {
        return false;
    }
    std::size_t i = body.find(':', key);
    if (i == std::string_view::npos) {
        return false;
    }
    ++i;
    while (i < body.size() && std::isspace(static_cast<unsigned char>(body[i]))) {
        ++i;
    }
    std::string value;
    if (i < body.size() && body[i] == '"') {
        const auto end = body.find('"', i + 1);
        value          = std::string(body.substr(i + 1, end == std::string_view::npos ? std::string_view::npos : end - i - 1));
    } else {
        while (i < body.size() && std::isalpha(static_cast<unsigned char>(body[i]))) {
            value += body[i++];
        }
    }
    for (auto& c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value == "false" || value.find("fail") != std::string::npos || value.find("error") != std::string::npos;
}

/// the connection_control transceiver name of transceiver `n`: Transceiver0, Transceiver1, ...
[[nodiscard]] inline std::string transceiverName(std::size_t n) { return std::format("Transceiver{}", n); }

class HardwareManager {
    std::string                      _baseUrl;
    std::string                      _transceiver = transceiverName(0UZ);
    std::unique_ptr<httplib::Client> _client;

public:
    RWTRadio::Logger trace = [](const std::string&) {}; // each request and its reply, for GR4_RWT_TRACE

    /// an empty base URL turns the RWT Hardware Manager off
    void configure(const std::string& baseUrl, std::size_t transceiver) {
        _client.reset();
        _baseUrl     = baseUrl;
        _transceiver = transceiverName(transceiver);
        if (!_baseUrl.empty()) {
            _client = std::make_unique<httplib::Client>(_baseUrl);
            _client->set_connection_timeout(2, 0);
            _client->set_read_timeout(10, 0); // a frequency change also switches the RF filters
            _client->set_write_timeout(2, 0);
        }
    }
    [[nodiscard]] bool               enabled() const noexcept { return _client != nullptr; }
    [[nodiscard]] const std::string& baseUrl() const noexcept { return _baseUrl; }

    /// PUT `path`; an empty string on success, otherwise why it failed
    [[nodiscard]] std::string put(const std::string& path) {
        if (!_client) {
            return "the RWT Hardware Manager is off";
        }
        const auto res = _client->Put(path, std::string{}, "application/json");
        if (!res) {
            return std::format("PUT {}{}: {}", _baseUrl, path, httplib::to_string(res.error()));
        }
        trace(std::format("PUT {}{} -> {} {}", _baseUrl, path, res->status, res->body.substr(0, 300)));
        if (res->status < 200 || res->status >= 300) {
            return std::format("PUT {}{}: HTTP {} {}", _baseUrl, path, res->status, res->body.substr(0, 300));
        }
        if (replyReportsFailure(res->body)) {
            return std::format("PUT {}{}: {}", _baseUrl, path, res->body.substr(0, 300));
        }
        return {};
    }

    [[nodiscard]] std::string setFrequency(Direction d, std::size_t path, double hz) {
        return put(std::format("/api/connection_control/{}/{}/{}/{}", d == Direction::RX ? "rx_freq" : "tx_freq", _transceiver, path, std::llround(hz)));
    }
    /// receive_enable (RX) or transmit_enable (TX) of one path
    [[nodiscard]] std::string pathEnable(Direction d, std::size_t path, bool enable) {
        return put(std::format("/api/connection_control/{}/{}/{}/{}", d == Direction::RX ? "receive_enable" : "transmit_enable", _transceiver, path, enable ? "true" : "false"));
    }
};

/// RX or TX paths a block switched on through the RWT Hardware Manager (receive_enable /
/// transmit_enable): on() switches the block's paths on, off() switches exactly those off again,
/// once. A path the RWT Hardware Manager refuses is reported and left out.
class PathEnables {
    Direction                _direction;
    std::vector<std::size_t> _on;

public:
    explicit PathEnables(Direction d) noexcept : _direction(d) {}

    void on(HardwareManager& hwm, const std::vector<std::size_t>& paths, const RWTRadio::Logger& report) {
        if (!hwm.enabled()) {
            return;
        }
        for (const std::size_t path : paths) {
            if (const std::string err = hwm.pathEnable(_direction, path, true); !err.empty()) {
                report(std::format("{}{} not switched on: {}", _direction == Direction::RX ? "RX" : "TX", path, err));
            } else {
                _on.push_back(path);
            }
        }
    }
    void off(HardwareManager& hwm, const RWTRadio::Logger& report) {
        for (const std::size_t path : _on) {
            if (const std::string err = hwm.pathEnable(_direction, path, false); !err.empty()) {
                report(std::format("{}{} not switched off: {}", _direction == Direction::RX ? "RX" : "TX", path, err));
            }
        }
        _on.clear();
    }
    [[nodiscard]] const std::vector<std::size_t>& paths() const noexcept { return _on; }
};

} // namespace gr::rwt::detail

#endif // GNURADIO_RWT_DETAIL_HARDWARE_MANAGER_HPP
