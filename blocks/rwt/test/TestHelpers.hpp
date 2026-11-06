// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GR4_RWT_TEST_HELPERS_HPP
#define GR4_RWT_TEST_HELPERS_HPP

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <type_traits>
#include <string_view>
#include <thread>
#include <vector>

namespace gr::rwt::test {

using Sched = gr::scheduler::Simple<>;

/// runs the scheduler, stopping it if the graph has not finished by `timeout`
inline auto runWithWatchdog(auto& sched, std::chrono::seconds timeout = std::chrono::seconds{10}) {
    auto watchdog = std::jthread([&sched, timeout](std::stop_token stoken) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline && !stoken.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!stoken.stop_requested()) {
            sched.requestStop();
        }
    });
    return sched.runAndWait();
}

/// the first tag at `index` holding `key`
inline const gr::Tag* findTag(const std::vector<gr::Tag>& tags, std::size_t index, std::string_view key) {
    for (const auto& t : tags) {
        if (t.index == index && t.map.find(key) != t.map.end()) {
            return &t;
        }
    }
    return nullptr;
}

/// every index carrying `key`
inline std::vector<std::size_t> tagIndices(const std::vector<gr::Tag>& tags, std::string_view key) {
    std::vector<std::size_t> out;
    for (const auto& t : tags) {
        if (t.map.find(key) != t.map.end()) {
            out.push_back(t.index);
        }
    }
    return out;
}

template<typename T>
std::optional<T> tagValue(const gr::Tag* tag, std::string_view key) {
    if (tag == nullptr) {
        return std::nullopt;
    }
    const auto it = tag->map.find(key);
    if (it == tag->map.end()) {
        return std::nullopt;
    }
    if constexpr (std::is_same_v<T, std::string>) {
        if (!it->second.is_string()) {
            return std::nullopt;
        }
        return it->second.value_or(std::string{});
    } else {
        if (const auto* p = it->second.template get_if<T>()) {
            return *p;
        }
        return std::nullopt;
    }
}

} // namespace gr::rwt::test

#endif // GR4_RWT_TEST_HELPERS_HPP
