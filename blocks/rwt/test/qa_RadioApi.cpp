// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_RadioApi.cpp — the Oxygen management API (connection_control): frequencies and transmit_enable,
// against a mock of the API's HTTP server, with the radio itself emulated by libiio's emu backend
#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/rwt/RWTSink.hpp>
#include <gnuradio-4.0/rwt/RWTSource.hpp>
#include <gnuradio-4.0/rwt/detail/RadioApi.hpp>

#include "EmuFixture.hpp"

#include <httplib.h>

#include <algorithm>
#include <chrono>
#include <complex>
#include <mutex>
#include <print>
#include <string>
#include <thread>
#include <vector>

using namespace boost::ut;
using namespace std::chrono_literals;
using gr::rwt::test::EmuFixture;
namespace rd = gr::rwt::detail;
using CF32 = std::complex<float>;

namespace {

/// the management API's connection_control endpoints: records each PUT path; `failPrefix` answers 500
class MockApi {
    httplib::Server          _server;
    std::thread              _thread;
    int                      _port = 0;
    mutable std::mutex       _mutex;
    std::vector<std::string> _requests;
    std::string              _failPrefix;

public:
    MockApi() {
        _server.Put(R"(/api/connection_control/.*)", [this](const httplib::Request& req, httplib::Response& res) {
            std::lock_guard lock(_mutex);
            _requests.push_back(req.path);
            if (!_failPrefix.empty() && req.path.starts_with(_failPrefix)) {
                res.status = 500;
                res.set_content(R"({"detail":"mock failure"})", "application/json");
                return;
            }
            res.set_content(R"({"status":"success","settings":{}})", "application/json");
        });
        _port   = _server.bind_to_any_port("127.0.0.1");
        _thread = std::thread([this] { _server.listen_after_bind(); });
        _server.wait_until_ready();
    }
    ~MockApi() {
        _server.stop();
        _thread.join();
    }
    [[nodiscard]] std::string url() const { return std::format("http://127.0.0.1:{}", _port); }
    void                      failWith(std::string prefix) {
        std::lock_guard lock(_mutex);
        _failPrefix = std::move(prefix);
    }
    [[nodiscard]] std::vector<std::string> requests() const {
        std::lock_guard lock(_mutex);
        return _requests;
    }
    [[nodiscard]] bool saw(const std::string& path) const {
        const auto r = requests();
        return std::ranges::find(r, path) != r.end();
    }
};

/// a source that never produces
template<typename T>
struct Silent : gr::Block<Silent<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(Silent, out);
    gr::work::Status processBulk(gr::OutputSpanLike auto& output) noexcept {
        output.publish(0UZ);
        return gr::work::Status::OK;
    }
};

/// a sink that takes everything
template<typename T>
struct Drain : gr::Block<Drain<T>> {
    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(Drain, in);
    gr::work::Status processBulk(gr::InputSpanLike auto& input) noexcept {
        std::ignore = input.consume(input.size());
        return gr::work::Status::OK;
    }
};

gr::property_map emuSettings(const EmuFixture& emu, const std::string& apiUrl, gr::property_map extra = {}) {
    gr::property_map m{{"uri", emu.uri()}, {"mode", std::string("local")}, {"load_fpga", false}, {"base_addr", std::uint64_t{0}}, {"sample_rate", 2.5e6f}, {"auto_filter", false}, {"buffer_size", gr::Size_t{64}}, {"api_url", apiUrl}};
    for (auto& [k, v] : extra) {
        m.insert_or_assign(k, v);
    }
    return m;
}

template<typename TSched>
void sendSettings(TSched& sched, std::string_view uniqueName, gr::property_map payload) {
    gr::MsgPortOutBuiltin request;
    std::ignore = request.connect(sched.msgIn);
    gr::sendMessage<gr::message::Command::Set>(request, uniqueName, gr::block::property::kStagedSetting, std::move(payload), "qa_RadioApi");
}

bool eventually(auto predicate, std::chrono::milliseconds timeout = 3000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    }
    return false;
}

using Sched = gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded>;

} // namespace

const boost::ut::suite<"management API"> radioApiTests = [] {
    "resolveApiUrl"_test = [] {
        using enum rd::Mode;
        expect(eq(rd::resolveApiUrl("", Local, ""), std::string("http://127.0.0.1"))) << "local mode: this Oxygen";
        expect(eq(rd::resolveApiUrl("", Local, "local:"), std::string("http://127.0.0.1")));
        expect(eq(rd::resolveApiUrl("", Remote, "ip:192.168.10.1"), std::string("http://192.168.10.1"))) << "remote: the uri's host";
        expect(eq(rd::resolveApiUrl("", Remote, "ip:192.168.10.1:30431"), std::string("http://192.168.10.1"))) << "not iiod's port";
        expect(rd::resolveApiUrl("", Local, "emu:/tmp/x.xml").empty()) << "the emulator has no API";
        expect(rd::resolveApiUrl("none", Local, "").empty());
        expect(rd::resolveApiUrl("off", Remote, "ip:10.0.0.1").empty());
        expect(eq(rd::resolveApiUrl("http://oxygen:8000", Local, ""), std::string("http://oxygen:8000"))) << "given as is";
    };

    "replyReportsFailure"_test = [] {
        expect(!rd::replyReportsFailure(R"({"status":"success","settings":{}})"));
        expect(!rd::replyReportsFailure(R"({"status": true})"));
        expect(!rd::replyReportsFailure(R"({"settings":{}})"));
        expect(rd::replyReportsFailure(R"({"status": false})"));
        expect(rd::replyReportsFailure(R"({"status":"Failed to set frequency"})"));
        expect(rd::replyReportsFailure(R"({"status":"error"})"));
    };

    if (!EmuFixture::available()) {
        std::println("libiio was built without the emu backend: skipping the block tests");
        return;
    }

    "the source sets RX frequencies through connection_control rx_freq"_test = [] {
        MockApi   api;
        EmuFixture emu;
        emu.setRxData(EmuFixture::words(std::vector<std::uint64_t>(64, 0U)));
        gr::Graph graph;
        auto&     rx  = graph.emplaceBlock<gr::rwt::RWTSource<CF32>>(emuSettings(emu, api.url(), {{"frequency", 915e6}, {"channel", gr::Size_t{1}}}));
        auto&     out = graph.emplaceBlock<Drain<CF32>>();
        expect(graph.connect<"out", "in">(rx, out).has_value());
        const std::string rxName(rx.unique_name);
        const auto        rxLo = [&rx] { return rx._dev.phy().readDouble("out_altvoltage0_RX_LO_frequency").value_or(-1.0); };
        Sched             sched;
        expect(sched.exchange(std::move(graph)).has_value());
        auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });

        expect(eventually([&] { return api.saw("/api/connection_control/rx_freq/Transceiver0/1/915000000"); })) << "start: RX1 path tuned through the API";
        const double loBefore = rxLo();
        expect(loBefore != 915e6) << "the API, not libiio, set the frequency";

        sendSettings(sched, rxName, {{"frequency", 433.92e6}});
        expect(eventually([&] { return api.saw("/api/connection_control/rx_freq/Transceiver0/1/433920000"); })) << "a live retune goes through the API";
        expect(std::ranges::none_of(api.requests(), [](const std::string& r) { return r.contains("/Transceiver0/0/"); })) << "only the path in use";
        sched.requestStop();
    };

    "the dual sink tunes both TX paths, switches them on at start and off at stop"_test = [] {
        MockApi api;
        {
            EmuFixture emu;
            gr::Graph  graph;
            auto&      s0 = graph.emplaceBlock<Silent<CF32>>();
            auto&      s1 = graph.emplaceBlock<Silent<CF32>>();
            auto&      tx = graph.emplaceBlock<gr::rwt::RWTDualSink<CF32>>(emuSettings(emu, api.url(), {{"frequency", 416e6}}));
            expect(graph.connect<"out", "in#0">(s0, tx).has_value());
            expect(graph.connect<"out", "in#1">(s1, tx).has_value());
            const std::string txName(tx.unique_name);
            Sched             sched;
            expect(sched.exchange(std::move(graph)).has_value());
            auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });

            expect(eventually([&] { return api.saw("/api/connection_control/transmit_enable/Transceiver0/1/true"); })) << "TX1 switched on";
            const auto atStart = api.requests();
            const auto at      = [&atStart](const std::string& r) { return std::ranges::find(atStart, r) - atStart.begin(); };
            expect(at("/api/connection_control/tx_freq/Transceiver0/0/416000000") < at("/api/connection_control/transmit_enable/Transceiver0/0/true")) << "tuned before switched on";
            expect(at("/api/connection_control/tx_freq/Transceiver0/1/416000000") < static_cast<long>(atStart.size()));
            expect(at("/api/connection_control/transmit_enable/Transceiver0/0/true") < static_cast<long>(atStart.size())) << "TX0 switched on";
            expect(!api.saw("/api/connection_control/transmit_enable/Transceiver0/0/false")) << "not off while running";

            sendSettings(sched, txName, {{"frequency", 434.5e6}});
            expect(eventually([&] { return api.saw("/api/connection_control/tx_freq/Transceiver0/1/434500000"); })) << "a live retune of both paths";

            sched.requestStop();
            runner.join();
            expect(api.saw("/api/connection_control/transmit_enable/Transceiver0/0/false")) << "TX0 switched off at stop";
            expect(api.saw("/api/connection_control/transmit_enable/Transceiver0/1/false")) << "TX1 switched off at stop";
        }
        const auto all = api.requests();
        expect(eq(std::ranges::count(all, std::string("/api/connection_control/transmit_enable/Transceiver0/0/false")), 1)) << "switched off once, not again when the block is destroyed";
    };

    "api_transceiver picks the transceiver: 1 = Transceiver1"_test = [] {
        expect(eq(rd::transceiverName(0UZ), std::string("Transceiver0")));
        expect(eq(rd::transceiverName(1UZ), std::string("Transceiver1")));
        MockApi    api;
        EmuFixture emu;
        gr::Graph  graph;
        auto&      src = graph.emplaceBlock<Silent<CF32>>();
        auto&      tx  = graph.emplaceBlock<gr::rwt::RWTSink<CF32>>(emuSettings(emu, api.url(), {{"frequency", 416e6}, {"api_transceiver", gr::Size_t{1}}, {"channel", gr::Size_t{1}}}));
        expect(graph.connect<"out", "in">(src, tx).has_value());
        Sched sched;
        expect(sched.exchange(std::move(graph)).has_value());
        auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });
        expect(eventually([&] { return api.saw("/api/connection_control/transmit_enable/Transceiver1/1/true"); })) << "TX1 of Transceiver1 switched on";
        expect(api.saw("/api/connection_control/tx_freq/Transceiver1/1/416000000")) << "and tuned";
        expect(std::ranges::none_of(api.requests(), [](const std::string& r) { return r.contains("Transceiver0"); })) << "nothing sent to Transceiver0";
        sched.requestStop();
        runner.join();
        expect(api.saw("/api/connection_control/transmit_enable/Transceiver1/1/false")) << "switched off at stop";
    };

    "a failing API falls back to libiio for the frequency"_test = [] {
        MockApi api;
        api.failWith("/api/connection_control/tx_freq");
        EmuFixture emu;
        gr::Graph  graph;
        auto&      src = graph.emplaceBlock<Silent<CF32>>();
        auto&      tx  = graph.emplaceBlock<gr::rwt::RWTSink<CF32>>(emuSettings(emu, api.url(), {{"frequency", 416e6}}));
        expect(graph.connect<"out", "in">(src, tx).has_value());
        const auto txLo = [&tx] { return tx._dev.phy().readDouble("out_altvoltage1_TX_LO_frequency").value_or(-1.0); };
        Sched      sched;
        expect(sched.exchange(std::move(graph)).has_value());
        auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });
        expect(eventually([&] { return tx._dev.isOpen() && txLo() == 416e6; })) << "the AD9361 LO set through libiio instead";
        expect(api.saw("/api/connection_control/transmit_enable/Transceiver0/0/true")) << "the TX path is still switched on";
        expect(tx.state() == gr::lifecycle::State::RUNNING) << "and the block runs";
        sched.requestStop();
    };

    "an unreachable API does not stop the block"_test = [] {
        std::string deadUrl;
        {
            MockApi gone;
            deadUrl = gone.url(); // nothing listens there once it is gone
        }
        EmuFixture emu;
        gr::Graph  graph;
        auto&      src = graph.emplaceBlock<Silent<CF32>>();
        auto&      tx  = graph.emplaceBlock<gr::rwt::RWTSink<CF32>>(emuSettings(emu, deadUrl, {{"frequency", 416e6}}));
        expect(graph.connect<"out", "in">(src, tx).has_value());
        const auto txLo = [&tx] { return tx._dev.phy().readDouble("out_altvoltage1_TX_LO_frequency").value_or(-1.0); };
        Sched      sched;
        expect(sched.exchange(std::move(graph)).has_value());
        auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });
        expect(eventually([&] { return tx._dev.isOpen() && txLo() == 416e6; }, 8000ms)) << "libiio LO, block running";
        sched.requestStop();
    };
};

int main() {}
