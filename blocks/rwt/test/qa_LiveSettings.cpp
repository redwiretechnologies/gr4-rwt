// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_LiveSettings.cpp — settings sent to running RWT blocks as messages, the way gnuradio4-control-plane
// (and so gr-studio) sends them: a Set to the block's unique name on the scheduler's message input.
#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/rwt/RWTSink.hpp>
#include <gnuradio-4.0/rwt/RWTSource.hpp>

#ifdef GR4_RWT_TEST_BLOCKING_BACKOFF
#include <gnuradio-4.0/scheduler/BlockingBackoff.hpp>
#endif

#include "EmuFixture.hpp"

#include <atomic>
#include <chrono>
#include <complex>
#include <format>
#include <print>
#include <string>
#include <thread>

using namespace boost::ut;
using namespace std::chrono_literals;
using gr::rwt::test::EmuFixture;
using CF32 = std::complex<float>;

namespace {

/// a source that never produces, as a TX chain with nothing to send yet
template<typename T>
struct Silent : gr::Block<Silent<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(Silent, out);
    gr::work::Status processBulk(gr::OutputSpanLike auto& output) noexcept {
        output.publish(0UZ);
        return gr::work::Status::OK;
    }
};

gr::property_map emuSettings(const EmuFixture& emu, gr::property_map extra = {}) {
    gr::property_map m{{"uri", emu.uri()}, {"mode", std::string("local")}, {"load_fpga", false}, {"base_addr", std::uint64_t{0}}, {"sample_rate", 2.5e6f}, {"auto_filter", false}};
    for (auto& [k, v] : extra) {
        m.insert_or_assign(k, v);
    }
    return m;
}

/// gnuradio4-control-plane's send_block_settings_request(): a Set on the scheduler's message input,
/// then wait for the block's reply on the scheduler's message output. gr-studio's default mode,
/// 'staged', is the staged_settings endpoint, which replies; 'immediate' is settings, which GR4
/// applies without a reply.
template<typename TSched>
std::optional<gr::Message> sendSettings(TSched& sched, std::string_view uniqueName, gr::property_map payload, std::chrono::milliseconds timeout = 1000ms, std::string_view endpoint = gr::block::property::kStagedSetting) {
    gr::MsgPortOutBuiltin request;
    gr::MsgPortInBuiltin  response;
    expect(request.connect(sched.msgIn).has_value());
    expect(sched.msgOut.connect(response).has_value());
    static std::atomic<int> id{0};
    const std::string        requestId = std::format("live-{}", id.fetch_add(1));
    gr::sendMessage<gr::message::Command::Set>(request, uniqueName, endpoint, std::move(payload), requestId);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto available = response.streamReader().available();
        if (available == 0UZ) {
            std::this_thread::sleep_for(5ms);
            continue;
        }
        auto replies = response.streamReader().get<gr::SpanReleasePolicy::ProcessAll>(available);
        for (const auto& reply : replies) {
            if (reply.clientRequestID == requestId && reply.serviceName == uniqueName) {
                return reply;
            }
        }
    }
    return std::nullopt;
}

/// polls `read` until it returns `want`
bool eventually(auto read, double want, std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (const auto v = read(); v && std::abs(*v - want) < 0.5) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    }
    return false;
}

template<typename TSched>
void repeaterTest(const char* label) {
    EmuFixture emu;
    emu.setRxData(EmuFixture::words(std::vector<std::uint64_t>(64, 0U)));

    gr::Graph graph;
    auto&     rx = graph.emplaceBlock<gr::rwt::RWTSource<CF32>>(emuSettings(emu, {{"frequency", 915e6}, {"buffer_size", gr::Size_t{64}}}));
    auto&     tx = graph.emplaceBlock<gr::rwt::RWTSink<CF32>>(emuSettings(emu, {{"frequency", 916e6}, {"attenuation1", 40.0}, {"buffer_size", gr::Size_t{64}}}));
    expect(graph.connect<"out", "in">(rx, tx).has_value());
    const std::string rxName(rx.unique_name);
    const std::string txName(tx.unique_name);

    TSched sched;
    expect(sched.exchange(std::move(graph)).has_value());
    auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });

    // wait until both blocks hold the radio
    const auto up = [](auto& blk) { return blk._dev.isOpen(); };
    for (int i = 0; i < 200 && !(up(rx) && up(tx)); ++i) {
        std::this_thread::sleep_for(10ms);
    }
    expect(up(rx) && up(tx)) << label << "both blocks started";

    const auto txLo  = [&tx] { return tx._dev.phy().readDouble("out_altvoltage1_TX_LO_frequency"); };
    const auto rxLo  = [&rx] { return rx._dev.phy().readDouble("out_altvoltage0_RX_LO_frequency"); };
    const auto txAtt = [&tx] { return tx._dev.phy().readDouble("out_voltage0_hardwaregain"); };

    const auto rxReply = sendSettings(sched, rxName, {{"frequency", 433.92e6}});
    expect(rxReply.has_value()) << label << "the source replies";
    expect(eventually(rxLo, 433.92e6)) << label << "the source's RX LO moves";

    const auto txReply = sendSettings(sched, txName, {{"frequency", 434.5e6}, {"attenuation1", 20.0}});
    expect(txReply.has_value()) << label << "the sink replies";
    if (txReply) {
        expect(txReply->data.has_value()) << label << "the sink accepts the settings";
    }
    expect(eventually(txLo, 434.5e6)) << label << "the sink's TX LO moves";
    expect(eventually(txAtt, -20.0)) << label << "the sink's TX attenuation changes";
    std::println("{}: RX LO {} Hz, TX LO {} Hz, TX gain {} dB", label, rxLo().value_or(-1.0), txLo().value_or(-1.0), txAtt().value_or(-1.0));

    // 'immediate' mode: applied, but GR4 sends no reply (the control plane reports a timeout)
    const auto immediate = sendSettings(sched, txName, {{"attenuation1", 30.0}}, 300ms, gr::block::property::kSetting);
    expect(!immediate.has_value()) << label << "GR4 does not reply to settings Set";
    expect(eventually(txAtt, -30.0)) << label << "an immediate settings change reaches the sink";

    // a rate the AD9361 cannot take is refused: the graph keeps running and the setting goes back
    std::ignore = sendSettings(sched, txName, {{"sample_rate", 1e9f}}, 300ms);
    std::this_thread::sleep_for(200ms);
    expect(tx.state() == gr::lifecycle::State::RUNNING) << label << "a refused sample_rate leaves the sink running";
    expect(eq(tx.sample_rate.value, 2.5e6f)) << label << "the refused sample_rate goes back to the value in force";
    expect(eventually(txLo, 434.5e6)) << label << "the sink still holds the radio";

    // a later RX retune tags the stream with the new frequency; the sink's own TX frequency must stay
    std::ignore = sendSettings(sched, rxName, {{"frequency", 440e6}}, 300ms);
    expect(eventually(rxLo, 440e6)) << label << "the source retunes again";
    std::this_thread::sleep_for(300ms);
    expect(eventually(txLo, 434.5e6, 100ms)) << label << "the RX retune leaves the sink's TX LO alone";
    std::println("{}: after RX retune: RX LO {} Hz, TX LO {} Hz, sink frequency setting {}", label, rxLo().value_or(-1.0), txLo().value_or(-1.0), tx.frequency.value);

    sched.requestStop();
}

template<typename TSched>
void idleSinkTest(const char* label) {
    EmuFixture emu;
    gr::Graph  graph;
    auto&      src = graph.emplaceBlock<Silent<CF32>>();
    auto&      tx  = graph.emplaceBlock<gr::rwt::RWTSink<CF32>>(emuSettings(emu, {{"frequency", 916e6}, {"attenuation1", 40.0}, {"buffer_size", gr::Size_t{64}}}));
    expect(graph.connect<"out", "in">(src, tx).has_value());
    const std::string txName(tx.unique_name);
    TSched            sched;
    expect(sched.exchange(std::move(graph)).has_value());
    auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });
    for (int i = 0; i < 200 && !tx._dev.isOpen(); ++i) {
        std::this_thread::sleep_for(10ms);
    }
    expect(tx._dev.isOpen()) << label << "the sink started";
    const auto txLo  = [&tx] { return tx._dev.phy().readDouble("out_altvoltage1_TX_LO_frequency"); };
    const auto reply = sendSettings(sched, txName, {{"frequency", 434.5e6}});
    expect(reply.has_value()) << label << "an idle sink replies";
    expect(eventually(txLo, 434.5e6)) << label << "an idle sink still retunes";
    std::println("{} idle sink: TX LO {} Hz", label, txLo().value_or(-1.0));
    sched.requestStop();
}

/// a sink whose start failed holds no radio: a settings change is reported, not silently lost
void failedStartTest() {
    EmuFixture emu;
    gr::Graph  graph;
    auto&      src = graph.emplaceBlock<Silent<CF32>>();
    auto&      tx  = graph.emplaceBlock<gr::rwt::RWTSink<CF32>>(emuSettings(emu, {{"channel", gr::Size_t{5}}}));
    expect(graph.connect<"out", "in">(src, tx).has_value());
    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
    expect(sched.exchange(std::move(graph)).has_value());
    auto runner = std::jthread([&sched] { std::ignore = sched.runAndWait(); });
    for (int i = 0; i < 200 && tx._startError.empty(); ++i) {
        std::this_thread::sleep_for(10ms);
    }
    expect(!tx._startError.empty()) << "the start failed";
    expect(!tx._dev.isOpen());
    std::println("failed start: {}", tx._startError);
    sched.requestStop();
}

} // namespace

const boost::ut::suite<"live settings"> liveSettingsTests = [] {
    if (!EmuFixture::available()) {
        std::println("libiio was built without the emu backend: skipping");
        return;
    }
    using enum gr::scheduler::ExecutionPolicy;
    "settings messages reach a running source and sink (Simple, single-threaded)"_test = [] { repeaterTest<gr::scheduler::Simple<singleThreaded>>("simple-single"); };
    "settings messages reach a running source and sink (Simple, multi-threaded)"_test  = [] { repeaterTest<gr::scheduler::Simple<multiThreaded>>("simple-multi"); };
    "settings messages reach a sink with no input (Simple, single-threaded)"_test = [] { idleSinkTest<gr::scheduler::Simple<singleThreaded>>("simple-single"); };
    "settings messages reach a sink with no input (Simple, multi-threaded)"_test  = [] { idleSinkTest<gr::scheduler::Simple<multiThreaded>>("simple-multi"); };
    "a sink whose start failed records why"_test = [] { failedStartTest(); };
#ifdef GR4_RWT_TEST_BLOCKING_BACKOFF // gr4-incubator's scheduler; on core 83d15cc it does not stop, so the test hangs at the end
    "settings messages reach a running source and sink (BlockingBackoff, multi-threaded)"_test = [] { repeaterTest<gr::incubator::scheduler::BlockingBackoff<multiThreaded>>("backoff-multi"); };
#endif
};

int main() {}
