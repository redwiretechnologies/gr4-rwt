// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_RWTSink.cpp — RWTSink / RWTDualSink in GR4 graphs, against libiio v1's emulated AD9361
#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/rwt/RWTSink.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include "EmuFixture.hpp"
#include "TestHelpers.hpp"

#include <complex>
#include <print>

using namespace boost::ut;
using namespace gr::rwt;
namespace rd = gr::rwt::detail;
using gr::rwt::test::EmuFixture;
using CF32 = std::complex<float>;
using W    = std::vector<std::uint64_t>;
template<typename T>
using TagSource = gr::blocks::testing::TagSource<T, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;

namespace {
constexpr std::uint64_t E = rd::DEFAULT_ESCAPE;

// tag keys: GNU Radio's gr:tx_* and this module's rwt:tx_sample
const std::pmr::string kSob(tag_key::kTxSob);
const std::pmr::string kEob(tag_key::kTxEob);
const std::pmr::string kTime(tag_key::kTxTime);
const std::pmr::string kSample(tag_key::kTxSample);

gr::property_map emuSettings(const EmuFixture& emu, std::string mode, gr::property_map extra = {}) {
    gr::property_map m{
        {"uri", emu.uri()},
        {"mode", std::move(mode)},
        {"load_fpga", false},
        {"base_addr", std::uint64_t{0}},
        {"sample_rate", 2.5e6f}, // within the driver range without a FIR
        {"auto_filter", false},
        {"buffer_size", gr::Size_t{64}},
    };
    for (auto& [k, v] : extra) {
        m.insert_or_assign(k, v);
    }
    return m;
}

template<typename T>
TagSource<T>& addSource(gr::Graph& graph, std::initializer_list<T> values, std::vector<gr::Tag> tags) {
    auto& src = graph.emplaceBlock<TagSource<T>>({{"n_samples_max", static_cast<gr::Size_t>(values.size())}, {"sample_rate", 2.5e6f}, {"mark_tag", false}});
    src.values = gr::Tensor<T>(gr::data_from, std::vector<T>(values));
    src._tags  = std::move(tags);
    return src;
}

void run(gr::Graph& graph) {
    test::Sched sched;
    expect(sched.exchange(std::move(graph)).has_value());
    expect(test::runWithWatchdog(sched).has_value());
}

std::string hex(const W& w) {
    std::string s;
    for (const auto x : w) {
        s += std::format("{:016x} ", x);
    }
    return s;
}
} // namespace

const boost::ut::suite<"RWTSink"> rwtSinkTests = [] {
    "basic instantiation"_test = [] {
        RWTSink<CF32> blk;
        blk.init(blk.progress);
        expect(eq(blk.attenuation1.value, 10.0));
        expect(blk.mute_on_stop.value);
    };

    if (!EmuFixture::available()) {
        std::println("libiio was built without the emu backend: skipping the emulator tests");
        return;
    }

    "local: gr:tx_sob + gr:tx_time ... gr:tx_eob is a timed burst"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        // 3 samples: an odd count, so the burst's last word is padded with a zero sample
        auto& src = addSource<CF32>(graph, {{0.25f, 0.5f}, {-0.25f, -0.5f}, {1.0f, -1.0f}},
            {gr::Tag{0UZ, {{kSob, true}, {kTime, std::uint64_t{2'000'000}}}}, // 2 ms = DAC sample 5000 at 2.5 MS/s
                gr::Tag{2UZ, {{kEob, true}}}});
        auto& snk = graph.emplaceBlock<RWTSink<CF32>>(emuSettings(emu, "local"));
        expect(graph.connect<"out", "in">(src, snk).has_value());
        run(graph);

        const W want = {
            E, rd::makeTagWord(rd::TAG_SOB, 0),
            E, rd::makeTagWord(rd::TAG_HOLD, 5000),
            rd::packSingle(8192, 16384, -8192, -16384),
            E, rd::makeTagWord(rd::TAG_EOB, 0), // precedes the word holding the burst's last sample
            rd::packSingle(32767, -32768, 0, 0),    // 1.0 saturates to 32767
        };
        const W got = EmuFixture::unwords(emu.txData());
        expect(got == want) << "got" << hex(got);
    };

    "local: gr-rwt's bare tx_sob / tx_time / tx_sample / tx_eob are read too"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        auto& src = addSource<CF32>(graph, {{0.25f, 0.5f}, {-0.25f, -0.5f}, {1.0f, -1.0f}},
            {gr::Tag{0UZ, {{"tx_sob", true}, {"tx_time", std::uint64_t{2'000'000}}}}, gr::Tag{1UZ, {{"tx_sample", std::uint64_t{7000}}}}, gr::Tag{2UZ, {{"tx_eob", true}}}});
        auto& snk = graph.emplaceBlock<RWTSink<CF32>>(emuSettings(emu, "local"));
        expect(graph.connect<"out", "in">(src, snk).has_value());
        run(graph);
        const W want = {
            E, rd::makeTagWord(rd::TAG_SOB, 0),
            E, rd::makeTagWord(rd::TAG_HOLD, 5000),
            rd::packSingle(8192, 16384, 0, 0),
            E, rd::makeTagWord(rd::TAG_HOLD, 7000),
            E, rd::makeTagWord(rd::TAG_EOB, 0),
            rd::packSingle(-8192, -16384, 32767, -32768),
        };
        const W got = EmuFixture::unwords(emu.txData());
        expect(got == want) << "got" << hex(got);
    };

    "local: rwt:tx_sample, escaped data, and a burst across chunks"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        const auto a   = static_cast<std::int16_t>(0xAAAA);
        // int16 ports: 2 items per sample; tags sit on item indices (sample 1 = item 2)
        auto& src = addSource<std::int16_t>(graph, {a, a, a, a, 7, 8, 9, 10},
            {gr::Tag{2UZ, {{kSample, std::uint64_t{123456}}}}, gr::Tag{6UZ, {{kEob, true}}}});
        auto& snk = graph.emplaceBlock<RWTSink<std::int16_t>>(emuSettings(emu, "local", {{"max_chunk_size", gr::Size_t{1}}}));
        expect(graph.connect<"out", "in">(src, snk).has_value());
        run(graph);

        // samples: (a,a) (a,a) (7,8) (9,10); HOLD on sample 1, EOB on sample 3
        const W want = {
            rd::packSingle(a, a, 0, 0), // HOLD starts a word of its own: sample 0 is padded out first
            E, rd::makeTagWord(rd::TAG_HOLD, 123456),
            rd::packSingle(a, a, 7, 8),
            E, rd::makeTagWord(rd::TAG_EOB, 0),
            rd::packSingle(9, 10, 0, 0),
        };
        const W got = EmuFixture::unwords(emu.txData());
        expect(got == want) << "got" << hex(got);
    };

    "local: a data word equal to the escape is sent escaped"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        const auto a   = static_cast<std::int16_t>(0xAAAA);
        auto&      src = addSource<std::int16_t>(graph, {a, a, a, a}, {gr::Tag{2UZ, {{kEob, true}}}});
        auto&      snk = graph.emplaceBlock<RWTSink<std::int16_t>>(emuSettings(emu, "local"));
        expect(graph.connect<"out", "in">(src, snk).has_value());
        run(graph);
        const W got = EmuFixture::unwords(emu.txData());
        expect(got == W{E, rd::makeTagWord(rd::TAG_EOB, 0), E, 0}) << "got" << hex(got);
    };

    "local: dual channel, the same tag on both ports is one marker"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        auto&      s1  = addSource<CF32>(graph, {{0.5f, 0.f}, {0.25f, 0.f}}, {gr::Tag{0UZ, {{kSob, true}}}, gr::Tag{1UZ, {{kEob, true}}}});
        auto&      s2  = addSource<CF32>(graph, {{0.f, 0.5f}, {0.f, 0.25f}}, {gr::Tag{0UZ, {{kSob, true}}}, gr::Tag{1UZ, {{kEob, true}}}});
        auto&      snk = graph.emplaceBlock<RWTDualSink<CF32>>(emuSettings(emu, "local"));
        expect(graph.connect<"out", "in#0">(s1, snk).has_value());
        expect(graph.connect<"out", "in#1">(s2, snk).has_value());
        run(graph);
        const W want = {
            E, rd::makeTagWord(rd::TAG_SOB, 0),
            rd::packDual(16384, 0, 0, 16384),
            E, rd::makeTagWord(rd::TAG_EOB, 0),
            rd::packDual(8192, 0, 0, 8192),
        };
        const W got = EmuFixture::unwords(emu.txData());
        expect(got == want) << "got" << hex(got);
    };

    "local: use_tags=false sends samples only"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        auto&      src = addSource<std::int16_t>(graph, {1, 2, 3, 4}, {gr::Tag{0UZ, {{kSob, true}}}});
        auto&      snk = graph.emplaceBlock<RWTSink<std::int16_t>>(emuSettings(emu, "local", {{"use_tags", false}}));
        expect(graph.connect<"out", "in">(src, snk).has_value());
        run(graph);
        const W got = EmuFixture::unwords(emu.txData());
        expect(got == W{rd::packSingle(1, 2, 3, 4)}) << "got" << hex(got);
    };

    "remote: standard AD9361 layout"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        auto&      src = addSource<std::int16_t>(graph, {100, -100, 200, -200, 300, -300}, {});
        auto&      snk = graph.emplaceBlock<RWTSink<std::int16_t>>(emuSettings(emu, "remote"));
        expect(graph.connect<"out", "in">(src, snk).has_value());
        run(graph);
        expect(emu.txData() == EmuFixture::int16s({100, -100, 200, -200, 300, -300})) << "the partial block is sent at stop";
    };
};

int main() {}
