// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_RWTSource.cpp — RWTSource / RWTDualSource in GR4 graphs, against libiio v1's emulated AD9361
#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/rwt/RWTSource.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include "EmuFixture.hpp"
#include "TestHelpers.hpp"

#include <complex>
#include <print>

using namespace boost::ut;
using namespace gr::rwt;
namespace rd = gr::rwt::detail;
using gr::rwt::test::EmuFixture;
using gr::rwt::test::findTag;
using gr::rwt::test::tagValue;
using CF32 = std::complex<float>;
template<typename T>
using TagSink = gr::blocks::testing::TagSink<T, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;

namespace {
constexpr std::uint64_t E = rd::DEFAULT_ESCAPE;

// emulator settings: no fpgaloader, no /dev/mem, AD9361 rate written directly (the emulator keeps
// the RX and TX rate attributes apart, where the real driver shares them)
gr::property_map emuSettings(const EmuFixture& emu, std::string mode, gr::property_map extra = {}) {
    gr::property_map m{
        {"uri", emu.uri()},
        {"mode", std::move(mode)},
        {"load_fpga", false},
        {"base_addr", std::uint64_t{0}},
        {"sample_rate", 2.5e6f}, // within the driver range without a FIR (2.083 .. 61.44 MS/s)
        {"auto_filter", false},
    };
    for (auto& [k, v] : extra) {
        m.insert_or_assign(k, v);
    }
    return m;
}

// the keys GNU Radio's tag constants are written under by this core: gr:trigger_time from c145ebc, trigger_time before
const std::string kTrigTime(rd::writtenKey(gr::tag::TRIGGER_TIME));
const std::string kTrigName(rd::writtenKey(gr::tag::TRIGGER_NAME));
const std::string kRate(rd::writtenKey(gr::tag::SAMPLE_RATE));
const std::string kFreq(rd::writtenKey(gr::tag::FREQUENCY));
const std::string kOverflowTag(rd::writtenKey(gr::tag::RX_OVERFLOW));

CF32 scaled(std::int16_t i, std::int16_t q) { return {static_cast<float>(i) / 2048.f, static_cast<float>(q) / 2048.f}; }
} // namespace

const boost::ut::suite<"RWTSource"> rwtSourceTests = [] {
    "basic instantiation"_test = [] {
        RWTSource<CF32> blk;
        blk.init(blk.progress);
        expect(eq(blk.sample_rate.value, 5e6f));
        expect(eq(blk.gain_mode.value, std::string("manual")));
    };

    if (!EmuFixture::available()) {
        std::println("libiio was built without the emu backend: skipping the emulator tests");
        return;
    }

    "local: FPGA time, overflow and PPS tags become GR4 tags"_test = [] {
        EmuFixture emu;
        // one pass of the file is one block: 28 frames x 4 bytes = 14 words = 12 samples
        emu.setRxData(EmuFixture::words({
            E, rd::makeTagWord(rd::TAG_TIME, 1000), // samples 0.. start at FPGA index 1000
            rd::packSingle(1, 2, 3, 4), rd::packSingle(5, 6, 7, 8),
            E, 0,                                            // a data word equal to the escape: two samples of 0xAAAA
            rd::packSingle(9, 10, 11, 12),
            E, rd::makeTagWord(rd::TAG_OVERFLOW, 1), // before sample 8
            rd::packSingle(13, 14, 15, 16),
            E, rd::makeTagWord(rd::TAG_PPS, 0, true), rd::makeTagWord(rd::TAG_TIME, 5000), // chained, before sample 10
            rd::packSingle(17, 18, 19, 20),
        }));

        gr::Graph graph;
        auto&     src = graph.emplaceBlock<RWTSource<CF32>>(emuSettings(emu, "local", {{"buffer_size", gr::Size_t{28}}, {"n_samples_max", gr::Size_t{12}}, {"frequency", 915e6}}));
        auto&     snk = graph.emplaceBlock<TagSink<CF32>>({{"log_tags", true}, {"log_samples", true}});
        expect(graph.connect<"out", "in">(src, snk).has_value());

        test::Sched sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(test::runWithWatchdog(sched).has_value());

        expect(eq(snk._samples.size(), 12UZ)) << "one file pass";
        if (snk._samples.size() == 12UZ) {
            const auto a = static_cast<std::int16_t>(0xAAAA);
            const std::vector<CF32> want = {scaled(1, 2), scaled(3, 4), scaled(5, 6), scaled(7, 8), scaled(a, a), scaled(a, a), scaled(9, 10), scaled(11, 12), scaled(13, 14), scaled(15, 16), scaled(17, 18), scaled(19, 20)};
            for (std::size_t i = 0; i < want.size(); ++i) {
                expect(snk._samples[i] == want[i]) << "sample" << i;
            }
        }

        // sample 0: stream metadata and the first FPGA time
        const gr::Tag* t0 = findTag(snk._tags, 0UZ, kTrigTime);
        expect(t0 != nullptr) << "time tag at sample 0";
        expect(tagValue<std::uint64_t>(t0, kTrigTime) == std::optional<std::uint64_t>(400'000U)) << "1000 samples at 2.5 MS/s = 400 us";
        expect(tagValue<std::uint64_t>(t0, tag_key::kSampleIndex) == std::optional<std::uint64_t>(1000U));
        expect(tagValue<std::string>(t0, kTrigName) == std::optional<std::string>("RWT_TIME"));
        const gr::Tag* m0 = findTag(snk._tags, 0UZ, kRate);
        expect(tagValue<float>(m0, kRate) == std::optional<float>(2.5e6f));
        expect(tagValue<double>(m0, kFreq) == std::optional<double>(915e6));

        // sample 8: overflow
        const gr::Tag* t8 = findTag(snk._tags, 8UZ, kOverflowTag);
        expect(t8 != nullptr) << "overflow tag at sample 8";
        expect(tagValue<bool>(t8, kOverflowTag) == std::optional<bool>(true));
        expect(tagValue<std::uint64_t>(t8, tag_key::kOverflow) == std::optional<std::uint64_t>(1U));
        expect(findTag(snk._tags, 8UZ, kTrigTime) == nullptr);

        // sample 10: PPS chained with a new time
        const gr::Tag* t10 = findTag(snk._tags, 10UZ, kTrigTime);
        expect(t10 != nullptr) << "time tag at sample 10";
        expect(tagValue<std::uint64_t>(t10, kTrigTime) == std::optional<std::uint64_t>(2'000'000U)) << "5000 samples at 2.5 MS/s";
        expect(tagValue<std::uint64_t>(t10, tag_key::kPps) == std::optional<std::uint64_t>(0U));
        expect(tagValue<std::string>(t10, kTrigName) == std::optional<std::string>("RWT_TIME"));

        // the key spellings: gr:/rwt: domains on gnuradio4-core from c145ebc
        if (kTrigTime.starts_with("gr:")) {
            expect(eq(kTrigTime, std::string("gr:trigger_time")));
            expect(eq(kOverflowTag, std::string("gr:rx_overflow")));
            expect(findTag(snk._tags, 0UZ, "gr:sample_rate") != nullptr);
            expect(findTag(snk._tags, 0UZ, "rwt:sample_index") != nullptr);
            expect(findTag(snk._tags, 0UZ, "trigger_time") == nullptr) << "no bare spelling";
        }
        expect(test::tagIndices(snk._tags, kTrigTime) == std::vector<std::size_t>{0UZ, 10UZ});
        expect(test::tagIndices(snk._tags, kOverflowTag) == std::vector<std::size_t>{8UZ});
    };

    "local: tags split across DMA blocks and chunks"_test = [] {
        EmuFixture emu;
        // 4-frame blocks (2 words): the tag word of the escape at the end of block 0 is in block 1
        emu.setRxData(EmuFixture::words({rd::packSingle(1, 1, 2, 2), E, rd::makeTagWord(rd::TAG_TIME, 42), rd::packSingle(3, 3, 4, 4)}));
        gr::Graph graph;
        auto&     src = graph.emplaceBlock<RWTSource<CF32>>(emuSettings(emu, "local", {{"buffer_size", gr::Size_t{4}}, {"max_chunk_size", gr::Size_t{3}}, {"n_samples_max", gr::Size_t{8}}}));
        auto&     snk = graph.emplaceBlock<TagSink<CF32>>();
        expect(graph.connect<"out", "in">(src, snk).has_value());
        test::Sched sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(test::runWithWatchdog(sched).has_value());

        expect(eq(snk._samples.size(), 8UZ));
        // the file loops: the time tag precedes samples 2 and 6 (4 samples per pass)
        expect(test::tagIndices(snk._tags, kTrigTime) == std::vector<std::size_t>{2UZ, 6UZ});
        expect(tagValue<std::uint64_t>(findTag(snk._tags, 2UZ, kTrigTime), tag_key::kSampleIndex) == std::optional<std::uint64_t>(42U));
        if (snk._samples.size() == 8UZ) {
            expect(snk._samples[1] == scaled(2, 2) && snk._samples[2] == scaled(3, 3) && snk._samples[7] == scaled(4, 4));
        }
    };

    "local: dual channel, raw int16 I/Q"_test = [] {
        EmuFixture emu;
        // both channels: one word per sample, RX2 in the upper half
        emu.setRxData(EmuFixture::words({rd::packDual(10, 11, 20, 21), rd::packDual(12, 13, 22, 23)}));
        gr::Graph graph;
        auto&     src  = graph.emplaceBlock<RWTDualSource<std::int16_t>>(emuSettings(emu, "local", {{"buffer_size", gr::Size_t{2}}, {"n_samples_max", gr::Size_t{4}}}));
        auto&     snk1 = graph.emplaceBlock<TagSink<std::int16_t>>();
        auto&     snk2 = graph.emplaceBlock<TagSink<std::int16_t>>();
        expect(graph.connect<"out#0", "in">(src, snk1).has_value());
        expect(graph.connect<"out#1", "in">(src, snk2).has_value());
        test::Sched sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(test::runWithWatchdog(sched).has_value());

        const std::vector<std::int16_t> rx1 = {10, 11, 12, 13, 10, 11, 12, 13};
        const std::vector<std::int16_t> rx2 = {20, 21, 22, 23, 20, 21, 22, 23};
        expect(std::vector<std::int16_t>(snk1._samples.begin(), snk1._samples.end()) == rx1) << "RX1 interleaved I,Q";
        expect(std::vector<std::int16_t>(snk2._samples.begin(), snk2._samples.end()) == rx2) << "RX2 interleaved I,Q";
        // the stream metadata reaches both ports, at item 0
        expect(findTag(snk1._tags, 0UZ, kRate) != nullptr && findTag(snk2._tags, 0UZ, kRate) != nullptr);
    };

    "remote: standard AD9361 layout, RX2 only"_test = [] {
        EmuFixture emu;
        // one enabled channel: the block holds RX2's I,Q pairs only
        emu.setRxData(EmuFixture::int16s({100, -100, 2047, -2048, 0, 1}));
        gr::Graph graph;
        auto&     src = graph.emplaceBlock<RWTSource<CF32>>(emuSettings(emu, "remote", {{"channel", gr::Size_t{1}}, {"buffer_size", gr::Size_t{3}}, {"n_samples_max", gr::Size_t{6}}}));
        auto&     snk = graph.emplaceBlock<TagSink<CF32>>();
        expect(graph.connect<"out", "in">(src, snk).has_value());
        test::Sched sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(test::runWithWatchdog(sched).has_value());

        const std::vector<CF32> want = {scaled(100, -100), scaled(2047, -2048), scaled(0, 1), scaled(100, -100), scaled(2047, -2048), scaled(0, 1)};
        expect(eq(snk._samples.size(), want.size()));
        if (snk._samples.size() == want.size()) {
            for (std::size_t i = 0; i < want.size(); ++i) {
                expect(snk._samples[i] == want[i]) << "sample" << i;
            }
        }
        expect(test::tagIndices(snk._tags, kTrigTime).empty()) << "no FPGA tags remotely";
        expect(findTag(snk._tags, 0UZ, kRate) != nullptr);
    };

    "an unavailable channel stops the graph with an error"_test = [] {
        EmuFixture emu;
        gr::Graph  graph;
        auto&      src = graph.emplaceBlock<RWTSource<CF32>>(emuSettings(emu, "remote", {{"channel", gr::Size_t{2}}}));
        auto&      snk = graph.emplaceBlock<TagSink<CF32>>();
        expect(graph.connect<"out", "in">(src, snk).has_value());
        test::Sched sched;
        expect(sched.exchange(std::move(graph)).has_value());
        std::ignore = test::runWithWatchdog(sched, std::chrono::seconds{5});
        expect(eq(snk._samples.size(), 0UZ));
    };
};

int main() {}
