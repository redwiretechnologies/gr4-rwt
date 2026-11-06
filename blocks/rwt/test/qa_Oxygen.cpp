// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_Oxygen.cpp — the device layer: gr-rwt wire format, rate planning, and AD9361 control on the emulator
#include <boost/ut.hpp>

#include <gnuradio-4.0/rwt/detail/BlockSupport.hpp>
#include <gnuradio-4.0/rwt/detail/Oxygen.hpp>
#include <gnuradio-4.0/rwt/detail/Wire.hpp>

#include "EmuFixture.hpp"

#include <print>

using namespace boost::ut;
using namespace gr::rwt::detail;
using gr::rwt::test::EmuFixture;

const boost::ut::suite<"wire format"> wireTests = [] {
    "packing matches gr-rwt's rwt_sink expressions"_test = [] {
        // gr-rwt lib/rwt_sink_impl.cc: single channel ptr0[0]<<48 | ptr0[1]<<32 | ptr0[2]<<16 | ptr0[3];
        // dual channel ch0 in the low half, ch1 in the high half
        expect(packSingle(1, 2, 3, 4) == 0x0001000200030004ULL);
        expect(packDual(1, 2, 3, 4) == 0x0003000400010002ULL);
        std::int16_t s[4];
        unpackSingle(packSingle(-2048, 2047, -1, 1234), s);
        expect(s[0] == -2048 && s[1] == 2047 && s[2] == -1 && s[3] == 1234);
        std::int16_t c0[2], c1[2];
        unpackDual(packDual(5, 6, 7, 8), c0, c1);
        expect(c0[0] == 5 && c0[1] == 6 && c1[0] == 7 && c1[1] == 8);
    };

    "decoder: literal escape, single and chained tags"_test = [] {
        StreamDecoder d(DEFAULT_ESCAPE, true);
        std::uint64_t data = 0, value = 0;
        std::uint8_t  type = 0;
        expect(d.feed(DEFAULT_ESCAPE, data, type, value) == StreamDecoder::CONSUMED);
        expect(d.feed(0, data, type, value) == StreamDecoder::DATA && data == DEFAULT_ESCAPE);
        expect(d.feed(DEFAULT_ESCAPE, data, type, value) == StreamDecoder::CONSUMED);
        expect(d.feed(makeTagWord(TAG_PPS, 7, true), data, type, value) == StreamDecoder::TAG && type == TAG_PPS && value == 7U);
        expect(d.feed(makeTagWord(TAG_TIME, 9), data, type, value) == StreamDecoder::TAG && type == TAG_TIME && value == 9U);
        expect(d.feed(0x1234, data, type, value) == StreamDecoder::DATA && data == 0x1234U);
    };
};

const boost::ut::suite<"rates and times"> rateTests = [] {
    "RX rate plan: the CIC takes over below the AD9361 floor (gr-rwt set_samplerate)"_test = [] {
        auto p = planRxRate(5e6, true);
        expect(p.valid && p.pow == 0 && p.adRate == 5e6);
        p = planRxRate(100e3, true);
        expect(p.valid && p.pow == 1 && p.adRate == 1e6) << "100 kS/s = 1 MS/s / 10";
        p = planRxRate(6.0, true);
        expect(p.valid && p.pow == 5) << "down to 6 S/s";
        p = planRxRate(1.0, true);
        expect(!p.valid);
        p = planRxRate(100e3, false);
        expect(!p.valid) << "no CIC without the FPGA";
        p = planRxRate(70e6, true);
        expect(!p.valid);
    };

    "time conversions (gr-rwt time_helper)"_test = [] {
        expect(gr::rwt::detail::parseTime("2,500000000", 1e6) == std::optional<std::uint64_t>(2'500'000U));
        expect(gr::rwt::detail::parseTime("3", 1e6) == std::optional<std::uint64_t>(3'000'000U));
        expect(!gr::rwt::detail::parseTime("x", 1e6).has_value());
        expect(gr::rwt::detail::nsToSamples(2'000'000U, 1e6) == 2000U);
        expect(gr::rwt::detail::samplesToNs(1500U, 1e6) == 1'500'000U);
        // far beyond 2^53 ns: no precision lost
        expect(gr::rwt::detail::samplesToNs(gr::rwt::detail::nsToSamples(1'800'000'000'000'000'000ULL, 61.44e6), 61.44e6) == 1'800'000'000'000'000'000ULL);
    };
};

const boost::ut::suite<"AD9361 control"> controlTests = [] {
    if (!EmuFixture::available()) {
        std::println("libiio was built without the emu backend: skipping the emulator tests");
        return;
    }

    "open in remote mode and drive the AD9361 attributes"_test = [] {
        EmuFixture emu;
        Oxygen     dev;
        dev.open(OxygenConfig{emu.uri(), Mode::Remote, "default", false, false, 0U, true});
        expect(dev.isOpen());
        expect(eq(dev.numChannels(Direction::RX), 2UZ));
        expect(eq(dev.numChannels(Direction::TX), 2UZ));
        expect(!dev.hasRegisters());
        const auto& phy = dev.phy();

        dev.setLo(Direction::RX, 915e6);
        dev.setLo(Direction::TX, 433.92e6);
        expect(phy.read("out_altvoltage0_RX_LO_frequency") == std::optional<std::string>("915000000"));
        expect(phy.read("out_altvoltage1_TX_LO_frequency") == std::optional<std::string>("433920000"));

        dev.setRxGainMode(1, "slow_attack");
        dev.setRxGainMode(0, "manual");
        dev.setRxGain(0, 30);
        expect(phy.read("in_voltage0_gain_control_mode") == std::optional<std::string>("manual"));
        expect(phy.read("in_voltage1_gain_control_mode") == std::optional<std::string>("slow_attack"));
        expect(phy.readDouble("in_voltage0_hardwaregain") == std::optional<double>(30.0));

        dev.setTxAttenuation(1, 9.8); // rounds to 0.25 dB steps
        expect(phy.read("out_voltage1_hardwaregain").value_or("").starts_with("-9.75"));
        dev.setTxAttenuation(0, 200.0); // clamps
        expect(phy.read("out_voltage0_hardwaregain").value_or("").starts_with("-89.75"));

        dev.setPort(Direction::RX, 1, "B_BALANCED");
        dev.setPort(Direction::TX, 0, "B");
        expect(phy.read("in_voltage1_rf_port_select") == std::optional<std::string>("B_BALANCED"));
        expect(phy.read("out_voltage0_rf_port_select") == std::optional<std::string>("B"));

        dev.setBandwidth(Direction::RX, 4e6);
        expect(phy.readDouble("in_voltage_rf_bandwidth") == std::optional<double>(4e6));

        dev.setRxTracking(false, true, false);
        expect(phy.read("in_voltage_quadrature_tracking_en") == std::optional<std::string>("0"));
        expect(phy.read("in_voltage_rf_dc_offset_tracking_en") == std::optional<std::string>("1"));
        expect(phy.read("in_voltage_bb_dc_offset_tracking_en") == std::optional<std::string>("0"));

        dev.setBasebandRate(Direction::RX, 2.5e6, false);
        expect(eq(dev.basebandRate(Direction::RX), 2.5e6));

        // gr-rwt "extra": aliases (tx_gain names both channels) and raw attribute names
        dev.applyExtra("rx_gain2=12,tx_gain=-20,in_voltage0_rf_port_select=C_BALANCED,bad,=x");
        expect(phy.readDouble("in_voltage1_hardwaregain") == std::optional<double>(12.0));
        expect(phy.read("out_voltage0_hardwaregain").value_or("").starts_with("-20"));
        expect(phy.read("out_voltage1_hardwaregain").value_or("").starts_with("-20"));
        expect(phy.read("in_voltage0_rf_port_select") == std::optional<std::string>("C_BALANCED"));

        // without registers the FPGA controls are no-ops
        dev.setTagConfig(Direction::RX, DEFAULT_ESCAPE, true);
        dev.setSampleIndex(5, false);
        expect(eq(dev.sampleIndex(Direction::RX), std::uint64_t{0}));
        dev.close();
        expect(!dev.isOpen());
    };

    "unknown attributes are reported, not fatal"_test = [] {
        EmuFixture  emu;
        Oxygen      dev;
        std::string lastWarning;
        dev.warn = [&lastWarning](const std::string& m) { lastWarning = m; };
        dev.open(OxygenConfig{emu.uri(), Mode::Remote, "default", false, false, 0U, true});
        expect(!dev.set("no_such_attribute", "1"));
        expect(lastWarning.find("no_such_attribute") != std::string::npos);
    };

    "a missing fpgaloader fails local mode with the reason"_test = [] {
        EmuFixture emu;
        Oxygen     dev;
        bool       threw = false;
        std::string what;
        try {
            dev.open(OxygenConfig{emu.uri(), Mode::Local, "default", true, false, 0U, true});
        } catch (const std::exception& e) {
            threw = true;
            what  = e.what();
        }
        expect(threw);
        expect(what.find("fpgaloader") != std::string::npos) << what;
    };
};

int main() {}
