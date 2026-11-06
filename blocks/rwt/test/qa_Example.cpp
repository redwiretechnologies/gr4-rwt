// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_Example.cpp — runs examples/oxygen_repeater.grc with GR4's rungraph, pointed at the emulator
#include <boost/ut.hpp>

#include "EmuFixture.hpp"

#include <cstdlib>
#include <format>
#include <fstream>
#include <print>
#include <sstream>
#include <string>

#ifndef GR4_RWT_RUNGRAPH
#error "GR4_RWT_RUNGRAPH must name the rungraph executable"
#endif
#ifndef GR4_RWT_EXAMPLE
#error "GR4_RWT_EXAMPLE must name examples/oxygen_repeater.grc"
#endif
#ifndef GR4_RWT_PLUGIN_DIR
#error "GR4_RWT_PLUGIN_DIR must name the directory of the block library"
#endif

using namespace boost::ut;
using gr::rwt::test::EmuFixture;

namespace {
std::string readFile(const std::string& path) {
    std::ifstream     in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void replaceOnce(std::string& s, const std::string& what, const std::string& with) {
    if (const auto pos = s.find(what); pos != std::string::npos) {
        s.replace(pos, what.size(), with);
    }
}
} // namespace

const boost::ut::suite<"example graph"> exampleTests = [] {
    if (!EmuFixture::available()) {
        std::println("libiio was built without the emu backend: skipping");
        return;
    }

    "oxygen_repeater.grc runs under rungraph and repeats RX to TX"_test = [] {
        EmuFixture                emu;
        std::vector<std::int16_t> rx;
        for (int i = 0; i < 64; ++i) {
            rx.push_back(static_cast<std::int16_t>(i));
            rx.push_back(static_cast<std::int16_t>(-i));
        }
        emu.setRxData(EmuFixture::int16s(rx));

        // the example as shipped, with the device pointed at the emulator
        std::string graph = readFile(GR4_RWT_EXAMPLE);
        expect(!graph.empty()) << "example file";
        const std::string device = std::format("      uri: \"{}\"\n      mode: remote\n      auto_filter: !!bool false\n", emu.uri());
        replaceOnce(graph, "      name: rx\n", "      name: rx\n" + device + "      n_samples_max: !!uint32 64\n");
        replaceOnce(graph, "      name: tx\n", "      name: tx\n" + device);
        const std::string graphPath = emu.uri().substr(4, emu.uri().rfind('/') - 4) + "/repeater.grc";
        std::ofstream(graphPath) << graph;

        const std::string cmd = std::format("'{}' --graph '{}' --plugin-dir '{}'", GR4_RWT_RUNGRAPH, graphPath, GR4_RWT_PLUGIN_DIR);
        expect(eq(std::system(cmd.c_str()), 0)) << cmd;

        // RX is 12-bit LSB aligned, TX MSB aligned: a sample passes through x16, as in gr-rwt
        std::vector<std::int16_t> tx;
        for (int i = 0; i < 64; ++i) {
            tx.push_back(static_cast<std::int16_t>(16 * i));
            tx.push_back(static_cast<std::int16_t>(-16 * i));
        }
        expect(emu.txData() == EmuFixture::int16s(tx)) << "all 64 samples reach TX";
    };
};

int main() {}
