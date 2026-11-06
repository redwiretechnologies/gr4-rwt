// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
// qa_Registry.cpp — the compiled block library registers every RWT block, creatable by name
#include <boost/ut.hpp>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

#include <gnuradio-4.0/GrGr4RwtRwtBlocks.hpp>

#include <algorithm>
#include <memory>
#include <string>

using namespace boost::ut;
namespace pmt = gr::pmt;

const boost::ut::suite<"block registry"> registryTests = [] {
    "the module registers all RWT blocks"_test = [] {
        // the shared library enters its blocks when it is loaded; init() only reports blocks it adds
        auto& registry = gr::globalBlockRegistry();
        std::ignore    = gr::blocklib::initGrGr4RwtRwtBlocks(registry);

        // keys use GR4's canonical type names
        for (const std::string name : {"gr::rwt::RWTSource", "gr::rwt::RWTDualSource", "gr::rwt::RWTSink", "gr::rwt::RWTDualSink"}) {
            for (const std::string type : {"complex<float32>", "int16"}) {
                const std::string key   = name + "<" + type + ">";
                const auto        keys  = registry.keys();
                const bool        found = std::ranges::any_of(keys, [&key](const std::string& k) { return k == key; });
                expect(found) << key;
            }
        }
    };

    "a graph as gr-studio writes it loads: integers unquoted, no unique_name or chunk sizes"_test = [] {
        auto& registry = gr::globalBlockRegistry();
        std::ignore    = gr::blocklib::initGrGr4RwtRwtBlocks(registry);
        // gr-studio's inline grc: every parameter it has a value for, numbers without a type tag
        constexpr std::string_view grc = R"(blocks:
  - id: gr::rwt::RWTSource<complex<float32>>
    parameters:
      name: rx
      api_transceiver: 0
      api_url: none
      channel: 0
      frequency: 915000000
      sample_rate: 1000000
  - id: gr::rwt::RWTSink<complex<float32>>
    parameters:
      name: tx
      api_transceiver: 0
      api_url: none
      channel: 1
      frequency: 416000000
      sample_rate: 1000000
      attenuation1: 10
connections:
  - [rx, out, tx, in]
)";
        bool loaded = false;
        try {
            auto graph = gr::loadGrc(gr::globalPluginLoader(), grc);
            loaded     = graph->blocks().size() == 2UZ;
        } catch (const std::exception& e) {
            expect(false) << e.what();
        }
        expect(loaded) << "both blocks loaded";
    };

    "a block created by name takes its settings"_test = [] {
        auto& registry = gr::globalBlockRegistry();
        std::ignore    = gr::blocklib::initGrGr4RwtRwtBlocks(registry);
        auto block     = registry.create("gr::rwt::RWTSource<complex<float32>>", {{"sample_rate", 2.5e6f}, {"frequency", 915e6}, {"gain_mode", std::string("slow_attack")}});
        expect(block != nullptr);
        if (block) {
            // the parameters are applied when the block is initialised, as a scheduler does
            block->init(std::make_shared<gr::Sequence>(), gr::thread_pool::kDefaultIoPoolId);
            const auto settings = block->settings().get();
            const auto value    = [&settings](std::string_view key) {
                const auto it = settings.find(key);
                return it == settings.end() ? pmt::Value{} : it->second;
            };
            expect(value("sample_rate").value_or<float>(0.f) == 2.5e6f);
            expect(value("frequency").value_or<double>(0.0) == 915e6);
            expect(value("gain_mode").value_or(std::string{}) == std::string("slow_attack"));
        }
    };
};

int main() {}
