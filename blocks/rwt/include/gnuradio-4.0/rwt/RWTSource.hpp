// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_SOURCE_HPP
#define GNURADIO_RWT_SOURCE_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <gnuradio-4.0/rwt/detail/BlockSupport.hpp>
#include <gnuradio-4.0/rwt/detail/RWTRadio.hpp>
#include <gnuradio-4.0/rwt/detail/HardwareManager.hpp>
#include <gnuradio-4.0/rwt/detail/Streams.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <print>
#include <string>
#include <thread>
#include <vector>

namespace gr::rwt {

GR_REGISTER_BLOCK("gr::rwt::RWTSource", gr::rwt::RWTSingleSource, [T], [ std::complex<float>, std::int16_t ])
GR_REGISTER_BLOCK("gr::rwt::RWTDualSource", gr::rwt::RWTDualSource, [T], [ std::complex<float>, std::int16_t ])

template<typename T, std::size_t nPorts = 1UZ>
requires((std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::int16_t>) && (nPorts == 1UZ || nPorts == 2UZ))
struct RWTSource : Block<RWTSource<T, nPorts>> {
    using Description = Doc<R"(RWT radio source (AD9361 receiver; the Oxygen and other Red Wire Technologies SDRs), the GNU Radio 4 counterpart of gr-rwt's rwt_source.

RWTSource receives one channel (`channel` picks RX1 or RX2); RWTDualSource receives both, port 0 = RX1, port 1 = RX2.
std::complex<float> ports are scaled so the AD9361's 12-bit full scale is 1.0; std::int16_t ports carry raw interleaved I,Q.

mode "local" (default when uri is empty) runs on the radio itself with the "default" FPGA personality, as gr-rwt does: it loads the
personality with fpgaloader, maps the FPGA registers from /dev/mem (root), and decodes the FPGA's in-band tags:
  FPGA time     -> gr:trigger_name "RWT_TIME", gr:trigger_time [ns] = sample index / sample_rate, rwt:sample_index
  FPGA PPS      -> gr:trigger_name "RWT_PPS", rwt:pps (and gr:trigger_time while the time is known)
  FPGA overflow -> gr:rx_overflow = true, rwt:overflow; trigger times resume at the next FPGA time tag
Below 600 kS/s the AD9361 runs at sample_rate x 10^n and the FPGA CIC decimates by 10^n (n <= 5), down to 6 S/s.
mode "remote" streams from an RWT radio running the "griio" personality over libiio (uri "ip:<host>"), without FPGA tags.

The first samples, and the first after a sample_rate or frequency change, carry gr:sample_rate and gr:frequency tags.
(Before gnuradio4-core c145ebc the gr: keys are written without the domain, as that core spells them.))">;

    using TPortType = std::conditional_t<nPorts == 1UZ, PortOut<T>, std::array<PortOut<T>, nPorts>>;

    TPortType out;

    // device
    Annotated<std::string, "uri", Visible, Doc<"libiio context URI; empty = this radio (local:)">>                    uri;
    Annotated<std::string, "mode", Doc<"local (default FPGA personality, tags) or remote (griio); empty = by uri">>       mode;
    // RF (gr-rwt rwt_source parameters)
    Annotated<float, "sample_rate", Unit<"Hz">, Visible, Doc<"RX sample rate; below 600 kS/s the FPGA CIC decimates">>    sample_rate = 5'000'000.f;
    Annotated<double, "frequency", Unit<"Hz">, Visible, Doc<"RX LO frequency (shared by both channels)">>                  frequency   = 2.4e9;
    Annotated<double, "bandwidth", Unit<"Hz">, Visible, Doc<"RX RF bandwidth">>                                            bandwidth   = 40e6;
    Annotated<gr::Size_t, "channel", Visible, Doc<"RX channel of the single-channel source: 0 = RX1, 1 = RX2">>            channel     = 0U;
    Annotated<std::string, "rf_port1", Doc<"RX1 port: A_BALANCED, B_BALANCED, C_BALANCED, A_N, A_P, ...">>                 rf_port1    = std::string("A_BALANCED");
    Annotated<std::string, "rf_port2", Doc<"RX2 port">>                                                                    rf_port2    = std::string("A_BALANCED");
    Annotated<std::string, "gain_mode", Visible, Doc<"manual, slow_attack, fast_attack or hybrid">>                        gain_mode   = std::string("manual");
    Annotated<double, "gain", Unit<"dB">, Visible, Doc<"manual RX gain of the channels in use">>                           gain        = 64.0;
    Annotated<bool, "quadrature", Doc<"AD9361 quadrature (IQ imbalance) tracking">>                                        quadrature  = true;
    Annotated<bool, "rfdc", Doc<"AD9361 RF DC offset tracking">>                                                           rfdc        = true;
    Annotated<bool, "bbdc", Doc<"AD9361 baseband DC offset tracking">>                                                     bbdc        = true;
    Annotated<std::string, "filter", Doc<"AD9361 .ftr FIR file (RX and TX); used when auto_filter is off">>                filter;
    Annotated<bool, "auto_filter", Doc<"load a FIR matching the rate (libad9361)">>                                        auto_filter = true;
    Annotated<std::string, "extra", Doc<"gr-rwt extra settings: comma-separated key=value (gr-rwt keys or AD9361 attributes)">> extra;
    // RWT Hardware Manager (http://<radio>/docs): connection_control sets the RX frequency with the RF path's filters, and switches the RX paths on at start and off at stop
    Annotated<std::string, "api_url", Doc<"RWT Hardware Manager base URL; empty = this radio (local) or the uri's host (ip:), none = off">> api_url;
    // an integer, not a string: gr-studio writes it unquoted, and GR4 refuses an integer for a string setting
    Annotated<gr::Size_t, "api_transceiver", Doc<"connection_control transceiver: N = TransceiverN (0 on an Oxygen)">>       api_transceiver = 0U;
    // FPGA ("default" personality, local mode)
    Annotated<bool, "use_tags", Doc<"decode the FPGA's in-band tags (time, PPS, overflow)">>                               use_tags          = true;
    Annotated<std::uint64_t, "escape", Doc<"escape word introducing in-band tags">>                                        escape            = detail::DEFAULT_ESCAPE;
    Annotated<bool, "use_ext_pps", Doc<"use the external PPS input instead of the internal one">>                          use_ext_pps       = false;
    Annotated<bool, "correction_enable", Doc<"CIC gain correction">>                                                       correction_enable = true;
    Annotated<bool, "bypass_enable", Doc<"hold the CIC in reset">>                                                         bypass_enable     = false;
    Annotated<std::uint32_t, "decimation_arbitrary", Doc<"raw CIC arbitrary decimation register (gr-rwt)">>                decimation_arbitrary = 0U;
    Annotated<std::string, "time_now", Doc<"set the FPGA time now: \"secs,nsecs\"">>                                       time_now;
    Annotated<std::string, "time_next_pps", Doc<"set the FPGA time at the next PPS: \"secs,nsecs\"">>                      time_next_pps;
    Annotated<std::string, "personality", Doc<"FPGA image for fpgaloader">>                                                personality  = std::string("default");
    Annotated<bool, "load_fpga", Doc<"run fpgaloader at start (local mode)">>                                              load_fpga    = true;
    Annotated<bool, "force_reload", Doc<"pass -f to fpgaloader">>                                                          force_reload = false;
    Annotated<std::uint64_t, "base_addr", Doc<"physical address of the FPGA user registers; 0 = do not map them">>        base_addr    = detail::DEFAULT_REG_BASE_ADDR;
    Annotated<bool, "check_blkid", Doc<"refuse an FPGA image that is not the default personality">>                        check_blkid  = true;
    // streaming
    Annotated<gr::Size_t, "buffer_size", Doc<"samples per IIO block">>                                                    buffer_size    = 32768U;
    Annotated<gr::Size_t, "num_buffers", Doc<"IIO blocks kept in flight">>                                                 num_buffers    = 4U;
    Annotated<gr::Size_t, "max_chunk_size", Doc<"max samples published at once">>                                          max_chunk_size = 8192U;
    Annotated<std::uint32_t, "timeout_us", Unit<"us">, Doc<"how long one read waits for the hardware">>                    timeout_us     = 100'000U;
    Annotated<gr::Size_t, "n_samples_max", Doc<"stop after this many samples (0 = run until stopped)">>                    n_samples_max  = 0U;

    GR_MAKE_REFLECTABLE(RWTSource, out, uri, mode, sample_rate, frequency, bandwidth, channel, rf_port1, rf_port2, gain_mode, gain, quadrature, rfdc, bbdc, filter, auto_filter, extra, api_url, api_transceiver, use_tags, escape, use_ext_pps, correction_enable, bypass_enable, decimation_arbitrary, time_now, time_next_pps, personality, load_fpga, force_reload, base_addr, check_blkid, buffer_size, num_buffers, max_chunk_size, timeout_us, n_samples_max);

    detail::RWTRadio                    _dev;
    std::string                       _startError; // why the last start() failed; empty while it holds the radio
    detail::HardwareManager                  _hwm;        // the RWT Hardware Manager, when api_url resolves to one
    detail::PathEnables               _rxOn{detail::Direction::RX}; // RX paths this block switched on through the RWT Hardware Manager
    std::unique_ptr<detail::RxStream> _rx;
    std::vector<std::size_t>          _channels;
    double                            _rate   = 0.0; // effective RX rate (after the CIC)
    int                               _cicPow = 0;
    std::uint64_t                     _streamPos = 0U; // samples published since start

    // FPGA time: the stream position whose FPGA sample index is _timeBaseIndex
    bool          _timeValid      = false;
    std::uint64_t _timeBasePos    = 0U;
    std::uint64_t _timeBaseIndex  = 0U;

    bool              _ioThreadDone = true;
    std::atomic<bool> _ioThreadStarted{false};
    std::atomic<bool> _ioStopRequested{false};
    std::atomic<bool> _metaDirty{true};

    // a teardown that never asked the io thread to stop (a scheduler ending in ERROR) would wait forever
    struct IoThreadGuard {
        RWTSource* self;
        ~IoThreadGuard() {
            if (lifecycle::isActive(self->state())) {
                self->requestStop();
            }
            gr::atomic_ref(self->_ioThreadDone).wait(false);
            self->setReceive(false); // a block destroyed without stop() leaves no RX path on
        }
    };
    IoThreadGuard _ioGuard{this};

    void settingsChanged(const property_map& oldSettings, property_map& newSettings, property_map& forwardSettings) {
        if (!_dev.isOpen()) {
            // before start() these are the initial settings, which start() applies; after a failed
            // start nothing holds the radio, and a change would otherwise vanish without a word
            if (!_startError.empty()) {
                std::println(stderr, "{}: settings change not applied, the block holds no radio ({}): {}", this->name.value, _startError, detail::describeSettings(newSettings));
            } else if (detail::traceEnabled()) {
                std::println(stderr, "{}: settings staged for start: {}", this->name.value, detail::describeSettings(newSettings));
            }
            return;
        }
        _rejected.clear();
        applySettings(&newSettings);
        // A refused value is reported on stderr and the setting goes back to the value in force. An
        // emitErrorMessage() here would stop the whole graph: GR4 turns an error a block reports into a
        // scheduler error (as gr4-uhd's notes say), so one bad value from gr-studio ends the run.
        for (const auto& [key, why] : _rejected) {
            const auto prev = oldSettings.find(std::pmr::string(key));
            std::println(stderr, "{}: settings change refused: {}; keeping {}", this->name.value, why, prev != oldSettings.end() ? detail::describeValue(prev->second) : std::string("the value in force"));
            if (prev != oldSettings.end()) {
                restoreSetting(key, prev->second);
            }
            newSettings.erase(std::pmr::string(key));
            forwardSettings.erase(std::pmr::string(key)); // nor is it passed downstream as a tag
        }
        if (detail::traceEnabled()) {
            traceApplied(newSettings);
        }
    }

    /// the RX frequency through the RWT Hardware Manager's connection_control, which also sets the RF path's
    /// filters, for each RX path the block uses; through libiio (AD9361 LO alone) without the RWT Hardware Manager or if it fails
    void applyFrequency() {
        if (_hwm.enabled()) {
            bool ok = true;
            for (const std::size_t ch : _channels) {
                if (const std::string err = _hwm.setFrequency(detail::Direction::RX, ch, frequency); !err.empty()) {
                    _dev.warn(err);
                    ok = false;
                }
            }
            if (ok) {
                return;
            }
            _dev.warn("the RWT Hardware Manager did not take the RX frequency; setting the AD9361 LO through libiio");
        }
        _dev.setLo(detail::Direction::RX, frequency);
    }

    /// RWT Hardware Manager receive_enable for the block's RX paths: on at start, off when the block
    /// stops, fails to start, or is destroyed. Only paths this block switched on are switched off.
    void setReceive(bool on) {
        if (on) {
            _rxOn.on(_hwm, _channels, [this](const std::string& msg) { _dev.warn(msg); });
        } else {
            _rxOn.off(_hwm, [this](const std::string& msg) { std::println(stderr, "{}: {}", this->name.value, msg); });
        }
    }

    /// a setting the radio cannot take: start() fails with it, a live change is refused (settingsChanged)
    std::vector<std::pair<std::string, std::string>> _rejected;
    void reject(std::string key, std::string why) { _rejected.emplace_back(std::move(key), std::move(why)); }

    /// puts a refused setting back to the value in force, as gr4-uhd's refuseRunningWrites() does
    void restoreSetting(std::string_view key, const pmt::Value& value) {
        if (key == "sample_rate") {
            if (const auto v = detail::asNumber(value)) {
                sample_rate = static_cast<float>(*v);
            }
        } else if (key == "time_now") {
            time_now = value.value_or(std::string{});
        } else if (key == "time_next_pps") {
            time_next_pps = value.value_or(std::string{});
        }
    }

    /// a stream that failed ends the run: on stderr, where the control plane's console shows it, and
    /// as a GR4 error, which stops the graph
    void reportStreamError(std::string_view where, const std::string& err) {
        std::println(stderr, "{}: {}", this->name.value, err);
        this->emitErrorMessage(where, err);
    }

    /// GR4_RWT_TRACE: what a settings change asked for and what the AD9361 reports back
    void traceApplied(const property_map& changed) const {
        static constexpr std::pair<std::string_view, std::string_view> kReadBack[] = {{"frequency", "out_altvoltage0_RX_LO_frequency"}, {"sample_rate", "in_voltage_sampling_frequency"}, {"bandwidth", "in_voltage_rf_bandwidth"}, {"gain", "in_voltage0_hardwaregain"}, {"gain_mode", "in_voltage0_gain_control_mode"}, {"rf_port1", "in_voltage0_rf_port_select"}, {"rf_port2", "in_voltage1_rf_port_select"}};
        std::string readBack;
        for (const auto& [key, attr] : kReadBack) {
            if (changed.contains(std::pmr::string(key))) {
                readBack += std::format("{}{}={}", readBack.empty() ? "" : ", ", attr, _dev.phy().read(std::string(attr)).value_or("?"));
            }
        }
        std::println(stderr, "{}: RX settings applied: {}{}{}", this->name.value, detail::describeSettings(changed), readBack.empty() ? "" : "; AD9361 reads back ", readBack);
    }

    void start() {
        _startError.clear();
        _ioThreadStarted.store(false, std::memory_order_relaxed);
        _ioStopRequested.store(false, std::memory_order_relaxed);
        _metaDirty.store(true, std::memory_order_relaxed);
        _streamPos = 0U;
        _timeValid = false;
        try {
            const detail::Mode m = detail::parseMode(mode, uri);
            _dev.warn            = [this](const std::string& msg) { std::println(stderr, "{}: {}", this->name.value, msg); };
            _dev.open(detail::RWTRadioConfig{uri, m, personality, load_fpga, force_reload, base_addr, check_blkid});
            _hwm.configure(detail::resolveHardwareManagerUrl(api_url, m, uri), static_cast<std::size_t>(api_transceiver.value));
            _hwm.trace = [this](const std::string& msg) {
                if (detail::traceEnabled()) {
                    std::println(stderr, "{}: {}", this->name.value, msg);
                }
            };

            _channels = nPorts == 2UZ ? std::vector<std::size_t>{0UZ, 1UZ} : std::vector<std::size_t>{static_cast<std::size_t>(channel.value)};
            for (const std::size_t ch : _channels) {
                if (ch >= _dev.numChannels(detail::Direction::RX)) {
                    throw std::invalid_argument(std::format("RX channel {} not available (the radio has {})", ch + 1UZ, _dev.numChannels(detail::Direction::RX)));
                }
            }
            if (_dev.hasRegisters() && _dev.blockVersion() < detail::RX_BLK_VER_MIN) {
                _dev.warn(std::format("FPGA user block version {} predates RX support in gr-rwt (needs {})", _dev.blockVersion(), detail::RX_BLK_VER_MIN));
            }
            _rejected.clear();
            applySettings(nullptr);
            if (!_rejected.empty()) {
                throw std::invalid_argument(_rejected.front().second);
            }
            _dev.setTagConfig(detail::Direction::RX, escape, use_tags);
            setReceive(true);

            _rx = std::make_unique<detail::RxStream>(_dev.streamDevice(detail::Direction::RX), detail::StreamConfig{m, _channels, buffer_size, num_buffers, escape, use_tags});
            if (const int r = _rx->activate(); r < 0) {
                throw std::runtime_error("unable to start the RX stream: " + detail::iioError(r));
            }
        } catch (const std::exception& e) {
            _startError = e.what();
            std::println(stderr, "{}: start failed: {}", this->name.value, _startError);
            this->emitErrorMessage("start()", e.what());
            _rx.reset();
            setReceive(false);
            _dev.close();
            this->requestStop();
            return;
        }
        _ioThreadStarted.store(true, std::memory_order_release);
        gr::atomic_ref(_ioThreadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this]() { ioReadLoop(); });
    }

    void stop() {
        _ioStopRequested.store(true, std::memory_order_release);
        if (_ioThreadStarted.load(std::memory_order_acquire)) {
            gr::atomic_ref(_ioThreadDone).wait(false);
        }
        _rx.reset();
        setReceive(false);
        _dev.close();
    }

    work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!lifecycle::isActive(this->state())) {
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        if (this->disconnect_on_done && this->hasNoDownStreamConnectedChildren()) {
            this->requestStop();
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        // the io thread never stops the block itself (stop() waits for it); its end is the stop
        if (_ioThreadStarted.load(std::memory_order_acquire) && gr::atomic_ref(_ioThreadDone).load_acquire()) {
            this->requestStop();
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        return {requestedWork, 0UZ, work::Status::OK};
    }

    // ---------------------------------------------------------------------------------------------
    // device settings: applied all at start (keys == nullptr), then as they change, on the io thread
    void applySettings(const property_map* keys) {
        const auto has = [keys](std::string_view k) { return keys == nullptr || keys->contains(std::pmr::string(k)); };
        const bool local = _dev.mode() == detail::Mode::Local;

        if (has("sample_rate") || has("auto_filter")) {
            applySampleRate();
        }
        if (has("filter") && !auto_filter && !filter->empty()) {
            if (const std::string err = _dev.loadFilter(filter); !err.empty()) {
                _dev.warn(err);
            }
        }
        if (has("frequency")) {
            applyFrequency();
            _metaDirty.store(true, std::memory_order_release);
        }
        if (has("bandwidth")) {
            _dev.setBandwidth(detail::Direction::RX, bandwidth);
        }
        if (has("rf_port1")) {
            _dev.setPort(detail::Direction::RX, 0, rf_port1);
        }
        if (has("rf_port2") && _dev.numChannels(detail::Direction::RX) > 1UZ) {
            _dev.setPort(detail::Direction::RX, 1, rf_port2);
        }
        // the gain mode precedes the gain: the driver refuses a gain while its AGC is on
        if (has("gain_mode") || has("gain")) {
            for (const std::size_t ch : _channels) {
                _dev.setRxGainMode(ch, gain_mode);
                if (gain_mode.value == "manual") {
                    _dev.setRxGain(ch, gain);
                }
            }
        }
        if (has("quadrature") || has("rfdc") || has("bbdc")) {
            _dev.setRxTracking(quadrature, rfdc, bbdc);
        }
        if (has("extra") && !extra->empty()) {
            _dev.applyExtra(extra);
        }
        if (!local) {
            return;
        }
        if (has("use_ext_pps")) {
            _dev.setExternalPps(use_ext_pps);
        }
        if (keys != nullptr && (has("escape") || has("use_tags"))) {
            _dev.setTagConfig(detail::Direction::RX, escape, use_tags);
            if (_rx) {
                _rx->setTagConfig(escape, use_tags);
            }
        }
        if (keys != nullptr && (has("correction_enable") || has("bypass_enable") || has("decimation_arbitrary"))) {
            _dev.applyCic(_cicPow, correction_enable, bypass_enable, decimation_arbitrary);
        }
        if (has("time_now") && !time_now->empty()) {
            applyTime(time_now, false);
        }
        if (has("time_next_pps") && !time_next_pps->empty()) {
            applyTime(time_next_pps, true);
        }
    }

    void applySampleRate() {
        const bool local = _dev.mode() == detail::Mode::Local;
        const auto plan  = detail::planRxRate(static_cast<double>(sample_rate.value), local && _dev.hasRegisters());
        if (!plan.valid) {
            reject("sample_rate", std::format("RX sample_rate {} S/s is not supported: the AD9361 takes up to {} S/s{}", sample_rate.value, detail::kMaxBasebandRate,
                                      local && _dev.hasRegisters() ? ", the FPGA CIC goes down to 6 S/s" : std::format(" and no less than {} S/s", detail::kMinBasebandRate)));
            return;
        }
        _dev.setBasebandRate(detail::Direction::RX, plan.adRate, auto_filter);
        _cicPow = plan.pow;
        if (local) {
            _dev.applyCic(_cicPow, correction_enable, bypass_enable, decimation_arbitrary);
        }
        const double bb = _dev.basebandRate(detail::Direction::RX);
        _rate           = (bb > 0.0 ? bb : plan.adRate) / std::pow(10.0, _cicPow);
        _metaDirty.store(true, std::memory_order_release);
    }

    void applyTime(const std::string& value, bool atNextPps) {
        if (const auto index = detail::parseTime(value, _rate)) {
            _dev.setSampleIndex(*index, atNextPps);
        } else {
            reject(atNextPps ? "time_next_pps" : "time_now", std::format("expected \"secs,nsecs\", got '{}'", value));
        }
    }

    // ---------------------------------------------------------------------------------------------
    // io thread
    [[nodiscard]] bool ioActive() const noexcept { return lifecycle::isActive(this->state()) && !_ioStopRequested.load(std::memory_order_acquire); }

    template<typename TPort>
    static void publishTagOn(TPort& port, property_map map, std::size_t sampleOffset) {
        port.publishTag(std::move(map), sampleOffset * detail::kItemsPerSample<T>);
    }

    /// tag maps for the events of a chunk, merged per sample offset, plus the stream metadata
    std::vector<std::pair<std::size_t, property_map>> buildTags(const std::vector<detail::StreamEvent>& events) {
        std::vector<std::pair<std::size_t, property_map>> tags;
        auto mapAt = [&tags](std::size_t offset) -> property_map& {
            if (tags.empty() || tags.back().first != offset) {
                tags.emplace_back(offset, property_map{});
            }
            return tags.back().second;
        };
        if (_metaDirty.exchange(false, std::memory_order_acq_rel)) {
            property_map& m = mapAt(0UZ);
            tag::put(m, tag::SAMPLE_RATE, static_cast<float>(_rate));
            tag::put(m, tag::FREQUENCY, frequency.value);
        }
        for (const auto& e : events) {
            property_map&       m   = mapAt(e.offset);
            const std::uint64_t pos = _streamPos + e.offset;
            switch (e.type) {
            case detail::TAG_TIME: // the FPGA sample index of this sample
                _timeValid     = true;
                _timeBasePos   = pos;
                _timeBaseIndex = e.value;
                tag::put(m, tag::TRIGGER_NAME, std::string(tag_key::kTriggerTime));
                tag::put(m, tag::TRIGGER_TIME, detail::samplesToNs(e.value, _rate));
                tag::put(m, tag::TRIGGER_OFFSET, 0.f);
                tag::put(m, tag_key::kSampleIndex, e.value);
                break;
            case detail::TAG_PPS:
                tag::put(m, tag_key::kPps, e.value);
                if (!m.contains(std::pmr::string(detail::writtenKey(tag::TRIGGER_NAME)))) {
                    tag::put(m, tag::TRIGGER_NAME, std::string(tag_key::kTriggerPps));
                    if (_timeValid) {
                        tag::put(m, tag::TRIGGER_TIME, detail::samplesToNs(_timeBaseIndex + (pos - _timeBasePos), _rate));
                        tag::put(m, tag::TRIGGER_OFFSET, 0.f);
                    }
                }
                break;
            case detail::TAG_OVERFLOW: // samples were lost before this one
                _timeValid = false;
                tag::put(m, tag::RX_OVERFLOW, true);
                tag::put(m, tag_key::kOverflow, e.value);
                break;
            default: break;
            }
        }
        return tags;
    }

    void ioReadLoop() {
        thread_pool::thread::setThreadName(std::format("rwt:{}", this->name.value));

        const std::size_t                      nCh      = _channels.size();
        const std::size_t                      maxChunk = std::max<std::size_t>(1UZ, max_chunk_size);
        std::vector<std::vector<std::int16_t>> iq(nCh, std::vector<std::int16_t>(2UZ * maxChunk));
        std::vector<std::int16_t*>             iqPtrs;
        for (auto& v : iq) {
            iqPtrs.push_back(v.data());
        }
        std::vector<detail::StreamEvent> events;

        while (ioActive()) {
            this->applyChangedSettings();

            std::size_t want = maxChunk;
            if (n_samples_max > 0U) {
                const std::uint64_t left = n_samples_max - std::min<std::uint64_t>(n_samples_max, _streamPos);
                if (left == 0U) {
                    break;
                }
                want = static_cast<std::size_t>(std::min<std::uint64_t>(want, left));
            }

            const int r = _rx->read(iqPtrs, want, events, static_cast<long>(timeout_us.value));
            if (r == -ETIMEDOUT || r == 0) {
                continue; // events seen so far stay pending for the next samples
            }
            if (r < 0) {
                reportStreamError("ioReadLoop()", "RX stream failed: " + detail::iioError(r));
                break;
            }
            const auto n    = static_cast<std::size_t>(r);
            auto       tags = buildTags(events);
            events.clear();
            if (!publish(iq, n, tags)) {
                break; // stopping
            }
            _streamPos += n;
        }

        this->publishEoS();
        _rx->deactivate();
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// publishes n samples per channel with their tags, waiting for buffer space; false if stopping
    bool publish(const std::vector<std::vector<std::int16_t>>& iq, std::size_t n, const std::vector<std::pair<std::size_t, property_map>>& tags) {
        constexpr std::size_t kItems = detail::kItemsPerSample<T>;
        const auto            waitForSpace = [this] {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            this->applyChangedSettings();
        };

        if constexpr (nPorts == 1UZ) {
            auto&       writer = out.streamWriter();
            std::size_t done   = 0UZ;
            std::size_t tagIdx = 0UZ;
            while (done < n) {
                if (!lifecycle::isActive(this->state())) {
                    return false;
                }
                auto              span  = writer.template tryReserve<SpanReleasePolicy::ProcessNone>((n - done) * kItems);
                const std::size_t nCopy = span.size() / kItems;
                if (nCopy == 0UZ) {
                    span.publish(0UZ);
                    waitForSpace();
                    continue;
                }
                detail::fromIq(iq[0].data() + 2UZ * done, nCopy, span.data());
                for (; tagIdx < tags.size() && tags[tagIdx].first < done + nCopy; ++tagIdx) {
                    publishTagOn(out, tags[tagIdx].second, tags[tagIdx].first - std::min(done, tags[tagIdx].first));
                }
                span.publish(nCopy * kItems);
                done += nCopy;
                this->progress->incrementAndGet();
                this->progress->notify_all();
            }
        } else {
            // hold the chunk until every port has room for all of it, so the channels stay aligned
            while (!std::ranges::all_of(out, [n](auto& p) { return p.streamWriter().available() >= n * kItems; })) {
                if (!lifecycle::isActive(this->state())) {
                    return false;
                }
                waitForSpace();
            }
            for (std::size_t k = 0; k < nPorts; ++k) {
                auto span = out[k].streamWriter().template tryReserve<SpanReleasePolicy::ProcessNone>(n * kItems);
                detail::fromIq(iq[k].data(), n, span.data());
                for (const auto& [offset, map] : tags) {
                    publishTagOn(out[k], map, offset);
                }
                span.publish(n * kItems);
            }
            this->progress->incrementAndGet();
            this->progress->notify_all();
        }
        return true;
    }
};

template<typename T>
using RWTSingleSource = RWTSource<T, 1UZ>;
template<typename T>
using RWTDualSource = RWTSource<T, 2UZ>;

} // namespace gr::rwt

#endif // GNURADIO_RWT_SOURCE_HPP
