// SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Red Wire Technologies
#ifndef GNURADIO_RWT_SINK_HPP
#define GNURADIO_RWT_SINK_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <gnuradio-4.0/rwt/detail/BlockSupport.hpp>
#include <gnuradio-4.0/rwt/detail/Oxygen.hpp>
#include <gnuradio-4.0/rwt/detail/RadioApi.hpp>
#include <gnuradio-4.0/rwt/detail/Streams.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <print>
#include <string>
#include <thread>
#include <vector>

namespace gr::rwt {

GR_REGISTER_BLOCK("gr::rwt::RWTSink", gr::rwt::RWTSingleSink, [T], [ std::complex<float>, std::int16_t ])
GR_REGISTER_BLOCK("gr::rwt::RWTDualSink", gr::rwt::RWTDualSink, [T], [ std::complex<float>, std::int16_t ])

template<typename T, std::size_t nPorts = 1UZ>
requires((std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::int16_t>) && (nPorts == 1UZ || nPorts == 2UZ))
struct RWTSink : Block<RWTSink<T, nPorts>> {
    using Description = Doc<R"(RWT Oxygen SDR sink (AD9361 transmitter), the GNU Radio 4 counterpart of gr-rwt's rwt_sink.

RWTSink transmits one channel (`channel` picks TX1 or TX2); RWTDualSink transmits both, port 0 = TX1, port 1 = TX2.
std::complex<float> input is scaled so 1.0 is the DAC's full scale; std::int16_t input is raw interleaved I,Q.

mode "local" (default when uri is empty) runs on the Oxygen with the "default" FPGA personality, as gr-rwt does, and turns
input tags into the FPGA's in-band burst markers (GNU Radio's gr:tx_* keys, and rwt:tx_sample):
  gr:tx_sob     (true)               start of burst
  gr:tx_eob     (true)               the tagged sample is the burst's last; the burst is sent at once
  gr:tx_time    (integer ns, or s)   hold the tagged sample until this time (FPGA DAC sample index = time x sample_rate)
  rwt:tx_sample (uint64)             hold the tagged sample until this FPGA DAC sample index
The bare spellings gr-rwt uses (tx_sob, tx_eob, tx_time, tx_sample) are read too.
mode "remote" streams to an Oxygen running the "griio" personality over libiio (uri "ip:<host>"); there gr:tx_eob only
flushes the partial buffer and the other tags are ignored.

Samples are handed to an io thread through a bounded queue, so a full DMA ring holds back the graph rather than a
scheduler thread. At stop the queue drains, the partial buffer is sent, and with mute_on_stop the outputs are attenuated
by 87.5 dB, as gr-rwt does.)">;

    using TPortType = std::conditional_t<nPorts == 1UZ, PortIn<T>, std::array<PortIn<T>, nPorts>>;

    TPortType in;

    // device
    Annotated<std::string, "uri", Visible, Doc<"libiio context URI; empty = this Oxygen (local:)">>              uri;
    Annotated<std::string, "mode", Doc<"local (default FPGA personality, tags) or remote (griio); empty = by uri">> mode;
    // RF (gr-rwt rwt_sink parameters)
    Annotated<float, "sample_rate", Unit<"Hz">, Visible, Doc<"TX sample rate (the AD9361 clock is shared with RX)">> sample_rate  = 5'000'000.f;
    Annotated<double, "frequency", Unit<"Hz">, Visible, Doc<"TX LO frequency (shared by both channels)">>            frequency    = 2.4e9;
    Annotated<double, "bandwidth", Unit<"Hz">, Visible, Doc<"TX RF bandwidth">>                                      bandwidth    = 40e6;
    Annotated<gr::Size_t, "channel", Visible, Doc<"TX channel of the single-channel sink: 0 = TX1, 1 = TX2">>        channel      = 0U;
    Annotated<std::string, "rf_port1", Doc<"TX1 port: A or B">>                                                      rf_port1     = std::string("A");
    Annotated<std::string, "rf_port2", Doc<"TX2 port: A or B">>                                                      rf_port2     = std::string("A");
    Annotated<double, "attenuation1", Unit<"dB">, Visible, Doc<"TX1 attenuation, 0 .. 89.75 dB">>                    attenuation1 = 10.0;
    Annotated<double, "attenuation2", Unit<"dB">, Visible, Doc<"TX2 attenuation, 0 .. 89.75 dB">>                    attenuation2 = 10.0;
    Annotated<std::string, "filter", Doc<"AD9361 .ftr FIR file (RX and TX); used when auto_filter is off">>          filter;
    Annotated<bool, "auto_filter", Doc<"load a FIR matching the rate (libad9361)">>                                  auto_filter  = true;
    Annotated<std::string, "extra", Doc<"gr-rwt extra settings: comma-separated key=value">>                         extra;
    // Oxygen management API (http://<radio>/docs): connection_control sets the TX frequency with the RF path's filters, and switches the TX paths on at start and off at stop
    Annotated<std::string, "api_url", Doc<"management API base URL; empty = this Oxygen (local) or the uri's host (ip:), none = off">> api_url;
    // an integer, not a string: gr-studio writes it unquoted, and GR4 refuses an integer for a string setting
    Annotated<gr::Size_t, "api_transceiver", Doc<"connection_control transceiver: N = TransceiverN (0 on an Oxygen)">>       api_transceiver = 0U;
    // FPGA ("default" personality, local mode)
    Annotated<bool, "use_tags", Doc<"encode tx_* tags as in-band FPGA markers (takes effect at start)">>              use_tags     = true;
    Annotated<std::uint64_t, "escape", Doc<"escape word introducing in-band markers (takes effect at start)">>         escape       = detail::DEFAULT_ESCAPE;
    Annotated<std::string, "personality", Doc<"FPGA image for fpgaloader">>                                          personality  = std::string("default");
    Annotated<bool, "load_fpga", Doc<"run fpgaloader at start (local mode)">>                                        load_fpga    = true;
    Annotated<bool, "force_reload", Doc<"pass -f to fpgaloader">>                                                    force_reload = false;
    Annotated<std::uint64_t, "base_addr", Doc<"physical address of the FPGA user registers; 0 = do not map them">>  base_addr    = detail::DEFAULT_REG_BASE_ADDR;
    Annotated<bool, "check_blkid", Doc<"refuse an FPGA image that is not the default personality">>                  check_blkid  = true;
    // streaming
    Annotated<gr::Size_t, "buffer_size", Doc<"samples per IIO block">>                                              buffer_size    = 32768U;
    Annotated<gr::Size_t, "num_buffers", Doc<"IIO blocks kept in flight">>                                           num_buffers    = 4U;
    Annotated<gr::Size_t, "max_chunk_size", Doc<"max samples taken from the input at once">>                         max_chunk_size = 8192U;
    Annotated<gr::Size_t, "max_queued_chunks", Doc<"chunks queued for the io thread before the input is held back">> max_queued_chunks = 8U;
    Annotated<bool, "mute_on_stop", Doc<"attenuate the outputs by 87.5 dB when stopping (gr-rwt)">>                  mute_on_stop   = true;

    GR_MAKE_REFLECTABLE(RWTSink, in, uri, mode, sample_rate, frequency, bandwidth, channel, rf_port1, rf_port2, attenuation1, attenuation2, filter, auto_filter, extra, api_url, api_transceiver, use_tags, escape, personality, load_fpga, force_reload, base_addr, check_blkid, buffer_size, num_buffers, max_chunk_size, max_queued_chunks, mute_on_stop);

    struct Chunk {
        std::vector<std::vector<std::int16_t>> iq;
        std::size_t                            n = 0UZ;
        std::vector<detail::StreamEvent>       events;
    };

    detail::Oxygen                    _dev;
    std::string                       _startError; // why the last start() failed; empty while it holds the radio
    detail::RadioApi                  _api;        // the management API, when api_url resolves to one
    std::vector<std::size_t>          _txOn;       // TX paths this block switched on through the API
    std::unique_ptr<detail::TxStream> _tx;
    std::vector<std::size_t>          _channels;
    std::atomic<double>               _rate{0.0}; // TX rate, for tx_time
    bool                              _warnedRemoteTags = false;

    std::mutex              _queueMutex;
    std::condition_variable _queueCv;
    std::deque<Chunk>       _queue;

    bool              _ioThreadDone = true;
    std::atomic<bool> _ioThreadStarted{false};
    std::atomic<bool> _ioStopRequested{false}; // drain the queue, then finish
    std::atomic<bool> _ioCancel{false};        // give up waiting for the hardware
    std::atomic<bool> _ioFailed{false};

    struct IoThreadGuard {
        RWTSink* self;
        ~IoThreadGuard() {
            if (lifecycle::isActive(self->state())) {
                self->requestStop();
            }
            self->_ioCancel.store(true, std::memory_order_release);
            self->_queueCv.notify_all();
            gr::atomic_ref(self->_ioThreadDone).wait(false);
            self->setTransmit(false); // a block destroyed without stop() leaves no TX path on
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

    /// the TX frequency through the management API's connection_control, which also sets the RF path's
    /// filters, for each TX path the block uses; through libiio (AD9361 LO alone) without the API or if it fails
    void applyFrequency() {
        if (_api.enabled()) {
            bool ok = true;
            for (const std::size_t ch : _channels) {
                if (const std::string err = _api.setFrequency(detail::Direction::TX, ch, frequency); !err.empty()) {
                    _dev.warn(err);
                    ok = false;
                }
            }
            if (ok) {
                return;
            }
            _dev.warn("the management API did not take the TX frequency; setting the AD9361 LO through libiio");
        }
        _dev.setLo(detail::Direction::TX, frequency);
    }

    /// connection_control transmit_enable for the block's TX paths: on at start, off when the block
    /// stops, fails to start, or is destroyed. Only paths this block switched on are switched off.
    void setTransmit(bool on) {
        if (!_api.enabled()) {
            return;
        }
        if (on) {
            for (const std::size_t ch : _channels) {
                if (const std::string err = _api.transmitEnable(ch, true); !err.empty()) {
                    _dev.warn("TX" + std::to_string(ch) + " not switched on: " + err);
                } else {
                    _txOn.push_back(ch);
                }
            }
            return;
        }
        for (const std::size_t ch : _txOn) {
            if (const std::string err = _api.transmitEnable(ch, false); !err.empty()) {
                std::println(stderr, "{}: TX{} not switched off: {}", this->name.value, ch, err);
            }
        }
        _txOn.clear();
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
        static constexpr std::pair<std::string_view, std::string_view> kReadBack[] = {{"frequency", "out_altvoltage1_TX_LO_frequency"}, {"sample_rate", "tx_sampling_frequency"}, {"bandwidth", "tx_rf_bandwidth"}, {"attenuation1", "out_voltage0_hardwaregain"}, {"attenuation2", "out_voltage1_hardwaregain"}, {"rf_port1", "out_voltage0_rf_port_select"}, {"rf_port2", "out_voltage1_rf_port_select"}};
        std::string readBack;
        for (const auto& [key, attr] : kReadBack) {
            if (changed.contains(std::pmr::string(key))) {
                readBack += std::format("{}{}={}", readBack.empty() ? "" : ", ", attr, _dev.phy().read(std::string(attr)).value_or("?"));
            }
        }
        std::println(stderr, "{}: TX settings applied: {}{}{}", this->name.value, detail::describeSettings(changed), readBack.empty() ? "" : "; AD9361 reads back ", readBack);
    }

    void start() {
        _startError.clear();
        _ioThreadStarted.store(false, std::memory_order_relaxed);
        _ioStopRequested.store(false, std::memory_order_relaxed);
        _ioCancel.store(false, std::memory_order_relaxed);
        _ioFailed.store(false, std::memory_order_relaxed);
        _warnedRemoteTags = false;
        {
            std::lock_guard lock(_queueMutex);
            _queue.clear();
        }
        try {
            const detail::Mode m = detail::parseMode(mode, uri);
            _dev.warn            = [this](const std::string& msg) { std::println(stderr, "{}: {}", this->name.value, msg); };
            _dev.open(detail::OxygenConfig{uri, m, personality, load_fpga, force_reload, base_addr, check_blkid});
            _api.configure(detail::resolveApiUrl(api_url, m, uri), static_cast<std::size_t>(api_transceiver.value));
            _api.trace = [this](const std::string& msg) {
                if (detail::traceEnabled()) {
                    std::println(stderr, "{}: {}", this->name.value, msg);
                }
            };

            _channels = nPorts == 2UZ ? std::vector<std::size_t>{0UZ, 1UZ} : std::vector<std::size_t>{static_cast<std::size_t>(channel.value)};
            for (const std::size_t ch : _channels) {
                if (ch >= _dev.numChannels(detail::Direction::TX)) {
                    throw std::invalid_argument(std::format("TX channel {} not available (the radio has {})", ch + 1UZ, _dev.numChannels(detail::Direction::TX)));
                }
            }
            _rejected.clear();
            applySettings(nullptr);
            if (!_rejected.empty()) {
                throw std::invalid_argument(_rejected.front().second);
            }
            _dev.setTagConfig(detail::Direction::TX, escape, use_tags);
            setTransmit(true);

            _tx = std::make_unique<detail::TxStream>(_dev.streamDevice(detail::Direction::TX), detail::StreamConfig{m, _channels, buffer_size, num_buffers, escape, use_tags});
            _tx->setStopFlag(&_ioCancel);
            if (const int r = _tx->activate(); r < 0) {
                throw std::runtime_error("unable to start the TX stream: " + detail::iioError(r));
            }
        } catch (const std::exception& e) {
            _startError = e.what();
            std::println(stderr, "{}: start failed: {}", this->name.value, _startError);
            this->emitErrorMessage("start()", e.what());
            _tx.reset();
            setTransmit(false);
            _dev.close();
            this->requestStop();
            return;
        }
        _ioThreadStarted.store(true, std::memory_order_release);
        gr::atomic_ref(_ioThreadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this]() { ioWriteLoop(); });
    }

    void stop() {
        if (_ioThreadStarted.load(std::memory_order_acquire)) {
            _ioStopRequested.store(true, std::memory_order_release);
            _queueCv.notify_all();
            // let queued samples go out; give up on the hardware after a while (e.g. a far-future tx_time)
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (!gr::atomic_ref(_ioThreadDone).load_acquire() && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            _ioCancel.store(true, std::memory_order_release);
            _queueCv.notify_all();
            gr::atomic_ref(_ioThreadDone).wait(false);
        }
        if (_dev.isOpen() && mute_on_stop) {
            for (const std::size_t ch : _channels) {
                _dev.setTxAttenuation(ch, 87.5);
            }
        }
        _tx.reset();
        setTransmit(false);
        _dev.close();
    }

    work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!lifecycle::isActive(this->state())) {
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        if (_ioThreadStarted.load(std::memory_order_acquire) && gr::atomic_ref(_ioThreadDone).load_acquire()) {
            this->requestStop();
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        return Block<RWTSink<T, nPorts>>::work(requestedWork);
    }

    // ---------------------------------------------------------------------------------------------
    void applySettings(const property_map* keys) {
        const auto has = [keys](std::string_view k) { return keys == nullptr || keys->contains(std::pmr::string(k)); };
        if (has("sample_rate") || has("auto_filter")) {
            const double rate = static_cast<double>(sample_rate.value);
            if (rate < detail::kMinBasebandRate || rate > detail::kMaxBasebandRate) {
                reject("sample_rate", std::format("TX sample_rate {} S/s is outside the AD9361's {} .. {} S/s", sample_rate.value, detail::kMinBasebandRate, detail::kMaxBasebandRate));
            } else {
                _dev.setBasebandRate(detail::Direction::TX, rate, auto_filter);
                const double bb = _dev.basebandRate(detail::Direction::TX);
                _rate.store(bb > 0.0 ? bb : rate, std::memory_order_release);
            }
        }
        if (has("filter") && !auto_filter && !filter->empty()) {
            if (const std::string err = _dev.loadFilter(filter); !err.empty()) {
                _dev.warn(err);
            }
        }
        if (has("frequency")) {
            applyFrequency();
        }
        if (has("bandwidth")) {
            _dev.setBandwidth(detail::Direction::TX, bandwidth);
        }
        if (has("rf_port1")) {
            _dev.setPort(detail::Direction::TX, 0, rf_port1);
        }
        if (has("rf_port2") && _dev.numChannels(detail::Direction::TX) > 1UZ) {
            _dev.setPort(detail::Direction::TX, 1, rf_port2);
        }
        if (has("attenuation1")) {
            _dev.setTxAttenuation(0, attenuation1);
        }
        if (has("attenuation2") && _dev.numChannels(detail::Direction::TX) > 1UZ) {
            _dev.setTxAttenuation(1, attenuation2);
        }
        if (has("extra") && !extra->empty()) {
            _dev.applyExtra(extra);
        }
    }

    /// gr-rwt's tx_* tags of one input, as stream events at sample offsets below nSamples
    void collectEvents(auto& input, std::size_t nSamples, std::vector<detail::StreamEvent>& events) {
        constexpr std::size_t kItems = detail::kItemsPerSample<T>;
        for (const auto& [relIndex, mapRef] : input.tags()) {
            const std::size_t item   = relIndex < 0 ? 0UZ : static_cast<std::size_t>(relIndex);
            const std::size_t offset = item / kItems;
            if (offset >= nSamples) {
                continue;
            }
            const property_map& map = mapRef.get();
            // the domain spelling (gr:tx_sob, rwt:tx_sample) first, then the bare one gr-rwt and older GNU Radio 4 use
            const auto get = [&map](std::string_view key) -> const pmt::Value* {
                for (const std::string_view k : {key, tag_key::legacyName(key)}) {
                    if (const auto it = map.find(k); it != map.end()) {
                        return &it->second;
                    }
                }
                return nullptr;
            };
            if (const auto* v = get(tag_key::kTxSob); v != nullptr && detail::asTrue(*v)) {
                events.push_back({offset, detail::TAG_SOB, 0U});
            }
            if (const auto* v = get(tag_key::kTxTime)) {
                // integer: nanoseconds (GR4 convention); floating point: seconds
                std::optional<std::uint64_t> ns;
                if (v->is_floating_point()) {
                    if (auto s = detail::asNumber(*v); s && *s >= 0.0) {
                        ns = static_cast<std::uint64_t>(std::llround(*s * 1e9));
                    }
                } else {
                    ns = detail::asUnsigned(*v);
                }
                if (ns) {
                    events.push_back({offset, detail::TAG_HOLD, detail::nsToSamples(*ns, _rate.load(std::memory_order_acquire))});
                }
            }
            if (const auto* v = get(tag_key::kTxSample)) {
                if (auto idx = detail::asUnsigned(*v)) {
                    events.push_back({offset, detail::TAG_HOLD, *idx});
                }
            }
            if (const auto* v = get(tag_key::kTxEob); v != nullptr && detail::asTrue(*v)) {
                events.push_back({offset, detail::TAG_EOB, 0U});
            }
        }
    }

    [[nodiscard]] bool queueFull() {
        std::lock_guard lock(_queueMutex);
        return _queue.size() >= std::max<std::size_t>(1UZ, max_queued_chunks);
    }

    void enqueue(Chunk&& chunk) {
        if (_dev.mode() == detail::Mode::Remote && !_warnedRemoteTags && std::ranges::any_of(chunk.events, [](const auto& e) { return e.type != detail::TAG_EOB; })) {
            _dev.warn("gr:tx_sob, gr:tx_time and rwt:tx_sample need the default FPGA personality (local mode); ignoring them");
            _warnedRemoteTags = true;
        }
        std::ranges::sort(chunk.events, [](const auto& a, const auto& b) { return a.offset < b.offset; });
        {
            std::lock_guard lock(_queueMutex);
            _queue.push_back(std::move(chunk));
        }
        _queueCv.notify_one();
    }

    [[nodiscard]] gr::work::Status processBulk(InputSpanLike auto& input) noexcept
    requires(nPorts == 1UZ)
    {
        constexpr std::size_t kItems = detail::kItemsPerSample<T>;
        if (!_ioThreadStarted.load(std::memory_order_acquire)) {
            std::ignore = input.consume(input.size()); // the radio did not open: drop, as SoapySink does
            return gr::work::Status::OK;
        }
        const std::size_t n = std::min<std::size_t>(input.size() / kItems, max_chunk_size);
        if (n == 0UZ) {
            std::ignore = input.consume(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        if (queueFull()) {
            std::ignore = input.consume(0UZ);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        Chunk chunk;
        chunk.n = n;
        chunk.iq.assign(1UZ, std::vector<std::int16_t>(2UZ * n));
        detail::toIq(std::span<const T>(input).data(), n, chunk.iq[0].data());
        collectEvents(input, n, chunk.events);
        enqueue(std::move(chunk));
        std::ignore = input.consume(n * kItems);
        return gr::work::Status::OK;
    }

    template<InputSpanLike TInSpan>
    [[nodiscard]] gr::work::Status processBulk(std::span<TInSpan>& inputs) noexcept
    requires(nPorts != 1UZ)
    {
        constexpr std::size_t kItems = detail::kItemsPerSample<T>;
        if (!_ioThreadStarted.load(std::memory_order_acquire)) {
            for (auto& input : inputs) {
                std::ignore = input.consume(input.size());
            }
            return gr::work::Status::OK;
        }
        std::size_t n = max_chunk_size;
        for (const auto& input : inputs) {
            n = std::min(n, input.size() / kItems);
        }
        if (n == 0UZ || queueFull()) {
            for (auto& input : inputs) {
                std::ignore = input.consume(0UZ);
            }
            return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        Chunk chunk;
        chunk.n = n;
        chunk.iq.assign(nPorts, std::vector<std::int16_t>(2UZ * n));
        for (std::size_t k = 0; k < nPorts; ++k) {
            detail::toIq(std::span<const T>(inputs[k]).data(), n, chunk.iq[k].data());
            collectEvents(inputs[k], n, chunk.events);
        }
        // the same marker on both ports is one marker
        std::ranges::sort(chunk.events, [](const auto& a, const auto& b) { return std::tie(a.offset, a.type, a.value) < std::tie(b.offset, b.type, b.value); });
        auto dup = std::ranges::unique(chunk.events, [](const auto& a, const auto& b) { return a.offset == b.offset && a.type == b.type && a.value == b.value; });
        chunk.events.erase(dup.begin(), dup.end());
        enqueue(std::move(chunk));
        for (auto& input : inputs) {
            std::ignore = input.consume(n * kItems);
        }
        return gr::work::Status::OK;
    }

    // ---------------------------------------------------------------------------------------------
    // io thread
    void ioWriteLoop() {
        thread_pool::thread::setThreadName(std::format("rwt:{}", this->name.value));
        std::vector<const std::int16_t*> ptrs;
        for (;;) {
            Chunk chunk;
            {
                std::unique_lock lock(_queueMutex);
                _queueCv.wait_for(lock, std::chrono::milliseconds(100), [this] { return !_queue.empty() || _ioStopRequested.load(std::memory_order_acquire) || _ioCancel.load(std::memory_order_acquire); });
                if (_ioCancel.load(std::memory_order_acquire)) {
                    break;
                }
                if (_queue.empty()) {
                    if (_ioStopRequested.load(std::memory_order_acquire)) {
                        break;
                    }
                    continue;
                }
                chunk = std::move(_queue.front());
                _queue.pop_front();
            }
            ptrs.clear();
            for (const auto& v : chunk.iq) {
                ptrs.push_back(v.data());
            }
            if (const int r = _tx->write(ptrs, chunk.n, chunk.events); r < 0) {
                if (r != -ECANCELED) {
                    reportStreamError("ioWriteLoop()", "TX stream failed: " + detail::iioError(r));
                    _ioFailed.store(true, std::memory_order_release);
                }
                break;
            }
            this->progress->incrementAndGet(); // queue space for processBulk
            this->progress->notify_all();
        }
        if (!_ioCancel.load(std::memory_order_acquire) && !_ioFailed.load(std::memory_order_acquire)) {
            if (const int r = _tx->flush(); r < 0 && r != -ECANCELED) {
                reportStreamError("ioWriteLoop()", "TX flush failed: " + detail::iioError(r));
            }
            _tx->deactivate(2'000'000L);
        } else {
            _tx->deactivate(0L);
        }
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }
};

template<typename T>
using RWTSingleSink = RWTSink<T, 1UZ>;
template<typename T>
using RWTDualSink = RWTSink<T, 2UZ>;

} // namespace gr::rwt

#endif // GNURADIO_RWT_SINK_HPP
