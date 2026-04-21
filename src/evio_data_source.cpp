// =========================================================================
// evio_data_source.cpp — EVIO file data source implementation
// =========================================================================

#include "evio_data_source.h"

#include <iostream>
#include <memory>

using namespace evc;

// =========================================================================
// Open / Close
// =========================================================================

std::string EvioDataSource::open(const std::string &path)
{
    close();
    filepath_ = path;

    // Open reader_ in evio random-access mode: the file is mmap'd and an
    // event-pointer table is built natively during Open.  We keep this
    // handle alive for the life of the data source so decodeEvent() can
    // jump directly via ReadEventByIndex().
    reader_.SetConfig(cfg_);
    if (reader_.OpenRandomAccess(path) != status::success) {
        invalidateReader();
        filepath_.clear();
        return "cannot open file (random-access mode)";
    }
    reader_path_ = path;
    last_decoded_index_ = -1;

    // Iterate evio events, Scan each (cheap — header parse only, no waveform
    // decode), and record physics sub-events in our index.
    int n_evio = reader_.GetRandomAccessEventCount();
    for (int ei = 0; ei < n_evio; ++ei) {
        if (reader_.ReadEventByIndex(ei) != status::success) continue;
        if (!reader_.Scan()) continue;
        // skip monitoring events (TI only, no waveforms) from viewer index
        if (cfg_.is_monitoring(reader_.GetEvHeader().tag)) continue;
        for (int si = 0; si < reader_.GetNEvents(); ++si)
            index_.push_back({ei, si});
    }
    // The index pass left reader_'s cache holding the last event scanned;
    // invalidate so decodeEvent() doesn't reuse a stale cached decode.
    last_decoded_index_ = -1;
    return "";
}

void EvioDataSource::close()
{
    index_.clear();
    invalidateReader();
    filepath_.clear();
}

// =========================================================================
// Capabilities
// =========================================================================

DataSourceCaps EvioDataSource::capabilities() const
{
    return {
        true,   // has_waveforms
        true,   // has_peaks (computed by WaveAnalyzer)
        true,   // has_pedestals
        true,   // has_clusters (computed by HyCalCluster)
        true,   // has_gem_raw
        true,   // has_gem_hits (computed by GemCluster)
        true,   // has_epics
        true,   // has_sync
        "evio"  // source_type
    };
}

// =========================================================================
// Random-access event decoding
// =========================================================================

void EvioDataSource::invalidateReader()
{
    reader_.Close();
    reader_path_.clear();
    last_decoded_index_ = -1;
}

std::string EvioDataSource::decodeEvent(int index, fdec::EventData &evt,
                                         ssp::SspEventData *ssp)
{
    if (index < 0 || index >= (int)index_.size())
        return "event out of range";

    std::lock_guard<std::mutex> lk(reader_mtx_);

    // Cache-hit path: same event as last decode → reader_'s lazy cache is
    // already valid, skip Scan + decode and just copy out.  This is the
    // viewer_server's common pattern of decodeEvent(N) immediately followed
    // by computeClusters(N).
    if (index != last_decoded_index_) {
        // Reopen the reader if something invalidated it (e.g. a prior error).
        if (reader_path_ != filepath_) {
            reader_.SetConfig(cfg_);
            if (reader_.OpenRandomAccess(filepath_) != status::success) {
                invalidateReader();
                return "cannot open file";
            }
            reader_path_ = filepath_;
        }
        const auto &ei = index_[index];
        if (reader_.ReadEventByIndex(ei.evio_event) != status::success) {
            last_decoded_index_ = -1;
            return "read error";
        }
        if (!reader_.Scan()) { last_decoded_index_ = -1; return "scan error"; }
        reader_.SelectEvent(ei.sub_event);
        last_decoded_index_ = index;
    }

    evt = reader_.Fadc();                      // lazy-decodes on first access,
    if (ssp) *ssp = reader_.Gem();             // returns cached ref thereafter
    return "";
}

// =========================================================================
// Full iteration (for histogram building)
// =========================================================================

void EvioDataSource::iterateAll(EventCallback ev_cb, ReconCallback /*recon_cb*/,
                                ControlCallback ctrl_cb, EpicsCallback epics_cb)
{
    EvChannel ch;
    ch.SetConfig(cfg_);
    if (ch.Open(filepath_) != status::success) return;

    auto event_ptr = std::make_unique<fdec::EventData>();
    auto &event = *event_ptr;
    auto ssp_ptr = std::make_unique<ssp::SspEventData>();
    auto &ssp_evt = *ssp_ptr;
    uint64_t last_ti_ts = 0;

    while (ch.Read() == status::success) {
        if (!ch.Scan()) continue;

        // control events (sync/prestart/go/end) — absolute unix time lands in
        // ch.Sync() along with run number / type / counter; Sync()'s snapshot
        // persists across events, so gate on event type so physics events
        // don't re-fire the callback.
        if (ctrl_cb) {
            auto et = ch.GetEventType();
            if (et == EventType::Prestart || et == EventType::Go ||
                et == EventType::End      || et == EventType::Sync)
            {
                const auto &s = ch.Sync();
                if (s.unix_time != 0) ctrl_cb(s.unix_time, last_ti_ts);
            }
        }

        // EPICS events
        if (epics_cb && ch.GetEventType() == EventType::Epics) {
            std::string text = ch.ExtractEpicsText();
            if (!text.empty())
                epics_cb(text, 0, last_ti_ts);
        }

        // physics events (skip monitoring — no waveforms to process)
        if (cfg_.is_monitoring(ch.GetEvHeader().tag)) continue;
        for (int i = 0; i < ch.GetNEvents(); ++i) {
            ssp_evt.clear();
            if (!ch.DecodeEvent(i, event, &ssp_evt)) continue;
            last_ti_ts = event.info.timestamp;
            if (ev_cb) ev_cb(i, event, &ssp_evt);
        }
    }
    ch.Close();
}
