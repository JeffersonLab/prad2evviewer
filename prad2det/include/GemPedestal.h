#pragma once
//=============================================================================
// GemPedestal.h — per-strip pedestal accumulator for GEM (APV25 front-end).
//
// Algorithm (per event, per APV):
//   1) Per time sample — sort 128 strip ADCs, drop the top CM_DISCARD and
//      bottom CM_DISCARD, average the middle 72 → common_mode[ts].
//      Subtract common_mode[ts] from every strip's value.
//   2) Per strip — average the CM-corrected values across the 6 time
//      samples → contribution.  Accumulate into per-strip mean/RMS.
//
// After all events, Write() writes a JSON file in the format
// GemSystem::LoadPedestals reads: one entry per APV with parallel
// ``offset`` (mean) and ``noise`` (RMS) arrays per strip.
//
// Usage:
//   gem::GemPedestal ped;
//   while (read_next_event(ssp_evt))
//       ped.Accumulate(ssp_evt);
//   ped.Write("gem_ped.json");
//=============================================================================

#include <memory>
#include <string>

namespace ssp { struct SspEventData; }

namespace gem {

class GemPedestal
{
public:
    GemPedestal();
    ~GemPedestal();

    GemPedestal(const GemPedestal &)            = delete;
    GemPedestal &operator=(const GemPedestal &) = delete;

    // Drop all accumulated stats.
    void Clear();

    // Fold one event's SSP data into the running accumulators.  APVs that
    // are not in full-readout mode (nstrips != 128) are silently skipped
    // for CM purposes — those strips contributed in online-ZS and can't
    // be pedestal-corrected offline.
    void Accumulate(const ssp::SspEventData &evt);

    // Number of APVs that received at least one contribution.
    int NumApvs() const;
    // Number of strips (across all APVs) with at least one contribution.
    int NumStrips() const;

    // Serialize the accumulated mean/RMS to JSON.  Returns the number of
    // APVs written, or a negative value on I/O failure.
    int Write(const std::string &output_path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace gem
