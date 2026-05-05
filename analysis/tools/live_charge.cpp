// =============================================================================
// live_charge.cpp — accumulate live charge from any replayed ROOT file.
//
// Inputs: one or more replayed ROOT files (output of replay_rawdata,
// replay_recon, OR replay_filter — anything that carries the `scalers`
// and `epics` side trees).  The integrator does the same arithmetic
// replay_filter applies internally to its passing checkpoint pairs:
//
//     Q = Σ live_fraction · Δt · ½(I_i + I_{i+1})
//
// where live_fraction is the slice-local DSC2 livetime (Δgated / Δungated
// since the previous scaler readout), Δt is the gap between adjacent
// merged checkpoints (TI ticks via the row-stamped `ti_ticks` and
// `ti_ticks_at_arrival`), and I is forward-filled from the configured
// EPICS beam-current channel.
//
// `good` filtering: when the `epics` and `scalers` trees carry the per-
// row `good` bool replay_filter writes (i.e. the input is a filter
// output), the tool integrates only over passing-passing pairs — yielding
// post-cut live charge.  When the column is absent (raw / recon), every
// adjacent pair contributes — yielding total live charge over the run,
// no quality cuts applied.
//
// Beam current is assumed to publish in nA (true for the Hall B IPM
// scalers); the resulting charge is therefore in nC.
// =============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <getopt.h>

#include <TFile.h>
#include <TTree.h>

#include <nlohmann/json.hpp>

#include "EventData.h"
#include "EventData_io.h"

using json = nlohmann::json;

namespace {

constexpr double TI_TICK_SEC = 4.0e-9;
constexpr int    DSC_NCH     = 16;

// ── In-memory rows ──────────────────────────────────────────────────────────

struct ScalerRow {
    int32_t  event_number   = 0;
    int64_t  ti_ticks       = 0;
    uint32_t ref_gated      = 0;
    uint32_t ref_ungated    = 0;
    uint32_t trg_gated[DSC_NCH]   = {};
    uint32_t trg_ungated[DSC_NCH] = {};
    uint32_t tdc_gated[DSC_NCH]   = {};
    uint32_t tdc_ungated[DSC_NCH] = {};
    bool     good           = true;        // defaults to "kept" when no col
};

struct EpicsRow {
    int32_t                       event_number = 0;
    int64_t                       ti_ticks     = 0;
    bool                          good         = true;
    std::map<std::string, double> updates;
};

// ── Tree readers ────────────────────────────────────────────────────────────

bool load_scalers(const std::vector<std::string> &files,
                  std::vector<ScalerRow>          &out,
                  bool                            &any_good_col)
{
    prad2::RawScalerData sc;
    any_good_col = false;
    for (const auto &path : files) {
        std::unique_ptr<TFile> f(TFile::Open(path.c_str(), "READ"));
        if (!f || f->IsZombie()) {
            std::cerr << "live_charge: cannot open " << path << "\n";
            return false;
        }
        TTree *t = dynamic_cast<TTree *>(f->Get("scalers"));
        if (!t) continue;
        prad2::SetScalerReadBranches(t, sc);
        bool good = true;
        const bool has_good = (t->GetBranch("good") != nullptr);
        any_good_col = any_good_col || has_good;
        if (has_good) t->SetBranchAddress("good", &good);
        Long64_t n = t->GetEntries();
        out.reserve(out.size() + n);
        for (Long64_t i = 0; i < n; ++i) {
            good = true;
            t->GetEntry(i);
            ScalerRow r;
            r.event_number = sc.event_number;
            r.ti_ticks     = sc.ti_ticks;
            r.ref_gated    = sc.ref_gated;
            r.ref_ungated  = sc.ref_ungated;
            std::memcpy(r.trg_gated,   sc.trg_gated,   DSC_NCH * sizeof(uint32_t));
            std::memcpy(r.trg_ungated, sc.trg_ungated, DSC_NCH * sizeof(uint32_t));
            std::memcpy(r.tdc_gated,   sc.tdc_gated,   DSC_NCH * sizeof(uint32_t));
            std::memcpy(r.tdc_ungated, sc.tdc_ungated, DSC_NCH * sizeof(uint32_t));
            r.good = good;
            out.push_back(r);
        }
    }
    return true;
}

bool load_epics(const std::vector<std::string> &files,
                std::vector<EpicsRow>           &out,
                bool                            &any_good_col)
{
    for (const auto &path : files) {
        std::unique_ptr<TFile> f(TFile::Open(path.c_str(), "READ"));
        if (!f || f->IsZombie()) {
            std::cerr << "live_charge: cannot open " << path << "\n";
            return false;
        }
        TTree *t = dynamic_cast<TTree *>(f->Get("epics"));
        if (!t) continue;

        prad2::RawEpicsData ep;
        std::vector<std::string> *cp = &ep.channel;
        std::vector<double>      *vp = &ep.value;
        t->SetBranchAddress("event_number_at_arrival", &ep.event_number_at_arrival);
        const bool has_ticks =
            (t->GetBranch("ti_ticks_at_arrival") != nullptr);
        if (has_ticks)
            t->SetBranchAddress("ti_ticks_at_arrival", &ep.ti_ticks_at_arrival);
        t->SetBranchAddress("channel", &cp);
        t->SetBranchAddress("value",   &vp);
        bool good = true;
        const bool has_good = (t->GetBranch("good") != nullptr);
        any_good_col = any_good_col || has_good;
        if (has_good) t->SetBranchAddress("good", &good);

        Long64_t n = t->GetEntries();
        out.reserve(out.size() + n);
        for (Long64_t i = 0; i < n; ++i) {
            ep.ti_ticks_at_arrival = 0;
            good = true;
            t->GetEntry(i);
            EpicsRow r;
            r.event_number = ep.event_number_at_arrival;
            r.ti_ticks     = has_ticks
                ? static_cast<int64_t>(ep.ti_ticks_at_arrival) : 0;
            r.good         = good;
            const size_t k_max = std::min(ep.channel.size(), ep.value.size());
            for (size_t k = 0; k < k_max; ++k)
                r.updates[ep.channel[k]] = ep.value[k];
            out.push_back(std::move(r));
        }
    }
    return true;
}

// ── Delta-livetime + integration ────────────────────────────────────────────

inline std::pair<uint32_t, uint32_t>
select_pair(const ScalerRow &r, const std::string &source, int channel)
{
    if (source == "ref") return {r.ref_gated, r.ref_ungated};
    const int c = std::clamp(channel, 0, DSC_NCH - 1);
    if (source == "trg") return {r.trg_gated[c], r.trg_ungated[c]};
    if (source == "tdc") return {r.tdc_gated[c], r.tdc_ungated[c]};
    return {0, 0};
}

template <class T>
std::vector<size_t> sort_by_event(const std::vector<T> &v)
{
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(),
              [&](size_t a, size_t b) { return v[a].event_number < v[b].event_number; });
    return idx;
}

// One full integration pass.  Returns the live-charge sum and side info.
struct ChargeResult {
    double  value_nC          = 0.0;
    double  live_seconds      = 0.0;
    int64_t n_pairs_total     = 0;     // adjacent (i, i+1) pairs walked
    int64_t n_pairs_kept      = 0;     // both endpoints had good=true
    int64_t n_pairs_integrated = 0;    // contributed to Q
    int64_t n_pairs_skipped   = 0;     // kept but missing data on either side
    bool    any_good_col      = false; // input carried the `good` bool
};

ChargeResult integrate(const std::vector<ScalerRow> &scalers,
                       const std::vector<EpicsRow>  &epics_rows,
                       bool any_good_col,
                       const std::string &source, int channel,
                       const std::string &beam_current_channel)
{
    ChargeResult r;
    r.any_good_col = any_good_col;

    // 1. delta livetime per scaler row, indexed by load order.
    auto sc_order = sort_by_event(scalers);
    std::vector<double> delta_lt(scalers.size(), -1.0);   // fraction in [0, 1]
    {
        uint32_t pg = 0, pu = 0;
        for (size_t k = 0; k < sc_order.size(); ++k) {
            const size_t orig = sc_order[k];
            const auto [g, u] = select_pair(scalers[orig], source, channel);
            if (g < pg || u < pu) { pg = pu = 0; }   // counter rebase
            const uint32_t dg = g - pg;
            const uint32_t du = u - pu;
            if (du > 0 && dg <= du)
                delta_lt[orig] = double(dg) / double(du);
            pg = g; pu = u;
        }
    }

    // 2. merged-timeline pass with forward-fill of livetime + beam current.
    auto ep_order = sort_by_event(epics_rows);
    struct Cp {
        int64_t ticks         = 0;
        double  live_fraction = std::nan("");
        double  beam_current  = std::nan("");
        bool    good          = true;
    };
    std::vector<Cp> tl;
    tl.reserve(scalers.size() + epics_rows.size());

    double cur_lt = std::nan("");
    double cur_I  = std::nan("");
    size_t i = 0, j = 0;
    while (i < sc_order.size() || j < ep_order.size()) {
        const bool take_sc =
            (i < sc_order.size()) &&
            (j >= ep_order.size() ||
             scalers[sc_order[i]].event_number <=
             epics_rows[ep_order[j]].event_number);
        Cp cp;
        if (take_sc) {
            const size_t orig = sc_order[i++];
            const auto &s = scalers[orig];
            cp.ticks = s.ti_ticks;
            const double v = delta_lt[orig];
            if (v >= 0.0) cur_lt = v;
            cp.live_fraction = cur_lt;
            cp.beam_current  = cur_I;
            cp.good          = s.good;
        } else {
            const size_t orig = ep_order[j++];
            const auto &e = epics_rows[orig];
            cp.ticks = e.ti_ticks;
            for (const auto &kv : e.updates)
                if (kv.first == beam_current_channel) cur_I = kv.second;
            cp.live_fraction = cur_lt;
            cp.beam_current  = cur_I;
            cp.good          = e.good;
        }
        tl.push_back(cp);
    }

    // 3. integrate over passing-passing pairs (or every pair when the input
    //    has no `good` column — raw/recon outputs).
    for (size_t k = 1; k < tl.size(); ++k) {
        ++r.n_pairs_total;
        const auto &a = tl[k - 1];
        const auto &b = tl[k];
        if (any_good_col && !(a.good && b.good)) continue;
        ++r.n_pairs_kept;
        if (a.ticks <= 0 || b.ticks <= 0 || b.ticks <= a.ticks
            || !std::isfinite(b.live_fraction) || b.live_fraction < 0
            || !std::isfinite(a.beam_current)  || !std::isfinite(b.beam_current)) {
            ++r.n_pairs_skipped;
            continue;
        }
        const double dt = (b.ticks - a.ticks) * TI_TICK_SEC;
        const double I  = 0.5 * (a.beam_current + b.beam_current);
        r.value_nC     += b.live_fraction * dt * I;
        r.live_seconds += b.live_fraction * dt;
        ++r.n_pairs_integrated;
    }
    return r;
}

// ── CLI ─────────────────────────────────────────────────────────────────────

void print_usage(const char *argv0)
{
    std::cerr <<
"usage: " << argv0 << " <input.root> [more.root ...]\n"
"        [-c|--beam-current CHAN]   EPICS channel (default hallb_IPM2C21A_CUR)\n"
"        [-s|--source ref|trg|tdc]  livetime DSC2 source (default ref)\n"
"        [-n|--channel N]           DSC2 channel for trg/tdc (default 0)\n"
"        [-j|--json PATH]           also write a JSON summary to PATH\n"
"        [-h|--help]\n"
"\n"
"Reads the `scalers` and `epics` side trees from one or more replayed\n"
"ROOT files and integrates Σ live_fraction · Δt · ½(Iₐ + I_b) over\n"
"adjacent slow-event checkpoints.  When the trees carry the per-row\n"
"`good` bool that replay_filter writes, only passing-passing pairs\n"
"contribute (post-cut live charge); otherwise every adjacent pair\n"
"contributes (total live charge over the run).  Beam current is assumed\n"
"to publish in nA, so the result is reported in nC.\n";
}

} // namespace

int main(int argc, char **argv)
{
    std::string beam_current = "hallb_IPM2C21A_CUR";
    std::string source       = "ref";
    int         channel      = 0;
    std::string json_path;

    static struct option long_opts[] = {
        {"beam-current", required_argument, nullptr, 'c'},
        {"source",       required_argument, nullptr, 's'},
        {"channel",      required_argument, nullptr, 'n'},
        {"json",         required_argument, nullptr, 'j'},
        {"help",         no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0},
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "c:s:n:j:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 'c': beam_current = optarg;        break;
        case 's': source       = optarg;        break;
        case 'n': channel      = std::atoi(optarg); break;
        case 'j': json_path    = optarg;        break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 2;
        }
    }
    std::vector<std::string> inputs;
    for (int i = optind; i < argc; ++i) inputs.emplace_back(argv[i]);
    if (inputs.empty()) { print_usage(argv[0]); return 2; }

    std::vector<ScalerRow> scalers;
    std::vector<EpicsRow>  epics_rows;
    bool any_good_sc = false, any_good_ep = false;
    if (!load_scalers(inputs, scalers,    any_good_sc)) return 1;
    if (!load_epics  (inputs, epics_rows, any_good_ep)) return 1;
    const bool any_good_col = any_good_sc || any_good_ep;

    if (scalers.empty()) {
        std::cerr << "live_charge: no scaler rows in input — cannot compute "
                     "livetime.\n";
        return 1;
    }
    if (epics_rows.empty()) {
        std::cerr << "live_charge: no EPICS rows in input — cannot read "
                     "beam current.\n";
        return 1;
    }

    const auto r = integrate(scalers, epics_rows, any_good_col,
                             source, channel, beam_current);

    std::cout
        << "live_charge: " << r.value_nC << " nC\n"
        << "  live time              : " << r.live_seconds << " s\n"
        << "  ⟨I⟩                    : "
        << (r.live_seconds > 0 ? r.value_nC / r.live_seconds : 0.0) << " nA\n"
        << "  beam current channel   : " << beam_current << "\n"
        << "  livetime source        : " << source
        << (source == "ref" ? "" : (" ch " + std::to_string(channel))) << "\n"
        << "  scaler rows            : " << scalers.size() << "\n"
        << "  EPICS rows             : " << epics_rows.size() << "\n"
        << "  adjacent pairs (total) : " << r.n_pairs_total << "\n"
        << "  pairs kept             : " << r.n_pairs_kept
        << (any_good_col ? " (good=true on both)" : " (no `good` column — every pair kept)")
        << "\n"
        << "  pairs integrated       : " << r.n_pairs_integrated << "\n"
        << "  pairs skipped          : " << r.n_pairs_skipped
        << " (missing tick / livetime / current on either side)\n";

    if (!json_path.empty()) {
        json j = {
            {"value_nC",             r.value_nC},
            {"unit",                 "nC"},
            {"beam_current_channel", beam_current},
            {"beam_current_unit",    "nA"},
            {"livetime_source",      source},
            {"livetime_channel",     channel},
            {"live_seconds",         r.live_seconds},
            {"average_current_nA",   r.live_seconds > 0
                                     ? r.value_nC / r.live_seconds : 0.0},
            {"n_scaler_rows",        scalers.size()},
            {"n_epics_rows",         epics_rows.size()},
            {"n_pairs_total",        r.n_pairs_total},
            {"n_pairs_kept",         r.n_pairs_kept},
            {"n_pairs_integrated",   r.n_pairs_integrated},
            {"n_pairs_skipped",      r.n_pairs_skipped},
            {"good_column_present",  any_good_col},
            {"input_files",          inputs},
        };
        std::ofstream of(json_path);
        if (!of) {
            std::cerr << "live_charge: cannot write " << json_path << "\n";
            return 1;
        }
        of << j.dump(2) << "\n";
    }
    return 0;
}
