#include "GemPedestal.h"
#include "SspData.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

namespace gem {

// --- per-strip running mean / RMS --------------------------------------------
namespace {

struct StripAccum {
    double sum   = 0.;
    double sum2  = 0.;
    int    count = 0;

    void   add(double v) { sum += v; sum2 += v * v; ++count; }
    double mean()  const { return count > 0 ? sum / count : 0.; }
    double rms()   const {
        if (count < 2) return 0.;
        double m = mean();
        double var = sum2 / count - m * m;
        return var > 0 ? std::sqrt(var) : 0.;
    }
};

// Pack (crate, mpd, apv, strip) → uint64 key — fits in a flat std::map without
// needing a custom hash for a 4-tuple.
inline uint64_t packKey(int crate, int mpd, int apv, int strip)
{
    return (static_cast<uint64_t>(crate & 0xFFFF) << 48) |
           (static_cast<uint64_t>(mpd   & 0xFFFF) << 32) |
           (static_cast<uint64_t>(apv   & 0xFFFF) << 16) |
            static_cast<uint64_t>(strip & 0xFFFF);
}

// Drop top/bottom N of 128 when computing the per-time-sample common mode.
constexpr int CM_DISCARD = 28;

}   // namespace

// --- Impl (pImpl) ------------------------------------------------------------
struct GemPedestal::Impl {
    std::map<uint64_t, StripAccum> accum;
};

GemPedestal::GemPedestal() : impl_(std::make_unique<Impl>()) {}
GemPedestal::~GemPedestal() = default;

void GemPedestal::Clear() { impl_->accum.clear(); }

int GemPedestal::NumStrips() const { return static_cast<int>(impl_->accum.size()); }

int GemPedestal::NumApvs() const
{
    std::map<uint64_t, int> apvs;
    for (const auto &[key, _] : impl_->accum) {
        uint64_t akey = (key >> 16);   // strip bits into the low half
        apvs[akey] += 1;
    }
    return static_cast<int>(apvs.size());
}

// --- accumulate --------------------------------------------------------------
void GemPedestal::Accumulate(const ssp::SspEventData &evt)
{
    float  sorted[ssp::APV_STRIP_SIZE];
    float  cm_corrected[ssp::APV_STRIP_SIZE][ssp::SSP_TIME_SAMPLES];

    for (int m = 0; m < evt.nmpds; ++m) {
        const auto &mpd = evt.mpds[m];
        if (!mpd.present) continue;

        for (int a = 0; a < ssp::MAX_APVS_PER_MPD; ++a) {
            const auto &apv = mpd.apvs[a];
            if (!apv.present || apv.nstrips == 0) continue;

            // Per-time-sample common-mode subtraction.
            for (int t = 0; t < ssp::SSP_TIME_SAMPLES; ++t) {
                for (int s = 0; s < ssp::APV_STRIP_SIZE; ++s)
                    sorted[s] = apv.hasStrip(s) ? float(apv.strips[s][t]) : 0.f;

                std::sort(sorted, sorted + ssp::APV_STRIP_SIZE);

                double cm_sum  = 0.;
                int    cm_count = ssp::APV_STRIP_SIZE - 2 * CM_DISCARD;
                for (int s = CM_DISCARD; s < ssp::APV_STRIP_SIZE - CM_DISCARD; ++s)
                    cm_sum += sorted[s];
                const double cm = cm_sum / cm_count;

                for (int s = 0; s < ssp::APV_STRIP_SIZE; ++s)
                    cm_corrected[s][t] = (apv.hasStrip(s) ? float(apv.strips[s][t]) : 0.f) - cm;
            }

            // Per-strip average over the 6 time samples, then accumulate.
            for (int s = 0; s < ssp::APV_STRIP_SIZE; ++s) {
                if (!apv.hasStrip(s)) continue;
                double avg = 0.;
                for (int t = 0; t < ssp::SSP_TIME_SAMPLES; ++t)
                    avg += cm_corrected[s][t];
                avg /= ssp::SSP_TIME_SAMPLES;

                impl_->accum[packKey(mpd.crate_id, mpd.mpd_id, a, s)].add(avg);
            }
        }
    }
}

// --- write JSON --------------------------------------------------------------
int GemPedestal::Write(const std::string &output_path) const
{
    std::ofstream of(output_path);
    if (!of.is_open()) {
        std::cerr << "GemPedestal::Write: cannot write " << output_path << "\n";
        return -1;
    }

    // Group strips by (crate, mpd, adc) for the one-entry-per-APV output.
    struct ApvKey { int crate, mpd, adc; };
    std::map<uint64_t, ApvKey>                                                apv_keys;
    std::map<uint64_t, std::vector<std::pair<int, const StripAccum*>>>        apv_strips;

    for (const auto &[key, acc] : impl_->accum) {
        const int crate = (key >> 48) & 0xFFFF;
        const int mpd   = (key >> 32) & 0xFFFF;
        const int apv   = (key >> 16) & 0xFFFF;
        const int strip =  key        & 0xFFFF;
        const uint64_t akey = (static_cast<uint64_t>(crate) << 32) |
                              (static_cast<uint64_t>(mpd)   << 16) |
                               static_cast<uint64_t>(apv);
        apv_keys[akey] = {crate, mpd, apv};
        apv_strips[akey].emplace_back(strip, &acc);
    }

    nlohmann::json arr = nlohmann::json::array();
    for (auto &[akey, strips] : apv_strips) {
        const auto &ak = apv_keys[akey];
        std::sort(strips.begin(), strips.end());

        nlohmann::json offsets = nlohmann::json::array();
        nlohmann::json noises  = nlohmann::json::array();
        for (const auto &[s, acc] : strips) {
            offsets.push_back(std::round(acc->mean() *  1000.) /  1000.);
            noises .push_back(std::round(acc->rms()  * 10000.) / 10000.);
        }
        arr.push_back({
            {"crate",  ak.crate}, {"mpd", ak.mpd}, {"adc", ak.adc},
            {"offset", offsets},  {"noise", noises}
        });
    }

    of << arr.dump(2) << "\n";
    return static_cast<int>(apv_strips.size());
}

} // namespace gem
