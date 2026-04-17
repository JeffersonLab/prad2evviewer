//============================================================================
// tagger_hycal_correlation.C — tagger TDC → HyCal coincidence study
//
// For each pair (T10R, Eₓ), x ∈ {49, 50, 51, 52, 53}:
//
//   Step 1  Build the event-wise TDC difference histogram
//              ΔT = tdc(T10R) − tdc(Eₓ)
//           and fit a Gaussian around the dominant peak to extract (μ, σ).
//
//   Step 2  Apply a ±Nσ timing cut (default N=3) and, for events passing
//           the cut, fill the W1156 (HyCal, ROC 0x8C slot 7 ch 3) FADC
//           peak-height and peak-integral histograms.
//
// A simple pedestal-and-max peak finder is used for W1156: pedestal is
// the mean of the first 10 samples, height is (max − ped), integral is
// the sum of (sample − ped) over ±8 bins around the max.  Swap in
// fdec::WaveAnalyzer if calibrated output is needed.
//
// Compile with ACLiC after loading rootlogon:
//
//     cd build
//     root -l ../analysis/scripts/rootlogon.C
//     .x ../analysis/scripts/tagger_hycal_correlation.C+( \
//        "/data/stage6/prad_023671/prad_023671.evio.00000", \
//        "tagger_w1156_corr.root", \
//        500000)
//
// Or one-liner:
//
//     root -l -b -q analysis/scripts/rootlogon.C \
//         'analysis/scripts/tagger_hycal_correlation.C+("path.evio","out.root",0)'
//============================================================================

#include "EvChannel.h"
#include "DaqConfig.h"
#include "load_daq_config.h"
#include "Fadc250Data.h"
#include "SspData.h"
#include "VtpData.h"
#include "TdcData.h"

#include <TCanvas.h>
#include <TF1.h>
#include <TFile.h>
#include <TH1D.h>
#include <TH1F.h>
#include <TString.h>
#include <TStyle.h>
#include <TSystem.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace evc;

//-----------------------------------------------------------------------------
// Channel layout (update if the DAQ map changes)
//-----------------------------------------------------------------------------
namespace {

constexpr int TAGGER_SLOT = 18;
constexpr int T10R_CH     = 0;

struct EChan { const char *name; int channel; };
constexpr EChan E_CHANNELS[] = {
    {"E49", 11}, {"E50", 12}, {"E51", 13}, {"E52", 14}, {"E53", 15},
};
constexpr int N_E = sizeof(E_CHANNELS) / sizeof(E_CHANNELS[0]);

constexpr uint32_t W1156_ROC     = 0x8C;
constexpr int      W1156_SLOT    = 7;
constexpr int      W1156_CH      = 3;

constexpr int PED_WINDOW    = 10;
constexpr int INT_HALFWIDTH = 8;

// ΔT histogram defaults (in TDC LSB units, ≈ 25 ps after rol2 shift).
constexpr int    DT_BINS  = 400;
constexpr double DT_RANGE = 200.0;

// W1156 output histograms.
constexpr int    H_BINS  = 200;
constexpr double H_MIN   = 0.0;
constexpr double H_MAX   = 4000.0;
constexpr int    I_BINS  = 200;
constexpr double I_MIN   = 0.0;
constexpr double I_MAX   = 40000.0;

//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------

// Earliest hit (smallest TDC value) for (slot, ch) in this event, or -1.
static int first_tdc(const tdc::TdcEventData &t, int slot, int ch)
{
    int best = -1;
    for (int i = 0; i < t.n_hits; ++i) {
        const auto &h = t.hits[i];
        if ((int)h.slot != slot || (int)h.channel != ch) continue;
        if (best < 0 || (int)h.value < best) best = (int)h.value;
    }
    return best;
}

// Simple FADC peak finder. Returns false if the channel has no samples.
static bool hycal_peak(const fdec::ChannelData &c, float &height, float &integral)
{
    if (c.nsamples <= PED_WINDOW) return false;
    double ped = 0.0;
    for (int i = 0; i < PED_WINDOW; ++i) ped += c.samples[i];
    ped /= PED_WINDOW;

    int tmax = 0;
    double maxv = (double)c.samples[0] - ped;
    for (int i = 1; i < c.nsamples; ++i) {
        double v = (double)c.samples[i] - ped;
        if (v > maxv) { maxv = v; tmax = i; }
    }
    height = (float)maxv;

    int lo = std::max(0, tmax - INT_HALFWIDTH);
    int hi = std::min<int>(c.nsamples, tmax + INT_HALFWIDTH + 1);
    double sum = 0.0;
    for (int i = lo; i < hi; ++i) sum += (double)c.samples[i] - ped;
    integral = (float)sum;
    return true;
}

// Pull W1156 peak (height, integral). Returns false if absent.
static bool w1156_peak(const fdec::EventData &evt,
                       float &height, float &integral)
{
    const fdec::RocData *roc = evt.findRoc(W1156_ROC);
    if (!roc) return false;
    const fdec::SlotData &s = roc->slots[W1156_SLOT];
    if (!s.present) return false;
    const fdec::ChannelData &c = s.channels[W1156_CH];
    if (c.nsamples <= 0) return false;
    return hycal_peak(c, height, integral);
}

//-----------------------------------------------------------------------------
// Per-event record accumulated during the single pass over the file.
//-----------------------------------------------------------------------------
struct Row {
    int   t10r;                 // TDC value of T10R, always set
    int   e[N_E];               // TDC values of E49..E53, -1 if missing
    float height;               // W1156 peak height
    float integral;             // W1156 peak integral
};

} // anonymous namespace

//=============================================================================
// Entry point (the symbol ACLiC exports to the ROOT command line)
//=============================================================================

int tagger_hycal_correlation(const char *evio_path,
                             const char *out_path   = "tagger_w1156_corr.root",
                             Long64_t    max_events = 0,
                             const char *daq_config = nullptr,
                             double      nsigma     = 3.0)
{
    //---- load DAQ config ----------------------------------------------------
    std::string cfg_path = daq_config ? daq_config : "";
    if (cfg_path.empty()) {
        const char *db = std::getenv("PRAD2_DATABASE_DIR");
        cfg_path = std::string(db ? db : "database") + "/daq_config.json";
    }

    DaqConfig cfg;
    if (!load_daq_config(cfg_path, cfg)) {
        std::cerr << "ERROR: cannot load " << cfg_path << "\n";
        return 1;
    }

    //---- open evio ----------------------------------------------------------
    EvChannel ch;
    ch.SetConfig(cfg);
    if (ch.Open(evio_path) != status::success) {
        std::cerr << "ERROR: cannot open " << evio_path << "\n";
        return 1;
    }
    std::cout << "reading " << evio_path << std::endl;

    //---- pass 1: collect per-event tuples into memory -----------------------
    auto event_ptr = std::make_unique<fdec::EventData>();
    auto tdc_ptr   = std::make_unique<tdc::TdcEventData>();
    auto &event   = *event_ptr;
    auto &tdc_evt = *tdc_ptr;

    std::vector<Row> rows;
    rows.reserve(1u << 20);

    Long64_t n_accepted = 0;
    while (ch.Read() == status::success) {
        if (!ch.Scan() || ch.GetEventType() != EventType::Physics) continue;

        int nsub = ch.GetNEvents();
        for (int i = 0; i < nsub; ++i) {
            if (!ch.DecodeEvent(i, event, nullptr, nullptr, &tdc_evt)) continue;

            int t0 = first_tdc(tdc_evt, TAGGER_SLOT, T10R_CH);
            if (t0 < 0) continue;

            Row r;
            r.t10r = t0;
            bool any_e = false;
            for (int k = 0; k < N_E; ++k) {
                r.e[k] = first_tdc(tdc_evt, TAGGER_SLOT, E_CHANNELS[k].channel);
                if (r.e[k] >= 0) any_e = true;
            }
            if (!any_e) continue;
            if (!w1156_peak(event, r.height, r.integral)) continue;

            rows.push_back(r);
            ++n_accepted;
            if (n_accepted % 100000 == 0)
                std::cout << "  pass 1: " << n_accepted << " events collected\n";
            if (max_events > 0 && n_accepted >= max_events) goto done;
        }
    }
done:
    ch.Close();
    std::cout << "pass 1 done: " << n_accepted << " events accepted\n";
    if (rows.empty()) {
        std::cerr << "no events survived initial filter — nothing to plot\n";
        return 1;
    }

    //---- pass 2: build + fit ΔT histograms, cut, fill W1156 -----------------
    TFile out(out_path, "RECREATE");
    gStyle->SetOptFit(1);
    gStyle->SetOptStat(1110);

    TCanvas *canvas = new TCanvas("summary", "tagger-W1156 correlations", 1500, 900);
    canvas->Divide(N_E, 3);

    struct Result { double mu, sigma; Long64_t n_total, n_sel; };
    std::vector<Result> results(N_E);

    // Reusable histograms created per pair.
    for (int k = 0; k < N_E; ++k) {
        const char *ename = E_CHANNELS[k].name;

        TH1D *hdt = new TH1D(
            TString::Format("dt_T10R_%s", ename),
            TString::Format("#DeltaT = T10R - %s;"
                            "tdc(T10R) - tdc(%s) [LSB];events", ename, ename),
            DT_BINS, -DT_RANGE, DT_RANGE);

        Long64_t n_total = 0;
        for (const auto &r : rows) {
            if (r.e[k] < 0) continue;
            hdt->Fill((double)r.t10r - (double)r.e[k]);
            ++n_total;
        }

        // fit Gaussian around largest bin
        int peak_bin = hdt->GetMaximumBin();
        double peak_x = hdt->GetXaxis()->GetBinCenter(peak_bin);
        double bw     = hdt->GetXaxis()->GetBinWidth(1);
        TF1 *gfit = new TF1(TString::Format("gfit_%s", ename), "gaus",
                            peak_x - 20.0 * bw, peak_x + 20.0 * bw);
        hdt->Fit(gfit, "RQ", "", peak_x - 20.0 * bw, peak_x + 20.0 * bw);
        double mu    = gfit->GetParameter(1);
        double sigma = std::max(std::fabs(gfit->GetParameter(2)), bw);

        hdt->Write();

        // ---- apply timing cut and fill W1156 histograms --------------------
        TH1F *h_height = new TH1F(
            TString::Format("W1156_height_%s", ename),
            TString::Format("W1156 peak height, "
                            "|#DeltaT - %.1f| < %.1f#sigma (T10R-%s);"
                            "height [ADC];events", mu, nsigma, ename),
            H_BINS, H_MIN, H_MAX);
        TH1F *h_integ  = new TH1F(
            TString::Format("W1156_integral_%s", ename),
            TString::Format("W1156 peak integral, "
                            "|#DeltaT - %.1f| < %.1f#sigma (T10R-%s);"
                            "integral [ADC#upoint sample];events", mu, nsigma, ename),
            I_BINS, I_MIN, I_MAX);

        Long64_t n_sel = 0;
        const double half = nsigma * sigma;
        for (const auto &r : rows) {
            if (r.e[k] < 0) continue;
            double dt = (double)r.t10r - (double)r.e[k];
            if (std::fabs(dt - mu) >= half) continue;
            h_height->Fill(r.height);
            h_integ->Fill(r.integral);
            ++n_sel;
        }
        h_height->Write();
        h_integ->Write();

        results[k] = {mu, sigma, n_total, n_sel};

        // summary canvas
        canvas->cd(k + 1);
        hdt->Draw();
        canvas->cd(k + 1 + N_E);
        h_height->Draw();
        canvas->cd(k + 1 + 2 * N_E);
        h_integ->Draw();
    }

    canvas->Write();
    out.Close();

    //---- terminal summary --------------------------------------------------
    std::cout << "\n=== Summary ===\n"
              << "   pair     mu[LSB]  sigma[LSB]    n_total   n_selected   keep\n";
    for (int k = 0; k < N_E; ++k) {
        const auto &r = results[k];
        double frac = r.n_total ? 100.0 * (double)r.n_sel / r.n_total : 0.0;
        std::cout << "  T10R-" << E_CHANNELS[k].name
                  << "  " << std::setw(8) << r.mu
                  << "   " << std::setw(8) << r.sigma
                  << "   " << std::setw(9) << r.n_total
                  << "    " << std::setw(9) << r.n_sel
                  << "   " << std::setw(5) << frac << "%\n";
    }
    std::cout << "\nhistograms written to " << out_path << "\n";
    return 0;
}
