#!/usr/bin/env python3
"""
plot_wave_deconv.py — visualisations for the prad2dec pile-up deconvolver.

Calls `fdec::WaveAnalyzer::Deconvolve` through the prad2py bindings on
exactly the same synthetic 3-pulse pile-up trace used by
`plot_wave_analysis.py:fig7`, plus (optionally) a real piled-up event
pulled from an EVIO file via `EvChannel`.  No part of the algorithm is
re-implemented in Python; the only formula evaluated here is the bare
two-tau pulse model `T(t) = (1−exp(−t/τ_r))·exp(−t/τ_f)` used to draw
the synthetic input and the deconv-output overlay.  The C++ NNLS / LM
solver is the authoritative source of every plotted amplitude.

Outputs:
  plots/fig8_deconv_synth.png   — synthetic 3-pulse pile-up + deconv result
  plots/fig9_deconv_real.png    — real EVIO piled-up event (when available)

Run:
  cd docs/technical_notes/waveform_analysis
  python scripts/plot_wave_deconv.py [--evio /path/to/run.evio.00000]

`--evio` is optional.  When omitted the real-event figure is skipped;
when given the script scans the file for the first piled-up channel
that has a usable per-type template (PbGlass / PbWO4 / LMS / Veto from
the daq_config-pointed JSON) and converges under the LM gates.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import prad2py.dec as dec     # actual C++ analyzers via pybind11


SCRIPT_DIR = Path(__file__).resolve().parent
OUT_DIR    = SCRIPT_DIR.parent / "plots"
OUT_DIR.mkdir(parents=True, exist_ok=True)

CLK_NS = 4.0


# ---------------------------------------------------------------------------
# Synthetic 3-pulse pile-up — same generator as plot_wave_analysis.py:fig7
# ---------------------------------------------------------------------------
def _pbwo_pulse(t0, height, n, rise_tau=2.5, decay_tau=12.0):
    """PbWO₄-like analytic pulse used to seed the synthetic pile-up.
    Identical to the helper in plot_wave_analysis.py so fig7 and fig8
    operate on the same input trace."""
    out = np.zeros(n)
    rise_done = height * (1 - np.exp(-6 / rise_tau))
    for i in range(n):
        if i < t0:
            continue
        if i - t0 < 6:
            out[i] = height * (1 - np.exp(-(i - t0) / rise_tau))
        else:
            out[i] = rise_done * np.exp(-(i - t0 - 6) / decay_tau)
    return out


def make_synth_trace():
    """Re-create fig7's 3-pulse PbWO₄-style pile-up.  Returns
    (samples_uint16, baseline_adc, true_inputs[(t0_sample, height), ...])."""
    np.random.seed(13)
    n = 80
    ped = 146.0
    inputs = [(20, 800.0), (35, 600.0), (50, 350.0)]
    crowd = ped + np.random.normal(0, 0.4, n)
    for t0, h in inputs:
        crowd += _pbwo_pulse(t0, h, n)
    return np.round(crowd).astype(np.uint16), ped, inputs


# ---------------------------------------------------------------------------
# Render one before/after panel — common to fig8 (synthetic) and fig9 (real)
# ---------------------------------------------------------------------------
PEAK_COLORS = ['#d62728', '#2ca02c', '#9467bd', '#8c564b', '#e377c2']


def render_panel(samples, ped_mean, wres, dec_out, tmpl, title, out_path,
                 truth=None):
    """Plot one channel-event with WA peaks, deconv heights, and the
    deconv model trace.  `truth` (optional) is a list of (peak_time_ns,
    true_height) used only to annotate the synthetic case."""
    n = samples.size
    t_ns = np.arange(n) * CLK_NS
    pedsub = samples.astype(np.float64) - ped_mean

    fig, ax = plt.subplots(figsize=(11, 4.6))

    raw_line, = ax.plot(t_ns, pedsub, 'o-', ms=2.8, lw=0.8, color='#1f77b4',
                        label='raw − ped (uint16 ADC)')
    ax.axhline(0, color='#888', lw=0.6, ls='--')

    # WaveAnalyzer-reported peaks: triangles at the local-maxima heights.
    wa_peaks = list(wres.peaks)
    for i, pk in enumerate(wa_peaks):
        col = PEAK_COLORS[i % len(PEAK_COLORS)]
        ax.plot([pk.time], [pk.height], 'v', color=col, ms=10, mfc='white',
                mec=col, mew=1.6)
        ax.axvline(pk.time, color=col, ls=':', lw=0.7, alpha=0.5)

    # Deconv result: per-peak amplitude, t0, τ_r, τ_f all from the
    # DeconvOutput.  We evaluate the same two-tau form the C++ solver
    # uses (T(t)=(1−exp(−t/τ_r))·exp(−t/τ_f)) only to draw the model
    # curve for visual comparison — the per-peak numbers are the
    # solver's output, not a Python re-fit.
    npk     = dec_out.n
    amps    = list(dec_out.amplitude)
    heights = list(dec_out.height)
    t0s     = list(dec_out.t0_ns)
    tau_rs  = list(dec_out.tau_r_ns)
    tau_fs  = list(dec_out.tau_f_ns)

    for i in range(npk):
        col = PEAK_COLORS[i % len(PEAK_COLORS)]
        # Deconv-recovered height — anchored to the WA peak time so the
        # before/after delta is read off the same x-coordinate.
        ax.plot([wa_peaks[i].time], [heights[i]], 'o',
                color=col, ms=8, mec='black', mew=0.6)

    # Σ a_k · T(t-t0_k; τ_r_k, τ_f_k) on a dense grid.
    t_dense = np.linspace(0.0, float(t_ns[-1]), 4 * n)
    model = np.zeros_like(t_dense)
    for ak, t0, tr, tf in zip(amps, t0s, tau_rs, tau_fs):
        m = t_dense > t0
        if m.any():
            dt = t_dense[m] - t0
            model[m] += ak * (1.0 - np.exp(-dt / tr)) * np.exp(-dt / tf)
    model_line, = ax.plot(t_dense, model, color='#ff7f0e', lw=1.6, alpha=0.85,
                          label=(f'Σ a_k · T(t-t0_k)  (init '
                                 f'τ_r={float(tmpl.tau_r_ns):.1f}, '
                                 f'τ_f={float(tmpl.tau_f_ns):.1f} ns; '
                                 f'χ²/dof={dec_out.chi2_per_dof:.2f})'))

    # Optional ground-truth markers (synthetic only).
    truth_handle = None
    if truth is not None:
        for tk_sample, h_true in truth:
            ax.plot([tk_sample * CLK_NS], [h_true], '+', color='black',
                    ms=14, mew=2.0)
        from matplotlib.lines import Line2D
        truth_handle = Line2D([], [], marker='+', color='black', ls='None',
                              ms=12, mew=2.0,
                              label=f'truth heights (n={len(truth)})')

    from matplotlib.lines import Line2D
    extra = [
        Line2D([], [], marker='v', color='#444', ls='None',
               mfc='white', ms=10, mec='#444', mew=1.6,
               label=f'WA peak heights (n={len(wa_peaks)})'),
        Line2D([], [], marker='o', color='#444', ls='None', ms=8,
               mec='black', mew=0.6,
               label=f'deconv heights (n={npk})'),
    ]
    if truth_handle is not None:
        extra.append(truth_handle)
    ax.legend(handles=[raw_line] + extra + [model_line],
              loc='upper right', fontsize=9, framealpha=0.94)

    ax.set_xlabel('time (ns)')
    ax.set_ylabel('ADC − ped')
    ax.set_title(title)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
# fig8 — synthetic pile-up
# ---------------------------------------------------------------------------
def make_fig8():
    samples, ped, truth = make_synth_trace()

    cfg = dec.WaveConfig()
    ana = dec.WaveAnalyzer(cfg)
    wres = ana.analyze_result(samples)

    # Hand-built template matching the synthesis (rise_tau=2.5 samples,
    # decay_tau=12 samples).  The script doesn't go through the per-type
    # store here because the synthetic data isn't keyed by (roc, slot,
    # channel); the explicit Deconvolve() API takes any PulseTemplate.
    tmpl = dec.PulseTemplate(tau_r_ns=10.0, tau_f_ns=48.0, is_global=False)
    dec_out = ana.deconvolve(samples, wres, tmpl)
    if dec_out.state not in (dec.Q_DECONV_APPLIED, dec.Q_DECONV_FALLBACK_GLOBAL):
        raise SystemExit(
            f"[fig8] deconv did not converge (state={dec_out.state})")

    render_panel(samples, wres.ped.mean, wres, dec_out, tmpl,
                 title=("Synthetic 3-pulse PbWO₄ pile-up — "
                        f"WaveAnalyzer found {wres.npeaks} peaks, "
                        f"deconv recovered {dec_out.n}"),
                 out_path=OUT_DIR / 'fig8_deconv_synth.png',
                 truth=truth)

    # Numeric comparison printed for the doc table.
    print()
    print("=== fig8 — synthetic 3-pulse pile-up ===")
    print(f"  pedestal: {wres.ped.mean:.2f} ± {wres.ped.rms:.2f} ADC")
    print(f"  deconv χ²/dof = {dec_out.chi2_per_dof:.2f}")
    print()
    print("    peak  truth   WA height   deconv height   "
          "Δ(WA-truth)   Δ(deconv-truth)   t0   τ_r   τ_f")
    for i, ((t0_sample, h_true), pk) in enumerate(
            zip(truth, list(wres.peaks))):
        wa_h = pk.height
        d_h  = list(dec_out.height)[i]
        d_t0 = list(dec_out.t0_ns)[i]
        d_tr = list(dec_out.tau_r_ns)[i]
        d_tf = list(dec_out.tau_f_ns)[i]
        print(f"    #{i}    {h_true:6.0f}  {wa_h:9.1f}   "
              f"{d_h:11.1f}    {wa_h-h_true:+10.1f}   "
              f"{d_h-h_true:+13.1f}   {d_t0:5.1f} {d_tr:5.2f} {d_tf:5.2f}")
    print()


# ---------------------------------------------------------------------------
# fig9 — real EVIO piled-up event (optional)
# ---------------------------------------------------------------------------
def find_piled_event(evio_path, daq_cfg_path=None, tmpl_path=None,
                     max_events=2000):
    """Scan an EVIO file and return (samples, wres, dec_out, tmpl,
    crate, slot, ch, event_idx) for the first piled-up channel with a
    usable per-type template + converged deconv.  Raises SystemExit on
    any pre-flight failure (file not found, no template, etc.)."""
    cfg = dec.load_daq_config(daq_cfg_path or "")
    wcfg = dec.WaveConfig(cfg.wave_cfg)

    if tmpl_path is None:
        tmpl_rel = cfg.wave_cfg.nnls_deconv.template_file
        db = os.environ.get("PRAD2_DATABASE_DIR", "database")
        tmpl_path = (tmpl_rel if os.path.isabs(tmpl_rel)
                     else os.path.join(db, tmpl_rel))
    if not Path(tmpl_path).is_file():
        raise SystemExit(f"[fig9] template file not found: {tmpl_path}")

    store = dec.PulseTemplateStore()
    if not store.load_from_file(tmpl_path, wcfg):
        raise SystemExit(f"[fig9] PulseTemplateStore.load_from_file failed")

    # roc tag → crate index for nicer titles.
    roc_to_crate = {}
    for entry in cfg.roc_tags:
        if entry.type in ("roc", "gem") and entry.crate >= 0:
            roc_to_crate[entry.tag] = entry.crate

    ch = dec.EvChannel()
    ch.set_config(cfg)
    if ch.open_auto(evio_path) != dec.Status.success:
        raise SystemExit(f"[fig9] cannot open {evio_path}")

    ana = dec.WaveAnalyzer(wcfg)

    n_events = 0
    while n_events < max_events:
        if ch.read() != dec.Status.success:
            break
        if not ch.scan() or ch.get_event_type() != dec.EventType.Physics:
            continue
        for ei in range(ch.get_n_events()):
            ch.select_event(ei)
            n_events += 1
            fadc = ch.fadc()
            for ri in range(fadc.nrocs):
                roc = fadc.roc(ri)
                crate = roc_to_crate.get(roc.tag, roc.tag)
                for s in roc.present_slots():
                    slot = roc.slot(s)
                    for c in slot.present_channels():
                        cd = slot.channel(c)
                        if cd.nsamples <= 0:
                            continue
                        samples = np.asarray(cd.samples, dtype=np.uint16)
                        wres = ana.analyze_result(samples)
                        peaks = list(wres.peaks)
                        if len(peaks) < 2:
                            continue
                        if not any(pk.quality & dec.Q_PEAK_PILED
                                   for pk in peaks):
                            continue
                        tmpl = store.lookup(roc.tag, s, c)
                        if tmpl is None:
                            continue
                        dec_out = ana.deconvolve(samples, wres, tmpl)
                        if dec_out.state not in (dec.Q_DECONV_APPLIED,
                                                 dec.Q_DECONV_FALLBACK_GLOBAL):
                            continue
                        return (samples, wres, dec_out, tmpl,
                                crate, s, c, n_events)
    raise SystemExit(f"[fig9] no usable piled-up event in first "
                     f"{max_events} physics events of {evio_path}")


def make_fig9(evio_path, daq_cfg_path=None, tmpl_path=None):
    samples, wres, dec_out, tmpl, crate, s, c, n_events = find_piled_event(
        evio_path, daq_cfg_path=daq_cfg_path, tmpl_path=tmpl_path)
    title = (f"Real PbWO₄ pile-up — "
             f"crate {crate} slot {s} ch {c}, event #{n_events}\n"
             f"WaveAnalyzer found {wres.npeaks} peaks, "
             f"deconv recovered {dec_out.n} (χ²/dof={dec_out.chi2_per_dof:.2f})")
    render_panel(samples, wres.ped.mean, wres, dec_out, tmpl,
                 title=title,
                 out_path=OUT_DIR / 'fig9_deconv_real.png')

    print("=== fig9 — real EVIO piled-up event ===")
    print(f"  source : {evio_path}")
    print(f"  channel: crate{crate} slot{s} ch{c}  (event #{n_events})")
    print(f"  template: τ_r={float(tmpl.tau_r_ns):.2f}  "
          f"τ_f={float(tmpl.tau_f_ns):.2f} ns "
          f"({'per-type' if tmpl.is_global else 'per-channel'})")
    print(f"  pedestal: {wres.ped.mean:.2f} ± {wres.ped.rms:.2f} ADC")
    print()
    print("    peak  WA height   deconv height   t0     τ_r    τ_f")
    for i, pk in enumerate(list(wres.peaks)):
        d_h  = list(dec_out.height)[i]
        d_t0 = list(dec_out.t0_ns)[i]
        d_tr = list(dec_out.tau_r_ns)[i]
        d_tf = list(dec_out.tau_f_ns)[i]
        flag = 'PILED' if (pk.quality & dec.Q_PEAK_PILED) else 'isolated'
        print(f"    #{i}   {pk.height:9.1f}    {d_h:11.1f}   "
              f"{d_t0:6.1f} {d_tr:6.2f} {d_tf:6.2f}   {flag}")
    print()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--evio", default="",
                    help="EVIO file for fig9 (real piled event).  Optional.")
    ap.add_argument("--daq-config", default="",
                    help='DAQ config (default "" → installed default).')
    ap.add_argument("--template", default="",
                    help="Override pulse_templates.json path for fig9.")
    args = ap.parse_args()

    make_fig8()

    if args.evio:
        make_fig9(args.evio,
                  daq_cfg_path=args.daq_config or None,
                  tmpl_path=args.template or None)
    else:
        print("(fig9 skipped — pass --evio /path/to/run.evio.00000 to render "
              "the real-data panel)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
