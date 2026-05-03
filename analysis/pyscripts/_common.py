#!/usr/bin/env python3
"""
_common.py — shared helpers for analysis/pyscripts/.

Both gem_hycal_matching.py and plot_hits_at_hycal.py do the same setup
(load DAQ config, runinfo, HyCal+GEM systems, discover EVIO splits) and
the same per-event boilerplate (waveform → cluster, GEM ProcessEvent +
Reconstruct, lab-frame transform).  This module factors that out so the
two scripts only differ in their per-event accumulation + output.

Mirrors the C++ helpers in:
  analysis/scripts/script_helpers.h     (path / file discovery)
  analysis/include/ConfigSetup.h        (RotateDetData + TransformDetData)

Requires:
  prad2py    (built from python/, exposes dec.* + det.*)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

try:
    from prad2py import dec, det
except ImportError as e:
    raise SystemExit(
        f"[ERROR] cannot import prad2py: {e}\n"
        "        Build the python bindings (cmake -DBUILD_PYTHON=ON) and "
        "ensure the install directory is on PYTHONPATH."
    )


# ============================================================================
# Path / run-number helpers (mirror script_helpers.h)
# ============================================================================

_RUN_PAT = re.compile(r"(?:prad|run)_0*(\d+)", re.IGNORECASE)


def extract_run_number(path: str) -> int:
    """Sniff the run number out of 'prad_NNNNNN.evio.*'-style names. -1 if none."""
    if not path:
        return -1
    m = _RUN_PAT.search(path)
    if not m:
        return -1
    try:
        return int(m.group(1))
    except ValueError:
        return -1


def resolve_db_path(p: str) -> str:
    """Resolve a possibly-relative database path against PRAD2_DATABASE_DIR."""
    if not p:
        return p
    if os.path.isabs(p):
        return p
    db = os.environ.get("PRAD2_DATABASE_DIR")
    if db is None:
        return p
    return os.path.join(db, p)


def discover_runinfo_path() -> Optional[str]:
    """Read database/reconstruction_config.json (under PRAD2_DATABASE_DIR or
    ./database) and return the resolved runinfo path; None if missing/malformed."""
    db = os.environ.get("PRAD2_DATABASE_DIR", "database")
    cfg_path = Path(db) / "reconstruction_config.json"
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            j = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None
    ri = j.get("runinfo")
    if not isinstance(ri, str):
        return None
    return resolve_db_path(ri)


def load_matching_config(
        ) -> tuple[tuple[float, float, float],
                   list[float],
                   tuple[float, float, float]]:
    """Read the 'matching' section from
    database/reconstruction_config.json and return
    ((A, B, C), gem_pos_res, (σ_target_x, σ_target_y, σ_target_z)).
    Missing keys / file fall back to (2.6, 0, 0), [0.1]*4, and
    (1.0, 1.0, 20.0) mm — preserving prior analysis behavior."""
    A, B, C = 2.6, 0.0, 0.0
    gem = [0.1, 0.1, 0.1, 0.1]
    tgt = (1.0, 1.0, 20.0)
    db = os.environ.get("PRAD2_DATABASE_DIR", "database")
    try:
        with open(Path(db) / "reconstruction_config.json", "r", encoding="utf-8") as f:
            j = json.load(f)
    except (OSError, json.JSONDecodeError):
        return (A, B, C), gem, tgt
    m = j.get("matching")
    if not isinstance(m, dict):
        return (A, B, C), gem, tgt
    h = m.get("hycal_pos_res")
    if isinstance(h, list) and len(h) >= 3:
        A, B, C = float(h[0]), float(h[1]), float(h[2])
    g = m.get("gem_pos_res")
    if isinstance(g, list) and g:
        gem = [float(v) for v in g]
    t = m.get("target_pos_res")
    if isinstance(t, list) and len(t) >= 3:
        tgt = (float(t[0]), float(t[1]), float(t[2]))
    return (A, B, C), gem, tgt


def _read_recon_config() -> dict:
    """Parse database/reconstruction_config.json once.  Returns {} on any
    failure (file missing, parse error, etc.) so callers can fall back to
    library defaults silently."""
    db = os.environ.get("PRAD2_DATABASE_DIR", "database")
    try:
        with open(Path(db) / "reconstruction_config.json", "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def make_hycal_cluster_config():
    """Build a det.HyCalClusterConfig from
    reconstruction_config.json:hycal.  Falls back to the C++ struct's
    library defaults for any missing key, so partial JSON works."""
    cfg = det.HyCalClusterConfig()
    j = _read_recon_config().get("hycal")
    if not isinstance(j, dict):
        return cfg
    if "min_module_energy"  in j: cfg.min_module_energy  = float(j["min_module_energy"])
    if "min_center_energy"  in j: cfg.min_center_energy  = float(j["min_center_energy"])
    if "min_cluster_energy" in j: cfg.min_cluster_energy = float(j["min_cluster_energy"])
    if "min_cluster_size"   in j: cfg.min_cluster_size   = int(j["min_cluster_size"])
    if "corner_conn"        in j: cfg.corner_conn        = bool(j["corner_conn"])
    if "split_iter"         in j: cfg.split_iter         = int(j["split_iter"])
    if "least_split"        in j: cfg.least_split        = float(j["least_split"])
    if "log_weight_thres"   in j: cfg.log_weight_thres   = float(j["log_weight_thres"])
    return cfg


def _apply_gem_cluster_overrides(cfg, j: dict) -> None:
    """Mutate a det.ClusterConfig in place from a JSON dict."""
    if "min_cluster_hits"    in j: cfg.min_cluster_hits    = int(j["min_cluster_hits"])
    if "max_cluster_hits"    in j: cfg.max_cluster_hits    = int(j["max_cluster_hits"])
    if "consecutive_thres"   in j: cfg.consecutive_thres   = int(j["consecutive_thres"])
    if "split_thres"         in j: cfg.split_thres         = float(j["split_thres"])
    if "cross_talk_width"    in j: cfg.cross_talk_width    = float(j["cross_talk_width"])
    if "charac_dists"        in j and isinstance(j["charac_dists"], list):
        cfg.charac_dists = [float(v) for v in j["charac_dists"]]
    if "match_mode"          in j: cfg.match_mode          = int(j["match_mode"])
    if "match_adc_asymmetry" in j: cfg.match_adc_asymmetry = float(j["match_adc_asymmetry"])
    if "match_time_diff"     in j: cfg.match_time_diff     = float(j["match_time_diff"])
    if "match_ts_period"     in j: cfg.ts_period           = float(j["match_ts_period"])


def make_gem_cluster_configs(n_dets: int) -> list:
    """Return one det.ClusterConfig per GEM detector.  Starts from
    reconstruction_config.json:gem.default and applies per-detector
    overrides under gem.{0..n_dets-1}.  If the JSON has no gem block we
    return n_dets default-initialized ClusterConfigs."""
    base = _read_recon_config().get("gem")
    out = [det.ClusterConfig() for _ in range(n_dets)]
    if not isinstance(base, dict):
        return out
    if isinstance(base.get("default"), dict):
        for c in out:
            _apply_gem_cluster_overrides(c, base["default"])
    for d in range(n_dets):
        per = base.get(str(d))
        if isinstance(per, dict):
            _apply_gem_cluster_overrides(out[d], per)
    return out


def hycal_pos_resolution(A: float, B: float, C: float, energy_mev: float) -> float:
    """sigma(E) at the HyCal face (mm), mirroring HyCalSystem::PositionResolution."""
    import math
    E_GeV = energy_mev / 1000.0 if energy_mev > 0 else 1e-6
    a = A / math.sqrt(E_GeV)
    b = B / E_GeV
    return math.sqrt(a * a + b * b + C * C)


def discover_split_files(any_path: str) -> list[str]:
    """Three modes by input shape:
      * '*' in path  → literal shell-style glob: expanded by Python's glob
        module so users can pick a subset (e.g. 'prad_024236.evio.0000*'
        gets splits .00000–.00009).  Quote it on the shell to keep the
        shell from expanding it first.
      * directory    → enumerate every prad_<run>.evio.<digits> in the dir,
        sniff run from dir name, warn (stderr) on gaps.
      * anything else → return [any_path] unchanged (single-file mode)."""
    if not any_path:
        return []
    p = Path(any_path)
    wants_glob = "*" in any_path
    is_dir = p.is_dir()

    if not wants_glob and not is_dir:
        return [any_path]

    # ---- glob mode: honor the literal pattern ------------------------------
    if wants_glob:
        import glob as _glob
        matches = sorted(_glob.glob(any_path))
        if not matches:
            sys.stderr.write(
                f"[WARN] discover_split_files: glob {any_path!r} matched "
                f"no files.\n"
            )
            return [any_path]
        return matches

    # ---- directory mode: enumerate by run number ---------------------------
    directory = p
    run = extract_run_number(p.name)

    if run < 0 or not directory.is_dir():
        sys.stderr.write(
            f"[WARN] discover_split_files: cannot resolve run/dir from "
            f"{any_path!r} — passing through as a single file.\n"
        )
        return [any_path]

    pat = re.compile(rf"^prad_0*{run}\.evio\.(\d+)$", re.IGNORECASE)
    matched: list[tuple[int, str]] = []
    for entry in directory.iterdir():
        m = pat.match(entry.name)
        if m:
            try:
                matched.append((int(m.group(1)), str(entry)))
            except ValueError:
                pass
    matched.sort()

    if matched:
        last = matched[-1][0]
        seen = {idx for idx, _ in matched}
        missing = [i for i in range(0, last + 1) if i not in seen]
        if missing:
            miss_str = " ".join(f".{i:05d}" for i in missing)
            sys.stderr.write(
                f"[WARN] split-file gaps in run {run} (found "
                f"{len(matched)} file(s), max suffix .{last:05d}): "
                f"missing {miss_str}\n"
            )

    if not matched:
        sys.stderr.write(
            f"[WARN] discover_split_files: no files matched 'prad_{run}.evio.*' "
            f"in {directory}\n"
        )
        return [any_path]

    return [path for _, path in matched]


# ============================================================================
# Runinfo (geometry + calibration paths)
# ============================================================================

@dataclass
class RunGeometry:
    """Subset of RunConfig the analysis scripts actually use.  Mirrors the
    fields read by ConfigSetup.h's RotateDetData / TransformDetData."""
    run_number:           int   = 0
    beam_energy:          float = 0.0
    target_x:             float = 0.0
    target_y:             float = 0.0
    target_z:             float = 0.0
    hycal_x:              float = 0.0
    hycal_y:              float = 0.0
    hycal_z:              float = 0.0
    hycal_tilt_x:         float = 0.0
    hycal_tilt_y:         float = 0.0
    hycal_tilt_z:         float = 0.0
    gem_x:                list[float] = field(default_factory=lambda: [0.0]*4)
    gem_y:                list[float] = field(default_factory=lambda: [0.0]*4)
    gem_z:                list[float] = field(default_factory=lambda: [0.0]*4)
    gem_tilt_x:           list[float] = field(default_factory=lambda: [0.0]*4)
    gem_tilt_y:           list[float] = field(default_factory=lambda: [0.0]*4)
    gem_tilt_z:           list[float] = field(default_factory=lambda: [0.0]*4)
    energy_calib_file:    str   = ""
    gem_pedestal_file:    str   = ""
    gem_common_mode_file: str   = ""


def load_run_geometry(runinfo_path: str, eff_run: int) -> RunGeometry:
    """Parse a runinfo JSON and pick the entry with the largest run_number ≤
    eff_run (or the largest entry overall if eff_run ≤ 0)."""
    with open(runinfo_path, "r", encoding="utf-8") as f:
        info = json.load(f)
    configs = info.get("configurations", [])
    if not configs:
        raise RuntimeError(f"runinfo {runinfo_path}: no 'configurations' entries")

    if eff_run > 0:
        ordered = sorted(configs, key=lambda c: c.get("run_number", 0))
        cfg = None
        for c in ordered:
            if c.get("run_number", 0) <= eff_run:
                cfg = c
        if cfg is None:
            cfg = ordered[0]
            sys.stderr.write(
                f"[WARN] no runinfo entry for run {eff_run}; using oldest "
                f"(run {cfg.get('run_number', 0)}).\n"
            )
    else:
        cfg = max(configs, key=lambda c: c.get("run_number", 0))
        sys.stderr.write(
            f"[WARN] no run number specified; using largest runinfo entry "
            f"(run {cfg.get('run_number', 0)}).\n"
        )

    g = RunGeometry()
    g.run_number  = int(cfg.get("run_number", 0))
    g.beam_energy = float(cfg.get("beam_energy", 0.0))

    tgt = cfg.get("target", [0.0, 0.0, 0.0])
    if isinstance(tgt, list) and len(tgt) >= 3:
        g.target_x, g.target_y, g.target_z = (float(v) for v in tgt[:3])

    hc = cfg.get("hycal", {})
    pos  = hc.get("position", [0.0, 0.0, 0.0])
    tilt = hc.get("tilting",  [0.0, 0.0, 0.0])
    g.hycal_x, g.hycal_y, g.hycal_z = (float(v) for v in pos)
    g.hycal_tilt_x, g.hycal_tilt_y, g.hycal_tilt_z = (float(v) for v in tilt)

    gem = cfg.get("gem", {})
    g.gem_pedestal_file    = gem.get("pedestal_file", "")
    g.gem_common_mode_file = gem.get("common_mode_file", "")
    for d in gem.get("detectors", []):
        i = int(d.get("id", -1))
        if 0 <= i < 4:
            pos  = d.get("position", [0.0, 0.0, 0.0])
            tilt = d.get("tilting",  [0.0, 0.0, 0.0])
            g.gem_x[i], g.gem_y[i], g.gem_z[i] = (float(v) for v in pos)
            g.gem_tilt_x[i], g.gem_tilt_y[i], g.gem_tilt_z[i] = \
                (float(v) for v in tilt)

    cal = cfg.get("calibration", {})
    g.energy_calib_file = cal.get("file", "")
    return g


# ============================================================================
# Coordinate transforms — use prad2det's DetectorTransform via the binding
# ============================================================================
# Single source of truth for "detector-frame → lab-frame" lives in
# prad2det/include/DetectorTransform.h (and AppState/Replay use it directly).
# build_lab_transforms() materializes the per-detector poses from a
# RunGeometry and stashes them on a Pipeline; per-hit transforms then call
# `xform.to_lab(x, y[, z])` so the rotation matrix is computed once per
# detector instead of per hit.


def build_lab_transforms(geo: RunGeometry
                         ) -> tuple["det.DetectorTransform",
                                    list["det.DetectorTransform"]]:
    """Return (hycal_xform, [gem_xform0, ...]) built from geo.  Each
    DetectorTransform's rotation matrix is precomputed via set()."""
    hycal_xform = det.DetectorTransform()
    hycal_xform.set(geo.hycal_x, geo.hycal_y, geo.hycal_z,
                    geo.hycal_tilt_x, geo.hycal_tilt_y, geo.hycal_tilt_z)
    gem_xforms: list[det.DetectorTransform] = []
    for d in range(4):
        t = det.DetectorTransform()
        t.set(geo.gem_x[d], geo.gem_y[d], geo.gem_z[d],
              geo.gem_tilt_x[d], geo.gem_tilt_y[d], geo.gem_tilt_z[d])
        gem_xforms.append(t)
    return hycal_xform, gem_xforms


def project_to_z(x: float, y: float, z: float, target_z: float
                 ) -> tuple[float, float, float]:
    """Straight-line target→hit projection to z = target_z.  Mirrors
    analysis::GetProjection (single-vertex assumption).  z must be > 0."""
    if z == 0.0:
        return x, y, target_z
    s = target_z / z
    return x * s, y * s, target_z


# ============================================================================
# Pipeline state + setup
# ============================================================================

@dataclass
class Pipeline:
    """Initialized HyCal + GEM systems plus geometry & EVIO file list.  One
    call to setup_pipeline() produces a ready-to-loop bundle.

    `hycal_xform` and `gem_xforms[d]` are prad2det DetectorTransform instances
    built from geo at setup time — call `xform.to_lab(x, y[, z])` per hit to
    get the lab-frame position (the rotation matrix is cached inside)."""
    cfg:           "dec.DaqConfig"       = None
    crate_map:     dict[int, int]        = field(default_factory=dict)
    geo:           RunGeometry           = field(default_factory=RunGeometry)
    hycal:         "det.HyCalSystem"     = None
    hc_clusterer:  "det.HyCalCluster"    = None
    wave_ana:      "dec.WaveAnalyzer"    = None
    gem_sys:       "det.GemSystem"       = None
    gem_clusterer: "det.GemCluster"      = None
    evio_files:    list[str]             = field(default_factory=list)
    hycal_xform:   "det.DetectorTransform" = None
    gem_xforms:    list                  = field(default_factory=list)


def _print(msg: str) -> None:
    print(msg, flush=True)


def setup_pipeline(*,
                   evio_path: str,
                   max_events: int = 0,
                   run_num: int = -1,
                   gem_ped_file: str = "",
                   gem_cm_file: str = "",
                   hc_calib_file: str = "",
                   daq_config: str = "",
                   gem_map_file: str = "",
                   hc_map_file: str = "",
                   ) -> Pipeline:
    """Mirror the C++ scripts' setup block.  All "_file" args accept "" to
    fall back to runinfo / database defaults.  Run number defaults to a
    sniff from the EVIO basename if -1."""
    p = Pipeline()

    # ---- DAQ config ------------------------------------------------------
    p.cfg = dec.load_daq_config(daq_config)  # "" → installed default
    _print(f"[setup] DAQ config : {daq_config or '(default)'}")
    p.crate_map = {int(roc.tag): int(roc.crate) for roc in p.cfg.roc_tags}

    # ---- runinfo ---------------------------------------------------------
    ri_path = discover_runinfo_path()
    if not ri_path:
        raise SystemExit("[ERROR] no runinfo pointer in database/reconstruction_config.json"
                         " — cannot resolve calibration / geometry.")
    eff_run = run_num
    if eff_run <= 0:
        sniff = extract_run_number(evio_path or "")
        if sniff > 0:
            eff_run = sniff
            _print(f"[setup] Run number : {eff_run} (extracted from filename)")
    else:
        _print(f"[setup] Run number : {eff_run} (caller-provided)")
    p.geo = load_run_geometry(ri_path, eff_run)
    p.hycal_xform, p.gem_xforms = build_lab_transforms(p.geo)
    _print(f"[setup] RunInfo    : {ri_path}  beam={p.geo.beam_energy:.0f} MeV  "
           f"hycal_z={p.geo.hycal_z:.1f} mm")

    # ---- HyCal -----------------------------------------------------------
    hc_map = hc_map_file or resolve_db_path("hycal_map.json")
    p.hycal = det.HyCalSystem()
    p.hycal.init(hc_map)

    hc_calib = hc_calib_file or resolve_db_path(p.geo.energy_calib_file)
    if hc_calib:
        n = p.hycal.load_calibration(hc_calib)
        _print(f"[setup] HC calib   : {hc_calib} ({n} modules)")
    else:
        _print("[WARN] no HyCal calibration file — energies will be wrong.")

    p.hc_clusterer = det.HyCalCluster(p.hycal)
    hc_cfg = make_hycal_cluster_config()
    p.hc_clusterer.set_config(hc_cfg)
    _print(f"[setup] HC cluster : min_mod_E={hc_cfg.min_module_energy:g}  "
           f"min_ctr_E={hc_cfg.min_center_energy:g}  "
           f"min_cl_E={hc_cfg.min_cluster_energy:g}  "
           f"split_iter={hc_cfg.split_iter}")

    p.wave_ana = dec.WaveAnalyzer(p.cfg.wave_cfg)

    # ---- GEM -------------------------------------------------------------
    gem_map = gem_map_file or resolve_db_path("gem_map.json")
    p.gem_sys = det.GemSystem()
    p.gem_sys.init(gem_map)
    _print(f"[setup] GEM map    : {gem_map}  "
           f"({p.gem_sys.get_n_detectors()} detectors)")

    ped_path = gem_ped_file or resolve_db_path(p.geo.gem_pedestal_file)
    cm_path  = gem_cm_file  or resolve_db_path(p.geo.gem_common_mode_file)
    # Crate remap (file-side hardware crate ID → logical crate ID in
    # gem_map.json) — built from daq_cfg.roc_tags entries with type == "gem".
    # Mirrors the C++ server's setup; without it, pedestals/CM keyed by raw
    # EVIO bank crate IDs (146/147) silently fail to match the gem_map's
    # remapped crates (1/2) and APVs run at default noise → no hits.
    gem_crate_remap = {int(re.tag): int(re.crate)
                       for re in p.cfg.roc_tags
                       if getattr(re, "type", "") == "gem"}
    if gem_crate_remap:
        _print(f"[setup] GEM crate remap: {gem_crate_remap}")
    if ped_path:
        p.gem_sys.load_pedestals(ped_path, gem_crate_remap)
        _print(f"[setup] GEM peds   : {ped_path}")
    else:
        _print("[WARN] no GEM pedestal file — full-readout data reconstructs empty.")
    if cm_path:
        p.gem_sys.load_common_mode_range(cm_path, gem_crate_remap)
        _print(f"[setup] GEM CM     : {cm_path}")
    # Pedestal checksum so we can prove the loaded values are identical
    # to the C++ server's [PEDSUM] line.  If the sums differ, the same
    # file is being applied differently between the two pipelines.
    n_apvs = p.gem_sys.get_n_apvs()
    sum_noise = 0.0
    sum_off   = 0.0
    n_strips  = 0
    for ai in range(n_apvs):
        apv = p.gem_sys.get_apv_config(ai)
        for ch in range(128):
            ped = apv.pedestal(ch)
            sum_noise += float(ped.noise)
            sum_off   += float(ped.offset)
            n_strips  += 1
    _print(f"[PEDSUM] n_apvs={n_apvs} n_strips={n_strips} "
           f"sum_noise={sum_noise:.6f} sum_offset={sum_off:.6f}")

    p.gem_clusterer = det.GemCluster()
    # gem_map.json globals — print so a startup diff against the C++ server
    # confirms identical strip-level filtering.
    _print(f"[GEMSYS] common_mode_thr={p.gem_sys.common_mode_threshold:g}"
           f" zero_sup_thr={p.gem_sys.zero_sup_threshold:g}"
           f" cross_talk_thr={p.gem_sys.cross_talk_threshold:g}"
           f" min_peak={p.gem_sys.min_peak_adc:g}"
           f" min_sum={p.gem_sys.min_sum_adc:g}"
           f" rej_first={int(p.gem_sys.reject_first_timebin)}"
           f" rej_last={int(p.gem_sys.reject_last_timebin)}")
    gem_cfgs = make_gem_cluster_configs(p.gem_sys.get_n_detectors())
    p.gem_sys.set_recon_configs(gem_cfgs)
    # Per-detector ClusterConfig dump — matches the C++ [GEMCFG] line so
    # the two startups can be diffed.
    for d, c in enumerate(gem_cfgs):
        _print(f"[GEMCFG] d{d}"
               f" min_hits={c.min_cluster_hits}"
               f" max_hits={c.max_cluster_hits}"
               f" consec={c.consecutive_thres}"
               f" split={c.split_thres:g}"
               f" xtalk={c.cross_talk_width:g}"
               f" match_mode={c.match_mode}"
               f" asym={c.match_adc_asymmetry:g}"
               f" tdiff={c.match_time_diff:g}"
               f" tperiod={c.ts_period:g}")

    # ---- EVIO discovery --------------------------------------------------
    p.evio_files = discover_split_files(evio_path or "")
    _print(f"[setup] EVIO       : {len(p.evio_files)} split file(s) for "
           f"input {evio_path or '(null)'}")
    for f in p.evio_files:
        _print(f"           {f}")
    return p


# ============================================================================
# Per-event reconstruction helpers
# ============================================================================
# Both scripts call these inside their own event loop to keep the pipeline
# idempotent — the user passes the already-decoded EventData / SspEventData
# in (so trigger filtering happens before we pay the reco cost).

# HyCal trigger window (ns) for picking the largest peak — matches the live
# monitor's "100..200" cut in app_state_init.cpp / the C++ scripts.
HC_TIME_LO = 100.0
HC_TIME_HI = 200.0


def reconstruct_hycal(p: Pipeline, fadc_evt,
                      time_window: Optional[Tuple[float, float]] = None) -> list:
    """Run HyCal waveform → energy → cluster on one decoded EventData.
    Returns a list of det.ClusterHit (detector frame).

    Per-channel peak selection mirrors the C++ server's clustering input
    (`AppState::processEvent`): pick the peak with the largest integral
    across all detected peaks (no time gate), then feed the integral as
    the channel's clustering energy.  Pass `time_window=(lo, hi)` to also
    require the chosen peak's time within those bounds — useful when an
    analysis wants to restrict to triggered peaks only, but NOT what the
    online monitor does."""
    p.hc_clusterer.clear()
    for ri in range(fadc_evt.nrocs):
        roc = fadc_evt.roc(ri)
        if not roc.present:
            continue
        crate = p.crate_map.get(roc.tag)
        if crate is None:
            continue
        for s in roc.present_slots():
            slot = roc.slot(s)
            for c in slot.present_channels():
                mod = p.hycal.module_by_daq(crate, s, c)
                if mod is None or not mod.is_hycal():
                    continue
                cd = slot.channel(c)
                if cd.nsamples <= 0:
                    continue
                _, _, peaks = p.wave_ana.analyze(cd.samples)
                # Best peak by INTEGRAL across all detected peaks — matches
                # `bestPeak()` in viewer_utils.h, which the server feeds into
                # HyCalCluster::AddHit.
                best = None
                best_i = -1.0
                for pk in peaks:
                    if time_window is not None:
                        if not (time_window[0] < pk.time < time_window[1]):
                            continue
                    if pk.integral > best_i:
                        best = pk
                        best_i = pk.integral
                if best is None:
                    continue
                p.hc_clusterer.add_hit(mod.index,
                                        mod.energize(best.integral),
                                        float(best.time))
    p.hc_clusterer.form_clusters()
    return p.hc_clusterer.reconstruct_hits()


def reconstruct_gem(p: Pipeline, ssp_evt) -> None:
    """Run GEM ProcessEvent + Reconstruct.  After this the per-detector
    hit lists are accessible via p.gem_sys.get_hits(d)."""
    p.gem_sys.clear()
    p.gem_sys.process_event(ssp_evt)
    p.gem_sys.reconstruct(p.gem_clusterer)


# ============================================================================
# Argparse helpers
# ============================================================================

def add_common_args(ap: argparse.ArgumentParser) -> None:
    """Register the args every analysis script accepts.  Mirrors the C++
    signatures of gem_hycal_matching / plot_hits_at_hycal."""
    ap.add_argument("evio_path",
                    help="EVIO input.  glob ('prad_NNNNNN.evio.*'), directory, "
                         "or single split (prad_NNNNNN.evio.00000).")
    ap.add_argument("out_path",
                    help="Output table path (.tsv / .csv).")
    ap.add_argument("--max-events", type=int, default=0,
                    help="Stop after N raw physics events (0 = all).")
    ap.add_argument("--run-num", type=int, default=-1,
                    help="Run number override (default -1 = sniff from filename).")
    ap.add_argument("--gem-ped-file",  default="",
                    help='GEM pedestal file (default "" = via runinfo).')
    ap.add_argument("--gem-cm-file",   default="",
                    help='GEM common-mode file (default "" = via runinfo).')
    ap.add_argument("--hc-calib-file", default="",
                    help='HyCal calibration file (default "" = via runinfo).')
    ap.add_argument("--daq-config",    default="",
                    help='DAQ config (default "" = installed default).')
    ap.add_argument("--gem-map-file",  default="",
                    help='GEM map (default "" = database/gem_map.json).')
    ap.add_argument("--hc-map-file",   default="",
                    help='HyCal map (default "" = database/hycal_map.json).')
    ap.add_argument("--csv", action="store_true",
                    help="Emit CSV instead of TSV.")
    ap.add_argument("--no-header", action="store_true",
                    help="Skip the column-name header row.")


# Physics trigger bits accepted for reconstruction.  Mirrors the server's
# `physics.accept_trigger_bits` set in monitor_config.json — currently
# {SSP0, SSP1, SSP2, SSP3} = bits {8,9,10,11} = mask 0xf00.  The check is
# bitwise-AND (an event passes if ANY accepted bit is set), matching
# `TriggerFilter::operator()` in src/app_state.h.  The previous behaviour
# (== 0x100) restricted the audit to SSP0-only events and silently
# excluded the cluster-counting triggers, which produced a 2× sample-bias
# vs the server in the GEM efficiency audit.
PHYSICS_TRIGGER_MASK = 0xf00


def passes_physics_trigger(trigger_bits: int) -> bool:
    """C++-equivalent physics trigger gate (`bits & PHYSICS_TRIGGER_MASK`).
    Use this in scripts instead of `== PHYSICS_TRIGGER_BITS`."""
    return bool(trigger_bits & PHYSICS_TRIGGER_MASK)


# Backwards-compat alias — old scripts that did `!= PHYSICS_TRIGGER_BITS`
# still work but only accept SSP0.  New code should call
# `passes_physics_trigger`.
PHYSICS_TRIGGER_BITS = 0x100


def open_table_writer(out_path: str, csv_mode: bool):
    """Open `out_path` for writing.  Returns (file_handle, write_row callable)
    where write_row(seq) writes one row.  Caller closes the file."""
    import csv as _csv
    fh = open(out_path, "w", encoding="utf-8", newline="")
    if csv_mode:
        w = _csv.writer(fh, lineterminator="\n")
        return fh, w.writerow
    sep = "\t"
    def write_row(row):
        fh.write(sep.join("" if v is None else str(v) for v in row))
        fh.write("\n")
    return fh, write_row
