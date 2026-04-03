#!/usr/bin/env python3
"""
HyCal Pedestal Monitor
======================
Measures and monitors FADC250 pedestals for all HyCal channels.
Maps DAQ addresses (crate/slot/channel) to HyCal modules and generates
an interactive HTML report with spatial colour maps.

**No external dependencies** -- uses only the Python standard library.

Usage
-----
    python hycal_pedestal_monitor.py                        # plot originals
    python hycal_pedestal_monitor.py --measure               # measure + compare
    python hycal_pedestal_monitor.py --latest-dir ./latest   # compare existing
    python hycal_pedestal_monitor.py --sim                   # test anywhere
"""

from __future__ import annotations

import argparse
import html as html_mod
import json
import math
import os
import random
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ===========================================================================
#  Paths & constants
# ===========================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
DB_DIR = SCRIPT_DIR / ".." / "database"
MODULES_JSON = DB_DIR / "hycal_modules.json"
DAQ_MAP_JSON = DB_DIR / "daq_map.json"

NUM_CRATES = 7
CRATE_NAMES = [f"adchycal{i}" for i in range(1, NUM_CRATES + 1)]
ORIGINAL_PED_DIR = Path("/usr/clas12/release/2.0.0/parms/fadc250/peds")
CHANNELS_PER_SLOT = 16


# ===========================================================================
#  Module database
# ===========================================================================

@dataclass
class Module:
    name: str
    mod_type: str   # "PbWO4", "PbGlass", "LMS"
    x: float        # centre-x  (mm)
    y: float        # centre-y  (mm)
    sx: float       # width  (mm)
    sy: float       # height (mm)


def load_modules(path: Path) -> List[Module]:
    with open(path) as f:
        data = json.load(f)
    return [Module(e["n"], e["t"], e["x"], e["y"], e["sx"], e["sy"])
            for e in data]


# ===========================================================================
#  DAQ map
# ===========================================================================

def load_daq_map(path: Path = DAQ_MAP_JSON) -> Dict[Tuple[int, int, int], str]:
    """(crate_index, slot, channel) -> module_name."""
    with open(path) as f:
        data = json.load(f)
    return {(d["crate"], d["slot"], d["channel"]): d["name"] for d in data}


# ===========================================================================
#  Pedestal file parser
# ===========================================================================

def parse_pedestal_file(filepath: Path) -> Dict[int, Dict[str, List[float]]]:
    """Parse one FADC250 pedestal .cnf file.

    Returns  slot_number -> {"ped": [16 floats], "noise": [16 floats]}
    """
    slots: Dict[int, Dict[str, List[float]]] = {}
    cur_slot: Optional[int] = None
    cur_key: Optional[str] = None
    vals: List[float] = []

    def _flush():
        nonlocal cur_key, vals
        if cur_slot is not None and cur_key and vals:
            slots.setdefault(cur_slot, {})[cur_key] = vals[:CHANNELS_PER_SLOT]
        vals = []
        cur_key = None

    with open(filepath) as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("FADC250_CRATE"):
                _flush()
            elif line.startswith("FADC250_SLOT"):
                _flush()
                cur_slot = int(line.split()[1])
            elif line.startswith("FADC250_ALLCH_PED"):
                _flush()
                cur_key = "ped"
                vals = [float(v) for v in
                        line[len("FADC250_ALLCH_PED"):].split()]
                if len(vals) >= CHANNELS_PER_SLOT:
                    _flush()
            elif line.startswith("FADC250_ALLCH_NOISE"):
                _flush()
                cur_key = "noise"
                vals = [float(v) for v in
                        line[len("FADC250_ALLCH_NOISE"):].split()]
                if len(vals) >= CHANNELS_PER_SLOT:
                    _flush()
            elif cur_key is not None:
                try:
                    vals.extend(float(v) for v in line.split())
                    if len(vals) >= CHANNELS_PER_SLOT:
                        _flush()
                except ValueError:
                    _flush()
    _flush()
    return slots


def read_all_pedestals(
    ped_dir: Path,
    suffix: str,
    daq_map: Dict[Tuple[int, int, int], str],
) -> Dict[str, Dict[str, float]]:
    """Read pedestal files for all 7 crates.

    Returns  module_name -> {"ped": float, "noise": float}
    """
    result: Dict[str, Dict[str, float]] = {}
    for crate_idx, crate_name in enumerate(CRATE_NAMES):
        fpath = ped_dir / f"{crate_name}{suffix}"
        if not fpath.exists():
            print(f"  Warning: {fpath} not found")
            continue
        for slot, data in parse_pedestal_file(fpath).items():
            for ch in range(CHANNELS_PER_SLOT):
                mod = daq_map.get((crate_idx, slot, ch))
                if mod is None:
                    continue
                entry: Dict[str, float] = {}
                if "ped" in data and ch < len(data["ped"]):
                    entry["ped"] = data["ped"][ch]
                if "noise" in data and ch < len(data["noise"]):
                    entry["noise"] = data["noise"][ch]
                if entry:
                    result[mod] = entry
    return result


# ===========================================================================
#  Pedestal measurement via SSH
# ===========================================================================

def measure_pedestals(latest_dir: Path) -> bool:
    print()
    print("=" * 60)
    print("  WARNING: Pedestal measurement will INTERRUPT DAQ running!")
    print("  Only proceed when DAQ is IDLE.")
    print("=" * 60)
    resp = input("\nProceed with pedestal measurement? [yes/no]: ").strip().lower()
    if resp not in ("yes", "y"):
        print("Measurement cancelled.")
        return False

    latest_dir.mkdir(parents=True, exist_ok=True)

    print("\nMeasuring pedestals on all crates ...")
    for cname in CRATE_NAMES:
        print(f"  {cname} ... ", end="", flush=True)
        cmd = f'ssh {cname} "faV3peds {cname}_ped_latest.cnf"'
        try:
            subprocess.run(cmd, shell=True, check=True, timeout=120)
            print("done")
        except subprocess.CalledProcessError as exc:
            print(f"FAILED (exit {exc.returncode})")
        except subprocess.TimeoutExpired:
            print("TIMEOUT")

    print("\nRetrieving pedestal files ...")
    for cname in CRATE_NAMES:
        src = f"{cname}:~/{cname}_ped_latest.cnf"
        dst = latest_dir / f"{cname}_ped_latest.cnf"
        try:
            subprocess.run(f"scp {src} {dst}",
                           shell=True, check=True, timeout=30)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            print(f"  Warning: scp from {cname} failed: {exc}")

    print(f"Pedestal files saved to {latest_dir}/")
    return True


# ===========================================================================
#  Pure-Python statistics helpers (no numpy)
# ===========================================================================

def _percentile(data: List[float], p: float) -> float:
    """Linear-interpolation percentile on already-sorted *data*."""
    if not data:
        return 0.0
    s = sorted(data)
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (k - lo) * (s[hi] - s[lo])


def _mean(data: List[float]) -> float:
    return sum(data) / len(data) if data else 0.0


def print_stats(label: str, peds: Dict[str, Dict[str, float]]):
    ped_vals = [v["ped"] for v in peds.values() if "ped" in v]
    noise_vals = [v["noise"] for v in peds.values() if "noise" in v]
    live = [v for v in ped_vals if v != 0.0]
    dead = sum(1 for v in ped_vals if v == 0.0)
    print(f"\n  {label}:")
    print(f"    Channels with data : {len(ped_vals)}")
    print(f"    Dead (ped == 0)    : {dead}")
    if live:
        print(f"    Pedestal  mean={_mean(live):.1f}  "
              f"min={min(live):.1f}  max={max(live):.1f}")
    if noise_vals:
        print(f"    Noise     mean={_mean(noise_vals):.2f}  "
              f"min={min(noise_vals):.2f}  max={max(noise_vals):.2f}")
    else:
        print("    (no noise/RMS data in files)")


# ===========================================================================
#  Colour-map helpers (no matplotlib)
# ===========================================================================

def _lerp(a: int, b: int, t: float) -> int:
    return int(a + (b - a) * t)


def _rgb_hex(r: int, g: int, b: int) -> str:
    return f"#{r:02x}{g:02x}{b:02x}"


# Viridis-inspired 5-stop gradient
_VIRIDIS = [
    (0.00, (68,   1,  84)),
    (0.25, (59,  82, 139)),
    (0.50, (33, 145, 140)),
    (0.75, (94, 201,  98)),
    (1.00, (253, 231, 37)),
]

# Diverging blue-white-red (RdBu_r-inspired)
_RDBU = [
    (0.00, ( 33, 102, 172)),
    (0.25, (103, 169, 207)),
    (0.50, (247, 247, 247)),
    (0.75, (239, 138,  98)),
    (1.00, (178,  24,  43)),
]


def _cmap_color(t: float, stops) -> str:
    t = max(0.0, min(1.0, t))
    for i in range(len(stops) - 1):
        t0, c0 = stops[i]
        t1, c1 = stops[i + 1]
        if t <= t1:
            s = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return _rgb_hex(_lerp(c0[0], c1[0], s),
                            _lerp(c0[1], c1[1], s),
                            _lerp(c0[2], c1[2], s))
    return _rgb_hex(*stops[-1][1])


def _val_color(val: float, vmin: float, vmax: float,
               cmap: str = "viridis") -> str:
    t = (val - vmin) / (vmax - vmin) if vmax > vmin else 0.5
    stops = _VIRIDIS if cmap == "viridis" else _RDBU
    return _cmap_color(t, stops)


# ===========================================================================
#  SVG / HTML generation
# ===========================================================================

_SVG_W = 700
_SVG_H = 700
_MARGIN = 55
_CBAR_H = 16
_BOT = 55            # room for colour bar + labels
_SHRINK = 0.92


def _svg_gradient(grad_id: str, stops) -> str:
    parts = [f'<linearGradient id="{grad_id}" x1="0" y1="0" x2="1" y2="0">']
    for t, (r, g, b) in stops:
        parts.append(f'<stop offset="{t*100:.0f}%" stop-color="{_rgb_hex(r,g,b)}"/>')
    parts.append("</linearGradient>")
    return "\n    ".join(parts)


def _generate_svg(
    modules: List[Module],
    values: Dict[str, float],
    title: str,
    cmap: str = "viridis",
    center_zero: bool = False,
    panel_id: int = 0,
) -> str:
    """Build one SVG panel for a HyCal colour map."""
    det = [m for m in modules if m.mod_type != "LMS"]
    if not det:
        return ""

    x_min = min(m.x - m.sx / 2 for m in det)
    x_max = max(m.x + m.sx / 2 for m in det)
    y_min = min(m.y - m.sy / 2 for m in det)
    y_max = max(m.y + m.sy / 2 for m in det)

    plot_w = _SVG_W - 2 * _MARGIN
    plot_h = _SVG_H - _MARGIN - _BOT
    scale = min(plot_w / (x_max - x_min), plot_h / (y_max - y_min))
    draw_w = (x_max - x_min) * scale
    draw_h = (y_max - y_min) * scale
    ox = _MARGIN + (plot_w - draw_w) / 2
    oy = _MARGIN + (plot_h - draw_h) / 2

    # colour limits
    raw = [v for v in values.values()]
    live = [v for v in raw if v != 0.0] if any(v != 0.0 for v in raw) else raw
    if live:
        vmin = _percentile(live, 2)
        vmax = _percentile(live, 98)
    else:
        vmin, vmax = 0.0, 1.0
    if center_zero:
        mx = max(abs(vmin), abs(vmax), 1e-9)
        vmin, vmax = -mx, mx

    rects: List[str] = []
    for m in det:
        w = m.sx * scale * _SHRINK
        h = m.sy * scale * _SHRINK
        cx = ox + (m.x - x_min) * scale
        cy = oy + (y_max - m.y) * scale
        rx, ry = cx - w / 2, cy - h / 2
        v = values.get(m.name)
        if v is not None:
            fill = _val_color(v, vmin, vmax, cmap)
            vtxt = f"{v:.2f}"
        else:
            fill = "#1a1a2e"
            vtxt = "N/A"
        esc = html_mod.escape(m.name)
        rects.append(
            f'<rect class="m" x="{rx:.1f}" y="{ry:.1f}" '
            f'width="{w:.1f}" height="{h:.1f}" fill="{fill}" '
            f'data-n="{esc}" data-v="{vtxt}"><title>{esc}: {vtxt}</title></rect>')

    # gradient for colour bar
    gid = f"g{panel_id}"
    stops = _VIRIDIS if cmap == "viridis" else _RDBU
    grad = _svg_gradient(gid, stops)

    cb_y = _SVG_H - _BOT + 12
    cb_x = _MARGIN
    cb_w = _SVG_W - 2 * _MARGIN

    return f"""\
<svg width="{_SVG_W}" height="{_SVG_H}" xmlns="http://www.w3.org/2000/svg"
     style="display:block">
  <defs>{grad}</defs>
  <rect width="{_SVG_W}" height="{_SVG_H}" fill="#0a0e14" rx="8"/>
  <text x="{_SVG_W/2}" y="28" text-anchor="middle" fill="#c9d1d9"
        font-family="monospace" font-size="13" font-weight="bold">{html_mod.escape(title)}</text>
  {"".join(rects)}
  <rect x="{cb_x}" y="{cb_y}" width="{cb_w}" height="{_CBAR_H}"
        fill="url(#{gid})" stroke="#555" stroke-width="0.5" rx="2"/>
  <text x="{cb_x}" y="{cb_y+_CBAR_H+14}" fill="#8b949e"
        font-family="monospace" font-size="10">{vmin:.1f}</text>
  <text x="{cb_x+cb_w}" y="{cb_y+_CBAR_H+14}" text-anchor="end" fill="#8b949e"
        font-family="monospace" font-size="10">{vmax:.1f}</text>
  <text x="{cb_x+cb_w/2}" y="{cb_y+_CBAR_H+14}" text-anchor="middle" fill="#8b949e"
        font-family="monospace" font-size="10">{(vmin+vmax)/2:.1f}</text>
</svg>"""


def _placeholder_svg(title: str, msg: str) -> str:
    return f"""\
<svg width="{_SVG_W}" height="{_SVG_H}" xmlns="http://www.w3.org/2000/svg"
     style="display:block">
  <rect width="{_SVG_W}" height="{_SVG_H}" fill="#0a0e14" rx="8"/>
  <text x="{_SVG_W/2}" y="28" text-anchor="middle" fill="#c9d1d9"
        font-family="monospace" font-size="13" font-weight="bold">{html_mod.escape(title)}</text>
  <text x="{_SVG_W/2}" y="{_SVG_H/2}" text-anchor="middle" fill="#555"
        font-family="monospace" font-size="14">{html_mod.escape(msg)}</text>
</svg>"""


def _stats_text(label: str, peds: Dict[str, Dict[str, float]]) -> str:
    ped_vals = [v["ped"] for v in peds.values() if "ped" in v]
    noise_vals = [v["noise"] for v in peds.values() if "noise" in v]
    live = [v for v in ped_vals if v != 0.0]
    dead = sum(1 for v in ped_vals if v == 0.0)
    lines = [f"{label}:  {len(ped_vals)} channels,  {dead} dead"]
    if live:
        lines.append(f"  Pedestal  mean={_mean(live):.1f}  "
                     f"min={min(live):.1f}  max={max(live):.1f}")
    if noise_vals:
        lines.append(f"  Noise     mean={_mean(noise_vals):.2f}  "
                     f"min={min(noise_vals):.2f}  max={max(noise_vals):.2f}")
    return "\n".join(lines)


def generate_report(
    modules: List[Module],
    original: Dict[str, Dict[str, float]],
    latest: Optional[Dict[str, Dict[str, float]]] = None,
    output: Optional[Path] = None,
):
    """Write an interactive HTML file with four SVG HyCal map panels."""

    def _extract(peds, key):
        return {n: v[key] for n, v in peds.items() if key in v}

    has_latest = latest is not None and len(latest) > 0
    cur = latest if has_latest else original
    label = "Current" if has_latest else "Original"

    ped_mean = _extract(cur, "ped")
    ped_noise = _extract(cur, "noise")
    has_noise = bool(ped_noise)

    # --- panel 1: pedestal mean ---
    svg1 = _generate_svg(modules, ped_mean, f"{label} Pedestal Mean",
                         panel_id=1)

    # --- panel 2: pedestal RMS ---
    if has_noise:
        svg2 = _generate_svg(modules, ped_noise, f"{label} Pedestal RMS",
                             panel_id=2)
    else:
        svg2 = _placeholder_svg(f"{label} Pedestal RMS",
                                "No noise/RMS data in pedestal files")

    # --- panels 3 & 4: differences ---
    if has_latest:
        delta_mean: Dict[str, float] = {}
        delta_noise: Dict[str, float] = {}
        for n in latest:
            if n in original:
                if "ped" in latest[n] and "ped" in original[n]:
                    delta_mean[n] = latest[n]["ped"] - original[n]["ped"]
                if "noise" in latest[n] and "noise" in original[n]:
                    delta_noise[n] = latest[n]["noise"] - original[n]["noise"]
        svg3 = _generate_svg(modules, delta_mean,
                             "Pedestal Mean Difference (Current \u2212 Original)",
                             cmap="rdbu", center_zero=True, panel_id=3)
        if delta_noise:
            svg4 = _generate_svg(modules, delta_noise,
                                 "Pedestal RMS Difference (Current \u2212 Original)",
                                 cmap="rdbu", center_zero=True, panel_id=4)
        else:
            svg4 = _placeholder_svg("RMS Difference",
                                    "No noise/RMS data in pedestal files")
    else:
        svg3 = _placeholder_svg("Pedestal Mean Difference",
                                "No comparison data (use --measure or --latest-dir)")
        svg4 = _placeholder_svg("Pedestal RMS Difference",
                                "No comparison data (use --measure or --latest-dir)")

    # --- statistics block ---
    stats_parts: List[str] = []
    if has_latest and latest:
        stats_parts.append(_stats_text("Original", original))
        stats_parts.append(_stats_text("Current", latest))
    else:
        stats_parts.append(_stats_text("Original", original))
    stats_html = html_mod.escape("\n\n".join(stats_parts))

    page = f"""\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>HyCal Pedestal Monitor</title>
<style>
  * {{ margin:0; padding:0; box-sizing:border-box; }}
  body {{ background:#0d1117; color:#c9d1d9; font-family:'Consolas','Courier New',monospace; }}
  h1 {{ text-align:center; padding:14px 0 6px; color:#58a6ff; font-size:20px; }}
  .grid {{ display:grid; grid-template-columns:1fr 1fr; gap:12px; padding:0 12px; }}
  .panel {{ background:#161b22; border:1px solid #30363d; border-radius:10px;
            padding:6px; display:flex; justify-content:center; }}
  .panel svg {{ max-width:100%; height:auto; }}
  .stats {{ margin:16px auto; max-width:720px; background:#161b22;
            border:1px solid #30363d; border-radius:8px; padding:14px 18px;
            white-space:pre; font-size:13px; color:#8b949e; line-height:1.5; }}
  #info {{ position:fixed; bottom:12px; left:50%; transform:translateX(-50%);
           background:#21262d; color:#c9d1d9; padding:6px 16px; border-radius:6px;
           font-size:13px; border:1px solid #30363d; pointer-events:none;
           transition:opacity 0.15s; opacity:0.85; }}
  rect.m {{ stroke:#333; stroke-width:0.3; }}
  rect.m:hover {{ stroke:#58a6ff; stroke-width:1.8; }}
</style>
</head>
<body>
<h1>HyCal Pedestal Monitor</h1>
<div class="grid">
  <div class="panel">{svg1}</div>
  <div class="panel">{svg2}</div>
  <div class="panel">{svg3}</div>
  <div class="panel">{svg4}</div>
</div>
<div class="stats">{stats_html}</div>
<div id="info">Hover over a module</div>
<script>
document.querySelectorAll('rect.m').forEach(function(r){{
  r.addEventListener('mouseenter',function(){{
    document.getElementById('info').textContent=r.dataset.n+':  '+r.dataset.v;
  }});
}});
</script>
</body>
</html>
"""
    if output is None:
        output = Path("pedestal_monitor.html")
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"\nReport saved to {output}")
    print(f"Open in browser:  firefox {output} &")


# ===========================================================================
#  Simulation
# ===========================================================================

def simulate_pedestals(
    modules: List[Module],
) -> Tuple[Dict[str, Dict[str, float]], Dict[str, Dict[str, float]]]:
    rng = random.Random(42)
    original: Dict[str, Dict[str, float]] = {}
    latest:   Dict[str, Dict[str, float]] = {}
    names: List[str] = []

    for m in modules:
        if m.mod_type == "LMS":
            continue
        names.append(m.name)
        o_ped = rng.gauss(160, 25)
        o_noi = abs(rng.gauss(4.0, 1.0))
        original[m.name] = {"ped": o_ped, "noise": o_noi}
        latest[m.name]   = {"ped": o_ped + rng.gauss(0, 3),
                            "noise": abs(o_noi + rng.gauss(0, 0.3))}

    for n in rng.sample(names, k=15):
        original[n]["ped"] = 0.0
        latest[n]["ped"]   = 0.0
    hot = rng.choice(names)
    original[hot]["ped"] = 2049.0
    latest[hot]["ped"]   = 2050.0

    return original, latest


# ===========================================================================
#  Main
# ===========================================================================

def main():
    ap = argparse.ArgumentParser(
        description="HyCal Pedestal Monitor - measure, read, and visualize "
                    "FADC250 pedestals for all HyCal channels")
    ap.add_argument("--measure", action="store_true",
                    help="Run pedestal measurement on all 7 crates via SSH "
                         "(only when DAQ is idle)")
    ap.add_argument("--original-dir", type=Path, default=ORIGINAL_PED_DIR,
                    help=f"Dir with original pedestal files "
                         f"(default: {ORIGINAL_PED_DIR})")
    ap.add_argument("--latest-dir", type=Path, default=None,
                    help="Dir with latest measured pedestal files")
    ap.add_argument("--output", "-o", type=Path, default=None,
                    help="Output HTML file (default: pedestal_monitor.html)")
    ap.add_argument("--sim", action="store_true",
                    help="Simulated data for testing (no SSH/files needed)")
    ap.add_argument("--modules-db", type=Path, default=MODULES_JSON)
    ap.add_argument("--daq-map", type=Path, default=DAQ_MAP_JSON)
    args = ap.parse_args()

    modules = load_modules(args.modules_db)
    print(f"Loaded {len(modules)} modules")

    if args.sim:
        print("=== Simulation Mode ===")
        original, latest = simulate_pedestals(modules)
        print_stats("Original (sim)", original)
        print_stats("Latest   (sim)", latest)
        generate_report(modules, original, latest, args.output)
        return

    daq_map = load_daq_map(args.daq_map)
    print(f"Loaded {len(daq_map)} DAQ channels")

    latest_dir = args.latest_dir
    if args.measure:
        latest_dir = latest_dir or Path("./pedestal_latest")
        if not measure_pedestals(latest_dir):
            latest_dir = None

    print(f"\nReading original pedestals from {args.original_dir} ...")
    original = read_all_pedestals(args.original_dir, "_ped.cnf", daq_map)
    print_stats("Original pedestals", original)

    latest = None
    if latest_dir and latest_dir.exists():
        print(f"\nReading latest pedestals from {latest_dir} ...")
        latest = read_all_pedestals(latest_dir, "_ped_latest.cnf", daq_map)
        print_stats("Latest pedestals", latest)

    if not original and not latest:
        print("\nERROR: No pedestal data found. Check file paths.")
        sys.exit(1)

    generate_report(modules, original, latest, args.output)


if __name__ == "__main__":
    main()
