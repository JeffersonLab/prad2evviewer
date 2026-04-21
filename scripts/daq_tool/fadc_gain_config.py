#!/usr/bin/env python3
"""
FADC Gain Config Generator
==========================
Generates a text-based ``adchycal_gain.cnf`` for the FADC250 DAQ, writing
``FAV3_ALLCH_GAIN`` entries (one per 16-channel slot) grouped by crate/slot.

Gains come from one of two sources:
  * ``-c/--calibration``: a JSON file (e.g. database/calibration/xxxx.json)
    containing a list of objects with ``name`` and ``factor`` fields.
  * Uniform values per module type via ``--pbwo4-gain`` / ``--pbglass-gain``.

If a calibration file is supplied, channels without a matching entry fall
back to the uniform value for their module type.

Usage
-----
    python fadc_gain_config.py
    python fadc_gain_config.py -c database/calibration/adc_to_mev_factors_cosmic.json
    python fadc_gain_config.py --pbwo4-gain 0.15 --pbglass-gain 0.12
    python fadc_gain_config.py -o /path/to/adchycal_gain.cnf
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent

NUM_CRATES = 7
CRATE_NAMES = [f"adchycal{i}" for i in range(1, NUM_CRATES + 1)]
CHANNELS_PER_SLOT = 16

DEFAULT_UNMAPPED_GAIN = 1.0
DEFAULT_LMS_GAIN = 1.0
DEFAULT_SCINT_GAIN = 1.0


# ---------------------------------------------------------------------------
#  Database auto-discovery
# ---------------------------------------------------------------------------

def find_database_dir(explicit: Optional[str] = None) -> Path:
    if explicit:
        p = Path(explicit).resolve()
        if not p.is_dir():
            sys.exit(f"error: --database path does not exist: {p}")
        return p

    candidates = [
        SCRIPT_DIR / ".." / ".." / "database",
        Path.cwd() / "database",
        Path.cwd(),
    ]
    for c in candidates:
        if (c / "daq_map.json").is_file() and (c / "hycal_modules.json").is_file():
            return c.resolve()
    sys.exit("error: could not locate database directory "
             "(looked for daq_map.json + hycal_modules.json)")


def load_modules(db_dir: Path) -> Dict[str, str]:
    """Return {module_name: module_type} for all HyCal modules."""
    with open(db_dir / "hycal_modules.json") as f:
        mods = json.load(f)
    return {m["n"]: m["t"] for m in mods}


def load_daq_map(db_dir: Path) -> List[Tuple[str, int, int, int]]:
    """Return list of (name, crate, slot, channel)."""
    with open(db_dir / "daq_map.json") as f:
        entries = json.load(f)
    return [(e["name"], e["crate"], e["slot"], e["channel"]) for e in entries]


# ---------------------------------------------------------------------------
#  Gain source
# ---------------------------------------------------------------------------

def load_calibration(path: Path) -> Dict[str, float]:
    """Return {module_name: gain_factor} from a calibration JSON file."""
    with open(path) as f:
        data = json.load(f)
    out: Dict[str, float] = {}
    for entry in data:
        name = entry.get("name")
        factor = entry.get("factor")
        if name is None or factor is None:
            continue
        out[name] = float(factor)
    return out


def resolve_gain(name: str,
                 mod_type: Optional[str],
                 cal: Dict[str, float],
                 pbwo4_gain: float,
                 pbglass_gain: float) -> float:
    if name in cal:
        return cal[name]
    if mod_type == "PbWO4":
        return pbwo4_gain
    if mod_type == "PbGlass":
        return pbglass_gain
    if mod_type == "LMS":
        return DEFAULT_LMS_GAIN
    # V1-V4 scintillators and anything else
    return DEFAULT_SCINT_GAIN


# ---------------------------------------------------------------------------
#  Config generation
# ---------------------------------------------------------------------------

def format_gain(g: float) -> str:
    return f"{g:.6f}"


def generate_config(daq: List[Tuple[str, int, int, int]],
                    mod_types: Dict[str, str],
                    cal: Dict[str, float],
                    pbwo4_gain: float,
                    pbglass_gain: float) -> Tuple[str, Dict[str, int]]:
    """Build the .cnf text. Returns (text, stats)."""
    # (crate, slot) -> {channel: (name, gain)}
    slots: Dict[Tuple[int, int], Dict[int, Tuple[str, float]]] = {}
    stats = {"from_cal": 0, "pbwo4_default": 0, "pbglass_default": 0,
             "other_default": 0, "unmapped": 0}

    for name, crate, slot, ch in daq:
        if crate < 0 or slot < 0 or ch < 0:
            continue
        mod_type = mod_types.get(name)
        gain = resolve_gain(name, mod_type, cal, pbwo4_gain, pbglass_gain)
        if name in cal:
            stats["from_cal"] += 1
        elif mod_type == "PbWO4":
            stats["pbwo4_default"] += 1
        elif mod_type == "PbGlass":
            stats["pbglass_default"] += 1
        else:
            stats["other_default"] += 1
        slots.setdefault((crate, slot), {})[ch] = (name, gain)

    lines: List[str] = []
    lines.append("# adchycal_gain.cnf")
    lines.append("# Generated by fadc_gain_config.py")
    lines.append(f"# Modules from calibration file : {stats['from_cal']}")
    lines.append(f"# PbWO4 default gain ({pbwo4_gain}) : {stats['pbwo4_default']}")
    lines.append(f"# PbGlass default gain ({pbglass_gain}) : {stats['pbglass_default']}")
    lines.append(f"# Other (LMS/V/..) default      : {stats['other_default']}")
    lines.append("")

    for ci in range(NUM_CRATES):
        crate_slots = sorted(s for (c, s) in slots if c == ci)
        if not crate_slots:
            continue
        lines.append(f"FAV3_CRATE {CRATE_NAMES[ci]}")
        for slot in crate_slots:
            ch_map = slots[(ci, slot)]
            gains: List[str] = []
            names: List[str] = []
            for ch in range(CHANNELS_PER_SLOT):
                entry = ch_map.get(ch)
                if entry is None:
                    gains.append(format_gain(DEFAULT_UNMAPPED_GAIN))
                    names.append(f"ch{ch}:unmapped")
                    stats["unmapped"] += 1
                else:
                    name, g = entry
                    gains.append(format_gain(g))
                    names.append(name)
            lines.append(f"# slot {slot}: {', '.join(names)}")
            lines.append(f"FAV3_SLOT {slot}")
            lines.append(f"FAV3_ALLCH_GAIN {' '.join(gains)}")
        lines.append("FAV3_CRATE end")
        lines.append("")

    return "\n".join(lines), stats


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate adchycal_gain.cnf from DAQ map + calibration")
    parser.add_argument("-c", "--calibration",
                        help="Path to calibration JSON (list of "
                             "{name, factor, ...} entries). Overrides uniform "
                             "values for matched channels.")
    parser.add_argument("--pbwo4-gain", type=float, default=1.0,
                        help="Uniform gain for PbWO4 crystals (default: 1.0)")
    parser.add_argument("--pbglass-gain", type=float, default=1.0,
                        help="Uniform gain for PbGlass modules (default: 1.0)")
    parser.add_argument("-d", "--database",
                        help="Database directory (default: auto-search)")
    parser.add_argument("-o", "--output", default="adchycal_gain.cnf",
                        help="Output file (default: adchycal_gain.cnf)")
    args = parser.parse_args()

    db_dir = find_database_dir(args.database)
    print(f"database : {db_dir}")

    mod_types = load_modules(db_dir)
    daq = load_daq_map(db_dir)
    print(f"modules  : {len(mod_types)}   daq entries: {len(daq)}")

    cal: Dict[str, float] = {}
    if args.calibration:
        cal_path = Path(args.calibration)
        if not cal_path.is_absolute() and not cal_path.is_file():
            alt = db_dir / "calibration" / cal_path.name
            if alt.is_file():
                cal_path = alt
        if not cal_path.is_file():
            sys.exit(f"error: calibration file not found: {args.calibration}")
        cal = load_calibration(cal_path)
        print(f"cal file : {cal_path}  ({len(cal)} entries)")
    else:
        print("cal file : <none>  (using uniform defaults)")

    text, stats = generate_config(daq, mod_types, cal,
                                  args.pbwo4_gain, args.pbglass_gain)

    out_path = Path(args.output).resolve()
    with open(out_path, "w") as f:
        f.write(text)
    print(f"wrote    : {out_path}")
    print(f"  from calibration : {stats['from_cal']}")
    print(f"  PbWO4 default    : {stats['pbwo4_default']}")
    print(f"  PbGlass default  : {stats['pbglass_default']}")
    print(f"  other default    : {stats['other_default']}")
    print(f"  unmapped slots   : {stats['unmapped']}")


if __name__ == "__main__":
    main()
