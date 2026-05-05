# GEM

Tools, scripts, and reference notes for the PRad-II GEM tracker.

| File | Purpose |
|---|---|
| `gem_dump.cpp` | C++ CLI: `raw` / `hits` / `clusters` / `evdump` / `summary` / `ped` modes. |
| `gem_event_viewer.py` | PyQt6 event-by-event GUI (EVIO file → live reconstruction). |
| `gem_cluster_view.py` | Static plotter for `gem_dump -m evdump` JSON output. |
| `gem_layout.py` | Visualises the strip geometry from `gem_map.json`. |
| `gem_strip_map.py` | Thin wrapper over `prad2py.det.map_strip` (library). |
| `gem_view.py` | Matplotlib rendering and `GemSystem` adapters (library). |
| `check_strip_map.py` | Dev: cross-validates the pipeline against PRadAnalyzer and `mpd_gem_view_ssp`. |
| `CMakeLists.txt` | Builds `gem_dump`; installs the binary and the Python scripts. |

## Common CLI conventions

Every tool here takes the same short / long flag pairs for configuration files:

| Short | Long | Meaning |
|---|---|---|
| `-D` | `--daq-config` | `daq_config.json` (`gem_dump`). |
| `-G` | `--gem-map` | `gem_map.json` (all tools). |
| `-P` | `--gem-ped` | `gem_ped.json` (`gem_dump`, `gem_event_viewer`). |
| `-o` | `--output` | output file (`gem_dump`, `gem_cluster_view`). |

When a flag is omitted, the Python tools look in `$PRAD2_DATABASE_DIR`
first (set by `prad2_setup.sh` / `prad2_setup.csh`), then next to the
script, then in CWD-relative fallbacks.  `gem_dump`'s C++ resolver
applies the same policy via `prad2::resolve_data_dir()`.

## Detector facts (as of 2026-04-18)

### Geometry

- **Four identical GEMs** — GEM0 + GEM1 (upstream plane), GEM2 + GEM3 (downstream).  Each plane consists of two half-sensors overlapping at the beam line; one is rotated 180° in XY so the beam holes align.
- **Active area:** 12 X-APVs × 128 ch at 0.4 mm pitch (X), 24 Y-APVs × 128 ch at 0.4 mm (Y).
- **Beam hole:** a 52 × 52 mm rectangle on the beam side, centred in Y.  The exact location is derived at runtime from the "match" APV strip positions — see the `hole` block in `gem_map.json`.

### Readout

- **Two crates** — `0x31` (`gemroc1`) and `0x34` (`gemroc2`), 72 APVs each → **144 APVs total**, evenly distributed across the four detectors (36 APVs per GEM).
- **52 MPDs / event** (observed in run 001137).  Each MPD hosts up to 16 APVs over individual fibres.
- **APV25 front-end:** 128 analogue channels multiplexed to a single ADC.  SSP firmware samples 6 time bins per trigger (13-bit signed ADC).
- **Bank tag:** `0x0DE9` (Hall-A standard MPD raw format; confirmed in run 001137).  Nested under the ROC bank (`0x31` or `0x34`) inside the physics-event bank (`0x82`).

### Data modes

| Mode | `ApvData.nstrips` | Typical use | Offline pipeline |
|---|---|---|---|
| **Full readout** | `== 128` (all channels sent) | Pedestal calibration runs and debug modes with CM passthrough. | `processApv` runs the full offline chain (pedestal subtract → sorting common-mode → `noise × zero_sup_threshold` ZS). |
| **Online ZS** | `< 128` (firmware dropped some) | Production runs. | `processApv` short-circuits: every strip present in the bank is a surviving hit (firmware already did pedestal + CM + ZS); no pedestal file required. |

The mode is auto-detected per APV from the `nstrips` count, **not**
from `has_online_cm` — the MPD can emit type-`0xD` CM-debug headers
(setting `has_online_cm = true`) while still sending all 128 strips
raw.  `nstrips < APV_STRIP_SIZE` is the only signal that reliably
indicates "firmware dropped some channels, so it also pedestal-subtracted."
See `prad2det/src/GemSystem.cpp`.

### Strip mapping (six-step pipeline)

The same code is shared between the live reconstruction
(`gem::MapStrip` in `prad2det/src/GemSystem.cpp`) and these scripts
(`gem_strip_map.py` wraps the same C++ entry point through
`prad2py.det`).  Per-APV configuration lives in `gem_map.json`:

| Field | Default | Meaning |
|---|---|---|
| `pos` | — | APV position on its plane (0 .. *n_apvs* − 1). |
| `orient` | 0 | `1` flips the APV (`strip → 127 − strip`). |
| `pin_rotate` | 0 | Rotated connector pins — `16` for `pos = 11` near the beam hole. |
| `shared_pos` | −1 | "Share another APV's plane slot" (−1 = use `pos`). |
| `hybrid_board` | `true` | MPD hybrid-board pin conversion (`false` for SRS). |
| `match` | `""` | `"+Y"` or `"-Y"` marks split APVs above / below the beam hole. |

Full geometry details — and the rationale for parameters that cannot
be collapsed — live in `memory/project_gem_geometry.md`.

## Typical workflows

### Open an event interactively

```bash
gem_event_viewer /volatile/hallb/prad/<run>/<file>.evio.00000
```

The GUI pre-scans the file (progress bar) and lets you step through
events with **Prev** / **Next** / **Goto**.  Threshold sliders re-run
reconstruction on cached SSP data, so adjustments cost no disk I/O.
The advanced tuning dock exposes every `GemSystem` and `GemCluster`
knob.

### Dump and visualise interesting events

```bash
# Pick up to 10 matching events; each is written to <stem>_<evnum>.json.
gem_dump -m evdump run.evio.00000 -P gem_ped.json \
         -n 10 -f clusters=2:3 -o /tmp/evt.json
# produces /tmp/evt_3.json, /tmp/evt_6.json, /tmp/evt_15.json, ...

# Render every dumped event → one PNG per JSON input.
gem_cluster_view /tmp/evt_*.json             # bash / zsh: shell expansion
gem_cluster_view "/tmp/evt_*.json"           # tcsh: quote so we expand
```

`-f` is a boolean filter — `clusters=2:3` reads "≥ 2 clusters in ≥ 3
detectors".  See `gem_dump --help` for the full grammar.  `-n` rules:

| `-n K` | Behaviour |
|---|---|
| omitted | Dump 1 event (default; single `<stem>.json` with no suffix). |
| `-n 1` | Same as omitted. |
| `-n K` (K ≥ 2) | Dump up to *K* matching events → suffixed files. |
| `-n 0` | Dump every matching event. |
| `-e N` | Dump only event *N*, ignoring `-n` and `-f`. |

### Pedestal calibration (full-readout test data only)

```bash
# 1. Compute per-strip pedestals from a full-readout run.
gem_dump -m ped /volatile/.../gem0gem1_001137.evio.00000 \
         -o /volatile/.../gem0gem1_001137/gem_ped.json

# 2. Run any downstream analysis with those pedestals loaded.
gem_dump -m clusters /volatile/.../gem0gem1_001137.evio.00000 \
         -P /volatile/.../gem0gem1_001137/gem_ped.json -n 20

# gem_event_viewer takes the same file via --gem-ped:
gem_event_viewer /volatile/.../gem0gem1_001137.evio.00000 \
                 --gem-ped /volatile/.../gem0gem1_001137/gem_ped.json
```

Production runs with online ZS skip step 1 — `processApv` bypasses
the offline pedestal/CM chain automatically.

**No auto-discovery.** Pedestals are per-run calibration products,
and a wrong file is worse than none.  Both `gem_dump` and
`gem_event_viewer` require the pedestal path to be passed explicitly
(`-P` / `--gem-ped`).  When run against full-readout data without
one, they emit a loud warning to stderr or a modal dialog rather than
silently reconstructing empty events.

### Summary diagnostics

```bash
gem_dump -m summary <file>.evio.00000 -n 100
```

Prints per-event MPD / APV / strip / hit / cluster / 2-D-hit counts.
A useful first step on a new file: non-zero strips → firmware is
sending data; non-zero hits → pedestals are working / online-ZS is
active; non-zero 2-D hits → XY matching is succeeding.

### Visualise the strip layout

```bash
gem_layout                           # auto-find gem_map.json via $PRAD2_DATABASE_DIR
gem_layout -G path/to/alt_map.json   # override
```

Draws every strip of one detector (all four are identical), overlays
APV boundaries and the beam hole, and writes `gem_layout.png`.

### Dev sanity check

```bash
python gem/check_strip_map.py                                            # from source tree
python $PRAD2_DIR/share/prad2evviewer/gem/check_strip_map.py             # from install
```

For every APV in `gem_map.json`, the script maps all 128 channels
through the reference implementations from `mpd_gem_view_ssp` and
`PRadAnalyzer` and through our own pipeline, and asserts that the
three agree.  Run after any change to the strip-mapping pipeline.

## References

- `prad2det/src/GemSystem.cpp` — pedestal / CM / ZS plus the strip-mapping implementation.
- `prad2det/src/GemCluster.cpp` — strip clustering and X/Y matching.
- `prad2det/src/PipelineBuilder.cpp` — one-stop wiring used by the live monitor and offline tools.
- `prad2dec/src/SspDecoder.cpp` — SSP / MPD / APV bitfield decoder.
- `database/gem_map.json` — detector geometry and per-APV mapping (source of truth).
- `docs/rols/banktags.md` — bank-tag reference, including MPD `0x0DE9`.
- `memory/project_gem_geometry.md` — deep-dive on the six-step mapping pipeline.
