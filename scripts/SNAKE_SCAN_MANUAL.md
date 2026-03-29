# HyCal Snake Scan -- Operator Manual

**HyCal Module Scanner for PRad-II, Jefferson Lab Hall B**

---

## Overview

Automates beam calibration of HyCal modules in a serpentine pattern.
Module positions are loaded from `database/hycal_modules.json`
(1152 PbWO4, 576 PbGlass). The scan always includes all PbWO4 modules; surrounding PbGlass layers
(0--6) are configurable in the GUI. The map always shows both types.

At each module the transporter pauses for a configurable dwell time
(default 120 s), then advances to the next.
A full PbWO4 scan takes ~40 hours at default settings.

**Safety:** The tool only writes to four EPICS PVs (`ptrans_{x,y}.VAL`
and `ptrans_{x,y}.SPMG`). All other channels are read-only.

---

## Requirements

| Component | Notes |
|-----------|-------|
| Python 3.8+ | Tkinter included on most platforms |
| pyepics | Only for `--real` mode: `pip install pyepics` |
| EPICS CA | `EPICS_CA_ADDR_LIST` must be set for real mode |

---

## Launching

```bash
# Simulation (no EPICS needed, title bar shows [SIMULATION])
python scripts/hycal_snake_scan.py

# Real EPICS
export EPICS_CA_ADDR_LIST="129.57.255.255"
export EPICS_CA_AUTO_ADDR_LIST="YES"
python scripts/hycal_snake_scan.py --real

# Custom database path
python scripts/hycal_snake_scan.py --database /path/to/hycal_modules.json
```

---

## Coordinate System

Beam-centre calibration offsets:

| Beam at HyCal (0,0) | ptrans_x = **-126.75 mm** | ptrans_y = **10.11 mm** |
|---|---|---|

To place the beam on a module at (mx, my) in HyCal coordinates:

```
ptrans_x = -126.75 - mx
ptrans_y =   10.11 - my
```

---

## Controls Quick Reference

### Scan Controls

| Control | What it does |
|---------|-------------|
| **LG layers (0-6)** | Number of PbGlass layers around PbWO4 to include (0 = PbWO4 only, 6 = all). Locked during scan. |
| **Dwell time** | Seconds at each module (default 120) |
| **Pos. threshold** | Max allowed position error in mm (default 0.5) |
| **Start module** | Pick via dropdown or click the module map |
| **Start Scan** | Begin from selected module through end of snake path |
| **Pause / Resume** | Pause motors (SPMG=1) and freeze dwell; Resume sets SPMG=3 |
| **Stop** | Abort scan, stop motors (SPMG=0), return to IDLE |
| **Skip Module** | During dwell only: skip remaining wait, advance to next module |
| **Ack Error** | After a position error: acknowledge and continue to next module |

### Direct Controls

| Button | What it does |
|--------|-------------|
| **Move to Selected Module** | Move beam to the selected module without starting a scan |
| **Reset to Beam Center** | Return transporter to (-126.75, 10.11) -- beam at HyCal centre |

### Module Map Colors

| Color | Meaning |
|-------|---------|
| Dark grey | Not visited | Yellow | Moving to |
| Green | Dwelling | Blue | Completed |
| Red | Position error | Orange | Selected start |

### Scan States

IDLE (grey) -- MOVING (yellow) -- DWELLING (green) -- PAUSED (orange) -- ERROR (red) -- COMPLETED (blue)

---

## Common Procedures

### Full Scan

1. Set dwell time and position threshold.
2. Ensure start module is **W1** (default).
3. Click **Start Scan**.

### Resume After Interruption

1. Find the last completed module (blue on map or in event log).
2. Select the **next** module via dropdown or map click.
3. Click **Start Scan**.

### Handle Position Error

The scan auto-pauses when position error exceeds threshold.
- **Ack Error** to skip and continue, or **Stop** to abort.
- Frequent errors: check motor speed, backlash settings, encoder health.

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| `ModuleNotFoundError: tkinter` | Install `python3-tk` or reinstall Python with Tk |
| `ModuleNotFoundError: epics` | `pip install pyepics` (only needed for `--real`) |
| PVs not connecting | Verify `EPICS_CA_ADDR_LIST`, IOC running, firewall (ports 5064/5065) |
| Motors don't move | Check SPMG = 3 (Go), hardware interlocks, motor enable, limit switches |
| Move timeout (>300 s) | Motor stall, limit switch, IOC down, or external SPMG change |

---

## EPICS PV Reference

### Written by this tool

| PV | Purpose |
|----|---------|
| `ptrans_{x,y}.VAL` | Absolute position set-point (mm) |
| `ptrans_{x,y}.SPMG` | Motor mode: Stop(0) Pause(1) Move(2) Go(3) |

### Move sequence per module

1. Write `.VAL` for both axes
2. Write `.SPMG` = 3 (Go) for both axes
3. Poll `.MOVN` until both = 0
4. Verify `.RBV` against target

### Key monitored PVs

`.RBV` (readback), `.MOVN` (in motion), `.SPMG` (mode), `.VELO` (velocity),
`.MSTA` (status word), `hallb_ptrans_{x,y}_encoder` (raw encoder)

---

## Module Grid & Snake Path

Module positions loaded from `database/hycal_modules.json`:

| Type | Count | Size (mm) | Extent |
|------|-------|-----------|--------|
| PbWO4 | 1152 (34x34 minus beam hole) | 20.77 x 20.75 | +/-342.7 mm |
| PbGlass | 576 (outer ring) | 38.15 x 38.15 | +/-562.9 mm |

Even rows scan left-to-right, odd rows right-to-left (serpentine).

```
Row 0:  W1  -> W2  -> ... -> W34    (L->R)
Row 1:  W68 -> W67 -> ... -> W35    (R->L)
Row 2:  W69 -> W70 -> ... -> W102   (L->R)
...
```
