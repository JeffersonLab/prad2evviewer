#!/usr/bin/env python3
"""
HyCal Snake Scan -- PbWO4 Module Scanner
=========================================
Tkinter GUI that drives the HyCal transporter in a snake pattern so the
beam centres on every inner PbWO4 module, dwells for a configurable time,
then advances to the next module.

Usage
-----
    python hycal_snake_scan.py              # simulation mode (no EPICS)
    python hycal_snake_scan.py --real       # real EPICS mode

Coordinate system
-----------------
    ptrans_x, ptrans_y = (-126.75, 10.11)  -->  beam at HyCal centre (0,0)
    ptrans_x = BEAM_CENTER_X - module_x
    ptrans_y = BEAM_CENTER_Y - module_y

Writable PVs (the ONLY PVs this tool writes to):
    ptrans_x.VAL / ptrans_y.VAL    -- absolute set-point
    ptrans_x.SPMG / ptrans_y.SPMG  -- motor mode  Stop(0) Pause(1) Move(2) Go(3)

All other PVs are read-only for monitoring.

Requirements
------------
    Python 3.8+
    pyepics  (only for --real mode)
"""

from __future__ import annotations

import argparse
import math
import random
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from typing import Dict, List, Optional, Tuple


# ============================================================================
#  CONSTANTS
# ============================================================================

# Transporter coordinates when the beam hits HyCal centre (0, 0)
BEAM_CENTER_X: float = -126.75   # mm
BEAM_CENTER_Y: float = 10.11     # mm

# PbWO4 module grid parameters (from module position map)
PBWO4_NX = 34           # columns
PBWO4_NY = 34           # rows
PBWO4_SX = 20.77        # module pitch x  (mm)
PBWO4_SY = 20.75        # module pitch y  (mm)
PBWO4_X0 = -342.705     # first column centre x  (HyCal frame)
PBWO4_Y0 = 342.375      # first row centre y      (HyCal frame, top row)

DEFAULT_DWELL = 120.0    # seconds
DEFAULT_POS_THRESHOLD = 0.5   # mm  -- alert if |RBV - target| exceeds this
MOVE_TIMEOUT = 300.0     # seconds per single move

SPMG_LABELS = {0: "Stop", 1: "Pause", 2: "Move", 3: "Go"}


class SPMG(IntEnum):
    STOP = 0
    PAUSE = 1
    MOVE = 2
    GO = 3


# -- EPICS PV names ----------------------------------------------------------

class PV:
    """All EPICS PV names used by this tool."""
    # ---- writable ----
    X_VAL  = "ptrans_x.VAL"
    Y_VAL  = "ptrans_y.VAL"
    X_SPMG = "ptrans_x.SPMG"
    Y_SPMG = "ptrans_y.SPMG"
    # ---- read-only monitoring ----
    X_ENCODER = "hallb_ptrans_x_encoder"
    Y_ENCODER = "hallb_ptrans_y_encoder"
    X_RBV  = "ptrans_x.RBV";   Y_RBV  = "ptrans_y.RBV"
    X_MOVN = "ptrans_x.MOVN";  Y_MOVN = "ptrans_y.MOVN"
    X_VELO = "ptrans_x.VELO";  Y_VELO = "ptrans_y.VELO"
    X_ACCL = "ptrans_x.ACCL";  Y_ACCL = "ptrans_y.ACCL"
    X_TDIR = "ptrans_x.TDIR";  Y_TDIR = "ptrans_y.TDIR"
    X_MSTA = "ptrans_x.MSTA";  Y_MSTA = "ptrans_y.MSTA"
    X_ATHM = "ptrans_x.ATHM";  Y_ATHM = "ptrans_y.ATHM"
    X_PREC = "ptrans_x.PREC";  Y_PREC = "ptrans_y.PREC"
    X_BVEL = "ptrans_x.BVEL";  Y_BVEL = "ptrans_y.BVEL"
    X_BACC = "ptrans_x.BACC";  Y_BACC = "ptrans_y.BACC"
    X_VBAS = "ptrans_x.VBAS";  Y_VBAS = "ptrans_y.VBAS"
    X_BDST = "ptrans_x.BDST";  Y_BDST = "ptrans_y.BDST"
    X_FRAC = "ptrans_x.FRAC";  Y_FRAC = "ptrans_y.FRAC"


# -- Colour palette (dark control-room theme) --------------------------------

class C:
    BG       = "#0d1117"
    PANEL    = "#161b22"
    BORDER   = "#30363d"
    TEXT     = "#c9d1d9"
    DIM      = "#8b949e"
    ACCENT   = "#58a6ff"
    GREEN    = "#3fb950"
    YELLOW   = "#d29922"
    RED      = "#f85149"
    ORANGE   = "#db6d28"
    # canvas module states
    MOD_TODO      = "#21262d"
    MOD_CURRENT   = "#d29922"
    MOD_DWELL     = "#3fb950"
    MOD_DONE      = "#1f6feb"
    MOD_ERROR     = "#f85149"
    MOD_SELECTED  = "#db6d28"


# ============================================================================
#  MODULE MAP & SNAKE PATH
# ============================================================================

@dataclass
class Module:
    name: str
    x: float          # centre x in HyCal frame (mm)
    y: float          # centre y in HyCal frame (mm)
    row: int           # grid row index  (0 = top)
    col: int           # grid col index  (0 = left)


def generate_modules() -> List[Module]:
    """Create the 34x34 PbWO4 module grid."""
    modules: List[Module] = []
    idx = 1
    for row in range(PBWO4_NY):
        y = PBWO4_Y0 - row * PBWO4_SY
        for col in range(PBWO4_NX):
            x = PBWO4_X0 + col * PBWO4_SX
            modules.append(Module(f"W{idx}", round(x, 3), round(y, 3), row, col))
            idx += 1
    return modules


def generate_snake_path(modules: List[Module]) -> List[Module]:
    """Order modules in a snake pattern: row-by-row, alternating direction."""
    rows: Dict[int, List[Module]] = {}
    for m in modules:
        rows.setdefault(m.row, []).append(m)
    path: List[Module] = []
    for r in sorted(rows):
        row_mods = sorted(rows[r], key=lambda m: m.col)
        if r % 2 == 1:          # odd rows: right-to-left
            row_mods.reverse()
        path.extend(row_mods)
    return path


def module_to_ptrans(mx: float, my: float) -> Tuple[float, float]:
    """HyCal-frame module centre --> transporter set-point."""
    return (BEAM_CENTER_X - mx, BEAM_CENTER_Y - my)


def ptrans_to_module(px: float, py: float) -> Tuple[float, float]:
    """Transporter position --> HyCal-frame coordinates."""
    return (BEAM_CENTER_X - px, BEAM_CENTER_Y - py)


# ============================================================================
#  EPICS INTERFACES
# ============================================================================

# Map from short key to PV name
_PV_MAP: List[Tuple[str, str]] = [
    # writable
    ("x_val",     PV.X_VAL),     ("y_val",     PV.Y_VAL),
    ("x_spmg",    PV.X_SPMG),   ("y_spmg",    PV.Y_SPMG),
    # read-only
    ("x_encoder", PV.X_ENCODER), ("y_encoder", PV.Y_ENCODER),
    ("x_rbv",     PV.X_RBV),    ("y_rbv",     PV.Y_RBV),
    ("x_movn",    PV.X_MOVN),   ("y_movn",    PV.Y_MOVN),
    ("x_velo",    PV.X_VELO),   ("y_velo",    PV.Y_VELO),
    ("x_accl",    PV.X_ACCL),   ("y_accl",    PV.Y_ACCL),
    ("x_tdir",    PV.X_TDIR),   ("y_tdir",    PV.Y_TDIR),
    ("x_msta",    PV.X_MSTA),   ("y_msta",    PV.Y_MSTA),
    ("x_athm",    PV.X_ATHM),   ("y_athm",    PV.Y_ATHM),
    ("x_prec",    PV.X_PREC),   ("y_prec",    PV.Y_PREC),
    ("x_bvel",    PV.X_BVEL),   ("y_bvel",    PV.Y_BVEL),
    ("x_bacc",    PV.X_BACC),   ("y_bacc",    PV.Y_BACC),
    ("x_vbas",    PV.X_VBAS),   ("y_vbas",    PV.Y_VBAS),
    ("x_bdst",    PV.X_BDST),   ("y_bdst",    PV.Y_BDST),
    ("x_frac",    PV.X_FRAC),   ("y_frac",    PV.Y_FRAC),
]


class RealEPICS:
    """Channel-access interface using pyepics."""

    def __init__(self):
        import epics as _epics          # type: ignore
        self._epics = _epics
        self._pvs: Dict[str, object] = {}

    def connect(self) -> Tuple[int, int]:
        for key, pvname in _PV_MAP:
            self._pvs[key] = self._epics.PV(pvname, connection_timeout=5.0)
        time.sleep(2.0)
        n = sum(1 for p in self._pvs.values() if p.connected)
        return n, len(self._pvs)

    def get(self, key: str, default=None):
        pv = self._pvs.get(key)
        if pv and pv.connected:
            v = pv.get()
            return v if v is not None else default
        return default

    def put(self, key: str, value) -> bool:
        pv = self._pvs.get(key)
        if pv and pv.connected:
            pv.put(value)
            return True
        return False


class SimulatedEPICS:
    """In-process motor simulation -- no EPICS needed."""

    def __init__(self):
        self._lock = threading.Lock()
        self._x = BEAM_CENTER_X
        self._y = BEAM_CENTER_Y
        self._tx = self._x
        self._ty = self._y
        self._x_spmg = int(SPMG.GO)
        self._y_spmg = int(SPMG.GO)
        self._x_movn = 0
        self._y_movn = 0
        self._speed = 50.0        # mm/s  (fast for simulation)
        self._moving = False
        self._thread: Optional[threading.Thread] = None

    def connect(self) -> Tuple[int, int]:
        return (0, 0)              # always "OK" in simulation

    # -- read ----------------------------------------------------------------

    def get(self, key: str, default=None):
        with self._lock:
            return {
                "x_encoder": round(self._x + random.gauss(0, 0.002), 4),
                "y_encoder": round(self._y + random.gauss(0, 0.002), 4),
                "x_rbv": round(self._x, 3),
                "y_rbv": round(self._y, 3),
                "x_val": round(self._tx, 3),
                "y_val": round(self._ty, 3),
                "x_movn": self._x_movn,
                "y_movn": self._y_movn,
                "x_spmg": self._x_spmg,
                "y_spmg": self._y_spmg,
                "x_velo": self._speed,
                "y_velo": self._speed,
                "x_accl": 2.0,  "y_accl": 2.0,
                "x_tdir": 1 if self._tx >= self._x else 0,
                "y_tdir": 1 if self._ty >= self._y else 0,
                "x_msta": 0x10B, "y_msta": 0x10B,
                "x_athm": int(abs(self._x - BEAM_CENTER_X) < 1.0),
                "y_athm": int(abs(self._y - BEAM_CENTER_Y) < 1.0),
                "x_prec": 3, "y_prec": 3,
                "x_bvel": 1.0, "y_bvel": 1.0,
                "x_bacc": 1.0, "y_bacc": 1.0,
                "x_vbas": 0.5, "y_vbas": 0.5,
                "x_bdst": 0.0, "y_bdst": 0.0,
                "x_frac": 1.0, "y_frac": 1.0,
            }.get(key, default)

    # -- write (ONLY x_val, y_val, x_spmg, y_spmg) --------------------------

    def put(self, key: str, value) -> bool:
        with self._lock:
            if key == "x_val":
                self._tx = float(value)
            elif key == "y_val":
                self._ty = float(value)
            elif key == "x_spmg":
                self._x_spmg = int(value)
            elif key == "y_spmg":
                self._y_spmg = int(value)
            else:
                return False
            self._evaluate_motion()
        return True

    def _evaluate_motion(self):
        """Start / stop simulated motion based on SPMG state."""
        if self._x_spmg == SPMG.STOP or self._y_spmg == SPMG.STOP:
            self._moving = False
            self._x_movn = 0
            self._y_movn = 0
        elif self._x_spmg == SPMG.PAUSE or self._y_spmg == SPMG.PAUSE:
            self._moving = False
            self._x_movn = 0
            self._y_movn = 0
        elif self._x_spmg == SPMG.GO and self._y_spmg == SPMG.GO:
            if not self._moving:
                self._moving = True
                t = threading.Thread(target=self._run_move, daemon=True)
                t.start()

    def _run_move(self):
        dt = 0.02
        while True:
            with self._lock:
                if not self._moving:
                    self._x_movn = 0
                    self._y_movn = 0
                    return
                dx = self._tx - self._x
                dy = self._ty - self._y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < 0.001:
                    self._x, self._y = self._tx, self._ty
                    self._x_movn = 0
                    self._y_movn = 0
                    self._moving = False
                    return
                step = min(self._speed * dt, dist)
                r = step / dist
                self._x += dx * r
                self._y += dy * r
                self._x_movn = 1 if abs(dx) > 0.001 else 0
                self._y_movn = 1 if abs(dy) > 0.001 else 0
            time.sleep(dt)


# -- helpers shared by both interfaces --------------------------------------

def epics_move_to(ep, x: float, y: float):
    """Command a move:  set VAL then ensure SPMG = Go."""
    ep.put("x_val", x)
    ep.put("y_val", y)
    ep.put("x_spmg", int(SPMG.GO))
    ep.put("y_spmg", int(SPMG.GO))

def epics_stop(ep):
    ep.put("x_spmg", int(SPMG.STOP))
    ep.put("y_spmg", int(SPMG.STOP))

def epics_pause(ep):
    ep.put("x_spmg", int(SPMG.PAUSE))
    ep.put("y_spmg", int(SPMG.PAUSE))

def epics_resume(ep):
    ep.put("x_spmg", int(SPMG.GO))
    ep.put("y_spmg", int(SPMG.GO))

def epics_is_moving(ep) -> bool:
    return bool(ep.get("x_movn", 0)) or bool(ep.get("y_movn", 0))

def epics_read_rbv(ep) -> Tuple[float, float]:
    return (ep.get("x_rbv", 0.0), ep.get("y_rbv", 0.0))


# ============================================================================
#  SCAN ENGINE  (runs in a background thread)
# ============================================================================

class ScanState:
    IDLE      = "IDLE"
    MOVING    = "MOVING"
    DWELLING  = "DWELLING"
    PAUSED    = "PAUSED"
    ERROR     = "ERROR"
    COMPLETED = "COMPLETED"


class ScanEngine:
    """Drives the snake scan in a background thread."""

    def __init__(self, epics, modules: List[Module], log_fn):
        self.ep = epics
        self.all_modules = modules
        self.path = generate_snake_path(modules)
        self.log = log_fn                # log_fn(msg, level="info")

        # -- tunables (set before start) --
        self.dwell_time: float = DEFAULT_DWELL
        self.pos_threshold: float = DEFAULT_POS_THRESHOLD

        # -- runtime state (read by GUI) --
        self.state: str = ScanState.IDLE
        self.current_idx: int = 0        # index into self.path
        self.dwell_remaining: float = 0.0
        self.completed: set = set()      # indices that finished successfully
        self.error_modules: set = set()  # indices that had position errors

        # -- thread control --
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._skip = threading.Event()
        self._paused = False
        self._ack_error = threading.Event()   # set by GUI to acknowledge error

    # -- public API (called from GUI / main thread) --------------------------

    @property
    def current_module(self) -> Optional[Module]:
        if 0 <= self.current_idx < len(self.path):
            return self.path[self.current_idx]
        return None

    @property
    def progress_text(self) -> str:
        done = len(self.completed)
        total = len(self.path)
        return f"{done}/{total}"

    @property
    def eta_seconds(self) -> float:
        remaining = len(self.path) - len(self.completed)
        avg_move = 4.0    # rough estimate seconds per move
        return remaining * (avg_move + self.dwell_time)

    def start(self, start_idx: int = 0):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._skip.clear()
        self._paused = False
        self.current_idx = start_idx
        self.completed.clear()
        self.error_modules.clear()
        self.state = ScanState.MOVING
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def resume_scan(self):
        """Resume from pause (during move or dwell)."""
        if self.state == ScanState.PAUSED:
            self._paused = False
            epics_resume(self.ep)
            self.state = ScanState.MOVING
            self.log("Scan resumed")

    def pause_scan(self):
        if self.state in (ScanState.MOVING, ScanState.DWELLING):
            self._paused = True
            epics_pause(self.ep)
            self.state = ScanState.PAUSED
            self.log("Scan paused", level="warn")

    def stop_scan(self):
        self._stop.set()
        self._skip.set()          # unblock any waits
        self._ack_error.set()     # unblock error wait
        self._paused = False
        epics_stop(self.ep)
        self.log("Scan stopped", level="warn")

    def skip_module(self):
        self._skip.set()

    def acknowledge_error(self):
        """User acknowledges position error -- scan continues."""
        self._ack_error.set()

    # -- background thread ---------------------------------------------------

    def _run(self):
        self.log(f"Scan started from {self.path[self.current_idx].name}, "
                 f"dwell {self.dwell_time:.0f}s, {len(self.path)} modules")
        try:
            for i in range(self.current_idx, len(self.path)):
                if self._stop.is_set():
                    break
                self.current_idx = i
                mod = self.path[i]
                px, py = module_to_ptrans(mod.x, mod.y)

                # -- move --
                self.state = ScanState.MOVING
                self.log(f"[{i+1}/{len(self.path)}] Moving to {mod.name} "
                         f"  ptrans({px:.3f}, {py:.3f})")
                epics_move_to(self.ep, px, py)

                if not self._wait_move_done(px, py):
                    break

                # -- check position error --
                rbv_x, rbv_y = epics_read_rbv(self.ep)
                err = math.sqrt((rbv_x - px)**2 + (rbv_y - py)**2)
                if err > self.pos_threshold:
                    self.error_modules.add(i)
                    self.state = ScanState.ERROR
                    self.log(f"POSITION ERROR at {mod.name}: "
                             f"error={err:.3f} mm  (threshold {self.pos_threshold})",
                             level="error")
                    self._ack_error.clear()
                    self._ack_error.wait()        # block until user acknowledges
                    if self._stop.is_set():
                        break

                # -- dwell --
                self.state = ScanState.DWELLING
                self.dwell_remaining = self.dwell_time
                self.log(f"Dwelling at {mod.name} for {self.dwell_time:.0f}s")
                result = self._wait_dwell()
                if result == "stop":
                    break

                self.completed.add(i)
                self.dwell_remaining = 0.0

        finally:
            if self._stop.is_set():
                self.state = ScanState.IDLE
                self.log("Scan stopped by user")
            elif self.current_idx >= len(self.path) - 1:
                self.state = ScanState.COMPLETED
                self.log("Scan COMPLETE -- all modules visited!", level="warn")
            else:
                self.state = ScanState.IDLE

    def _wait_move_done(self, target_x: float, target_y: float) -> bool:
        """Wait until MOVN=0 on both axes.  Returns False if stopped."""
        t0 = time.time()
        while not self._stop.is_set():
            # handle pause
            while self._paused and not self._stop.is_set():
                self.state = ScanState.PAUSED
                time.sleep(0.1)
            if self._stop.is_set():
                return False
            self.state = ScanState.MOVING
            if not epics_is_moving(self.ep):
                return True
            if time.time() - t0 > MOVE_TIMEOUT:
                self.log(f"MOVE TIMEOUT after {MOVE_TIMEOUT:.0f}s", level="error")
                return False
            time.sleep(0.1)
        return False

    def _wait_dwell(self) -> str:
        """Wait for dwell_time seconds.  Returns 'done', 'skip', or 'stop'."""
        end = time.time() + self.dwell_time
        while time.time() < end:
            if self._stop.is_set():
                return "stop"
            if self._skip.is_set():
                self._skip.clear()
                self.log("Module skipped by user")
                return "skip"
            while self._paused and not self._stop.is_set():
                end += 0.1       # freeze the countdown while paused
                self.state = ScanState.PAUSED
                time.sleep(0.1)
            if self._stop.is_set():
                return "stop"
            self.state = ScanState.DWELLING
            self.dwell_remaining = max(0.0, end - time.time())
            time.sleep(0.1)
        return "done"


# ============================================================================
#  GUI
# ============================================================================

class SnakeScanGUI:

    CANVAS_SIZE = 510       # pixels
    CANVAS_PAD  = 5

    def __init__(self, root: tk.Tk, epics, simulation: bool):
        self.root = root
        self.ep = epics
        self.simulation = simulation

        self.modules = generate_modules()
        self.engine = ScanEngine(epics, self.modules, self._log)

        # module index lookup by (row, col)
        self._mod_grid: Dict[Tuple[int, int], int] = {}
        for idx, m in enumerate(self.engine.path):
            self._mod_grid[(m.row, m.col)] = idx

        self._selected_start_idx = 0
        self._log_lines: List[str] = []

        # canvas item ids per (row, col)
        self._cell_ids: Dict[Tuple[int, int], int] = {}

        self._build_ui()
        self._poll()

    # -----------------------------------------------------------------------
    #  UI construction
    # -----------------------------------------------------------------------

    def _build_ui(self):
        self.root.title("HyCal Snake Scan" +
                        ("  [SIMULATION]" if self.simulation else "  [REAL EPICS]"))
        self.root.configure(bg=C.BG)
        self.root.resizable(True, True)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=C.BG, foreground=C.TEXT,
                         fieldbackground=C.PANEL, bordercolor=C.BORDER)
        style.configure("TLabel", background=C.BG, foreground=C.TEXT)
        style.configure("TLabelframe", background=C.BG, foreground=C.ACCENT)
        style.configure("TLabelframe.Label", background=C.BG,
                         foreground=C.ACCENT, font=("Consolas", 9, "bold"))
        style.configure("TButton", background=C.PANEL, foreground=C.TEXT,
                         padding=4)
        style.map("TButton",
                  background=[("active", C.BORDER)],
                  foreground=[("disabled", "#484f58")])
        style.configure("Accent.TButton", background="#1f6feb",
                         foreground="white")
        style.configure("Danger.TButton", background="#da3633",
                         foreground="white")
        style.configure("Warn.TButton", background="#9e6a03",
                         foreground="white")
        style.configure("Green.TButton", background="#238636",
                         foreground="white")

        # -- top status bar --------------------------------------------------
        top = tk.Frame(self.root, bg="#0d1520", height=32)
        top.pack(fill="x")
        tk.Label(top, text="  HYCAL SNAKE SCAN  ",
                 bg="#0d1520", fg=C.GREEN,
                 font=("Consolas", 13, "bold")).pack(side="left", padx=8)
        mode_text = "SIMULATION" if self.simulation else "REAL EPICS"
        mode_fg = C.YELLOW if self.simulation else C.GREEN
        tk.Label(top, text=mode_text, bg="#0d1520", fg=mode_fg,
                 font=("Consolas", 9, "bold")).pack(side="left", padx=4)

        self._lbl_state = tk.Label(top, text="IDLE", bg="#0d1520",
                                    fg=C.DIM, font=("Consolas", 10, "bold"))
        self._lbl_state.pack(side="right", padx=12)

        # -- main area -------------------------------------------------------
        main = tk.Frame(self.root, bg=C.BG)
        main.pack(fill="both", expand=True, padx=6, pady=4)

        # column 0: canvas
        left = tk.Frame(main, bg=C.BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        # column 1: controls + status
        right = tk.Frame(main, bg=C.BG)
        right.grid(row=0, column=1, sticky="nsew")

        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        self._build_canvas(left)
        self._build_controls(right)

        # -- bottom: log -----------------------------------------------------
        log_frame = ttk.LabelFrame(self.root, text=" Event Log ")
        log_frame.pack(fill="both", expand=False, padx=6, pady=(0, 6))

        self._log_text = tk.Text(log_frame, height=8, bg="#0d1117",
                                  fg=C.DIM, font=("Consolas", 9),
                                  wrap="word", state="disabled",
                                  borderwidth=0, highlightthickness=0)
        self._log_text.pack(fill="both", expand=True, padx=2, pady=2)
        self._log_text.tag_configure("info", foreground=C.TEXT)
        self._log_text.tag_configure("warn", foreground=C.YELLOW)
        self._log_text.tag_configure("error", foreground=C.RED)

    # -- canvas (module map) -------------------------------------------------

    def _build_canvas(self, parent):
        frm = ttk.LabelFrame(parent, text=" Module Map (PbWO4 34x34) ")
        frm.pack(fill="both", expand=True)

        sz = self.CANVAS_SIZE
        self._canvas = tk.Canvas(frm, width=sz, height=sz,
                                  bg="#0a0e14", highlightthickness=0)
        self._canvas.pack(padx=4, pady=4)
        self._canvas.bind("<Button-1>", self._on_canvas_click)

        self._draw_modules()

        # legend
        leg = tk.Frame(frm, bg=C.BG)
        leg.pack(fill="x", padx=4, pady=(0, 4))
        for label, colour in [("Todo", C.MOD_TODO), ("Moving", C.MOD_CURRENT),
                               ("Dwell", C.MOD_DWELL), ("Done", C.MOD_DONE),
                               ("Error", C.MOD_ERROR), ("Start", C.MOD_SELECTED)]:
            tk.Canvas(leg, width=10, height=10, bg=colour,
                      highlightthickness=0).pack(side="left", padx=(6, 1))
            tk.Label(leg, text=label, bg=C.BG, fg=C.DIM,
                     font=("Consolas", 8)).pack(side="left")

    def _cell_size(self) -> float:
        return (self.CANVAS_SIZE - 2 * self.CANVAS_PAD) / max(PBWO4_NX, PBWO4_NY)

    def _draw_modules(self):
        cs = self._cell_size()
        pad = self.CANVAS_PAD
        gap = max(1, int(cs * 0.06))
        for m in self.modules:
            x0 = pad + m.col * cs + gap
            y0 = pad + m.row * cs + gap
            x1 = pad + (m.col + 1) * cs - gap
            y1 = pad + (m.row + 1) * cs - gap
            rid = self._canvas.create_rectangle(
                x0, y0, x1, y1, fill=C.MOD_TODO, outline="", width=0)
            self._cell_ids[(m.row, m.col)] = rid

    def _on_canvas_click(self, event):
        cs = self._cell_size()
        col = int((event.x - self.CANVAS_PAD) / cs)
        row = int((event.y - self.CANVAS_PAD) / cs)
        col = max(0, min(col, PBWO4_NX - 1))
        row = max(0, min(row, PBWO4_NY - 1))

        # find this module's index in the snake path
        idx = self._mod_grid.get((row, col))
        if idx is not None:
            self._selected_start_idx = idx
            mod = self.engine.path[idx]
            self._start_var.set(mod.name)
            self._log(f"Selected start module: {mod.name} "
                      f"(row {row}, col {col})")

    def _update_canvas(self):
        eng = self.engine
        for i, mod in enumerate(eng.path):
            rid = self._cell_ids.get((mod.row, mod.col))
            if rid is None:
                continue
            if i == eng.current_idx and eng.state == ScanState.DWELLING:
                colour = C.MOD_DWELL
            elif i == eng.current_idx and eng.state in (ScanState.MOVING,
                                                         ScanState.PAUSED):
                colour = C.MOD_CURRENT
            elif i in eng.error_modules:
                colour = C.MOD_ERROR
            elif i in eng.completed:
                colour = C.MOD_DONE
            elif (eng.state == ScanState.IDLE and
                  i == self._selected_start_idx):
                colour = C.MOD_SELECTED
            else:
                colour = C.MOD_TODO
            self._canvas.itemconfigure(rid, fill=colour)

    # -- controls panel ------------------------------------------------------

    def _build_controls(self, parent):
        # === Scan Control ===
        sc = ttk.LabelFrame(parent, text=" Scan Control ")
        sc.pack(fill="x", pady=(0, 4))

        # dwell
        r = tk.Frame(sc, bg=C.BG)
        r.pack(fill="x", padx=6, pady=2)
        tk.Label(r, text="Dwell time (s):", bg=C.BG, fg=C.TEXT,
                 font=("Consolas", 9)).pack(side="left")
        self._dwell_var = tk.DoubleVar(value=DEFAULT_DWELL)
        tk.Spinbox(r, from_=1, to=9999, textvariable=self._dwell_var,
                   width=8, bg=C.PANEL, fg=C.TEXT, font=("Consolas", 9),
                   buttonbackground=C.BORDER,
                   insertbackground=C.TEXT).pack(side="right")

        # threshold
        r2 = tk.Frame(sc, bg=C.BG)
        r2.pack(fill="x", padx=6, pady=2)
        tk.Label(r2, text="Pos. threshold (mm):", bg=C.BG, fg=C.TEXT,
                 font=("Consolas", 9)).pack(side="left")
        self._thresh_var = tk.DoubleVar(value=DEFAULT_POS_THRESHOLD)
        tk.Spinbox(r2, from_=0.01, to=10.0, increment=0.1,
                   textvariable=self._thresh_var,
                   width=8, bg=C.PANEL, fg=C.TEXT, font=("Consolas", 9),
                   buttonbackground=C.BORDER,
                   insertbackground=C.TEXT).pack(side="right")

        # start module selector
        r3 = tk.Frame(sc, bg=C.BG)
        r3.pack(fill="x", padx=6, pady=2)
        tk.Label(r3, text="Start module:", bg=C.BG, fg=C.TEXT,
                 font=("Consolas", 9)).pack(side="left")
        names = [m.name for m in self.engine.path]
        self._start_var = tk.StringVar(value=names[0])
        self._start_combo = ttk.Combobox(r3, textvariable=self._start_var,
                                          values=names, width=10,
                                          font=("Consolas", 9))
        self._start_combo.pack(side="right")
        self._start_combo.bind("<<ComboboxSelected>>", self._on_start_selected)

        # buttons
        bf = tk.Frame(sc, bg=C.BG)
        bf.pack(fill="x", padx=6, pady=6)

        self._btn_start = ttk.Button(bf, text="Start Scan",
                                      style="Green.TButton",
                                      command=self._cmd_start)
        self._btn_start.pack(side="left", expand=True, fill="x", padx=2)

        self._btn_pause = ttk.Button(bf, text="Pause",
                                      style="Warn.TButton",
                                      command=self._cmd_pause)
        self._btn_pause.pack(side="left", expand=True, fill="x", padx=2)

        self._btn_stop = ttk.Button(bf, text="Stop",
                                     style="Danger.TButton",
                                     command=self._cmd_stop)
        self._btn_stop.pack(side="left", expand=True, fill="x", padx=2)

        bf2 = tk.Frame(sc, bg=C.BG)
        bf2.pack(fill="x", padx=6, pady=(0, 6))

        self._btn_skip = ttk.Button(bf2, text="Skip Module",
                                     command=self._cmd_skip)
        self._btn_skip.pack(side="left", expand=True, fill="x", padx=2)

        self._btn_ack = ttk.Button(bf2, text="Ack Error",
                                    style="Warn.TButton",
                                    command=self._cmd_ack_error)
        self._btn_ack.pack(side="left", expand=True, fill="x", padx=2)

        # progress
        self._lbl_progress = tk.Label(sc, text="Progress: --/--",
                                       bg=C.BG, fg=C.TEXT,
                                       font=("Consolas", 9))
        self._lbl_progress.pack(padx=6, anchor="w")
        self._lbl_current = tk.Label(sc, text="Current: --",
                                      bg=C.BG, fg=C.TEXT,
                                      font=("Consolas", 9))
        self._lbl_current.pack(padx=6, anchor="w")
        self._lbl_eta = tk.Label(sc, text="ETA: --",
                                  bg=C.BG, fg=C.DIM,
                                  font=("Consolas", 9))
        self._lbl_eta.pack(padx=6, anchor="w")
        self._lbl_dwell_cd = tk.Label(sc, text="",
                                       bg=C.BG, fg=C.GREEN,
                                       font=("Consolas", 9))
        self._lbl_dwell_cd.pack(padx=6, anchor="w", pady=(0, 4))

        # === Direct Control ===
        dc = ttk.LabelFrame(parent, text=" Direct Control ")
        dc.pack(fill="x", pady=(0, 4))

        dcb = tk.Frame(dc, bg=C.BG)
        dcb.pack(fill="x", padx=6, pady=6)

        ttk.Button(dcb, text="Move to Selected Module",
                   command=self._cmd_move_to_module
                   ).pack(fill="x", pady=1)
        ttk.Button(dcb, text="Reset to Beam Center",
                   style="Accent.TButton",
                   command=self._cmd_reset_center
                   ).pack(fill="x", pady=1)

        # === Motor Status  (scrollable) ===
        ms = ttk.LabelFrame(parent, text=" Motor Status ")
        ms.pack(fill="both", expand=True, pady=(0, 4))

        # use a canvas + frame for scrolling if needed
        inner = tk.Frame(ms, bg=C.BG)
        inner.pack(fill="both", expand=True, padx=4, pady=4)

        self._status_labels: Dict[str, tk.Label] = {}
        self._build_motor_block(inner, "X Motor", [
            ("Encoder",  "x_encoder"), ("RBV",  "x_rbv"),
            ("VAL",      "x_val"),     ("MOVN", "x_movn"),
            ("SPMG",     "x_spmg"),    ("VELO", "x_velo"),
            ("ACCL",     "x_accl"),    ("TDIR", "x_tdir"),
            ("MSTA",     "x_msta"),    ("ATHM", "x_athm"),
        ], row=0)

        self._build_motor_block(inner, "Y Motor", [
            ("Encoder",  "y_encoder"), ("RBV",  "y_rbv"),
            ("VAL",      "y_val"),     ("MOVN", "y_movn"),
            ("SPMG",     "y_spmg"),    ("VELO", "y_velo"),
            ("ACCL",     "y_accl"),    ("TDIR", "y_tdir"),
            ("MSTA",     "y_msta"),    ("ATHM", "y_athm"),
        ], row=1)

        # === Position Error ===
        pe = ttk.LabelFrame(parent, text=" Position Check ")
        pe.pack(fill="x", pady=(0, 4))
        pef = tk.Frame(pe, bg=C.BG)
        pef.pack(fill="x", padx=6, pady=4)

        self._lbl_expected = tk.Label(pef, text="Expected: --",
                                       bg=C.BG, fg=C.TEXT,
                                       font=("Consolas", 9))
        self._lbl_expected.pack(anchor="w")
        self._lbl_actual = tk.Label(pef, text="Actual:   --",
                                     bg=C.BG, fg=C.TEXT,
                                     font=("Consolas", 9))
        self._lbl_actual.pack(anchor="w")
        self._lbl_error = tk.Label(pef, text="Error:    --",
                                    bg=C.BG, fg=C.TEXT,
                                    font=("Consolas", 9, "bold"))
        self._lbl_error.pack(anchor="w")

    def _build_motor_block(self, parent, title: str,
                           fields: List[Tuple[str, str]], row: int):
        frm = tk.Frame(parent, bg=C.BG)
        frm.grid(row=row, column=0, sticky="nsew", padx=2, pady=2)
        parent.rowconfigure(row, weight=1)
        parent.columnconfigure(0, weight=1)

        tk.Label(frm, text=title, bg=C.BG, fg=C.ACCENT,
                 font=("Consolas", 9, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 2))

        half = (len(fields) + 1) // 2
        for i, (label, key) in enumerate(fields):
            c = 0 if i < half else 2
            r = (i % half) + 1
            tk.Label(frm, text=f"{label}:", bg=C.BG, fg=C.DIM,
                     font=("Consolas", 8), anchor="e"
                     ).grid(row=r, column=c, sticky="e", padx=(4, 1))
            lbl = tk.Label(frm, text="--", bg=C.BG, fg=C.TEXT,
                           font=("Consolas", 9), anchor="w", width=12)
            lbl.grid(row=r, column=c + 1, sticky="w", padx=(0, 6))
            self._status_labels[key] = lbl

    # -----------------------------------------------------------------------
    #  Commands
    # -----------------------------------------------------------------------

    def _on_start_selected(self, _event=None):
        name = self._start_var.get()
        for i, m in enumerate(self.engine.path):
            if m.name == name:
                self._selected_start_idx = i
                break

    def _cmd_start(self):
        self._on_start_selected()
        self.engine.dwell_time = self._dwell_var.get()
        self.engine.pos_threshold = self._thresh_var.get()
        self.engine.start(self._selected_start_idx)

    def _cmd_pause(self):
        eng = self.engine
        if eng.state == ScanState.PAUSED:
            eng.resume_scan()
            self._btn_pause.configure(text="Pause")
        elif eng.state in (ScanState.MOVING, ScanState.DWELLING):
            eng.pause_scan()
            self._btn_pause.configure(text="Resume")

    def _cmd_stop(self):
        self.engine.stop_scan()
        self._btn_pause.configure(text="Pause")

    def _cmd_skip(self):
        self.engine.skip_module()

    def _cmd_ack_error(self):
        self.engine.acknowledge_error()

    def _cmd_move_to_module(self):
        self._on_start_selected()
        mod = self.engine.path[self._selected_start_idx]
        px, py = module_to_ptrans(mod.x, mod.y)
        self._log(f"Direct move to {mod.name}  ptrans({px:.3f}, {py:.3f})")
        epics_move_to(self.ep, px, py)

    def _cmd_reset_center(self):
        self._log("Resetting to beam centre "
                  f"ptrans({BEAM_CENTER_X}, {BEAM_CENTER_Y})")
        epics_move_to(self.ep, BEAM_CENTER_X, BEAM_CENTER_Y)

    # -----------------------------------------------------------------------
    #  Logging
    # -----------------------------------------------------------------------

    def _log(self, msg: str, level: str = "info"):
        ts = datetime.now().strftime("%H:%M:%S")
        tag = level.upper().ljust(5)
        line = f"[{ts}] {tag} {msg}"
        self._log_lines.append(line)
        # schedule text widget update on the main thread
        self.root.after_idle(self._append_log, line, level)

    def _append_log(self, line: str, level: str):
        self._log_text.configure(state="normal")
        self._log_text.insert("end", line + "\n", level)
        self._log_text.see("end")
        self._log_text.configure(state="disabled")

    # -----------------------------------------------------------------------
    #  Periodic refresh (5 Hz)
    # -----------------------------------------------------------------------

    def _poll(self):
        self._update_status()
        self._update_canvas()
        self._update_scan_info()
        self._update_buttons()
        self.root.after(200, self._poll)

    def _update_status(self):
        """Refresh motor status labels from EPICS readback."""
        for key, lbl in self._status_labels.items():
            val = self.ep.get(key, "--")
            if val == "--" or val is None:
                lbl.configure(text="--", fg=C.DIM)
                continue

            # format value
            if key.endswith("_msta"):
                txt = f"0x{int(val):X}"
            elif key.endswith("_spmg"):
                txt = f"{SPMG_LABELS.get(int(val), '?')}({int(val)})"
            elif key.endswith("_movn") or key.endswith("_athm") or key.endswith("_tdir"):
                txt = str(int(val))
            elif isinstance(val, float):
                txt = f"{val:.3f}"
            else:
                txt = str(val)

            # colour moving indicators
            fg = C.TEXT
            if key.endswith("_movn") and int(val) == 1:
                fg = C.YELLOW
            elif key.endswith("_spmg") and int(val) != SPMG.GO:
                fg = C.ORANGE if int(val) == SPMG.PAUSE else C.RED

            lbl.configure(text=txt, fg=fg)

        # position check
        eng = self.engine
        mod = eng.current_module
        if mod and eng.state != ScanState.IDLE:
            px, py = module_to_ptrans(mod.x, mod.y)
            rx = self.ep.get("x_rbv", 0.0)
            ry = self.ep.get("y_rbv", 0.0)
            err = math.sqrt((rx - px)**2 + (ry - py)**2)
            self._lbl_expected.configure(
                text=f"Expected: ({px:.3f}, {py:.3f})")
            self._lbl_actual.configure(
                text=f"Actual:   ({rx:.3f}, {ry:.3f})")
            err_fg = C.RED if err > eng.pos_threshold else C.GREEN
            self._lbl_error.configure(
                text=f"Error:    {err:.3f} mm", fg=err_fg)
        else:
            rx = self.ep.get("x_rbv", 0.0)
            ry = self.ep.get("y_rbv", 0.0)
            self._lbl_expected.configure(text="Expected: --")
            self._lbl_actual.configure(
                text=f"Actual:   ({rx:.3f}, {ry:.3f})")
            self._lbl_error.configure(text="Error:    --", fg=C.DIM)

    def _update_scan_info(self):
        eng = self.engine

        # state badge
        state_colours = {
            ScanState.IDLE:      C.DIM,
            ScanState.MOVING:    C.YELLOW,
            ScanState.DWELLING:  C.GREEN,
            ScanState.PAUSED:    C.ORANGE,
            ScanState.ERROR:     C.RED,
            ScanState.COMPLETED: C.ACCENT,
        }
        self._lbl_state.configure(
            text=eng.state,
            fg=state_colours.get(eng.state, C.DIM))

        # progress
        self._lbl_progress.configure(text=f"Progress: {eng.progress_text}")

        mod = eng.current_module
        if mod:
            self._lbl_current.configure(text=f"Current:  {mod.name}")
        else:
            self._lbl_current.configure(text="Current:  --")

        # ETA
        eta = eng.eta_seconds
        if eng.state in (ScanState.MOVING, ScanState.DWELLING, ScanState.PAUSED):
            h, rem = divmod(int(eta), 3600)
            m, s = divmod(rem, 60)
            self._lbl_eta.configure(text=f"ETA:      {h}h {m:02d}m {s:02d}s")
        else:
            self._lbl_eta.configure(text="ETA:      --")

        # dwell countdown
        if eng.state == ScanState.DWELLING:
            self._lbl_dwell_cd.configure(
                text=f"Dwell:    {eng.dwell_remaining:.1f}s remaining")
        else:
            self._lbl_dwell_cd.configure(text="")

    def _update_buttons(self):
        eng = self.engine
        running = eng.state in (ScanState.MOVING, ScanState.DWELLING,
                                 ScanState.PAUSED, ScanState.ERROR)
        self._btn_start.configure(
            state="disabled" if running else "normal")
        self._btn_pause.configure(
            state="normal" if running else "disabled")
        self._btn_stop.configure(
            state="normal" if running else "disabled")
        self._btn_skip.configure(
            state="normal" if eng.state == ScanState.DWELLING else "disabled")
        self._btn_ack.configure(
            state="normal" if eng.state == ScanState.ERROR else "disabled")
        self._start_combo.configure(
            state="readonly" if not running else "disabled")


# ============================================================================
#  MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="HyCal Snake Scan -- PbWO4 module scanner")
    parser.add_argument("--real", action="store_true",
                        help="Use real EPICS (requires pyepics)")
    args = parser.parse_args()

    simulation = not args.real

    if simulation:
        ep = SimulatedEPICS()
    else:
        ep = RealEPICS()

    n_ok, n_total = ep.connect()
    if not simulation:
        print(f"EPICS: connected {n_ok}/{n_total} PVs")
        if n_ok < n_total * 0.5:
            print("WARNING: many PVs not connected -- check IOC / network")

    root = tk.Tk()
    SnakeScanGUI(root, ep, simulation)
    root.mainloop()


if __name__ == "__main__":
    main()
