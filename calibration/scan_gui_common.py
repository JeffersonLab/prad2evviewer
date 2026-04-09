"""
Shared GUI utilities for HyCal calibration tools.

Used by both ``hycal_snake_scan.py`` and ``hycal_gain_equalizer.py``.
Centralizes the bits that were otherwise duplicated between the two
scripts: session log file setup, log line formatting, the position
check panel, encoder-drift monitoring, profile loading, and the
EPICS / scaler bring-up boilerplate.
"""

from __future__ import annotations

import html as html_mod
import json
import math
import os
from datetime import datetime
from typing import Any, Dict, List, Optional

from PyQt6.QtWidgets import QGroupBox, QLabel, QVBoxLayout

from scan_utils import C, Module
from scan_engine import DEFAULT_VELO_X, DEFAULT_VELO_Y


# ============================================================================
#  Constants shared by both GUI scripts
# ============================================================================

PATHS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "paths.json")

POLL_MS = 200             # main UI poll interval (5 Hz)
SCALER_POLL_MS = 5_000    # default scaler poll interval (5 s)

PROFILE_AUTOGEN = "(autogen)"
PROFILE_NONE = "(none)"

ENCODER_DRIFT_WARN = 0.5   # mm — yellow threshold
ENCODER_DRIFT_ERR  = 1.5   # mm — red threshold


# ============================================================================
#  Session log file
# ============================================================================

def open_session_log(tool_prefix: str, simulation: bool, observer: bool):
    """Create / append today's session log file.

    Returns ``None`` in observer mode (read-only — never writes to disk).
    Otherwise opens ``logs/{SIM_}{tool_prefix}_YYYYMMDD.log`` and writes
    a session-start banner.
    """
    if observer:
        return None
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    prefix = "SIM_" if simulation else ""
    name = datetime.now().strftime(f"{prefix}{tool_prefix}_%Y%m%d.log")
    f = open(os.path.join(log_dir, name), "a")
    f.write(
        "\n" + "=" * 70 + "\n"
        f"=== Session start: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n"
        + "=" * 70 + "\n")
    f.flush()
    return f


def format_log_line(msg: str, level: str = "info") -> str:
    """Return the timestamped log line both GUIs use."""
    ts = datetime.now().strftime("%H:%M:%S")
    return f"[{ts}] {level.upper().ljust(5)} {msg}"


def html_log_line(line: str, level: str) -> str:
    """Return the HTML span for inserting a log line into a QTextEdit."""
    colors = {"info": C.TEXT, "warn": C.YELLOW, "error": C.RED}
    c = colors.get(level, C.DIM)
    return (
        f'<span style="color:{c};font-family:Consolas;font-size:13pt;">'
        f'{html_mod.escape(line)}</span>')


def append_log_line(text_edit, line: str, level: str) -> None:
    """Append a colour-formatted log line to a QTextEdit and auto-scroll.

    Uses the "scroll-stick" pattern: if the user is already at the
    bottom of the log (within a few pixels) the view follows the new
    entry; if they have scrolled up to inspect history the view is
    left where it is.
    """
    sb = text_edit.verticalScrollBar()
    at_bottom = sb.value() >= sb.maximum() - 4
    text_edit.append(html_log_line(line, level))
    if at_bottom:
        sb.setValue(sb.maximum())


# ============================================================================
#  Position-check panel
# ============================================================================

def build_position_check_panel(parent_layout) -> Dict[str, QLabel]:
    """Build the standard "Position Check" group box.

    Adds the group box to ``parent_layout`` and returns a dict with
    keys ``target``, ``actual``, ``diff``, ``drift`` mapping to the
    QLabel widgets so the caller can update them later.
    """
    pe = QGroupBox("Position Check")
    lo = QVBoxLayout(pe)
    lbl_target = QLabel("Target: --"); lo.addWidget(lbl_target)
    lbl_actual = QLabel("Actual: --"); lo.addWidget(lbl_actual)
    lbl_diff = QLabel("Diff:   --")
    lbl_diff.setStyleSheet("font: bold 13pt 'Consolas';")
    lo.addWidget(lbl_diff)
    lbl_drift = QLabel("Drift:   --"); lo.addWidget(lbl_drift)
    parent_layout.addWidget(pe)
    return {"target": lbl_target, "actual": lbl_actual,
            "diff": lbl_diff, "drift": lbl_drift}


def update_position_check(labels: Dict[str, QLabel], ep: Any,
                          target_px: Optional[float],
                          target_py: Optional[float],
                          target_name: str = "",
                          scanning: bool = False,
                          pos_threshold: float = 0.5) -> None:
    """Refresh the target / actual / diff labels from live PVs.

    Adds an ETA suffix to the diff label based on the motor velocities.
    If ``scanning`` is True, the diff is colored red/green against
    ``pos_threshold`` so the operator can see at a glance whether the
    motor has reached the target.  Otherwise it stays dim.
    """
    rx = ep.get("x_rbv", 0.0) or 0.0
    ry = ep.get("y_rbv", 0.0) or 0.0
    labels["actual"].setText(f"Actual: ({rx:.3f}, {ry:.3f})")
    if target_px is None or target_py is None:
        labels["target"].setText("Target: --")
        labels["diff"].setText("Diff:   --")
        labels["diff"].setStyleSheet(f"color: {C.DIM}; font: bold 13pt 'Consolas';")
        return
    err = math.sqrt((rx - target_px) ** 2 + (ry - target_py) ** 2)
    name_html = (f' <b style="color:{C.ACCENT}">{target_name}</b>'
                 if target_name else "")
    labels["target"].setText(
        f"Target: ({target_px:.3f}, {target_py:.3f}){name_html}")
    vx = ep.get("x_velo", DEFAULT_VELO_X) or DEFAULT_VELO_X
    vy = ep.get("y_velo", DEFAULT_VELO_Y) or DEFAULT_VELO_Y
    dx, dy = abs(rx - target_px), abs(ry - target_py)
    eta_sec = max(dx / vx if vx > 0 else 0, dy / vy if vy > 0 else 0)
    if eta_sec >= 60:
        eta_str = f" ({int(eta_sec)//60}m {int(eta_sec)%60}s)"
    elif eta_sec >= 1:
        eta_str = f" ({eta_sec:.0f}s)"
    else:
        eta_str = ""
    labels["diff"].setText(f"Diff:   {err:.3f} mm{eta_str}")
    if scanning:
        fg = C.RED if err > pos_threshold else C.GREEN
    else:
        fg = C.DIM
    labels["diff"].setStyleSheet(f"color: {fg}; font: bold 13pt 'Consolas';")


# ============================================================================
#  Encoder drift checker
# ============================================================================

class EncoderDriftChecker:
    """Tracks motor encoder vs RBV drift for the position-check panel.

    Calibrates the encoder offset on the first call (once both encoders
    and RBVs are available), then reports absolute drift in the supplied
    QLabel, color-coded against ``ENCODER_DRIFT_WARN`` / ``_ERR``.
    """

    def __init__(self) -> None:
        self.offset_x: Optional[float] = None
        self.offset_y: Optional[float] = None

    def update(self, ep: Any, log_fn, drift_label: QLabel) -> None:
        enc_x = ep.get("x_encoder", None)
        enc_y = ep.get("y_encoder", None)
        rbv_x = ep.get("x_rbv", None)
        rbv_y = ep.get("y_rbv", None)
        if enc_x is None or enc_y is None or rbv_x is None or rbv_y is None:
            return
        if self.offset_x is None:
            self.offset_x = enc_x - rbv_x
            self.offset_y = enc_y - rbv_y
            log_fn(f"Encoder calibrated: offset "
                   f"X={self.offset_x:.4f} Y={self.offset_y:.4f}")
            return
        dx = abs((enc_x - self.offset_x) - rbv_x)
        dy = abs((enc_y - self.offset_y) - rbv_y)
        fx = (C.RED if dx > ENCODER_DRIFT_ERR
              else C.YELLOW if dx > ENCODER_DRIFT_WARN
              else C.GREEN)
        fy = (C.RED if dy > ENCODER_DRIFT_ERR
              else C.YELLOW if dy > ENCODER_DRIFT_WARN
              else C.GREEN)
        drift_label.setText(
            f'Drift:   X <span style="color:{fx}">{dx:.4f}</span>  '
            f'Y <span style="color:{fy}">{dy:.4f}</span>')


# ============================================================================
#  Profile loading
# ============================================================================

def load_profiles(paths_file: str) -> Dict[str, List[str]]:
    """Load the path profiles JSON, returning ``{}`` if missing."""
    if os.path.exists(paths_file):
        with open(paths_file) as f:
            return json.load(f)
    return {}


# ============================================================================
#  EPICS bring-up
# ============================================================================

def setup_motor_epics(observer: bool, simulation: bool):
    """Create and connect the appropriate motor EPICS group.

    Echoes connection counts and disconnected PVs to stdout in non-sim
    mode so the operator sees them at startup.
    """
    from scan_epics import (MotorEPICS, ObserverEPICS, SimulatedMotorEPICS)
    if observer:
        ep = ObserverEPICS()
    elif simulation:
        ep = SimulatedMotorEPICS()
    else:
        ep = MotorEPICS(writable=True)
    n_ok, n_total = ep.connect()
    if not simulation:
        print(f"EPICS: {n_ok}/{n_total} PVs connected")
        for pv in ep.disconnected_pvs():
            print(f"  NOT connected: {pv}")
    return ep


def setup_scaler_epics(simulation: bool, all_modules: List[Module]):
    """Create and connect the appropriate scaler EPICS group."""
    from scan_epics import (ScalerPVGroup, SimulatedScalerEPICS)
    if simulation:
        ep = SimulatedScalerEPICS(all_modules)
    else:
        ep = ScalerPVGroup(all_modules)
    s_ok, s_total = ep.connect()
    if not simulation:
        print(f"Scalers: {s_ok}/{s_total} PVs connected")
    return ep
