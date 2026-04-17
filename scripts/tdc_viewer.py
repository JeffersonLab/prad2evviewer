#!/usr/bin/env python3
"""
Tagger TDC Viewer (PyQt6)
=========================

Interactive viewer for the V1190 TDC banks (0xE107) produced by the tagger
crate (ROC 0x008E).  Accepts two input forms:

  1. An evio file (``*.evio``, ``*.evio.*``) — decoded in-process via the
     ``prad2py`` pybind11 module.  Requires building with
     ``-DBUILD_PYTHON=ON`` and adding ``build/python`` to ``PYTHONPATH``.
  2. A binary hit dump produced by ``tdc_dump -b`` (no C++ extension
     needed, works as a fallback on hosts without prad2py built).

Displays:

  * A bar chart of hits-per-channel for the selected slot (auto-sized
    to 16 / 32 / 64 / 128 based on the highest channel actually hit —
    click a bar to pick that channel).
  * A TDC value histogram for the selected (slot, channel).
  * A tree on the left with per-slot / per-channel hit counts.

Usage
-----

    # Direct: read an evio file (needs prad2py on PYTHONPATH)
    python scripts/tdc_viewer.py /data/.../prad_023667.evio.00000

    # Indirect: pre-dumped binary file
    ./build/bin/tdc_dump /data/.../prad_023667.evio.00000 -b hits.bin
    python scripts/tdc_viewer.py hits.bin

Only PyQt6 and numpy are required.  Plots are drawn with QPainter, so the
usual matplotlib/pyqtgraph stack is *not* needed.

Binary file format (produced by tdc_dump -b)
--------------------------------------------

    magic        : 16 ASCII bytes "PRAD2_TDC_HITS_1"
    record_count : uint32_le
    records      : record_count × 16-byte BinHit  (see test/tdc_dump.cpp)
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from PyQt6.QtCore import Qt, QRectF, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)


# ---------------------------------------------------------------------------
# Binary file loader
# ---------------------------------------------------------------------------

BIN_MAGIC = b"PRAD2_TDC_HITS_1"
RAW_DTYPE = np.dtype(
    [
        ("event_num", "<u4"),
        ("trigger_bits", "<u4"),
        ("roc_tag", "<u2"),
        ("slot", "u1"),
        ("channel_edge", "u1"),  # bit 7 = edge, bits 6:0 = channel
        ("tdc", "<u4"),
    ]
)
assert RAW_DTYPE.itemsize == 16, "BinHit must be 16 bytes"

# Exposed view with edge/channel split out for convenience.
RECORD_DTYPE = np.dtype(
    [
        ("event_num", "<u4"),
        ("trigger_bits", "<u4"),
        ("roc_tag", "<u2"),
        ("slot", "u1"),
        ("channel", "u1"),
        ("edge", "u1"),
        ("tdc", "<u4"),
    ]
)


def load_hits(path: str) -> np.ndarray:
    """Load hits from a binary dump. Returns a structured numpy array
    with (event_num, trigger_bits, roc_tag, slot, channel, edge, tdc)."""
    size = os.path.getsize(path)
    if size < 20:
        raise ValueError(f"{path}: file too small ({size} bytes)")

    with open(path, "rb") as f:
        magic = f.read(16)
        if magic != BIN_MAGIC:
            raise ValueError(
                f"{path}: bad magic {magic!r} (expected {BIN_MAGIC!r})"
            )
        count_bytes = f.read(4)
        count = int.from_bytes(count_bytes, "little")
        payload_bytes = size - 20
        if count * RAW_DTYPE.itemsize > payload_bytes:
            # File was truncated or count header wasn't finalised.
            count = payload_bytes // RAW_DTYPE.itemsize
        raw = np.fromfile(f, dtype=RAW_DTYPE, count=count)

    hits = np.empty(raw.size, dtype=RECORD_DTYPE)
    hits["event_num"]    = raw["event_num"]
    hits["trigger_bits"] = raw["trigger_bits"]
    hits["roc_tag"]      = raw["roc_tag"]
    hits["slot"]         = raw["slot"]
    hits["channel"]      = raw["channel_edge"] & 0x7F
    hits["edge"]         = (raw["channel_edge"] >> 7) & 0x1
    hits["tdc"]          = raw["tdc"]
    return hits


# ---------------------------------------------------------------------------
# In-process evio loader via prad2py
# ---------------------------------------------------------------------------

_SCRIPT_DIR = Path(__file__).resolve().parent
# Common locations for the freshly built prad2py extension. If the user has
# not set PYTHONPATH we still try a couple of obvious candidates so the
# viewer "just works" after ``cmake --build``.
for _cand in (
    _SCRIPT_DIR.parent / "build" / "python",
    _SCRIPT_DIR.parent / "build-release" / "python",
    _SCRIPT_DIR.parent / "build" / "Release" / "python",
):
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))

try:
    import prad2py  # type: ignore
    HAVE_PRAD2PY = True
    PRAD2PY_ERROR = ""
except Exception as _exc:  # noqa: BLE001
    prad2py = None  # type: ignore
    HAVE_PRAD2PY = False
    PRAD2PY_ERROR = f"{type(_exc).__name__}: {_exc}"


def _looks_like_evio(path: str) -> bool:
    name = os.path.basename(path).lower()
    return name.endswith(".evio") or ".evio." in name


def load_hits_from_evio(
    path: str,
    *,
    max_events: int = 0,
    daq_config: str = "",
    roc_filter: int = -1,
) -> np.ndarray:
    """Decode TDC hits directly from an evio file via prad2py."""
    if not HAVE_PRAD2PY:
        raise RuntimeError(
            "prad2py module not available "
            f"({PRAD2PY_ERROR or 'not importable'}).\n"
            "Build it with:\n"
            "    cmake -DBUILD_PYTHON=ON -S . -B build && cmake --build build\n"
            "and add build/python/ to PYTHONPATH."
        )
    return prad2py.load_tdc_hits(
        path,
        daq_config=daq_config,
        max_events=int(max_events),
        roc_filter=int(roc_filter),
    )


def load_hits_auto(path: str, **kwargs) -> np.ndarray:
    """Dispatch on filename: .evio -> prad2py, otherwise -> binary loader."""
    if _looks_like_evio(path):
        return load_hits_from_evio(path, **kwargs)
    return load_hits(path)


def round_up_channels(max_ch: int) -> int:
    """Round the observed max channel number up to a V1190-friendly power
    of two (16, 32, 64, 128). Channels are 0-indexed in the data, so we
    compare against (max_ch + 1)."""
    need = max(int(max_ch) + 1, 1)
    for n in (16, 32, 64, 128):
        if need <= n:
            return n
    return 128


# ---------------------------------------------------------------------------
# Plot widgets
# ---------------------------------------------------------------------------


class BarChart(QWidget):
    """
    Horizontal index → count bar chart painted with QPainter.
    Emits ``barClicked(index)`` when a bar is clicked.
    """

    barClicked = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._counts: np.ndarray = np.zeros(0, dtype=np.int64)
        self._labels: Dict[int, str] = {}
        self._highlight: Optional[int] = None
        self._title = ""
        self.setMinimumHeight(180)
        self.setMouseTracking(True)

    # --- data ------------------------------------------------------------

    def setData(self, counts: np.ndarray, labels: Optional[Dict[int, str]] = None):
        self._counts = np.asarray(counts, dtype=np.int64)
        self._labels = labels or {}
        self._highlight = None
        self.update()

    def setTitle(self, title: str):
        self._title = title
        self.update()

    def setHighlight(self, idx: Optional[int]):
        self._highlight = idx
        self.update()

    # --- geometry --------------------------------------------------------

    def _plotRect(self) -> QRectF:
        m = 30.0
        return QRectF(m + 20, 18, self.width() - m - 30, self.height() - m - 18)

    def _indexAtX(self, x: float) -> Optional[int]:
        r = self._plotRect()
        if not r.contains(x, r.center().y()):
            return None
        n = self._counts.size
        if n <= 0:
            return None
        rel = (x - r.left()) / r.width()
        idx = int(rel * n)
        if 0 <= idx < n:
            return idx
        return None

    # --- events ----------------------------------------------------------

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            idx = self._indexAtX(ev.position().x())
            if idx is not None:
                self.barClicked.emit(idx)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        p.fillRect(self.rect(), QColor(250, 250, 250))

        r = self._plotRect()
        p.setPen(QPen(QColor(60, 60, 60)))
        p.drawRect(r)

        if self._title:
            f = QFont()
            f.setPointSize(9)
            f.setBold(True)
            p.setFont(f)
            p.drawText(int(r.left()), int(r.top() - 4), self._title)

        n = self._counts.size
        if n <= 0:
            p.setPen(QColor(120, 120, 120))
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "(no hits)")
            return

        cmax = int(self._counts.max()) if self._counts.size else 1
        cmax = max(cmax, 1)
        bar_w = r.width() / n

        # bars
        for i, c in enumerate(self._counts):
            h = (c / cmax) * r.height()
            x0 = r.left() + i * bar_w
            y0 = r.bottom() - h
            color = QColor(80, 140, 200)
            if self._highlight is not None and i == self._highlight:
                color = QColor(230, 120, 40)
            elif c == 0:
                color = QColor(220, 220, 220)
            p.fillRect(QRectF(x0 + 0.5, y0, max(bar_w - 1.0, 1.0), h), color)

        # y-axis ticks
        p.setPen(QColor(100, 100, 100))
        f = QFont()
        f.setPointSize(8)
        p.setFont(f)
        for frac in (0.0, 0.5, 1.0):
            y = r.bottom() - frac * r.height()
            p.drawLine(int(r.left() - 3), int(y), int(r.left()), int(y))
            p.drawText(
                int(r.left() - 38),
                int(y + 4),
                f"{int(cmax * frac):,}",
            )

        # x-axis ticks
        step = max(1, n // 16)
        for i in range(0, n, step):
            x = r.left() + (i + 0.5) * bar_w
            p.drawLine(int(x), int(r.bottom()), int(x), int(r.bottom() + 3))
            label = self._labels.get(i, str(i))
            p.drawText(int(x - 14), int(r.bottom() + 14), label)


class Histogram(QWidget):
    """1-D histogram painted with QPainter."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._counts: np.ndarray = np.zeros(0, dtype=np.int64)
        self._edges: np.ndarray = np.zeros(0)
        self._title = ""
        self._xlabel = ""
        self.setMinimumHeight(260)

    def setData(self, counts: np.ndarray, edges: np.ndarray):
        self._counts = np.asarray(counts, dtype=np.int64)
        self._edges = np.asarray(edges, dtype=np.float64)
        self.update()

    def setTitle(self, title: str):
        self._title = title
        self.update()

    def setXLabel(self, label: str):
        self._xlabel = label
        self.update()

    def _plotRect(self) -> QRectF:
        m = 40.0
        return QRectF(m + 25, 20, self.width() - m - 35, self.height() - m - 20)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        p.fillRect(self.rect(), QColor(250, 250, 250))

        r = self._plotRect()
        p.setPen(QColor(60, 60, 60))
        p.drawRect(r)

        if self._title:
            f = QFont()
            f.setPointSize(10)
            f.setBold(True)
            p.setFont(f)
            p.drawText(int(r.left()), int(r.top() - 4), self._title)

        n = self._counts.size
        if n <= 0 or self._counts.sum() == 0:
            p.setPen(QColor(120, 120, 120))
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "(no hits for this channel)")
            return

        cmax = int(self._counts.max())
        cmax = max(cmax, 1)
        bar_w = r.width() / n

        p.setPen(Qt.PenStyle.NoPen)
        for i, c in enumerate(self._counts):
            if c == 0:
                continue
            h = (c / cmax) * r.height()
            x0 = r.left() + i * bar_w
            y0 = r.bottom() - h
            p.fillRect(
                QRectF(x0, y0, max(bar_w, 1.0), h),
                QColor(80, 140, 200),
            )

        # y ticks
        p.setPen(QColor(100, 100, 100))
        f = QFont()
        f.setPointSize(8)
        p.setFont(f)
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = r.bottom() - frac * r.height()
            p.drawLine(int(r.left() - 3), int(y), int(r.left()), int(y))
            p.drawText(
                int(r.left() - 46),
                int(y + 4),
                f"{int(cmax * frac):,}",
            )

        # x ticks
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = r.left() + frac * r.width()
            val_idx = int(round(frac * (self._edges.size - 1)))
            val = self._edges[val_idx] if self._edges.size > 0 else 0
            p.drawLine(int(x), int(r.bottom()), int(x), int(r.bottom() + 3))
            p.drawText(int(x - 30), int(r.bottom() + 14), f"{val:.0f}")

        if self._xlabel:
            p.drawText(
                int(r.center().x() - 60),
                int(r.bottom() + 30),
                self._xlabel,
            )


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------


class TdcViewer(QMainWindow):
    DEFAULT_BINS = 200

    def __init__(
        self,
        hits: Optional[np.ndarray] = None,
        path: str = "",
        *,
        max_events: int = 0,
        daq_config: str = "",
        roc_filter: int = -1,
    ):
        super().__init__()
        self.setWindowTitle("Tagger TDC Viewer")
        self.resize(1280, 800)

        self._hits: np.ndarray = (
            hits if hits is not None else np.zeros(0, dtype=RECORD_DTYPE)
        )
        self._path = path
        self._slot_ch_counts: Dict[Tuple[int, int], int] = {}
        self._current: Optional[Tuple[int, int]] = None
        self._load_max_events = max_events
        self._load_daq_config = daq_config
        self._load_roc_filter = roc_filter

        self._build_ui()
        self._make_menu()

        if self._hits.size:
            self._rebuild_index()

    # --- UI layout -------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Top row: file label + edge filter + bins
        top = QHBoxLayout()
        self.file_label = QLabel("(no file)")
        top.addWidget(self.file_label, 1)

        top.addWidget(QLabel("Edge:"))
        self.edge_combo = QComboBox()
        self.edge_combo.addItems(["both", "leading (0)", "trailing (1)"])
        self.edge_combo.currentIndexChanged.connect(self._refresh)
        top.addWidget(self.edge_combo)

        top.addWidget(QLabel("Bins:"))
        self.bins_spin = QSpinBox()
        self.bins_spin.setRange(10, 2000)
        self.bins_spin.setValue(self.DEFAULT_BINS)
        self.bins_spin.setSingleStep(10)
        self.bins_spin.valueChanged.connect(self._refresh_histogram)
        top.addWidget(self.bins_spin)

        main_layout.addLayout(top)

        # Main splitter: left (tree) | right (plots)
        splitter = QSplitter(Qt.Orientation.Horizontal)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Slot / Channel", "Hits"])
        self.tree.setColumnWidth(0, 150)
        self.tree.itemSelectionChanged.connect(self._on_tree_select)
        splitter.addWidget(self.tree)

        right = QWidget()
        rlay = QVBoxLayout(right)
        rlay.setContentsMargins(4, 4, 4, 4)

        self.channel_bar = BarChart()
        self.channel_bar.setTitle("Hits per channel (selected slot) — click a bar")
        self.channel_bar.barClicked.connect(self._on_bar_clicked)
        rlay.addWidget(self.channel_bar)

        self.tdc_hist = Histogram()
        self.tdc_hist.setTitle("TDC value histogram — select a channel")
        self.tdc_hist.setXLabel("TDC value (LSB = 25 ps after rol2 shift)")
        rlay.addWidget(self.tdc_hist, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        main_layout.addWidget(splitter, 1)

        self.setStatusBar(QStatusBar())

    def _make_menu(self):
        m = self.menuBar().addMenu("&File")
        a_open = QAction("&Open…", self)
        a_open.setShortcut("Ctrl+O")
        a_open.triggered.connect(self._open_dialog)
        m.addAction(a_open)
        m.addSeparator()
        a_quit = QAction("&Quit", self)
        a_quit.setShortcut("Ctrl+Q")
        a_quit.triggered.connect(self.close)
        m.addAction(a_quit)

    # --- loading ---------------------------------------------------------

    def _open_dialog(self):
        filt = ("All supported (*.evio *.evio.* *.bin);;"
                "EVIO files (*.evio *.evio.*);;"
                "Binary dumps (*.bin);;"
                "All files (*)")
        path, _ = QFileDialog.getOpenFileName(self, "Open TDC source", "", filt)
        if path:
            self.load(path)

    def load(self, path: str):
        self.statusBar().showMessage(f"Loading {path}…")
        QApplication.processEvents()
        try:
            if _looks_like_evio(path):
                hits = load_hits_from_evio(
                    path,
                    max_events=self._load_max_events,
                    daq_config=self._load_daq_config,
                    roc_filter=self._load_roc_filter,
                )
            else:
                hits = load_hits(path)
        except Exception as exc:
            QMessageBox.critical(self, "Load failed", f"{path}\n\n{exc}")
            self.statusBar().showMessage("")
            return
        self._hits = hits
        self._path = path
        self._rebuild_index()

    # --- indexing --------------------------------------------------------

    def _rebuild_index(self):
        hits = self._hits
        self.file_label.setText(
            f"{os.path.basename(self._path)} — {hits.size:,} hits"
            if self._path
            else f"(in-memory) — {hits.size:,} hits"
        )
        self.tree.clear()
        self._slot_ch_counts.clear()
        self._current = None

        if hits.size == 0:
            self._refresh()
            return

        slots = np.unique(hits["slot"])
        for slot in slots:
            smask = hits["slot"] == slot
            sub = hits[smask]
            slot_item = QTreeWidgetItem([f"slot {int(slot)}", f"{sub.size:,}"])
            slot_item.setData(0, Qt.ItemDataRole.UserRole, ("slot", int(slot)))
            self.tree.addTopLevelItem(slot_item)

            chs, counts = np.unique(sub["channel"], return_counts=True)
            for ch, c in zip(chs, counts):
                self._slot_ch_counts[(int(slot), int(ch))] = int(c)
                ch_item = QTreeWidgetItem([f"  ch {int(ch):3d}", f"{int(c):,}"])
                ch_item.setData(
                    0, Qt.ItemDataRole.UserRole, ("channel", int(slot), int(ch))
                )
                slot_item.addChild(ch_item)
            slot_item.setExpanded(False)

        # Auto-select the slot with the most hits
        best_slot = int(
            max(slots, key=lambda s: int(np.count_nonzero(hits["slot"] == s)))
        )
        for i in range(self.tree.topLevelItemCount()):
            it = self.tree.topLevelItem(i)
            data = it.data(0, Qt.ItemDataRole.UserRole)
            if data and data[0] == "slot" and data[1] == best_slot:
                self.tree.setCurrentItem(it)
                break

        self._refresh()

    # --- interaction -----------------------------------------------------

    def _current_edge_mask(self) -> Optional[int]:
        idx = self.edge_combo.currentIndex()
        return None if idx == 0 else (idx - 1)

    def _on_tree_select(self):
        items = self.tree.selectedItems()
        if not items:
            return
        data = items[0].data(0, Qt.ItemDataRole.UserRole)
        if not data:
            return
        if data[0] == "slot":
            self._current = (data[1], None)
            self._refresh()
        elif data[0] == "channel":
            self._current = (data[1], data[2])
            self._refresh()

    def _on_bar_clicked(self, idx: int):
        if self._current is None:
            return
        slot, _ = self._current
        self._current = (slot, int(idx))
        # Select the matching tree item (if it exists).
        for i in range(self.tree.topLevelItemCount()):
            it = self.tree.topLevelItem(i)
            data = it.data(0, Qt.ItemDataRole.UserRole)
            if data and data[0] == "slot" and data[1] == slot:
                it.setExpanded(True)
                for j in range(it.childCount()):
                    child = it.child(j)
                    cdata = child.data(0, Qt.ItemDataRole.UserRole)
                    if cdata and cdata[0] == "channel" and cdata[2] == idx:
                        self.tree.setCurrentItem(child)
                        return
                break
        self._refresh()

    def _refresh(self):
        self._refresh_bar()
        self._refresh_histogram()

    def _refresh_bar(self):
        hits = self._hits
        if hits.size == 0 or self._current is None:
            self.channel_bar.setData(np.zeros(0))
            self.channel_bar.setTitle("Hits per channel")
            return

        slot, ch = self._current
        mask = hits["slot"] == slot
        edge_sel = self._current_edge_mask()
        if edge_sel is not None:
            mask = mask & (hits["edge"] == edge_sel)
        sub = hits[mask]

        # Auto-size the channel axis: look at *any* hit in this slot (before
        # the edge cut) so that switching "leading/trailing/both" doesn't
        # reshape the chart under the user.
        slot_hits = hits[hits["slot"] == slot]
        if slot_hits.size > 0:
            max_ch = int(slot_hits["channel"].max())
        else:
            max_ch = 0
        nbars = round_up_channels(max_ch)

        channels = sub["channel"].astype(np.int32)
        counts = np.bincount(channels, minlength=nbars)
        if counts.size > nbars:
            counts = counts[:nbars]

        self.channel_bar.setData(counts)
        self.channel_bar.setTitle(
            f"Hits per channel — slot {slot} ({nbars}-channel axis)"
        )
        self.channel_bar.setHighlight(ch if ch is not None else None)

    def _refresh_histogram(self):
        hits = self._hits
        if hits.size == 0 or self._current is None or self._current[1] is None:
            self.tdc_hist.setData(np.zeros(0), np.zeros(0))
            self.tdc_hist.setTitle("TDC value histogram — select a channel")
            self.statusBar().showMessage("")
            return

        slot, ch = self._current
        mask = (hits["slot"] == slot) & (hits["channel"] == ch)
        edge_sel = self._current_edge_mask()
        if edge_sel is not None:
            mask = mask & (hits["edge"] == edge_sel)
        sub = hits[mask]

        if sub.size == 0:
            self.tdc_hist.setData(np.zeros(0), np.zeros(0))
            self.tdc_hist.setTitle(
                f"TDC histogram — slot {slot}, ch {ch} (no hits)"
            )
            self.statusBar().showMessage("")
            return

        tdc_vals = sub["tdc"].astype(np.int64)
        tmin = int(tdc_vals.min())
        tmax = int(tdc_vals.max())
        if tmax <= tmin:
            tmax = tmin + 1
        nbins = self.bins_spin.value()
        counts, edges = np.histogram(tdc_vals, bins=nbins, range=(tmin, tmax + 1))
        self.tdc_hist.setData(counts, edges)

        edge_name = (
            "both edges"
            if edge_sel is None
            else ("leading" if edge_sel == 0 else "trailing")
        )
        title = (
            f"TDC histogram — slot {slot}, ch {ch}, {edge_name} "
            f"— {sub.size:,} hits, mean={tdc_vals.mean():.1f}, rms={tdc_vals.std():.1f}"
        )
        self.tdc_hist.setTitle(title)

        self.statusBar().showMessage(
            f"slot={slot} ch={ch}  n={sub.size:,}  "
            f"min={tmin}  max={tmax}  mean={tdc_vals.mean():.2f}"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _parse_roc(value: str) -> int:
    s = value.strip().lower()
    if not s:
        return -1
    return int(s, 0)  # handles 0x..., 0..., decimal


def _cli_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="tdc_viewer.py",
        description="Interactive viewer for V1190 TDC hits (bank 0xE107).",
    )
    p.add_argument(
        "path",
        nargs="?",
        help="Input file. .evio(.*) -> decoded via prad2py; "
             ".bin -> read as tdc_dump binary dump.",
    )
    p.add_argument(
        "-n", "--max-events",
        type=int,
        default=0,
        help="When reading an .evio file, stop after N physics events (0 = all).",
    )
    p.add_argument(
        "-D", "--daq-config",
        default="",
        help="Override daq_config.json path (prad2py uses the installed default otherwise).",
    )
    p.add_argument(
        "--roc",
        default="",
        help="Only keep hits from this parent ROC tag, e.g. --roc 0x8E (default: all).",
    )
    return p


def main(argv):
    args = _cli_parser().parse_args(argv[1:])
    roc = _parse_roc(args.roc)

    app = QApplication(argv[:1])

    hits = None
    path = ""
    if args.path:
        path = args.path
        try:
            if _looks_like_evio(path):
                hits = load_hits_from_evio(
                    path,
                    max_events=args.max_events,
                    daq_config=args.daq_config,
                    roc_filter=roc,
                )
            else:
                hits = load_hits(path)
        except Exception as exc:
            QMessageBox.critical(None, "Load failed", f"{path}\n\n{exc}")
            return 1

    win = TdcViewer(
        hits=hits,
        path=path,
        max_events=args.max_events,
        daq_config=args.daq_config,
        roc_filter=roc,
    )
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
