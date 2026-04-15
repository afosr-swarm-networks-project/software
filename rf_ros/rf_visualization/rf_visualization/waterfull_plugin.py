from __future__ import annotations

import colorsys
import hashlib
from typing import Optional

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from python_qt_binding.QtCore import QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rqt_gui_py.plugin import Plugin

from rf_msgs.msg import RfDetectionArray, StftFrame


class WaterfullWidget(QWidget):
    # Emitted from the ROS callback thread; Qt delivers them on the GUI thread.
    _stft_ready = Signal(object)
    _det_ready  = Signal(object)

    def __init__(self, node) -> None:
        super().__init__()
        self._node = node

        self._stft_msg: Optional[StftFrame] = None
        self._detections_msg: Optional[RfDetectionArray] = None
        self._stft_sub = None
        self._det_sub = None
        self._image = None
        self._rects: list = []
        self._labels: list = []
        self._class_visibility: dict[str, bool] = {}
        self._class_checkboxes: dict[str, QCheckBox] = {}
        self._confidence_threshold = 0.0
        self._updating_topics = False

        self._build_ui()

        self._stft_ready.connect(self._on_stft_ready)
        self._det_ready.connect(self._on_det_ready)

        # Topic discovery only — no render timer.
        self._topic_timer = QTimer(self)
        self._topic_timer.timeout.connect(self._refresh_topic_lists)
        self._topic_timer.start(1500)

        self._refresh_topic_lists()

    def shutdown(self) -> None:
        self._topic_timer.stop()
        self._unsubscribe_stft()
        self._unsubscribe_detections()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("STFT Topic:"))
        self._stft_combo = QComboBox()
        self._stft_combo.currentTextChanged.connect(self._on_stft_topic_changed)
        controls.addWidget(self._stft_combo, 2)

        controls.addWidget(QLabel("Detections Topic:"))
        self._det_combo = QComboBox()
        self._det_combo.currentTextChanged.connect(self._on_detections_topic_changed)
        controls.addWidget(self._det_combo, 2)

        refresh_button = QPushButton("Refresh Topics")
        refresh_button.clicked.connect(self._refresh_topic_lists)
        controls.addWidget(refresh_button)

        layout.addLayout(controls)

        class_controls = QHBoxLayout()
        class_controls.addWidget(QLabel("Show Classes:"))
        self._class_filter_widget = QWidget()
        self._class_filter_layout = QHBoxLayout(self._class_filter_widget)
        self._class_filter_layout.setContentsMargins(0, 0, 0, 0)
        self._class_filter_layout.setSpacing(8)
        self._class_filter_layout.addStretch()
        class_controls.addWidget(self._class_filter_widget, 1)
        class_controls.addWidget(QLabel("Min Confidence:"))
        self._confidence_spinbox = QDoubleSpinBox()
        self._confidence_spinbox.setRange(0.0, 1.0)
        self._confidence_spinbox.setSingleStep(0.05)
        self._confidence_spinbox.setDecimals(2)
        self._confidence_spinbox.setValue(self._confidence_threshold)
        self._confidence_spinbox.valueChanged.connect(self._on_confidence_threshold_changed)
        class_controls.addWidget(self._confidence_spinbox)
        layout.addLayout(class_controls)

        self._figure = Figure(figsize=(9, 6))
        self._canvas = FigureCanvasQTAgg(self._figure)
        self._ax = self._figure.add_subplot(111)
        self._ax.set_title("RF Waterfull Viewer")
        self._ax.set_xlabel("Frequency (GHz)")
        self._ax.set_ylabel("Time (s)")
        self._figure.tight_layout()
        layout.addWidget(self._canvas)

    # ── Topic management ──────────────────────────────────────────────────────

    def _refresh_topic_lists(self) -> None:
        topic_names_and_types = self._node.get_topic_names_and_types()
        stft_topics = sorted(
            name for name, types in topic_names_and_types
            if "rf_msgs/msg/StftFrame" in types
        )
        det_topics = sorted(
            name for name, types in topic_names_and_types
            if "rf_msgs/msg/RfDetectionArray" in types
        )

        self._updating_topics = True
        try:
            self._reset_combo(self._stft_combo, stft_topics)
            self._reset_combo(self._det_combo, det_topics)
        finally:
            self._updating_topics = False

        if self._stft_combo.count() > 0 and self._stft_sub is None:
            self._on_stft_topic_changed(self._stft_combo.currentText())
        if self._det_combo.count() > 0 and self._det_sub is None:
            self._on_detections_topic_changed(self._det_combo.currentText())

    def _reset_combo(self, combo: QComboBox, topics: list) -> None:
        current = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItems(topics)
        if current:
            idx = combo.findText(current)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def _on_stft_topic_changed(self, topic: str) -> None:
        if self._updating_topics or not topic:
            return
        self._unsubscribe_stft()
        self._stft_msg = None
        self._stft_sub = self._node.create_subscription(
            StftFrame, topic, self._on_stft, 10
        )

    def _on_detections_topic_changed(self, topic: str) -> None:
        if self._updating_topics or not topic:
            return
        self._unsubscribe_detections()
        self._detections_msg = None
        self._sync_class_filters([])
        self._redraw_detections()
        self._canvas.draw_idle()
        self._det_sub = self._node.create_subscription(
            RfDetectionArray, topic, self._on_detections, 10
        )

    def _unsubscribe_stft(self) -> None:
        if self._stft_sub is not None:
            self._node.destroy_subscription(self._stft_sub)
            self._stft_sub = None

    def _unsubscribe_detections(self) -> None:
        if self._det_sub is not None:
            self._node.destroy_subscription(self._det_sub)
            self._det_sub = None

    # ── ROS callbacks (executor thread) ──────────────────────────────────────

    def _on_stft(self, msg: StftFrame) -> None:
        self._stft_ready.emit(msg)

    def _on_detections(self, msg: RfDetectionArray) -> None:
        self._det_ready.emit(msg)

    # ── Qt slots (GUI thread) ─────────────────────────────────────────────────

    @Slot(object)
    def _on_stft_ready(self, msg: StftFrame) -> None:
        """New waterfall frame: update the image then redraw detection boxes."""
        self._stft_msg = msg
        self._redraw_waterfall()
        self._redraw_detections()

    @Slot(object)
    def _on_det_ready(self, msg: RfDetectionArray) -> None:
        """New detections: swap the boxes without touching the waterfall image."""
        self._detections_msg = msg
        self._sync_class_filters(sorted({det.kind for det in msg.detections if det.kind}))
        if self._image is None:
            return  # no axes extent yet — boxes will appear with the next STFT frame
        self._redraw_detections()
        self._canvas.draw_idle()

    # ── Rendering ─────────────────────────────────────────────────────────────

    def _redraw_waterfall(self) -> None:
        msg = self._stft_msg
        if msg is None or msg.num_frames == 0 or msg.fft_size == 0:
            return

        try:
            sxx = np.asarray(msg.waterfall_db, dtype=np.float64).reshape(
                int(msg.num_frames), int(msg.fft_size)
            )
            f = np.asarray(msg.freq_ghz, dtype=np.float64)
            t = np.asarray(msg.time_s, dtype=np.float64)
        except ValueError:
            return

        if f.size < 2 or t.size < 2:
            return

        extent = [float(f[0]), float(f[-1]), float(t[0]), float(t[-1])]

        if self._image is None:
            self._image = self._ax.imshow(
                sxx,
                origin="lower",
                aspect="auto",
                extent=extent,
                cmap="inferno",
            )
            self._figure.colorbar(self._image, ax=self._ax, label="STFT (dB)")
        else:
            self._image.set_data(sxx)
            self._image.set_extent(extent)

        self._ax.set_xlim(extent[0], extent[1])
        self._ax.set_ylim(extent[2], extent[3])
        self._ax.set_title(
            f"RF Waterfull Viewer | {int(msg.fft_size)} bins x {int(msg.num_frames)} frames"
        )
        self._canvas.draw_idle()

    def _redraw_detections(self) -> None:
        """Replace all bbox patches. Caller is responsible for draw_idle()."""
        for rect in self._rects:
            rect.remove()
        self._rects.clear()

        for label in self._labels:
            label.remove()
        self._labels.clear()

        if self._detections_msg is None:
            return

        for det in self._detections_msg.detections:
            if det.confidence < self._confidence_threshold:
                continue
            if det.kind and not self._class_visibility.get(det.kind, True):
                continue
            color = self._color_for_kind(det.kind)
            rect = Rectangle(
                (det.f_lo_ghz, det.t_lo_s),
                det.f_hi_ghz - det.f_lo_ghz,
                det.t_hi_s - det.t_lo_s,
                linewidth=2.0,
                edgecolor=color,
                facecolor="none",
            )
            self._ax.add_patch(rect)
            self._rects.append(rect)

            text = self._ax.text(
                det.f_lo_ghz,
                det.t_hi_s,
                f"{det.kind} {det.confidence:.0%}",
                color=color,
                fontsize=9,
                verticalalignment="bottom",
                bbox={"facecolor": "black", "alpha": 0.45, "pad": 2, "edgecolor": "none"},
            )
            self._labels.append(text)

    def _sync_class_filters(self, kinds: list[str]) -> None:
        ordered_kinds = sorted(dict.fromkeys(kinds))
        current_kinds = set(self._class_checkboxes)
        next_kinds = set(ordered_kinds)

        if current_kinds == next_kinds:
            return

        for kind in list(self._class_checkboxes):
            if kind in next_kinds:
                continue
            checkbox = self._class_checkboxes.pop(kind)
            self._class_filter_layout.removeWidget(checkbox)
            checkbox.deleteLater()
            self._class_visibility.pop(kind, None)

        for kind in ordered_kinds:
            if kind in self._class_checkboxes:
                continue
            checkbox = QCheckBox(kind)
            checkbox.setChecked(self._class_visibility.get(kind, True))
            checkbox.toggled.connect(
                lambda checked, class_name=kind: self._on_class_filter_toggled(class_name, checked)
            )
            checkbox.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
            insert_at = max(0, self._class_filter_layout.count() - 1)
            self._class_filter_layout.insertWidget(insert_at, checkbox)
            self._class_checkboxes[kind] = checkbox
            self._class_visibility[kind] = checkbox.isChecked()

    def _on_class_filter_toggled(self, kind: str, checked: bool) -> None:
        self._class_visibility[kind] = checked
        self._redraw_detections()
        self._canvas.draw_idle()

    def _on_confidence_threshold_changed(self, value: float) -> None:
        self._confidence_threshold = value
        self._redraw_detections()
        self._canvas.draw_idle()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _color_for_kind(self, kind: str) -> str:
        if not kind:
            return "#ffffff"
        digest = hashlib.md5(kind.encode("utf-8")).hexdigest()
        hue = (int(digest[:8], 16) % 360) / 360.0
        r, g, b = colorsys.hls_to_rgb(hue, 0.60, 0.85)
        return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


class WaterfullPlugin(Plugin):
    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName("WaterfullPlugin")

        self._widget = WaterfullWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"RF Waterfull Viewer ({context.serial_number()})"
            )
        context.add_widget(self._widget)

    def shutdown_plugin(self) -> None:
        self._widget.shutdown()
