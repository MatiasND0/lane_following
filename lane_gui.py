#!/usr/bin/env python3
"""
lane_gui.py
-----------
GUI PyQt5 que se suscribe a los topics de debug del nodo C++
y muestra cada etapa del pipeline de detección de carril.
Incluye un panel de calibración BEV para ajustar la homografía en tiempo real.

Topics suscritos:
  /lane_debug/original          sensor_msgs/CompressedImage
  /lane_debug/hls_mask          sensor_msgs/CompressedImage
  /lane_debug/bev               sensor_msgs/CompressedImage
  /lane_debug/sliding_window    sensor_msgs/CompressedImage
  /lane_debug/overlay           sensor_msgs/CompressedImage
  /lane_errors_est              std_msgs/Float32MultiArray

Uso:
    source /opt/ros/humble/setup.bash
    python3 lane_gui.py
"""

import sys
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RosParameter, ParameterValue, ParameterType

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor, QPalette, QFontDatabase
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel,
    QGridLayout, QVBoxLayout, QHBoxLayout,
    QGroupBox, QSizePolicy, QFrame, QSlider,
    QPushButton, QSplitter, QTabWidget, QSpinBox,
    QDoubleSpinBox, QComboBox, QFormLayout, QScrollArea
)

# Evita conflicto de plugins Qt entre OpenCV y PyQt5.
# OpenCV puede setear QT_QPA_PLATFORM_PLUGIN_PATH hacia su propio bundle
# (cv2/qt/plugins), lo que rompe la inicialización de QApplication.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_FONTDIR", None)


# ---------------------------------------------------------------------------
# Señales Qt para comunicación desde el hilo ROS2
# ---------------------------------------------------------------------------

class RosSignals(QObject):
    image_received  = pyqtSignal(str, np.ndarray)   # (topic_label, frame)
    errors_received = pyqtSignal(float, float, float)  # e2, e3, k
    control_command = pyqtSignal(str)
    set_bev_params  = pyqtSignal(list, list)  # (src_pts_flat, dst_pts_flat)
    set_camera_params = pyqtSignal(dict)      # parámetros de orientación de cámara
    set_tuning_params = pyqtSignal(dict)      # parámetros de tuning del detector


# ---------------------------------------------------------------------------
# Nodo ROS2 (corre en hilo separado)
# ---------------------------------------------------------------------------

class LaneGuiNode(Node):
    def __init__(self, signals: RosSignals):
        super().__init__("lane_gui_node")
        self.signals = signals

        topics = {
            "/lane_debug/original":       "Original",
            "/lane_debug/hls_mask":       "HLS Mask",
            "/lane_debug/bev":            "Bird's Eye View",
            "/lane_debug/sliding_window": "Sliding Window",
            "/lane_debug/overlay":        "Lane Overlay",
        }

        self.subs = []
        for topic, label in topics.items():
            sub = self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, l=label: self._on_image(msg, l),
                10,
            )
            self.subs.append(sub)

        self.errors_sub = self.create_subscription(
            Float32MultiArray,
            "/lane_errors_est",
            self._on_errors,
            10,
        )

        self.ctrl_pub = self.create_publisher(String, "/lane_control/cmd", 10)
        self.signals.control_command.connect(self._on_control_command)

        # Cliente para SetParameters del nodo pipeline
        self.set_params_client = self.create_client(
            SetParameters,
            "/lane_pipeline_node/set_parameters",
        )
        self.signals.set_bev_params.connect(self._on_set_bev_params)
        self.signals.set_camera_params.connect(self._on_set_camera_params)
        self.signals.set_tuning_params.connect(self._on_set_tuning_params)

        self.get_logger().info("LaneGuiNode suscrito a los topics de debug.")

    def _on_image(self, msg: CompressedImage, label: str):
        np_arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.signals.image_received.emit(label, frame_rgb)

    def _on_errors(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.signals.errors_received.emit(
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
            )

    def _on_control_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.ctrl_pub.publish(msg)

    def _on_set_bev_params(self, src_flat: list, dst_flat: list):
        """Envía los nuevos puntos de homografía al nodo C++ via SetParameters."""
        if not self.set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Servicio SetParameters del nodo pipeline no disponible.")
            return

        request = SetParameters.Request()

        # bev_src_pts
        src_param = RosParameter()
        src_param.name = "bev_src_pts"
        src_param.value = ParameterValue()
        src_param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        src_param.value.double_array_value = [float(v) for v in src_flat]
        request.parameters.append(src_param)

        # bev_dst_pts
        dst_param = RosParameter()
        dst_param.name = "bev_dst_pts"
        dst_param.value = ParameterValue()
        dst_param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        dst_param.value.double_array_value = [float(v) for v in dst_flat]
        request.parameters.append(dst_param)

        future = self.set_params_client.call_async(request)
        future.add_done_callback(self._on_set_params_done)

    def _on_set_camera_params(self, params: dict):
        """Envía parámetros de orientación de cámara al nodo C++ via SetParameters."""
        if not self.set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Servicio SetParameters del nodo pipeline no disponible.")
            return

        request = SetParameters.Request()

        # Mapeo nombre_gui → nombre_param_ros
        param_map = {
            "bev_mode":               ParameterType.PARAMETER_STRING,
            "camera_pitch":           ParameterType.PARAMETER_DOUBLE,
            "camera_yaw":             ParameterType.PARAMETER_DOUBLE,
            "camera_roll":            ParameterType.PARAMETER_DOUBLE,
            "camera_height":          ParameterType.PARAMETER_DOUBLE,
            "camera_lateral_offset":  ParameterType.PARAMETER_DOUBLE,
        }

        for name, ptype in param_map.items():
            if name not in params:
                continue
            p = RosParameter()
            p.name = name
            p.value = ParameterValue()
            p.value.type = ptype
            if ptype == ParameterType.PARAMETER_STRING:
                p.value.string_value = str(params[name])
            else:
                p.value.double_value = float(params[name])
            request.parameters.append(p)

        future = self.set_params_client.call_async(request)
        future.add_done_callback(self._on_set_params_done)

    def _on_set_tuning_params(self, params: dict):
        """Envía parámetros de tuning del detector al nodo C++ via SetParameters."""
        if not self.set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Servicio SetParameters del nodo pipeline no disponible.")
            return

        request = SetParameters.Request()

        param_map = {
            "center_inference_extra_px": ParameterType.PARAMETER_DOUBLE,
        }

        for name, ptype in param_map.items():
            if name not in params:
                continue
            p = RosParameter()
            p.name = name
            p.value = ParameterValue()
            p.value.type = ptype
            p.value.double_value = float(params[name])
            request.parameters.append(p)

        future = self.set_params_client.call_async(request)
        future.add_done_callback(self._on_set_params_done)

    def _on_set_params_done(self, future):
        try:
            response = future.result()
            ok = all(r.successful for r in response.results)
            if ok:
                self.get_logger().info("Parámetros actualizados exitosamente.")
            else:
                reasons = [r.reason for r in response.results if not r.successful]
                self.get_logger().warn(f"Fallo al actualizar parámetros: {reasons}")
        except Exception as e:
            self.get_logger().error(f"Error al llamar SetParameters: {e}")


# ---------------------------------------------------------------------------
# Label interactiva para arrastrar puntos de BEV
# ---------------------------------------------------------------------------

class InteractiveLabel(QLabel):
    point_dragged = pyqtSignal(int, float, float)  # idx, x, y (en espacio imagen)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []  # [(x, y), ...] en espacio imagen (640x360)
        self.selected_point = -1
        self.setMouseTracking(True)
        self.image_size = (640, 360)

    def set_points(self, points):
        self.points = points
        self.update()

    def _map_to_image(self, pos):
        """Mapea coordenadas del widget a coordenadas de la imagen (640x360)."""
        if not self.pixmap() or self.pixmap().isNull():
            return -1, -1

        pw = self.pixmap().width()
        ph = self.pixmap().height()
        ww = self.width()
        wh = self.height()

        # Offset por el centrado del pixmap en el label
        ox = (ww - pw) / 2
        oy = (wh - ph) / 2

        # Coordenadas relativas al pixmap
        px = pos.x() - ox
        py = pos.y() - oy

        # Validar si está dentro del pixmap
        if px < 0 or px > pw or py < 0 or py > ph:
            return -1, -1

        # Mapear a 640x360
        ix = (px / pw) * self.image_size[0]
        iy = (py / ph) * self.image_size[1]
        return ix, iy

    def _map_from_image(self, ix, iy):
        """Mapea coordenadas de la imagen (640x360) a coordenadas del widget."""
        if not self.pixmap() or self.pixmap().isNull():
            return -1, -1

        pw = self.pixmap().width()
        ph = self.pixmap().height()
        ww = self.width()
        wh = self.height()

        ox = (ww - pw) / 2
        oy = (wh - ph) / 2

        px = (ix / self.image_size[0]) * pw
        py = (iy / self.image_size[1]) * ph

        return px + ox, py + oy

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            ix, iy = self._map_to_image(event.pos())
            if ix == -1: return

            # Buscar punto más cercano
            best_dist = 20.0  # Umbral de captura en px imagen (ajustado)
            self.selected_point = -1
            for i, (px, py) in enumerate(self.points):
                dist = np.sqrt((ix - px)**2 + (iy - py)**2)
                if dist < best_dist:
                    best_dist = dist
                    self.selected_point = i
            self.update()

    def mouseMoveEvent(self, event):
        if self.selected_point != -1:
            ix, iy = self._map_to_image(event.pos())
            if ix != -1:
                # Permitir arrastrar un poco fuera de la imagen
                self.point_dragged.emit(self.selected_point, ix, iy)

    def mouseReleaseEvent(self, event):
        self.selected_point = -1
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self.points or not self.pixmap():
            return

        from PyQt5.QtGui import QPainter, QPen, QBrush

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 1) Dibujar líneas del trapecio
        pen = QPen(QColor(0, 229, 255, 180), 2)
        painter.setPen(pen)
        
        widget_pts = [self._map_from_image(x, y) for x, y in self.points]
        for i in range(4):
            p1 = widget_pts[i]
            p2 = widget_pts[(i + 1) % 4]
            if p1[0] != -1 and p2[0] != -1:
                painter.drawLine(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]))

        # 2) Dibujar puntos
        for i, (wx, wy) in enumerate(widget_pts):
            if wx == -1: continue
            
            color = QColor(105, 255, 71) if i != self.selected_point else QColor(255, 255, 0)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(Qt.black, 1))
            painter.drawEllipse(int(wx - 5), int(wy - 5), 10, 10)
            
            painter.setPen(QPen(Qt.white))
            painter.drawText(int(wx + 8), int(wy + 5), f"P{i+1}")


# ---------------------------------------------------------------------------
# Widget de imagen individual con título
# ---------------------------------------------------------------------------

class ImagePanel(QFrame):
    def __init__(self, title: str, parent=None, interactive=False):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setObjectName("imagePanel")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        # Título
        self.title_lbl = QLabel(title)
        self.title_lbl.setObjectName("panelTitle")
        self.title_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_lbl)

        # Imagen
        if interactive:
            self.image_lbl = InteractiveLabel()
        else:
            self.image_lbl = QLabel()
            
        self.image_lbl.setObjectName("imageLabel")
        self.image_lbl.setAlignment(Qt.AlignCenter)
        self.image_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_lbl.setMinimumSize(200, 150)
        self.image_lbl.setText("Esperando datos...")
        layout.addWidget(self.image_lbl)

        # FPS counter
        self.fps_lbl = QLabel("— FPS")
        self.fps_lbl.setObjectName("fpsLabel")
        self.fps_lbl.setAlignment(Qt.AlignRight)
        layout.addWidget(self.fps_lbl)

        self._last_time = time.time()
        self._fps_alpha = 0.1
        self._fps = 0.0

    def update_frame(self, frame: np.ndarray):
        # Calcular FPS
        now = time.time()
        dt = now - self._last_time
        if dt > 0:
            self._fps = self._fps_alpha * (1.0 / dt) + (1.0 - self._fps_alpha) * self._fps
        self._last_time = now
        self.fps_lbl.setText(f"{self._fps:.1f} FPS")

        # Convertir frame a QPixmap escalado
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        # Escalar manteniendo aspect ratio
        lbl_size = self.image_lbl.size()
        scaled = pixmap.scaled(lbl_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_lbl.setPixmap(scaled)


# ---------------------------------------------------------------------------
# Widget de métricas del MPC
# ---------------------------------------------------------------------------

class MetricsPanel(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setObjectName("metricsPanel")
        self.setFixedHeight(140)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(32)

        self.e2_display = self._make_metric("e₂  Lateral", "m", "#00E5FF")
        self.e3_display = self._make_metric("e₃  Angular", "rad", "#69FF47")
        self.k_display  = self._make_metric("k  Curvatura", "m⁻¹", "#FF6B35")

        for widget in [self.e2_display, self.e3_display, self.k_display]:
            layout.addWidget(widget)

        # Historial para mini-gráfico textual
        self.e2_history = deque(maxlen=40)
        self.e3_history = deque(maxlen=40)

    def _make_metric(self, name, unit, color):
        frame = QFrame()
        frame.setObjectName("metricFrame")
        vbox = QVBoxLayout(frame)
        vbox.setSpacing(2)
        vbox.setContentsMargins(0, 0, 0, 0)

        name_lbl = QLabel(name)
        name_lbl.setObjectName("metricName")

        value_lbl = QLabel("—")
        value_lbl.setObjectName("metricValue")
        value_lbl.setStyleSheet(f"color: {color};")

        unit_lbl = QLabel(unit)
        unit_lbl.setObjectName("metricUnit")

        vbox.addWidget(name_lbl)
        vbox.addWidget(value_lbl)
        vbox.addWidget(unit_lbl)

        frame._value_lbl = value_lbl
        return frame

    def update_errors(self, e2: float, e3: float, k: float):
        self.e2_display._value_lbl.setText(f"{e2:+.4f}")
        self.e3_display._value_lbl.setText(f"{e3:+.4f}")
        self.k_display._value_lbl.setText(f"{k:+.4f}")


# ---------------------------------------------------------------------------
# Panel de calibración BEV
# ---------------------------------------------------------------------------

class BevCalibrationPanel(QFrame):
    """
    Panel interactivo para calibrar los puntos de la homografía BEV.
    Contiene tabs para puntos SRC/DST, cámara y tuning del detector.
    """

    # Defaults (mismos que el nodo C++)
    DEFAULT_SRC = [
        (-1.0, 274.0),
        (652.0, 275.0),
        (822.0, 344.0),
        (-159.0, 344.0),
    ]

    DEFAULT_DST = [
        (32.0,    0.0),   # top-left     (320*0.10, 0)
        (288.0,   0.0),   # top-right    (320*0.90, 0)
        (288.0, 240.0),   # bottom-right (320*0.90, 240)
        (32.0,  240.0),   # bottom-left  (320*0.10, 240)
    ]

    DEFAULT_TUNE = {
        "center_inference_extra_px": 15.0,
    }

    POINT_NAMES = [
        "Sup-Izq (lejos)",
        "Sup-Der (lejos)",
        "Inf-Der (cerca)",
        "Inf-Izq (cerca)",
    ]

    def __init__(self, signals: RosSignals, parent=None):
        super().__init__(parent)
        self.signals = signals
        self.setFrameShape(QFrame.StyledPanel)
        self.setObjectName("calibPanel")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Título
        title = QLabel("⚙  CALIBRACIÓN BEV")
        title.setObjectName("calibTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Tabs SRC / DST / Cámara
        tabs = QTabWidget()
        tabs.setObjectName("calibTabs")

        self.src_spins = self._build_point_tab("Trapecio (Perspectiva)", 640, 360)
        self.dst_spins = self._build_point_tab("Rectángulo (BEV)", 320, 240)
        self.camera_tab = self._build_camera_tab()
        self.tune_tab = self._build_tuning_tab()

        tabs.addTab(self.src_spins["widget"], "SRC")
        tabs.addTab(self.dst_spins["widget"], "DST")
        tabs.addTab(self.camera_tab["widget"], "CAM")
        tabs.addTab(self.tune_tab["widget"], "TUNE")
        layout.addWidget(tabs, stretch=1)

        # Botones
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(6)

        self.apply_btn = QPushButton("▶ Aplicar Puntos")
        self.apply_btn.setObjectName("calibApplyBtn")
        self.apply_btn.clicked.connect(self._on_apply)

        self.apply_cam_btn = QPushButton("📷 Aplicar Cámara")
        self.apply_cam_btn.setObjectName("calibApplyBtn")
        self.apply_cam_btn.clicked.connect(self._on_apply_camera)

        self.apply_tune_btn = QPushButton("🎯 Aplicar Tuning")
        self.apply_tune_btn.setObjectName("calibApplyBtn")
        self.apply_tune_btn.clicked.connect(self._on_apply_tuning)

        self.reset_btn = QPushButton("↻ Reset")
        self.reset_btn.setObjectName("calibResetBtn")
        self.reset_btn.clicked.connect(self._on_reset)

        btn_layout.addWidget(self.apply_btn)
        btn_layout.addWidget(self.apply_cam_btn)
        btn_layout.addWidget(self.apply_tune_btn)
        btn_layout.addWidget(self.reset_btn)
        layout.addLayout(btn_layout)

        # Status
        self.status_lbl = QLabel("Listo")
        self.status_lbl.setObjectName("calibStatus")
        self.status_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_lbl)

        # Inicializar con defaults
        self._set_values(self.src_spins, self.DEFAULT_SRC)
        self._set_values(self.dst_spins, self.DEFAULT_DST)

    def _build_point_tab(self, title: str, max_x: int, max_y: int) -> dict:
        widget = QWidget()
        widget.setObjectName("calibTabContent")
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        spins = []
        for i, name in enumerate(self.POINT_NAMES):
            group = QFrame()
            group.setObjectName("calibPointGroup")
            glayout = QHBoxLayout(group)
            glayout.setContentsMargins(4, 2, 4, 2)
            glayout.setSpacing(4)

            lbl = QLabel(f"P{i+1}")
            lbl.setObjectName("calibPointLabel")
            lbl.setFixedWidth(22)
            lbl.setToolTip(name)

            x_spin = QSpinBox()
            x_spin.setObjectName("calibSpin")
            x_spin.setRange(-1000, 2000)
            x_spin.setPrefix("x:")
            x_spin.setSuffix("px")

            y_spin = QSpinBox()
            y_spin.setObjectName("calibSpin")
            y_spin.setRange(-1000, 2000)
            y_spin.setPrefix("y:")
            y_spin.setSuffix("px")

            glayout.addWidget(lbl)
            glayout.addWidget(x_spin, 1)
            glayout.addWidget(y_spin, 1)

            layout.addWidget(group)
            spins.append((x_spin, y_spin))

        layout.addStretch()

        return {"widget": widget, "spins": spins}

    def _get_values(self, tab_data: dict) -> list:
        """Devuelve lista flat de 8 doubles desde los spinboxes."""
        result = []
        for x_spin, y_spin in tab_data["spins"]:
            result.append(float(x_spin.value()))
            result.append(float(y_spin.value()))
        return result

    def _set_values(self, tab_data: dict, points: list):
        """Setea los spinboxes desde una lista de tuplas [(x,y), ...]."""
        for i, (x_spin, y_spin) in enumerate(tab_data["spins"]):
            x_spin.blockSignals(True)
            x_spin.setValue(int(points[i][0]))
            x_spin.blockSignals(False)
            
            y_spin.blockSignals(True)
            y_spin.setValue(int(points[i][1]))
            y_spin.blockSignals(False)

    def get_src_points(self):
        """Devuelve la lista de puntos [(x,y), ...] del tab SRC."""
        pts = []
        for x_spin, y_spin in self.src_spins["spins"]:
            pts.append((x_spin.value(), y_spin.value()))
        return pts

    def set_src_point(self, idx, x, y):
        """Setea un punto específico (x, y) en el tab SRC."""
        if 0 <= idx < len(self.src_spins["spins"]):
            sx, sy = self.src_spins["spins"][idx]
            sx.setValue(int(x))
            sy.setValue(int(y))

    def _build_camera_tab(self) -> dict:
        """Construye el tab con los controles de orientación de cámara."""
        widget = QWidget()
        widget.setObjectName("calibTabContent")
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        spins = {}

        # Modo BEV
        mode_group = QFrame()
        mode_group.setObjectName("calibPointGroup")
        mg_layout = QHBoxLayout(mode_group)
        mg_layout.setContentsMargins(4, 2, 4, 2)
        mg_layout.setSpacing(4)
        mg_layout.addWidget(QLabel("Modo"))
        mode_combo = QComboBox()
        mode_combo.setObjectName("calibSpin")
        mode_combo.addItems(["points", "model"])
        mg_layout.addWidget(mode_combo, 1)
        layout.addWidget(mode_group)
        spins["mode"] = mode_combo

        # Parámetros de orientación
        cam_params = [
            ("Pitch",  "°", -30.0, 30.0, 0.1,  -8.0),
            ("Yaw",    "°", -30.0, 30.0, 0.1,   0.0),
            ("Roll",   "°", -15.0, 15.0, 0.1,   0.0),
            ("Altura", "m",  0.05,  1.0, 0.001, 0.230),
            ("Offset", "mm",-100.0,100.0, 0.5,  32.5),
        ]

        for name, unit, vmin, vmax, step, default in cam_params:
            group = QFrame()
            group.setObjectName("calibPointGroup")
            glayout = QHBoxLayout(group)
            glayout.setContentsMargins(4, 2, 4, 2)
            glayout.setSpacing(4)

            lbl = QLabel(name)
            lbl.setObjectName("calibPointLabel")
            lbl.setFixedWidth(42)

            spin = QDoubleSpinBox()
            spin.setObjectName("calibSpin")
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            spin.setDecimals(3)
            spin.setSuffix(f" {unit}")
            spin.setValue(default)

            glayout.addWidget(lbl)
            glayout.addWidget(spin, 1)
            layout.addWidget(group)
            spins[name.lower()] = spin

        layout.addStretch()
        return {"widget": widget, "spins": spins}

    def _build_tuning_tab(self) -> dict:
        """Construye el tab con parámetros de tuning del detector."""
        widget = QWidget()
        widget.setObjectName("calibTabContent")
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        spins = {}

        group = QFrame()
        group.setObjectName("calibPointGroup")
        glayout = QHBoxLayout(group)
        glayout.setContentsMargins(4, 2, 4, 2)
        glayout.setSpacing(4)

        lbl = QLabel("Ctr +px")
        lbl.setObjectName("calibPointLabel")
        lbl.setFixedWidth(56)
        lbl.setToolTip("Separación extra (px) del centro inferido respecto al medio carril")

        spin = QDoubleSpinBox()
        spin.setObjectName("calibSpin")
        spin.setRange(-20.0, 20.0)
        spin.setSingleStep(0.1)
        spin.setDecimals(2)
        spin.setSuffix(" px")
        spin.setValue(self.DEFAULT_TUNE["center_inference_extra_px"])

        glayout.addWidget(lbl)
        glayout.addWidget(spin, 1)
        layout.addWidget(group)

        spins["center_inference_extra_px"] = spin
        layout.addStretch()
        return {"widget": widget, "spins": spins}

    def _on_apply(self):
        src_flat = self._get_values(self.src_spins)
        dst_flat = self._get_values(self.dst_spins)
        self.signals.set_bev_params.emit(src_flat, dst_flat)
        self.status_lbl.setText("⬤ Puntos enviados")
        self.status_lbl.setStyleSheet("color: #69FF47;")

    def _on_apply_camera(self):
        """Lee los valores del tab Cámara y los envía al nodo C++."""
        s = self.camera_tab["spins"]
        params = {
            "bev_mode":              s["mode"].currentText(),
            "camera_pitch":          np.deg2rad(s["pitch"].value()),
            "camera_yaw":            np.deg2rad(s["yaw"].value()),
            "camera_roll":           np.deg2rad(s["roll"].value()),
            "camera_height":         s["altura"].value(),
            "camera_lateral_offset": s["offset"].value() / 1000.0,  # mm → m
        }
        self.signals.set_camera_params.emit(params)
        self.status_lbl.setText(f"📷 Cámara ({s['mode'].currentText()})")
        self.status_lbl.setStyleSheet("color: #69FF47;")

    def _on_apply_tuning(self):
        """Lee los valores del tab TUNE y los envía al nodo C++."""
        s = self.tune_tab["spins"]
        params = {
            "center_inference_extra_px": s["center_inference_extra_px"].value(),
        }
        self.signals.set_tuning_params.emit(params)
        self.status_lbl.setText(f"🎯 Tuning (extra={params['center_inference_extra_px']:.2f}px)")
        self.status_lbl.setStyleSheet("color: #69FF47;")

    def _on_reset(self):
        self._set_values(self.src_spins, self.DEFAULT_SRC)
        self._set_values(self.dst_spins, self.DEFAULT_DST)
        # Reset camera tab
        s = self.camera_tab["spins"]
        s["mode"].setCurrentIndex(0)  # "points"
        s["pitch"].setValue(-8.0)
        s["yaw"].setValue(0.0)
        s["roll"].setValue(0.0)
        s["altura"].setValue(0.230)
        s["offset"].setValue(32.5)
        self.tune_tab["spins"]["center_inference_extra_px"].setValue(
            self.DEFAULT_TUNE["center_inference_extra_px"]
        )
        self.status_lbl.setText("↻ Valores por defecto")
        self.status_lbl.setStyleSheet("color: #FFA726;")


# ---------------------------------------------------------------------------
# Ventana principal
# ---------------------------------------------------------------------------

class LaneDebugWindow(QMainWindow):

    PIPELINE_LABELS = [
        "Original",
        "HLS Mask",
        "Bird's Eye View",
        "Sliding Window",
        "Lane Overlay",
    ]

    def __init__(self, signals: RosSignals):
        super().__init__()
        self.signals = signals
        self.panels: dict[str, ImagePanel] = {}
        self.is_paused = False
        self._setup_ui()
        self._apply_stylesheet()
        self._connect_signals()

    # ------------------------------------------------------------------
    # UI setup
    # ------------------------------------------------------------------

    def _setup_ui(self):
        self.setWindowTitle("BFMC — Lane Detection Pipeline Debug")
        self.resize(1400, 860)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 8)
        root_layout.setSpacing(8)

        # Header
        header = self._build_header()
        root_layout.addWidget(header)

        # Grid de imágenes — 2 filas, la primera 3 paneles, la segunda 2 + calib
        grid = QGridLayout()
        grid.setSpacing(8)

        labels_row0 = self.PIPELINE_LABELS[:3]
        labels_row1 = self.PIPELINE_LABELS[3:]

        for col, label in enumerate(labels_row0):
            interactive = (label == "Original")
            panel = ImagePanel(label, interactive=interactive)
            self.panels[label] = panel
            grid.addWidget(panel, 0, col)
            
            if interactive:
                # Sincronizar puntos arrastrados con spinboxes
                panel.image_lbl.point_dragged.connect(self._on_panel_point_drag_update)
        for col, label in enumerate(labels_row1):
            panel = ImagePanel(label)
            self.panels[label] = panel
            grid.addWidget(panel, 1, col)

        # Panel de calibración BEV en la tercera celda de la segunda fila
        self.calib_panel = BevCalibrationPanel(self.signals)
        grid.addWidget(self.calib_panel, 1, 2)

        # Columnas con el mismo peso
        for c in range(3):
            grid.setColumnStretch(c, 1)
        for r in range(2):
            grid.setRowStretch(r, 1)

        root_layout.addLayout(grid, stretch=1)

        # Panel de métricas
        self.metrics = MetricsPanel()
        root_layout.addWidget(self.metrics)

    def _build_header(self):
        frame = QFrame()
        frame.setObjectName("headerFrame")
        frame.setFixedHeight(52)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(16, 0, 16, 0)

        title = QLabel("BFMC  ·  Lane Detection Pipeline")
        title.setObjectName("headerTitle")

        self.status_lbl = QLabel("⬤  Esperando topics...")
        self.status_lbl.setObjectName("statusLabel")

        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setObjectName("ctrlButton")
        self.back_btn = QPushButton("Back")
        self.back_btn.setObjectName("ctrlButton")
        self.step_btn = QPushButton("Step")
        self.step_btn.setObjectName("ctrlButton")
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.setObjectName("ctrlButton")

        self.pause_btn.clicked.connect(self._on_pause_resume)
        self.back_btn.clicked.connect(self._on_back)
        self.step_btn.clicked.connect(self._on_step)
        self.reset_btn.clicked.connect(self._on_reset)

        layout.addWidget(title)
        layout.addStretch()
        layout.addWidget(self.pause_btn)
        layout.addWidget(self.back_btn)
        layout.addWidget(self.step_btn)
        layout.addWidget(self.reset_btn)
        layout.addWidget(self.status_lbl)

        return frame

    # ------------------------------------------------------------------
    # Conexión de señales
    # ------------------------------------------------------------------

    def _connect_signals(self):
        self.signals.image_received.connect(self._on_image_received)
        self.signals.errors_received.connect(self._on_errors_received)
        
        # Enviar puntos iniciales al panel interactivo
        pts = self.calib_panel.get_src_points()
        self.panels["Original"].image_lbl.set_points(pts)
        
        # Suscribirse a cambios en los spinboxes para actualizar el overlay
        for sx, sy in self.calib_panel.src_spins["spins"]:
            sx.valueChanged.connect(self._update_overlay_from_gui)
            sy.valueChanged.connect(self._update_overlay_from_gui)

    def _on_panel_point_drag_update(self, idx, x, y):
        """Callback cuando se arrastra un punto en el panel 'Original'."""
        self.calib_panel.set_src_point(idx, x, y)
        # El cambio en el spinbox disparará _update_overlay_from_gui

    def _update_overlay_from_gui(self):
        """Actualiza los puntos dibujados en el panel 'Original' leyendo los spinboxes."""
        pts = self.calib_panel.get_src_points()
        self.panels["Original"].image_lbl.set_points(pts)
    def _on_image_received(self, label: str, frame: np.ndarray):
        if label in self.panels:
            self.panels[label].update_frame(frame)
            self.status_lbl.setText("⬤  Recibiendo datos")
            self.status_lbl.setStyleSheet("color: #69FF47; font-weight: bold;")

    def _on_errors_received(self, e2: float, e3: float, k: float):
        self.metrics.update_errors(e2, e3, k)

    def _on_pause_resume(self):
        if self.is_paused:
            self.signals.control_command.emit("resume")
            self.is_paused = False
            self.pause_btn.setText("Pause")
        else:
            self.signals.control_command.emit("pause")
            self.is_paused = True
            self.pause_btn.setText("Resume")

    def _on_step(self):
        self.signals.control_command.emit("step")
        self.is_paused = True
        self.pause_btn.setText("Resume")

    def _on_back(self):
        self.signals.control_command.emit("back")
        self.is_paused = True
        self.pause_btn.setText("Resume")

    def _on_reset(self):
        self.signals.control_command.emit("reset")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self._on_pause_resume()
            return
        if event.key() == Qt.Key_Right:
            self._on_step()
            return
        if event.key() == Qt.Key_Left:
            self._on_back()
            return
        if event.key() == Qt.Key_R:
            self._on_reset()
            return
        super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Stylesheet industrial / dark
    # ------------------------------------------------------------------

    def _apply_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0D0F14;
                color: #C8CDD8;
            }

            #headerFrame {
                background-color: #161A23;
                border-bottom: 1px solid #2A2F3E;
                border-radius: 4px;
            }

            #headerTitle {
                font-size: 15px;
                font-weight: 700;
                letter-spacing: 1.5px;
                color: #E8ECF5;
                font-family: "Courier New", monospace;
            }

            #statusLabel {
                font-size: 12px;
                color: #FFA726;
                font-family: "Courier New", monospace;
            }

            #ctrlButton {
                background-color: #1E2330;
                border: 1px solid #2E3550;
                border-radius: 4px;
                color: #C8CDD8;
                padding: 4px 10px;
                font-size: 11px;
                font-family: "Courier New", monospace;
            }

            #ctrlButton:hover {
                background-color: #2A3144;
            }

            #imagePanel {
                background-color: #13161E;
                border: 1px solid #1E2330;
                border-radius: 6px;
            }

            #imagePanel:hover {
                border: 1px solid #2E3550;
            }

            #panelTitle {
                font-size: 11px;
                font-weight: 600;
                letter-spacing: 1.2px;
                color: #7B8CB0;
                text-transform: uppercase;
                font-family: "Courier New", monospace;
                padding-bottom: 2px;
                border-bottom: 1px solid #1E2330;
            }

            #imageLabel {
                background-color: #0A0C10;
                border-radius: 3px;
                color: #3A4060;
                font-size: 11px;
            }

            #fpsLabel {
                font-size: 10px;
                color: #3A4060;
                font-family: "Courier New", monospace;
            }

            #metricsPanel {
                background-color: #161A23;
                border: 1px solid #1E2330;
                border-top: 2px solid #2E3550;
                border-radius: 6px;
            }

            #metricFrame {
                background: transparent;
            }

            #metricName {
                font-size: 10px;
                letter-spacing: 1px;
                color: #5A6080;
                font-family: "Courier New", monospace;
            }

            #metricValue {
                font-size: 26px;
                font-weight: 700;
                font-family: "Courier New", monospace;
            }

            #metricUnit {
                font-size: 10px;
                color: #5A6080;
                font-family: "Courier New", monospace;
            }

            /* ---- Panel de Calibración BEV ---- */

            #calibPanel {
                background-color: #13161E;
                border: 1px solid #1E2330;
                border-radius: 6px;
            }

            #calibTitle {
                font-size: 11px;
                font-weight: 600;
                letter-spacing: 1.2px;
                color: #00E5FF;
                font-family: "Courier New", monospace;
                padding-bottom: 4px;
                border-bottom: 1px solid #1E2330;
            }

            #calibTabs {
                background: transparent;
                font-size: 10px;
                font-family: "Courier New", monospace;
            }

            #calibTabs::pane {
                border: 1px solid #1E2330;
                border-radius: 3px;
                background-color: #0D0F14;
            }

            #calibTabs::tab-bar {
                alignment: center;
            }

            QTabBar::tab {
                background-color: #1E2330;
                color: #7B8CB0;
                border: 1px solid #2E3550;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                padding: 4px 16px;
                font-size: 10px;
                font-family: "Courier New", monospace;
                font-weight: 600;
                letter-spacing: 1px;
            }

            QTabBar::tab:selected {
                background-color: #0D0F14;
                color: #00E5FF;
                border-bottom: 2px solid #00E5FF;
            }

            QTabBar::tab:hover:!selected {
                background-color: #2A3144;
            }

            #calibTabContent {
                background: transparent;
            }

            #calibPointGroup {
                background-color: #161A23;
                border: 1px solid #1E2330;
                border-radius: 3px;
            }

            #calibPointLabel {
                font-size: 10px;
                font-weight: 700;
                color: #00E5FF;
                font-family: "Courier New", monospace;
            }

            #calibSpin {
                background-color: #0A0C10;
                border: 1px solid #2E3550;
                border-radius: 3px;
                color: #C8CDD8;
                font-size: 10px;
                font-family: "Courier New", monospace;
                padding: 2px 4px;
            }

            #calibSpin:focus {
                border: 1px solid #00E5FF;
            }

            #calibApplyBtn {
                background-color: #0D3B3B;
                border: 1px solid #00E5FF;
                border-radius: 4px;
                color: #00E5FF;
                padding: 6px 12px;
                font-size: 11px;
                font-weight: 600;
                font-family: "Courier New", monospace;
            }

            #calibApplyBtn:hover {
                background-color: #145050;
            }

            #calibResetBtn {
                background-color: #1E2330;
                border: 1px solid #2E3550;
                border-radius: 4px;
                color: #FFA726;
                padding: 6px 12px;
                font-size: 11px;
                font-family: "Courier New", monospace;
            }

            #calibResetBtn:hover {
                background-color: #2A3144;
            }

            #calibStatus {
                font-size: 10px;
                color: #5A6080;
                font-family: "Courier New", monospace;
            }
        """)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _ros_spin_worker(node: Node):
    try:
        rclpy.spin(node)
    except Exception:
        # Cierre normal de ROS durante shutdown de la aplicación.
        pass


def main():
    rclpy.init()

    # Forzar ruta de plugins de Qt provista por PyQt5.
    from PyQt5.QtCore import QLibraryInfo
    qt_plugins_path = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    if qt_plugins_path:
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = qt_plugins_path

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    signals = RosSignals()
    node    = LaneGuiNode(signals)
    window  = LaneDebugWindow(signals)
    window.show()

    # Hilo ROS2 separado del hilo Qt
    ros_thread = threading.Thread(target=_ros_spin_worker, args=(node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    try:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    except Exception:
        # Puede ocurrir si ya se pidió shutdown desde otro contexto.
        pass

    ros_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
