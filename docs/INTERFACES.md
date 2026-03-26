# INTERFACES.md
> Contrato de interfaces ROS2 del paquete lane_detection.
> Un agente que modifique este paquete NO debe cambiar los nombres de topics
> ni el formato de `/lane_errors_est` sin actualizar este documento.

---

## Nodo: lane_pipeline_node

**Ejecutable:** `lane_pipeline_node`
**Package:** `lane_detection`
**Lenguaje:** C++17
**Tipo de nodo:** publicador puro (no suscribe a ningún topic en la implementación offline)

---

## Topics publicados

### `/lane_debug/original`
| Campo | Valor |
|-------|-------|
| Tipo | `sensor_msgs/msg/CompressedImage` |
| Encoding | JPEG, calidad 85 |
| Resolución | 640×480 px, 3 canales BGR |
| Frame ID | `"original"` |
| Frecuencia | igual a `publish_rate_hz` |
| Propósito | Frame crudo de entrada, sin procesamiento |

---

### `/lane_debug/hls_mask`
| Campo | Valor |
|-------|-------|
| Tipo | `sensor_msgs/msg/CompressedImage` |
| Encoding | JPEG, calidad 85 |
| Resolución | 640×480 px, 3 canales (máscara binaria convertida a BGR para visualización) |
| Frame ID | `"hls_mask"` |
| Frecuencia | igual a `publish_rate_hz` |
| Propósito | Resultado del umbralizado HLS: blanco = píxel activo |
| Nota | Internamente es CV_8UC1; se convierte a BGR antes de publicar |

---

### `/lane_debug/bev`
| Campo | Valor |
|-------|-------|
| Tipo | `sensor_msgs/msg/CompressedImage` |
| Encoding | JPEG, calidad 85 |
| Resolución | 320×240 px, 3 canales (BEV binario convertido a BGR) |
| Frame ID | `"bev"` |
| Frecuencia | igual a `publish_rate_hz` |
| Propósito | Imagen binaria después de la homografía BEV |
| Escala | 5 mm/px (configurable con `bev_scale_mpp`) |
| Orientación | y=0 (arriba) = lejos del vehículo; y=239 (abajo) = cerca (cámara) |

---

### `/lane_debug/sliding_window`
| Campo | Valor |
|-------|-------|
| Tipo | `sensor_msgs/msg/CompressedImage` |
| Encoding | JPEG, calidad 85 |
| Resolución | 320×240 px, 3 canales BGR |
| Frame ID | `"sliding_window"` |
| Frecuencia | igual a `publish_rate_hz` |
| Propósito | BEV con ventanas dibujadas + polinomios ajustados superpuestos |
| Colores | Izquierda: amarillo (255,200,0) · Derecha: cyan (0,200,255) · Centro: verde (0,255,0) |

---

### `/lane_debug/overlay`
| Campo | Valor |
|-------|-------|
| Tipo | `sensor_msgs/msg/CompressedImage` |
| Encoding | JPEG, calidad 85 |
| Resolución | 640×480 px, 3 canales BGR |
| Frame ID | `"overlay"` |
| Frecuencia | igual a `publish_rate_hz` |
| Propósito | Imagen original con el polinomio central proyectado de BEV a perspectiva |
| Color overlay | Verde (0,255,0), grosor 3 px |

---

### `/lane_errors_est`  ⚠️ Topic de salida crítico — no renombrar

| Campo | Valor |
|-------|-------|
| Tipo | `std_msgs/msg/Float32MultiArray` |
| Tamaño de `data` | 3 elementos, siempre |
| `data[0]` | **e2** — error lateral [m] |
| `data[1]` | **e3** — error angular [rad] |
| `data[2]` | **k**  — curvatura de referencia [m⁻¹] |
| Frecuencia | igual a `publish_rate_hz` |
| Publicado solo si | `state_filtered_.center.valid == true` |

**Convención de signos:**
```
e2 > 0  →  vehículo está a la DERECHA del centro del carril
e2 < 0  →  vehículo está a la IZQUIERDA del centro del carril

e3 > 0  →  vehículo apunta a la DERECHA respecto a la tangente del carril
e3 < 0  →  vehículo apunta a la IZQUIERDA

k  > 0  →  carril curva hacia la IZQUIERDA
k  < 0  →  carril curva hacia la DERECHA
k  = 0  →  tramo recto
```

---

## Parámetros ROS2 del nodo

Todos los parámetros se declaran en el constructor con `declare_parameter` y se leen
con `get_parameter`. Se pueden setear en runtime:

```bash
ros2 run lane_detection lane_pipeline_node --ros-args \
    -p frames_dir:=/path/to/frames \
    -p publish_rate_hz:=25.0 \
    -p bev_scale_mpp:=0.005 \
    -p alpha_filter:=0.3 \
    -p lane_width_m:=0.35 \
    -p camera_offset_m:=0.23
```

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `frames_dir` | string | `""` | Ruta a carpeta con .jpg. **Requerido.** Nodo hace shutdown si está vacío. |
| `publish_rate_hz` | double | 25.0 | Frecuencia del timer de procesamiento. |
| `bev_scale_mpp` | double | 0.005 | Metros por píxel en la imagen BEV. Afecta todas las conversiones métricas. |
| `alpha_filter` | double | 0.3 | Coeficiente del filtro exponencial temporal. Rango útil: [0.1, 0.5]. |
| `lane_width_m` | double | 0.35 | Ancho nominal del carril BFMC en metros. Usado para inferir línea faltante. |
| `camera_offset_m` | double | 0.23 | Distancia cámara → eje trasero en metros. Afecta el cálculo de e2, e3, k. |

---

## QoS

Todos los publishers usan `rclpy.QoS(depth=10)` (equivalente a `rclcpp::QoS(10)`),
que corresponde a `RELIABLE` + `VOLATILE`. No se usa `BEST_EFFORT` porque la GUI
necesita recibir todos los frames de debug sin pérdida.

---

## Nodo: lane_gui_node (Python)

**Archivo:** `lane_gui.py`
**Lenguaje:** Python 3 + rclpy + PyQt5

### Topics suscritos

Todos con QoS depth=10:

| Topic | Tipo | Callback |
|-------|------|----------|
| `/lane_debug/original` | `CompressedImage` | Actualiza panel "Original" |
| `/lane_debug/hls_mask` | `CompressedImage` | Actualiza panel "HLS Mask" |
| `/lane_debug/bev` | `CompressedImage` | Actualiza panel "Bird's Eye View" |
| `/lane_debug/sliding_window` | `CompressedImage` | Actualiza panel "Sliding Window" |
| `/lane_debug/overlay` | `CompressedImage` | Actualiza panel "Lane Overlay" |
| `/lane_errors_est` | `Float32MultiArray` | Actualiza HUD de métricas |

### Arquitectura de threading en la GUI

```
Hilo principal (Qt):
  QApplication.exec_()
  Todos los widgets se crean y actualizan en este hilo.

Hilo secundario (ROS2):
  threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
  Los callbacks ROS2 corren en este hilo.

Comunicación entre hilos:
  pyqtSignal — thread-safe por diseño de Qt.
  RosSignals.image_received(str, np.ndarray)
  RosSignals.errors_received(float, float, float)
```

**Regla crítica:** Nunca tocar widgets Qt desde el hilo ROS2. Siempre usar signals.
