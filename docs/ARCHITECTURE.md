# ARCHITECTURE.md
> Visión general del sistema, flujo de datos, decisiones de diseño de alto nivel.

---

## Diagrama de nodos ROS2

```
┌─────────────────────────────────────────────────────────────────┐
│                        OFFLINE SOURCE                           │
│                                                                 │
│   rosbag2  ──►  extract_frames.py  ──►  /frames/*.jpg           │
└─────────────────────────────────┬───────────────────────────────┘
                                  │ lee archivos directamente
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│              lane_pipeline_node  (C++17, ROS2 Humble)           │
│                                                                 │
│  Timer @ publish_rate_hz                                        │
│  │                                                              │
│  ├──► compute_hls_mask()       ──► /lane_debug/hls_mask         │
│  ├──► warpPerspective (BEV)    ──► /lane_debug/bev              │
│  ├──► run_sliding_window()     ──► /lane_debug/sliding_window   │
│  ├──► apply_temporal_filter()                                   │
│  ├──► draw_lane_overlay()      ──► /lane_debug/overlay          │
│  └──► publish_lane_errors()    ──► /lane_errors_est             │
│                                    ──► /lane_debug/original     │
└──────────────────────────────┬──────────────────────────────────┘
                               │
              ┌────────────────┴────────────────┐
              │                                 │
              ▼                                 ▼
┌─────────────────────────┐       ┌─────────────────────────────┐
│   lane_gui.py           │       │   MPC Controller Node       │
│   (Python, PyQt5)       │       │   (fuera de este paquete)   │
│                         │       │                             │
│  /lane_debug/* ──► GUI  │       │  /lane_errors_est           │
│  /lane_errors_est ──► HUD│      │   data[0] = e2  [m]         │
└─────────────────────────┘       │   data[1] = e3  [rad]       │
                                  │   data[2] = k   [m⁻¹]       │
                                  └─────────────────────────────┘
```

---

## Decisiones de diseño de alto nivel

### Por qué un solo nodo C++

Todo el pipeline vive en **un solo nodo ROS2** (`LanePipelineNode`), ahora segmentado en
múltiples `.cpp` por etapa dentro de `src/pipeline/stages/`. Se descartó dividirlo en múltiples nodos
(uno por etapa) porque:
- La latencia entre nodos en ROS2 por IPC agrega ~2–5 ms por salto, inaceptable con budget de 40 ms.
- Compartir la imagen BEV entre nodos requeriría copias de memoria innecesarias.
- El pipeline es secuencial sin branching: no hay ganancia en paralelismo por nodo.

### Por qué Python solo para la GUI

La GUI es puramente de visualización; no está en el camino crítico del pipeline.
Python + PyQt5 permite iterar el layout rápido. La comunicación GUI ↔ nodo C++ va
enteramente por topics ROS2, lo que permite apagar la GUI en producción sin ningún
cambio en el nodo de percepción.

### Por qué offline (frames en disco) y no live

El flujo offline (bag → frames → nodo) permite:
1. Reproducibilidad exacta: el mismo frame produce el mismo resultado siempre.
2. Comparación contra ground truth del bag (e2/e3 de referencia).
3. Depuración frame a frame sin timing constraints.

El nodo está diseñado para ser **plug-and-play** con la cámara en vivo: basta reemplazar
el timer + lectura de disco por una suscripción a `/camera/color/image_raw` o
`/telemetry/camera_record/compressed`. La lógica del pipeline no cambia.

### Por qué Bird's Eye View obligatorio

Sin BEV, el ajuste polinomial se hace en coordenadas de imagen (píxeles perspectivos)
donde 1 px no tiene significado métrico consistente. Las consecuencias son:
- Los coeficientes del polinomio no tienen unidades interpretables.
- e2 y e3 no pueden expresarse en metros/radianes directamente.
- La curvatura k dependería de la distancia a la cámara.

Con BEV a 5 mm/px, todas las variables tienen unidades físicas reales.

### Por qué polinomio grado 2 y no spline

| Criterio | Polinomio grado 2 | Spline cúbica |
|----------|-------------------|---------------|
| Parámetros a filtrar | 3 (a, b, c) | N puntos de control |
| Curvatura | Constante (2a) | Variable por segmento |
| Estabilidad ante ruido | Alta (LSQ global) | Media (sensible a outliers locales) |
| Costo computacional | Muy bajo | Bajo-medio |
| Suficiente para BFMC | Sí (radios ≥ 0.8 m) | Sí, pero excesivo |

El filtrado temporal exponencial sobre los 3 coeficientes `[a, b, c]` es la pieza
clave de estabilidad. Ver `PIPELINE.md` sección 5.

---

## Flujo de datos detallado por frame

```
cv::Mat bgr (640×480)
    │
    ▼ GaussianBlur 5×5 + cvtColor BGR→HLS + inRange × 2 + bitwise_or + morphClose
cv::Mat binary_mask (640×480, CV_8UC1)
    │
    ▼ warpPerspective con H_ (homografía precalibrada)
cv::Mat bev_binary (320×240, CV_8UC1)
    │
    ▼ histograma columnas (mitad inferior) → picos L/R → 9 sliding windows
std::vector<cv::Point2f> left_pts, right_pts
    │
    ▼ cv::solve (SVD least squares) → PolyCoeffs {a, b, c}
LaneState raw_state {left, right, center}
    │
    ▼ filtro exponencial α=0.3 sobre [a,b,c] con validación de salto
LaneState state_filtered_
    │
    ├──► draw_lane_overlay() → perspectiveTransform(H_inv_) → polylines sobre bgr
    └──► publish_lane_errors() → e2, e3, k → /lane_errors_est
```

---

## Árbol de código del pipeline (refactor modular sin headers)

```
src/
├── main.cpp
└── pipeline/
        ├── lane_pipeline_node.cpp
        └── stages/
                ├── 01_params_and_init.cpp
                ├── 02_frame_source.cpp
                ├── 03_preprocess_hls.cpp
                ├── 04_bev_transform.cpp
                ├── 05_sliding_window.cpp
                ├── 06_temporal_filter.cpp
                ├── 07_errors_mpc.cpp
                └── 08_debug_publish.cpp
```

Notas:
- `src/main.cpp` contiene únicamente `main()`.
- `src/pipeline/lane_pipeline_node.cpp` declara `LanePipelineNode`, tipos y estado compartido.
- Las definiciones por etapa se incluyen al final de `lane_pipeline_node.cpp` con:
    `#include "stages/xx_*.cpp"`.
