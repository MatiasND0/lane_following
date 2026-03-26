# AGENT_CONTEXT.md
> Cargá este archivo primero. Es el índice de todo lo que necesitás saber.
> Si necesitás más detalle sobre algún punto, el archivo correspondiente está indicado.

---

## ¿Qué es este sistema?

Pipeline de detección de carril para el **Bosch Future Mobility Challenge (BFMC)**.
Vehículo a escala con cámara RealSense D435. Sin LiDAR, sin mapas, solo visión clásica (OpenCV).
La salida alimenta directamente un **controlador MPC** que consume `e2`, `e3`, `k`.

---

## Stack tecnológico

| Capa | Tecnología |
|------|------------|
| OS | Ubuntu 22.04 |
| Middleware | ROS2 Humble |
| Lenguaje principal | C++17 |
| GUI de debug | Python 3 + PyQt5 + rclpy |
| Visión | OpenCV 4.x |
| Hardware de cómputo | Jetson Nano (producción) / PC Ubuntu (desarrollo) |
| Cámara | Intel RealSense D435 |

---

## Archivos del paquete

```
lane_detection/
├── src/
│   └── lane_pipeline_node.cpp   ← TODO el pipeline en un solo nodo C++
├── CMakeLists.txt
├── package.xml
├── extract_frames.py            ← Extrae .jpg de un rosbag2
├── lane_gui.py                  ← GUI PyQt5 con debug + panel de calibración BEV
└── doc/
    ├── AGENT_CONTEXT.md         ← Este archivo (leer primero)
    ├── ARCHITECTURE.md          ← Visión general, diagrama de nodos
    ├── PIPELINE.md              ← Cada etapa del pipeline con parámetros
    ├── INTERFACES.md            ← Topics ROS2, tipos, formatos exactos
    ├── COORDINATES.md           ← Sistemas de referencia, fórmulas e2/e3/k
    ├── CALIBRATION.md           ← Cómo calibrar homografía y rangos HLS
    └── BEV_HOMOGRAPHY_GUIDE.md  ← Guía educativa: qué es BEV, homografía, cómo calibrar
```

---

## Topics que existen hoy

| Topic | Tipo | Dirección | Descripción |
|-------|------|-----------|-------------|
| `/lane_debug/original` | `CompressedImage` | pub | Frame crudo |
| `/lane_debug/hls_mask` | `CompressedImage` | pub | Máscara binaria HLS |
| `/lane_debug/bev` | `CompressedImage` | pub | BEV binarizado |
| `/lane_debug/sliding_window` | `CompressedImage` | pub | BEV con ventanas y polys |
| `/lane_debug/overlay` | `CompressedImage` | pub | Overlay sobre imagen original |
| `/lane_errors_est` | `Float32MultiArray` | pub | `[e2, e3, k]` para el MPC |

> Detalle de formatos y unidades → `INTERFACES.md`

---

## Parámetros ROS2 del nodo C++

| Parámetro | Default | Unidad |
|-----------|---------|--------|
| `frames_dir` | — | path |
| `publish_rate_hz` | 25.0 | Hz |
| `bev_scale_mpp` | 0.005 | m/px |
| `alpha_filter` | 0.3 | — |
| `lane_width_m` | 0.35 | m |
| `camera_offset_m` | 0.23 | m |

---

## Variables de salida para el MPC

```
e2  [m]     Error lateral: distancia del vehículo al centro del carril
e3  [rad]   Error angular: ángulo entre heading del vehículo y tangente del carril
k   [m⁻¹]  Curvatura de la trayectoria de referencia
```

> Fórmulas exactas y sistema de referencia → `COORDINATES.md`

---

## Lo que NO hace este sistema (limitaciones conocidas)

- No usa deep learning ni redes neuronales
- No fusiona sensores (solo cámara)
- No hace relocalización global ni mapas
- La homografía BEV asume terreno plano (sin desniveles)
- El polinomio de grado 2 puede ser insuficiente en curvas > 90°

---

## Estado de implementación

| Componente | Estado |
|------------|--------|
| Extracción de frames del bag | ✅ Completo |
| Preprocesamiento HLS | ✅ Completo |
| Transformación BEV | ✅ Completo (requiere calibración) |
| Sliding window + polinomio grado 2 | ✅ Completo |
| Filtrado temporal exponencial | ✅ Completo |
| Cálculo e2/e3/k | ✅ Completo |
| GUI de debug | ✅ Completo |
| Calibración interactiva de homografía | ✅ Completo (panel en GUI + parámetros dinámicos) |
| Validación contra ground truth del bag | ⬜ Pendiente |

---

## Reglas para el agente

1. **No usar deep learning** bajo ningún concepto en este módulo.
2. **No cambiar los nombres de los topics de salida** (`/lane_errors_est`, `/lane_debug/*`).
3. **El sistema de referencia es fijo**: origen en eje trasero, Y adelante, X izquierda. No cambiarlo.
4. **El polinomio siempre es `x = a·y² + b·y + c`** en coordenadas BEV métricas.
5. Cualquier modificación al cálculo de e2/e3/k debe estar justificada en `COORDINATES.md`.
6. El código C++ debe compilar con `-std=c++17` sin warnings bajo `-Wall -Wextra`.
