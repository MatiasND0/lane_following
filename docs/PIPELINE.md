# PIPELINE.md
> Descripción exhaustiva de cada etapa del pipeline.
> Para cada etapa: qué hace, por qué, parámetros ajustables, fallos esperados.

---

## Índice de etapas

1. [Lectura de imagen](#1-lectura-de-imagen)
2. [Preprocesamiento HLS](#2-preprocesamiento-hls)
3. [Transformación Bird's Eye View](#3-transformación-birds-eye-view)
4. [Sliding Window + Ajuste Polinomial](#4-sliding-window--ajuste-polinomial)
5. [Filtrado Temporal](#5-filtrado-temporal)
6. [Cálculo de Errores para el MPC](#6-cálculo-de-errores-para-el-mpc)

---

## Mapeo de implementación por etapas (layout modular)

| Etapa | Archivo | Entradas / Salidas | Parámetros relevantes |
|------|---------|---------------------|-----------------------|
| 01. Parámetros e init | `src/pipeline/stages/01_params_and_init.cpp` | Entrada: parámetros ROS2. Salida: timer, publishers, estado inicial. | `frames_dir`, `publish_rate_hz`, `bev_scale_mpp`, `alpha_filter`, `lane_width_m`, `camera_offset_m` |
| 02. Fuente de frames | `src/pipeline/stages/02_frame_source.cpp` | Entrada: `.jpg/.png` desde `frames_dir`. Salida: orquestación del ciclo por frame y publicación `original`. | `frames_dir`, `publish_rate_hz` |
| 03. Preprocesamiento HLS | `src/pipeline/stages/03_preprocess_hls.cpp` | Entrada: `cv::Mat BGR`. Salida: máscara binaria HLS. | Umbrales HLS hardcoded |
| 04. Transformación BEV | `src/pipeline/stages/04_bev_transform.cpp` | Entrada: máscara binaria perspectiva. Salida: BEV + proyección inversa para overlay. | `bev_scale_mpp` + puntos de homografía |
| 05. Sliding window + fit | `src/pipeline/stages/05_sliding_window.cpp` | Entrada: BEV binaria. Salida: `LaneState` (`left/right/center`) con polinomio `x=a·y²+b·y+c`. | `lane_width_m`, `bev_scale_mpp` |
| 06. Filtro temporal | `src/pipeline/stages/06_temporal_filter.cpp` | Entrada: `LaneState` crudo. Salida: `state_filtered_`. | `alpha_filter` |
| 07. Errores MPC | `src/pipeline/stages/07_errors_mpc.cpp` | Entrada: `state_filtered_`. Salida: `/lane_errors_est` = `[e2,e3,k]`. | `camera_offset_m`, `bev_scale_mpp` |
| 08. Debug publish | `src/pipeline/stages/08_debug_publish.cpp` | Entrada: imágenes intermedias. Salida: `/lane_debug/*`. | calidad JPEG fija 85 |

Archivo agregador del nodo:
- `src/pipeline/lane_pipeline_node.cpp` (declara `LanePipelineNode` e incluye `stages/*.cpp` al final)

---

## 1. Lectura de imagen

**Función:** `process_next_frame()` en `lane_pipeline_node.cpp`

**Entrada:** Archivo `.jpg` desde `frames_dir`, leído en orden alfanumérico.
Los nombres de archivo son `frame_<timestamp_ns>.jpg`, por lo que el orden
alfanumérico es igual al orden cronológico.

**Salida:** `cv::Mat bgr` de dimensiones **640×480×3** (BGR, CV_8UC3).

**Parámetros:**
- `frames_dir` (string): ruta a la carpeta con los frames extraídos.
- `publish_rate_hz` (double): frecuencia del timer. No controla la velocidad de
  reproducción respecto al tiempo real del bag; solo determina cuántos frames
  por segundo se procesan.

**Fallo esperado:** Si la carpeta está vacía o el frame está corrupto,
`cv::imread` devuelve un `Mat` vacío. El nodo loguea un warning y salta el frame.

---

## 2. Preprocesamiento HLS

**Función:** `compute_hls_mask(const cv::Mat& bgr)` → `cv::Mat` (CV_8UC1, binaria)

**Por qué HLS y no HSV o BGR:**
El canal L (Lightness) en HLS separa la luminosidad de la crominancia. Esto permite
umbralizar líneas blancas por L alto independientemente del color, y líneas amarillas
por H+S sin que la sombra las apague. HSV mezcla valor con saturación de forma que
sombras fuertes destruyen la detección de blanco.

**Secuencia interna:**

```
bgr
 │
 ▼  cv::GaussianBlur(5×5, σ=0)
blurred
 │
 ▼  cv::cvtColor(BGR → HLS)
hls  [H: 0-180, L: 0-255, S: 0-255]
 │
 ├──► inRange(H:0-180, L:180-255, S:0-60)    →  white_mask
 │    (líneas blancas: L alto, S bajo)
 │
 └──► inRange(H:15-35, L:80-220, S:100-255)  →  yellow_mask
      (líneas amarillas: H amarillo, S alto)
           │
           ▼  cv::bitwise_or
      combined_mask
           │
           ▼  cv::morphologyEx(MORPH_CLOSE, kernel 3×3)
      final_mask  ← salida
```

**Parámetros ajustables (hardcoded, modificar en `compute_hls_mask`):**

| Variable | Valor actual | Efecto si se sube | Efecto si se baja |
|----------|-------------|-------------------|-------------------|
| L mínimo blanco | 180 | Menos falsos positivos | Detecta más blanco pero más ruido |
| S máximo blanco | 60 | Acepta blancos más saturados | Solo acepta blanco puro |
| H mínimo amarillo | 15 | Ignora amarillos verdosos | Captura más naranja |
| H máximo amarillo | 35 | Captura más naranja | Solo amarillo puro |
| S mínimo amarillo | 100 | Ignora amarillos pálidos | Más susceptible a luz amarilla ambiental |

**Kernel morfológico:** `MORPH_CLOSE` con kernel 3×3. Cierra huecos pequeños en líneas
discontinuas sin ensancharlas demasiado. No usar kernel mayor a 5×5: dilata las líneas
y corrompe el ajuste polinomial.

**Fallo esperado:**
- Sobreexposición: L de toda la imagen sube → falsos positivos en blanco.
  Mitigación: ajustar L mínimo dinámicamente como `mean(L_roi) + 1.5*std(L_roi)`.
- Subexposición nocturna: ninguna máscara activa. El filtro temporal retiene el estado previo.

---

## 3. Transformación Bird's Eye View

**Función:** `init_bev_homography()` (init), `cv::warpPerspective` (por frame)

**Concepto:**
Se mapea un trapecio en la imagen perspectiva (que corresponde a un rectángulo real
en el suelo) a un rectángulo en la imagen BEV. La transformación es una homografía
3×3 calculada con `cv::getPerspectiveTransform`.

**Dimensiones de salida BEV:**
- **Resolución:** 320×240 px
- **Escala:** 5 mm/px (`bev_scale_mpp = 0.005`)
- **Campo cubierto:** 1.6 m lateral × 1.2 m longitudinal

```
Imagen BEV (320×240 px @ 5mm/px)

y=0 (arriba)   ←  lejos del vehículo (~1.2 m adelante de la cámara)
│
│   ┌──────────────────────────────┐
│   │                              │ ← 1.6 m de ancho
│   │    zona útil de detección    │
│   │                              │
│   └──────────────────────────────┘
▼
y=239 (abajo)  ←  cerca del vehículo (posición de la cámara)

x=0 (izq)                    x=319 (der)
```

**Puntos de la homografía (valores iniciales — requieren calibración):**

```cpp
// Imagen perspectiva fuente (640×480)
src_pts = {
    {640 * 0.40, 480 * 0.55},   // top-left  del trapecio
    {640 * 0.60, 480 * 0.55},   // top-right
    {640 * 0.95, 480 * 0.95},   // bottom-right
    {640 * 0.05, 480 * 0.95},   // bottom-left
};

// Imagen BEV destino (320×240)
dst_pts = {
    {320 * 0.10,   0},          // top-left
    {320 * 0.90,   0},          // top-right
    {320 * 0.90, 240},          // bottom-right
    {320 * 0.10, 240},          // bottom-left
};
```

**Para calibrar estos valores:** ver `CALIBRATION.md`.

**`H_inv_`:** Se calcula la homografía inversa simultáneamente. Se usa en
`draw_lane_overlay()` para proyectar los puntos del polinomio BEV de vuelta
a la imagen perspectiva original.

**Supuesto crítico:** El suelo es plano. En rampas o desniveles la homografía
introduce error. En BFMC la pista es plana; si hay rampas, la homografía debe
recalibrarse por segmento.

---

## 4. Sliding Window + Ajuste Polinomial

**Función:** `run_sliding_window(const cv::Mat& bev_binary, cv::Mat& viz)` → `LaneState`

### 4.1 Histograma de base

Se suma la mitad inferior del BEV binarizado por columnas (`cv::reduce REDUCE_SUM`).
Los dos picos del histograma (mitad izquierda y mitad derecha) dan la posición
inicial x de cada línea.

**Por qué solo la mitad inferior:** La mitad inferior está más cerca del vehículo,
donde la detección es más confiable y la perspectiva residual es menor.

### 4.2 Sliding Window

```
Parámetros:
  n_windows   = 9        (ventanas verticales)
  win_half_w  = 30 px    (semiancho = ±15 cm en el suelo)
  min_pixels  = 40       (mínimo de píxeles para recentrar la ventana)
```

Para cada ventana (de abajo hacia arriba):
1. Definir ROI: `[cx ± win_half_w] × [y_low, y_high]`
2. Extraer píxeles no-nulos con `cv::findNonZero`
3. Si count ≥ min_pixels: recentrar `cx` en el centroide de los píxeles
4. Acumular todos los píxeles de la ventana en `left_pts` / `right_pts`

**Si una línea tiene < 20 píxeles totales al final:** `PolyCoeffs.valid = false`.
La línea se considera no detectada y se infiere desde la otra línea + offset nominal.

### 4.3 Ajuste polinomial grado 2

Modelo: **`x = a·y² + b·y + c`**  donde y es longitudinal, x es lateral (ambos en px BEV).

El ajuste se hace por mínimos cuadrados usando `cv::solve(A, B, result, cv::DECOMP_SVD)`.

**Normalización de y:** Se escala `y_norm = y / bev_h` antes de construir la matriz A
para evitar mal condicionamiento numérico (y² puede ser hasta 57600 en px, vs 1.0 normalizado).
Los coeficientes se desescalan después:
```
a_real = a_norm / bev_h²
b_real = b_norm / bev_h
c_real = c_norm              (término independiente no cambia)
```

### 4.4 Centro del carril

Si ambas líneas son válidas:
```
a_c = (a_L + a_R) / 2
b_c = (b_L + b_R) / 2
c_c = (c_L + c_R) / 2
```

Si solo una línea es válida, se desplaza ±`lane_width_m / bev_scale_mpp` píxeles.

**Validación del ancho del carril:**
```
ancho_px = c_R - c_L   (evaluado en y = bev_h, base de la imagen)
ancho_m  = ancho_px * bev_scale_mpp

Rango válido: [0.25 m, 0.45 m]
Si ancho_m está fuera de este rango → detección sospechosa → reducir alpha en el filtro
```

---

## 5. Filtrado Temporal

**Función:** `apply_temporal_filter(const LaneState& raw)` — modifica `state_filtered_` in-place

**Filtro exponencial de primer orden sobre coeficientes:**
```
coef_filtered = α · coef_raw + (1 - α) · coef_filtered_prev
```

**Parámetro `alpha_filter` (default 0.3):**
- α = 0.1 → muy suave, respuesta lenta, lag visible en curvas
- α = 0.3 → buen balance para 25 FPS en BFMC
- α = 0.5 → más reactivo, puede oscilar con detecciones ruidosas
- α = 1.0 → sin filtro (equivalente a usar raw directamente)

**Inicialización:** En el primer frame válido, `state_filtered_` se iguala directamente
a `raw` (sin promedio). La flag `valid` lo gestiona.

**Validación antes de actualizar:**
```
Si |a_raw - a_filtered| / (|a_filtered| + ε) > 1.5
   → Rechazar: curvatura saltó más del 150%, probablemente detección espuria
   → No actualizar state_filtered_ para este frame
```

**Por qué filtrar coeficientes y no variables derivadas (e2, e3, k):**
Filtrar e2/e3 directamente introduce retardo de fase que el MPC interpreta como
error real y sobrecompensa. Filtrar los coeficientes mantiene la geometría coherente
y los errores derivados son siempre consistentes entre sí.

---

## 6. Cálculo de Errores para el MPC

**Función:** `publish_lane_errors(const LaneState& state)`

Ver `COORDINATES.md` para la derivación matemática completa.

**Resumen de fórmulas:**

```cpp
// Punto de evaluación: eje trasero del vehículo
// La cámara está camera_offset_m adelante del eje trasero
// En BEV, y=bev_h corresponde a la posición de la cámara
// → y_eval desplaza hacia atrás (valores negativos en px BEV)

double y_eval_px = -camera_offset_m / bev_scale_mpp;

// e2: error lateral [m]
double x_center_px = a·y² + b·y + c   (evaluado en y_eval_px)
double e2 = -(x_center_px - bev_w/2) * bev_scale_mpp

// e3: error angular [rad]
double dxdy = 2·a·y_eval_px + b
double e3 = atan(dxdy)

// k: curvatura [m⁻¹]
double k_px = 2·a / (1 + dxdy²)^(3/2)
double k = k_px / bev_scale_mpp
```

**Publicación:**
```
/lane_errors_est  →  Float32MultiArray
data[0] = e2  [m]
data[1] = e3  [rad]
data[2] = k   [m⁻¹]
```
