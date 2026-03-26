# CALIBRATION.md
> Procedimientos de calibración para la homografía BEV y los rangos HLS.
> Ejecutar cada vez que cambie la posición de la cámara o las condiciones de iluminación.

---

## 1. Calibración de la Homografía BEV

### ¿Cuándo recalibrar?
- Al cambiar la altura o ángulo de montaje de la cámara.
- Al cambiar el vehículo o la posición de la cámara sobre el vehículo.
- Si el overlay de la línea central está desplazado respecto al carril real.
- Si el ancho del carril detectado en BEV es diferente a 35 cm.

### Qué se necesita
- Un frame del bag donde el vehículo esté sobre una recta, centrado en el carril.
- Una cuadrícula o cinta métrica en el suelo (opcional pero recomendado).
- El script de selección de puntos (ver abajo).

---

### Procedimiento paso a paso

#### Paso 1: Extraer un frame de referencia

```bash
python3 extract_frames.py \
    --bag /path/to/bag \
    --output /tmp/calib_frames \
    --every 30
# Esto da ~1 frame por segundo. Elegir uno donde el vehículo esté recto.
```

#### Paso 2: Seleccionar puntos del trapecio en la imagen perspectiva

El trapecio debe corresponder a un rectángulo en el suelo frente al vehículo.
Idealmente usar marcas físicas conocidas (ej: bordes del carril a distancias medidas).

Abrir el frame con el selector interactivo:

```bash
python3 - <<'EOF'
import cv2, sys

img = cv2.imread("/tmp/calib_frames/<nombre_frame>.jpg")
pts = []
clone = img.copy()

def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pts.append((x, y))
        cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(clone, f"{len(pts)}: ({x},{y})", (x+8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        print(f"Punto {len(pts)}: ({x}, {y})")
        if len(pts) == 4:
            print("\n--- 4 puntos seleccionados ---")
            print("Copiar en lane_pipeline_node.cpp, init_bev_homography():")
            print(f"src_pts = {{")
            for p in pts:
                print(f"    {{{float(p[0]):.1f}f, {float(p[1]):.1f}f}},")
            print(f"}}")

cv2.namedWindow("calibracion", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("calibracion", on_click)

print("Hacer click en 4 puntos en este orden:")
print("  1. Esquina superior izquierda del trapecio (lejos, izquierda)")
print("  2. Esquina superior derecha del trapecio (lejos, derecha)")
print("  3. Esquina inferior derecha del trapecio (cerca, derecha)")
print("  4. Esquina inferior izquierda del trapecio (cerca, izquierda)")
print("Presionar ESC para salir.\n")

while True:
    cv2.imshow("calibracion", clone)
    if cv2.waitKey(1) == 27:
        break
cv2.destroyAllWindows()
EOF
```

**Criterio para elegir los 4 puntos:**
- Los puntos top (1 y 2) deben estar a la misma distancia del vehículo (~1.0–1.2 m adelante).
- Los puntos bottom (3 y 4) deben estar justo delante de la cámara (~0.1–0.2 m adelante).
- Los 4 puntos deben estar sobre el asfalto, no en las líneas del carril.
- Si hay una cuadrícula de 35×35 cm en el suelo, usar sus esquinas.

#### Paso 3: Definir los puntos destino en BEV

Los puntos destino en el BEV (320×240 px) deben corresponder al mismo rectángulo real.
Usar la escala `bev_scale_mpp = 0.005` (5 mm/px):

```
Si el rectángulo real mide W metros de ancho y H metros de largo:
  W_px = W / 0.005
  H_px = H / 0.005

Ejemplo con W=0.8m, H=1.0m:
  W_px = 160 px
  H_px = 200 px

Centrado en la imagen BEV (320×240):
  dst_pts = {
      {(320 - 160) / 2,       (240 - 200) / 2},   // top-left
      {(320 + 160) / 2,       (240 - 200) / 2},   // top-right
      {(320 + 160) / 2,  240 - (240 - 200) / 2},  // bottom-right
      {(320 - 160) / 2,  240 - (240 - 200) / 2},  // bottom-left
  }
  = {
      {80,  20},
      {240, 20},
      {240, 220},
      {80,  220},
  }
```

#### Paso 4: Actualizar el código

Abrir `src/lane_pipeline_node.cpp`, función `init_bev_homography()`, y reemplazar
los valores de `src_pts` y `dst_pts` con los obtenidos en los pasos anteriores.

#### Paso 5: Verificar

Compilar y correr el nodo. En la GUI, verificar en el panel **Bird's Eye View** que:
- Las líneas del carril son paralelas y verticales (no convergentes).
- El ancho entre líneas en la base de la imagen BEV sea ≈ 70 px (= 35 cm / 5mm).
- Las líneas no se curvan artificialmente en tramos rectos.

**Si las líneas se curvan en rectas:** los puntos src no estaban sobre el plano del suelo
(probable error: se seleccionaron sobre el capó del vehículo o sobre objetos elevados).

---

## 2. Calibración de Rangos HLS

### ¿Cuándo recalibrar?
- Al cambiar de entorno (interior/exterior, con/sin luz natural).
- Si el panel `HLS Mask` en la GUI muestra la máscara vacía o con exceso de ruido.
- Si solo se detectan las líneas de un color (blanco sí, amarillo no, o viceversa).

### Diagnóstico visual

Usar el panel **HLS Mask** en la GUI:
- **Máscara vacía:** umbrales demasiado restrictivos → bajar mínimos.
- **Ruido por toda la imagen:** umbrales muy permisivos → subir mínimos o bajar máximos.
- **Solo franjas horizontales:** la cámara está capturando reflejos del suelo → agregar
  erosión morfológica adicional.

### Procedimiento de ajuste interactivo

```bash
python3 - <<'EOF'
import cv2
import numpy as np

# Cargar un frame representativo
img = cv2.imread("/tmp/calib_frames/<nombre_frame>.jpg")
blurred = cv2.GaussianBlur(img, (5,5), 0)
hls = cv2.cvtColor(blurred, cv2.COLOR_BGR2HLS)

# Valores iniciales (iguales a los del nodo C++)
params = {
    "W_L_min": 180, "W_L_max": 255, "W_S_max": 60,
    "Y_H_min": 15,  "Y_H_max": 35,
    "Y_L_min": 80,  "Y_S_min": 100,
}

def update(_=None):
    wl_min = cv2.getTrackbarPos("W_L_min", "HLS Calibration")
    wl_max = cv2.getTrackbarPos("W_L_max", "HLS Calibration")
    ws_max = cv2.getTrackbarPos("W_S_max", "HLS Calibration")
    yh_min = cv2.getTrackbarPos("Y_H_min", "HLS Calibration")
    yh_max = cv2.getTrackbarPos("Y_H_max", "HLS Calibration")
    yl_min = cv2.getTrackbarPos("Y_L_min", "HLS Calibration")
    ys_min = cv2.getTrackbarPos("Y_S_min", "HLS Calibration")

    white  = cv2.inRange(hls, (0, wl_min, 0), (180, wl_max, ws_max))
    yellow = cv2.inRange(hls, (yh_min, yl_min, ys_min), (yh_max, 220, 255))
    combined = cv2.bitwise_or(white, yellow)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

    overlay = img.copy()
    overlay[combined > 0] = [0, 255, 0]
    display = np.hstack([img, overlay])
    cv2.imshow("HLS Calibration", display)

cv2.namedWindow("HLS Calibration", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HLS Calibration", 1280, 480)
for name, val in params.items():
    max_val = 255 if "H_" not in name else 180
    cv2.createTrackbar(name, "HLS Calibration", val, max_val, update)

update()
print("Ajustar sliders. Presionar 's' para imprimir valores finales. ESC para salir.")
while True:
    k = cv2.waitKey(1)
    if k == 27:
        break
    if k == ord('s'):
        print("\n--- Valores finales (copiar en compute_hls_mask) ---")
        print(f"white:  L in [{cv2.getTrackbarPos('W_L_min','HLS Calibration')}, "
              f"{cv2.getTrackbarPos('W_L_max','HLS Calibration')}], "
              f"S max {cv2.getTrackbarPos('W_S_max','HLS Calibration')}")
        print(f"yellow: H in [{cv2.getTrackbarPos('Y_H_min','HLS Calibration')}, "
              f"{cv2.getTrackbarPos('Y_H_max','HLS Calibration')}], "
              f"L min {cv2.getTrackbarPos('Y_L_min','HLS Calibration')}, "
              f"S min {cv2.getTrackbarPos('Y_S_min','HLS Calibration')}")
cv2.destroyAllWindows()
EOF
```

### Cómo actualizar el código con los nuevos valores

En `src/lane_pipeline_node.cpp`, función `compute_hls_mask()`:

```cpp
// Líneas blancas
cv::inRange(hls,
    cv::Scalar(0,   W_L_min, 0),
    cv::Scalar(180, W_L_max, W_S_max),
    white_mask);

// Líneas amarillas
cv::inRange(hls,
    cv::Scalar(Y_H_min, Y_L_min, Y_S_min),
    cv::Scalar(Y_H_max, 220,     255),
    yellow_mask);
```

Reemplazar los valores numéricos con los obtenidos del script de calibración.

---

## 3. Verificación post-calibración

Después de calibrar homografía y HLS, ejecutar el pipeline completo sobre un bag
conocido y verificar:

| Check | Herramienta | Criterio de éxito |
|-------|------------|-------------------|
| BEV con líneas paralelas | GUI panel BEV | Líneas verticales, ancho ~70 px en base |
| Máscara sin ruido excesivo | GUI panel HLS Mask | Solo píxeles en las líneas del carril |
| Polinomios estables | GUI panel Sliding Window | Sin oscilaciones frame a frame |
| e2 cerca de 0 en tramos centrados | `ros2 topic echo /lane_errors_est` | \|e2\| < 0.05 m en rectas centradas |
| k cerca de 0 en rectas | `ros2 topic echo /lane_errors_est` | \|k\| < 0.1 m⁻¹ en rectas |
