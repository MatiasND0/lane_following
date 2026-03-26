# Lane Detection Pipeline — BFMC

Sistema de detección de carril para el Bosch Future Mobility Challenge.
Pipeline completo en C++ (ROS2 Humble) con GUI de debug en Python/PyQt5.

---

## Estructura del paquete

```
lane_detection/
├── src/
│   └── lane_pipeline_node.cpp    ← Nodo C++: todo el pipeline
├── CMakeLists.txt
├── package.xml
├── extract_frames.py             ← Script de extracción de frames del bag
└── lane_gui.py                   ← GUI de debug en Python
```

---

## 1. Extracción de frames del rosbag2

```bash
source /opt/ros/humble/setup.bash

python3 extract_frames.py \
    --bag  /path/to/your/bag_directory \
    --output /path/to/frames \
    --every 1
```

Opciones:
- `--every N`: guardar 1 de cada N frames (útil para bags largos; usar 2 o 3 para reducir volumen)
- `--topic`: topic a extraer (default: `/telemetry/camera_record/compressed`)

Los frames se guardan como `.jpg` con nombre `frame_<timestamp_ns>.jpg`
(orden cronológico garantizado por nombre de archivo).

---

## 2. Compilar el paquete ROS2

```bash
# Crear workspace si no existe
mkdir -p ~/bfmc_ws/src
cd ~/bfmc_ws/src

# Copiar o clonar el paquete
cp -r /path/to/lane_detection .

# Compilar
cd ~/bfmc_ws
colcon build --packages-select lane_detection --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 3. Ejecutar el nodo C++

```bash
source ~/bfmc_ws/install/setup.bash

ros2 run lane_detection lane_pipeline_node \
    --ros-args \
    -p frames_dir:=/path/to/frames \
    -p publish_rate_hz:=25.0 \
    -p bev_scale_mpp:=0.005 \
    -p alpha_filter:=0.3 \
    -p lane_width_m:=0.35 \
    -p camera_offset_m:=0.23
```

### Parámetros

| Parámetro         | Default | Descripción                                          |
|-------------------|---------|------------------------------------------------------|
| `frames_dir`      | —       | **Requerido.** Ruta a la carpeta con los .jpg        |
| `publish_rate_hz` | 25.0    | Frecuencia de procesamiento en Hz                    |
| `bev_scale_mpp`   | 0.005   | Metros por píxel en la imagen BEV (5 mm/px)          |
| `alpha_filter`    | 0.3     | Coeficiente del filtro exponencial temporal          |
| `lane_width_m`    | 0.35    | Ancho nominal del carril en metros (BFMC: 35 cm)     |
| `camera_offset_m` | 0.23    | Distancia cámara → eje trasero en metros             |

---

## 4. Ejecutar la GUI

En una terminal separada (con el nodo C++ ya corriendo):

```bash
source /opt/ros/humble/setup.bash
source ~/bfmc_ws/install/setup.bash

python3 lane_gui.py
```

La GUI muestra 5 paneles:
- **Original**: frame crudo de la cámara
- **HLS Mask**: máscara binaria resultante del umbralizado HLS
- **Bird's Eye View**: imagen binarizada transformada a vista aérea
- **Sliding Window**: ventanas de detección + polinomios ajustados
- **Lane Overlay**: polinomio central proyectado sobre la imagen original

Panel inferior: valores en tiempo real de `e2`, `e3`, `k` para el MPC.

### Controles de reproducción (frame a frame)

La GUI publica comandos en `/lane_control/cmd` para controlar el nodo offline:
- `Pause/Resume` (barra superior, o tecla `Space`)
- `Step` (avanza exactamente 1 frame, o tecla `→`)
- `Reset` (reinicia al primer frame, o tecla `R`)

Si usás `./run_all.sh`, por defecto el pipeline arranca **pausado** para depuración
frame-a-frame. Para arrancar reproduciendo en continuo:

```bash
./run_all.sh --start-running
```

---

## 5. Verificar los topics de debug

```bash
# Ver todos los topics activos
ros2 topic list

# Verificar que llegan imágenes
ros2 topic hz /lane_debug/original
ros2 topic hz /lane_debug/sliding_window

# Ver los errores del MPC
ros2 topic echo /lane_errors_est

# Visualizar una etapa con rqt_image_view
ros2 run rqt_image_view rqt_image_view /lane_debug/bev
```

---

## 6. Calibración de la homografía BEV

Los puntos de la homografía en `lane_pipeline_node.cpp` son valores iniciales
estimados para la D435 montada a 0.23 m del eje trasero.

**Para calibrar correctamente:**

1. Colocar el vehículo sobre una superficie plana con un patrón de referencia
   (ej: cinta métrica o cuadrícula en el suelo).
2. Identificar 4 puntos en la imagen perspectiva que forman un rectángulo
   conocido en el suelo.
3. Medir sus posiciones en píxeles en la imagen original.
4. Calcular los puntos destino en BEV usando la escala `bev_scale_mpp`.
5. Actualizar los arrays `src_pts` y `dst_pts` en `init_bev_homography()`.

Herramienta útil para seleccionar puntos interactivamente:
```bash
python3 -c "
import cv2, sys
img = cv2.imread(sys.argv[1])
pts = []
def cb(e,x,y,f,p):
    if e == cv2.EVENT_LBUTTONDOWN:
        pts.append((x,y)); print(f'Punto: ({x}, {y})')
cv2.namedWindow('cal'); cv2.setMouseCallback('cal', cb)
while True:
    cv2.imshow('cal', img)
    if cv2.waitKey(1) == 27: break
" /path/to/frames/frame_*.jpg
```

---

## 7. Troubleshooting

**Los topics de debug no aparecen:**
- Verificar que `frames_dir` apunta a una carpeta con archivos `.jpg`.
- Revisar logs: `ros2 run lane_detection lane_pipeline_node --ros-args -p frames_dir:=/path/to/frames`

**La GUI no recibe imágenes:**
- Confirmar que el nodo C++ está corriendo: `ros2 node list`
- Confirmar que los topics existen: `ros2 topic list | grep lane_debug`

**Las líneas del carril no se detectan:**
- Ajustar los rangos HLS en `compute_hls_mask()` para las condiciones de luz específicas.
- Verificar la calibración de la homografía (ver sección 6).

**FPS bajo en la GUI:**
- Reducir `publish_rate_hz` a 15 Hz.
- La GUI escala las imágenes en cada repaint; en hardware limitado considerar ventanas más pequeñas.

---

## Sistema de referencia

```
         y (adelante)
         ^
         |
         |  ← cámara (+0.23 m en y)
         |
    -----O-----> x (izquierda)
    (eje trasero = origen)
```

- **e2 > 0**: vehículo desplazado a la derecha del centro del carril
- **e3 > 0**: vehículo apuntando a la derecha respecto a la tangente del carril
- **k > 0**: curva hacia la izquierda (convención ROS/RHR)