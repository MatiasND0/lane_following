# Guía de BEV y Homografía para Detección de Carriles

> Documento orientado a personas sin experiencia previa en visión por computadora.
> Explica qué es la BEV, qué es una homografía, y cómo ajustar los parámetros
> desde el panel de calibración de la GUI.

---

## 1. ¿Qué es la BEV (Bird's Eye View)?

La **BEV** o **vista de pájaro** es una imagen transformada que muestra el suelo
**como si lo estuvieras viendo desde arriba**, directamente hacia abajo.

```
  Perspectiva normal (cámara)          BEV (vista de pájaro)
  ┌────────────────────────┐           ┌────────────────────────┐
  │       cielo            │           │                        │
  │   ╲   carril   ╱      │    →      │  │   carril    │       │
  │    ╲          ╱       │    →      │  │              │       │
  │     ╲________╱        │    →      │  │              │       │
  │      (cerca)          │           │  │              │       │
  └────────────────────────┘           └────────────────────────┘
  Las líneas convergen               Las líneas son paralelas
  hacia un punto de fuga              (como en un plano real)
```

### ¿Por qué se usa?

En la imagen de la cámara, las líneas del carril **convergen hacia un punto de fuga**
a medida que se alejan. Esto hace difícil medir distancias reales.

En la BEV, las líneas son **paralelas y rectas** (en tramos rectos), lo que permite:

- **Medir** el ancho del carril en centímetros reales
- **Calcular** la posición lateral del vehículo (e2)
- **Estimar** el ángulo del vehículo respecto al carril (e3)
- **Ajustar** un polinomio para curvas sin distorsión perspectiva

### ¿Cuándo falla?

La BEV asume que el suelo es **perfectamente plano**. Si hay rampas, baches o
desniveles, la imagen se distorsiona. En la pista BFMC esto no es un problema
porque la superficie es plana.

---

## 2. ¿Qué es la Homografía?

La **homografía** es la transformación matemática que convierte la imagen de la
cámara (perspectiva) a la vista de pájaro (BEV). Es una **matriz de 3×3** que
mapea cada punto de una imagen a un punto en la otra.

### La idea detrás: el trapecio y el rectángulo

Imagina que en el suelo frente al vehículo hay un rectángulo (por ejemplo,
delimitado por las líneas del carril). En la imagen de la cámara, ese rectángulo
se ve como un **trapecio** (más angosto arriba, más ancho abajo):

```
    Imagen de la cámara                  Imagen BEV

    P1 ─────────── P2                P1 ─────────── P2
     ╲             ╱                 │               │
      ╲           ╱                  │               │
       ╲         ╱                   │               │
        ╲       ╱                    │               │
    P4 ──────── P3                   P4 ─────────── P3

    (trapecio)                       (rectángulo)
```

La homografía se calcula eligiendo **4 puntos en el trapecio** (puntos SRC,
en la imagen de la cámara) y **4 puntos en el rectángulo** (puntos DST,
en la imagen BEV). OpenCV usa estos 8 puntos para calcular la matriz 3×3 que
transforma cualquier punto de una imagen a la otra.

### Los 4 puntos y su orden

```
  Punto 1 (P1): Esquina superior izquierda   → lejos, lado izquierdo
  Punto 2 (P2): Esquina superior derecha     → lejos, lado derecho
  Punto 3 (P3): Esquina inferior derecha     → cerca, lado derecho
  Punto 4 (P4): Esquina inferior izquierda   → cerca, lado izquierdo

  "Superior" = lejos del vehículo (arriba en la imagen)
  "Inferior" = cerca del vehículo (abajo en la imagen)
```

> [!IMPORTANT]
> El orden de los puntos es crucial. Si se invierten o mezclan,
> la BEV saldrá volteada, estirada o completamente distorsionada.

---

## 3. ¿Cómo elegir buenos puntos SRC y DST?

### Puntos SRC (en la imagen perspectiva 640×480)

Los puntos SRC definen el **trapecio** en la imagen de la cámara:

- **P1 y P2** (arriba): Deben estar a la misma altura (mismo Y) en la zona
  lejana visible del carril. Típicamente entre `y = 250` y `y = 300`.
  No demasiado arriba o se incluirá cielo/horizonte.

- **P3 y P4** (abajo): Deben estar cerca de la base de la imagen
  (zona próxima al vehículo). Típicamente entre `y = 430` y `y = 470`.
  No en el borde absoluto o se incluirá el capó del auto.

- **Lateralmente**: Los puntos de arriba (P1, P2) están más juntos
  (por la convergencia perspectiva). Los de abajo (P3, P4) están más separados.

### Puntos DST (en la BEV 320×240)

Los puntos DST definen **dónde caen los puntos SRC en la BEV**:

- Deben formar un **rectángulo** (lados paralelos).
- El ancho del rectángulo determina cuánta carretera lateral se ve.
- La altura del rectángulo determina cuánta profundidad se cubre.

Valores típicos recomendados:

| Punto | x   | y   | Descripción |
|-------|-----|-----|-------------|
| P1    | 32  | 0   | Esquina superior izquierda |
| P2    | 288 | 0   | Esquina superior derecha |
| P3    | 288 | 240 | Esquina inferior derecha |
| P4    | 32  | 240 | Esquina inferior izquierda |

Esto deja un margen de ~10% a cada lado (32 px de 320), lo cual evita
artefactos en los bordes.

---

## 4. Usando el Panel de Calibración en la GUI

### Ubicación

El panel de calibración está en la **esquina inferior derecha** de la ventana
principal, al lado de "Sliding Window" y "Lane Overlay".

```
  ┌──────────────┬──────────────┬──────────────┐
  │  Original    │  HLS Mask    │ Bird's Eye   │
  │              │              │    View      │
  ├──────────────┼──────────────┼──────────────┤
  │  Sliding     │  Lane        │ ⚙ CALIBRA-  │
  │  Window      │  Overlay     │   CIÓN BEV  │
  └──────────────┴──────────────┴──────────────┘
```

### Paso a paso

1. **Abrir la GUI** con `python3 lane_gui.py` (el nodo C++ debe estar corriendo).

2. **Verificar la BEV actual**: Mirar el panel "Bird's Eye View".
   Si las líneas del carril no son paralelas o la imagen se ve distorsionada,
   la homografía necesita ajuste.

3. **Pestaña SRC**: Ajustar los 4 puntos del trapecio en la imagen de la cámara.
   - P1 y P2 controlan dónde "mira" la BEV en la zona lejana.
   - P3 y P4 controlan la zona cercana al vehículo.
   - Los valores son en **píxeles** de la imagen original (640×480).

4. **Pestaña DST**: Ajustar los puntos destino en la BEV.
   Normalmente no es necesario cambiar estos valores, salvo que se
   quiera cambiar la región cubierta.

5. **Presionar "▶ Aplicar"**: Los valores se envían al nodo C++ en tiempo real.
   La BEV se actualizará inmediatamente en el siguiente frame.

6. **Verificar el resultado**:
   - Las líneas del carril deben ser **verticales y paralelas** en tramos rectos.
   - El ancho entre líneas debe ser ~**70 px** (= 35 cm / 5 mm por px).
   - Si es correcto, anotar los valores para usarlos como nuevos defaults.

7. **Presionar "↻ Reset"** si se quiere volver a los valores por defecto.

---

## 5. Diagnóstico de Problemas

### Las líneas convergen en la BEV (no son paralelas)

**Causa**: Los puntos SRC superiores (P1, P2) están demasiado arriba
(incluyen zona por encima del horizonte) o demasiado juntos.

**Solución**: Bajar P1.y y P2.y (valores más altos = más abajo en la imagen).
Separar P1.x y P2.x.

### La BEV aparece completamente negra

**Causa**: Los puntos SRC están fuera del área útil de la imagen o
la imagen no contiene información en esa zona.

**Solución**: Verificar que P3 y P4 estén dentro de los píxeles visibles
(no más allá del borde de la imagen).

### Las líneas rectas se curvan en la BEV

**Causa**: Los puntos SRC no están sobre el plano del suelo. Probablemente
algún punto está sobre el capó del vehículo, sobre un objeto elevado,
o sobre el cielo.

**Solución**: Todos los puntos deben corresponder a **puntos en el asfalto**.
Usar el panel "Original" como referencia.

### La BEV está rotada o invertida

**Causa**: El orden de los puntos está mal. P1-P2 deben ser los de arriba
(lejos) y P3-P4 los de abajo (cerca).

**Solución**: Verificar el orden: P1=sup-izq, P2=sup-der, P3=inf-der, P4=inf-izq.

### El ancho del carril no mide ~70 px

**Causa**: La escala de la homografía no coincide con `bev_scale_mpp = 0.005`
(5 mm/px). El rectángulo DST no tiene las proporciones correctas.

**Solución**: Ajustar el ancho del rectángulo DST. Si el carril real mide
35 cm, en BEV debe medir 35 / 0.5 = **70 px**.

---

## 6. Referencia Rápida de Parámetros

| Parámetro ROS2 | Tipo | Descripción |
|----------------|------|-------------|
| `bev_src_pts` | `double[]` (8) | 4 pares (x,y) del trapecio en imagen 640×480 |
| `bev_dst_pts` | `double[]` (8) | 4 pares (x,y) del rectángulo en BEV 320×240 |
| `bev_scale_mpp` | `double` | Escala: metros por píxel (default 0.005 = 5mm/px) |

### Cómo setear por línea de comandos

```bash
ros2 run lane_detection lane_pipeline_node --ros-args \
  -p frames_dir:=/path/to/frames \
  -p bev_src_pts:="[256.0, 264.0, 384.0, 264.0, 608.0, 456.0, 32.0, 456.0]" \
  -p bev_dst_pts:="[32.0, 0.0, 288.0, 0.0, 288.0, 240.0, 32.0, 240.0]"
```

### Cómo cambiar en tiempo real (desde otra terminal)

```bash
ros2 param set /lane_pipeline_node bev_src_pts "[256.0, 264.0, 384.0, 264.0, 608.0, 456.0, 32.0, 456.0]"
```

---

## 7. Glosario

| Término | Significado |
|---------|-------------|
| **BEV** | Bird's Eye View — vista de pájaro |
| **Homografía** | Matriz 3×3 que transforma una perspectiva en otra |
| **SRC** | Source — puntos de origen en la imagen de la cámara |
| **DST** | Destination — puntos de destino en la imagen BEV |
| **Trapecio** | Forma geométrica de 4 lados con dos paralelos — lo que se ve en perspectiva |
| **mpp** | Meters per pixel — metros por píxel |
| **e2** | Error lateral del vehículo respecto al centro del carril |
| **e3** | Error angular del vehículo respecto a la tangente del carril |
| **MPC** | Model Predictive Control — controlador que consume e2, e3, k |
