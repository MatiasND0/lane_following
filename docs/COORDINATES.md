# COORDINATES.md
> Sistema de referencia del vehículo, del BEV, y derivación matemática completa
> de e2, e3 y k. Todo agente que modifique el cálculo de errores debe leer este archivo.

---

## 1. Sistema de referencia del vehículo

```
         ^ Y  (adelante, dirección de avance)
         |
         |
         |   * cámara (y = +0.23 m)
         |
    ─────O────► X  (izquierda)
   (eje trasero = origen)

   Z sube (out of page, convención mano derecha)
```

| Eje | Dirección | Origen |
|-----|-----------|--------|
| Y | Hacia adelante del vehículo | Eje trasero (proyectado al suelo) |
| X | Hacia la izquierda del vehículo | Eje trasero |
| Z | Hacia arriba | Suelo bajo el eje trasero |

**Convención:** mano derecha (ROS standard). Yaw positivo = giro en sentido antihorario
visto desde arriba (giro a la izquierda).

---

## 2. Sistema de referencia de la imagen BEV

La imagen BEV tiene ejes de imagen (píxeles), relacionados con el mundo real así:

```
Imagen BEV (320×240 px)

(0,0) ──────────────────► u  (píxeles, hacia la derecha)
  │       lejos del vehículo
  │
  │         [zona de detección]
  │
  ▼
  v  (píxeles, hacia abajo = hacia el vehículo)
(0,239)     cerca del vehículo
```

**Relación BEV ↔ mundo real:**

```
x_mundo [m] = (u_bev - bev_w/2) * bev_scale_mpp   ← positivo = derecha en imagen = izquierda del vehículo
                                                       ⚠️  Ver nota de signo más abajo
y_mundo [m] = (bev_h - v_bev) * bev_scale_mpp      ← positivo = lejos del vehículo = adelante
```

**Nota de signo en X:**
En la imagen BEV, u=0 es el borde izquierdo de la imagen (que corresponde al lado
izquierdo del vehículo). u = bev_w/2 = 160 es el centro (directamente al frente).
Por tanto, u > 160 está a la derecha del vehículo → X_mundo es negativo (X del vehículo
es positivo a la izquierda).

```
x_vehiculo = -(u_bev - bev_w/2) * bev_scale_mpp
```

---

## 3. Relación cámara — eje trasero

La cámara D435 está montada a **`camera_offset_m = 0.23 m`** delante del eje trasero.

En el sistema BEV:
- La fila `v = bev_h - 1 = 239` corresponde a la posición de la **cámara**.
- El eje trasero está `camera_offset_m` detrás de la cámara.
- En coordenadas BEV: `v_eje_trasero = bev_h - 1 + camera_offset_m / bev_scale_mpp`

Como el BEV solo cubre la región **delante** de la cámara, el eje trasero queda
**fuera del dominio de la imagen** (v > bev_h). Se evalúa el polinomio en la extensión
matemática fuera del rango visible:

```
y_eval_px = -(camera_offset_m / bev_scale_mpp)
```

Esto es negativo: está "debajo" de la base de la imagen BEV, es decir, detrás de la cámara.
En la coordenada `v` de la imagen: `v_eval = bev_h + |y_eval_px|`, que está fuera de la imagen.

---

## 4. Modelo del polinomio de carril

El polinomio describe la posición lateral `u` en función de la posición longitudinal `v` en BEV:

```
u = a·v² + b·v + c          [px BEV]
```

Donde:
- `v` es la coordenada vertical en la imagen BEV (0 = arriba = lejos, bev_h = abajo = cerca)
- `u` es la coordenada horizontal en la imagen BEV (0 = izquierda, bev_w = derecha)
- `a, b, c` son los coeficientes ajustados en **píxeles BEV**

Para trabajar en metros, se puede convertir:
```
x_m = (u - bev_w/2) * bev_scale_mpp
y_m = (bev_h - v) * bev_scale_mpp
```

El polinomio en metros es:
```
x_m = A·y_m² + B·y_m + C
```

Donde A, B, C se obtienen sustituyendo y haciendo el cambio de variable. En la implementación
se trabaja en píxeles y se convierte al final para evitar acumulación de errores de escala.

---

## 5. Derivación de e2

**Definición:** distancia lateral entre el origen (eje trasero) y la línea central del carril,
en metros, en el eje X del vehículo.

**Paso 1:** Evaluar el polinomio central en `v = y_eval_px`:
```
u_center = a_c · y_eval_px² + b_c · y_eval_px + c_c     [px BEV]
```

**Paso 2:** Convertir a distancia lateral desde el centro de la imagen:
```
desplazamiento_px = u_center - bev_w / 2
```

Un valor positivo significa que el centro del carril está a la derecha en la imagen,
es decir, a la **derecha del vehículo** (X negativo en sistema del vehículo).

**Paso 3:** Convertir a metros con signo correcto:
```
e2 = -desplazamiento_px * bev_scale_mpp     [m]
```

El signo negativo convierte "derecha en imagen" → "e2 positivo" (vehículo está a la
derecha del carril). Esta convención es estándar para controladores de carril donde
e2 > 0 requiere corrección a la izquierda.

**Implementación:**
```cpp
double x_center_px = state.center.a * y_eval_px * y_eval_px
                   + state.center.b * y_eval_px
                   + state.center.c;
double e2 = -(x_center_px - bev_w_ / 2.0) * bev_scale_mpp_;
```

---

## 6. Derivación de e3

**Definición:** ángulo entre el eje longitudinal del vehículo (Y) y la tangente al
carril en el punto de evaluación. Positivo = carril gira a la derecha respecto al vehículo.

**Tangente al polinomio:**
El polinomio es `u(v) = a·v² + b·v + c`. La derivada es:
```
du/dv = 2a·v + b
```

Esta derivada está en unidades de px/px (adimensional). Representa el cambio en
posición lateral por unidad de avance longitudinal, es decir, la tangente del ángulo.

**Evaluación en el eje trasero:**
```
(du/dv)|_{v=y_eval_px} = 2·a_c·y_eval_px + b_c
```

**Ángulo:**
```
e3 = atan(du/dv)     [rad]
```

El argumento de atan es adimensional: el `bev_scale_mpp` cancela porque afecta
igualmente al numerador (du en px) y al denominador (dv en px).

**Nota de signo:** `du/dv > 0` significa que el carril se inclina hacia la derecha
conforme avanzamos → e3 > 0. Como el eje Y del vehículo apunta adelante y el vehículo
está alineado con ese eje (e3 = 0 en perfecto seguimiento), e3 > 0 indica que el carril
gira a la derecha → el vehículo debe girar a la derecha → acción de control positiva.

**Implementación:**
```cpp
double dxdy = 2.0 * state.center.a * y_eval_px + state.center.b;
double e3 = std::atan(dxdy);
```

---

## 7. Derivación de k

**Definición:** curvatura de la trayectoria de referencia en el punto de evaluación, en m⁻¹.

**Fórmula general de curvatura para una curva paramétrica u(v):**
```
k = (d²u/dv²) / (1 + (du/dv)²)^(3/2)
```

Para el polinomio de grado 2:
```
d²u/dv² = 2a
```

Por tanto:
```
k_px = 2·a_c / (1 + (du/dv)²)^(3/2)     [px⁻¹]
```

**Conversión a m⁻¹:**
La curvatura tiene unidades de [longitud]⁻¹. Como k_px está en px⁻¹:
```
k = k_px / bev_scale_mpp     [m⁻¹]
```

**Aproximación de curvatura pequeña:**
Cuando `|du/dv| << 1` (tramos casi rectos), se puede aproximar:
```
k ≈ 2·a_c / bev_scale_mpp
```

En BFMC, el radio mínimo es ~0.8 m → k_max ≈ 1.25 m⁻¹.
Con bev_scale_mpp = 0.005, eso es k_px_max ≈ 0.00625 px⁻¹.
En ese punto du/dv puede llegar a ~0.4 en curvas cerradas, por lo que
**no se debe usar la aproximación** en curvas: usar la fórmula completa.

**Implementación:**
```cpp
double k_px_inv = 2.0 * state.center.a / std::pow(1.0 + dxdy * dxdy, 1.5);
double k = k_px_inv / bev_scale_mpp_;
```

---

## 8. Resumen de convenciones de signo

| Variable | Positivo significa | Negativo significa |
|----------|-------------------|--------------------|
| e2 | Vehículo a la derecha del carril | Vehículo a la izquierda |
| e3 | Carril gira a la derecha / vehículo apunta izq | Carril gira a la izquierda |
| k | Curva hacia la izquierda | Curva hacia la derecha |
| a (coef. cuadrático) | Parábola abre hacia la derecha (curva izq en perspectiva BEV) | Parábola abre hacia la izquierda |

---

## 9. Diagrama unificado

```
              BEV (imagen, 320×240 px)
              ┌──────────────────────────┐
         v=0  │       lejos (>1.2m)      │
              │                          │
              │  u_L   u_C    u_R        │  ← polinomios
              │   │     │     │          │
              │   └─────┘─────┘          │
              │      carril              │
v=bev_h=240  │  ← posición de la cámara │
              └──────────────────────────┘
                u=0            u=320

              v_eval = bev_h + camera_offset_m/bev_scale_mpp
              (fuera de la imagen, debajo)

              ┌──────────────────────────┐
              │  eje trasero (origen)    │  ← evaluación del polinomio aquí
              └──────────────────────────┘

Sistema de referencia del vehículo:
  Y arriba = adelante
  X derecha = izquierda del vehículo
  Origen = eje trasero
```
