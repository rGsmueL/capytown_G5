# capytown_G20 — RC-0: primer arranque y calibración

**Universidad ESAN · Robótica (08079) · 2026-2**
Proyecto CapyTown · Semana 8 · Robot Yahboom MicroROS-Pi5

| Dato | Valor |
|---|---|
| **Sección** | S-004 |
| **Variante de calibración (Partes B y C)** | A3 — Hombro |
| **Variante de validación (Parte D)** | A4 — Gancho |
| **ROS_DOMAIN_ID** | **20** (Pi5 y firmware de la placa) |
| **ROS 2** | Humble + micro-ROS |
| **Tag de entrega** | `v1.0-rc0` |
| **Fecha de pruebas** | 15–16/07/2026 |
| **Radio final `r`** | **0.0313029273082 m** |
| **Ancho de vía efectivo `b_eff`** | **0.2501 m** |

## Integrantes y roles

| Integrante | Rol               | Responsabilidad en RC-0                                                   |
| ---------- | ----------------- | ------------------------------------------------------------------------- |
| Maykol     | **Integración**   | Bringup, `ROS_DOMAIN_ID`, instalación de rosbag2, bags, repositorio y tag |
| Joaquin    | **Control**       | Calibración de `r` y `b_eff`; actualización de `wheel_params.yaml`        |
| Samuel     | **Planificación** | Script con la secuencia de comandos de prueba (recta y giros)             |
| Wilmer     | **Percepción**    | `plot_odom.py`: lectura de bags y gráficas                                |

## Parámetros finales (`config/wheel_params.yaml`)

| Parámetro                        | Valor                 | Nota                                              |
| -------------------------------- | --------------------- | ------------------------------------------------- |
| `wheel_radius` (**r**)           | **0.0313029273082 m** | calibrado en A3; error final 0.16–0.17 %          |
| `wheel_separation` (**b_eff**)   | **0.2501 m**          | adoptado en A3 y usado en la validación A4        |
| `firmware_wheel_radius` (r₀)     | 0.0325 m              | no se toca                                        |
| `firmware_wheel_separation` (b₀) | 0.2400 m              | no se toca                                        |
| `encoder_ticks_per_rev`          | 1320                  |                                                   |
| `max_linear_vel`                 | 0.30 m/s              | límite del `odom_corrector`                       |
| `max_angular_vel`                | 1.50 rad/s            | límite del `odom_corrector`                       |
| `publish_tf`                     | `true`                | publica la TF de `/odom`                          |

Fórmulas usadas:

- `r_nuevo = r · d_real / d_odom`
- `b_eff_nuevo = b_eff · 360° / ángulo_medido`

Sobre `b_eff`: el valor **usado en las pruebas y adoptado en el YAML es 0.2501 m**. La última fila de `calibration_log.csv` sugiere `0.2509 m` como corrección adicional (error 0.33 %), pero no se aplicó; la validación A4 se hizo con **0.2501 m**.

## Resultados

### B — Recta de 1.0 m: calibración de `r` (A3)

| Corrida | `r` usado [m]      | `d_cinta` [cm] | `d_odom` [cm] | Error [%] | Comando |
| ------- | ------------------ | -------------- | ------------- | --------- | ------- |
| 1       | 0.03161623         | 94.00          | 96.65         | 2.82      | 10 s    |
| 2       | 0.0307493597516    | 95.00          | 93.32         | 1.77      | 10 s    |
| 3       | 0.0313029273082    | 94.50          | 94.34         | 0.17      | 10 s    |
| 4       | 0.0313029273082    | 98.60          | 98.44         | 0.16      | 10.5 s  |

La corrida 4 es la corrida final con **10.5 s** (bag `rc0_recta`): `/odom` acumuló 98.44 cm de desplazamiento neto, con desvío lateral máximo de 0.23 cm y giro acumulado de +0.84°. El error se calcula contra la cinta (`|d_cinta − d_odom| / d_cinta`), no contra `v × t`. Las corridas 1–3 usaron 10 s.

### C — Giro en sitio de 360°: calibración de `b_eff` (A3)

| # | Fecha | `b_eff` antes [m] | Objetivo | Medido | `b_eff` después [m] | Error [%] |
| - | ----- | ----------------- | -------- | ------ | ------------------- | --------- |
| 1 | 15/07 | 0.2329            | 360°     | 343.2° | 0.2443              | 4.67      |
| 2 | 15/07 | 0.2443            | 360°     | 387.3° | 0.2271              | 7.58      |
| 3 | 15/07 | 0.2271            | 360°     | 363.4° | 0.2250              | 0.94      |
| 4 | 16/07 | 0.2535            | 360°     | 364.9° | 0.2501              | 1.36      |
| 5 | 16/07 | 0.2501            | 360°     | 358.8° | 0.2509              | 0.33      |

`b_eff antes` es el valor usado en la prueba; `b_eff después` es la corrección sugerida. Se adoptó **0.2501 m** (fila 4) en `wheel_params.yaml` y con ese valor se hizo la validación A4. La fila 5 sugiere 0.2509 m (0.33 %) como corrección adicional.

### D — Validación en la segunda variante (A4, sin recalibrar)

| Prueba    | Valor real | Valor `/odom` | Error [%]              | Bag                | Desvío lateral máx. | Giro acumulado |
| --------- | ---------- | ------------- | ---------------------- | ------------------ | ------------------- | -------------- |
| Recta 1   | 98.60 cm   | 98.92 cm      | 0.32                   | `rc0_val_recta1`   | 1.95 cm             | −2.27°         |
| Recta 2   | 98.90 cm   | 98.73 cm      | 0.17                   | `rc0_val_recta2`   | 0.93 cm             | +0.39°         |
| Giro 360° | 360.5°     | 361.7°        | 0.33 (0.14 vs 360°)    | `rc0_val_giro`     | 0.64 cm             | +361.89°       |

Las dos rectas de validación usaron **10.5 s**; el giro se hizo a **0.3 rad/s**. El error del giro es **0.33 % real vs `/odom`** y **0.14 % contra el objetivo de 360°**. No se recalculó ningún parámetro en A4.

## Bags y métricas de `/odom`

Métricas calculadas por `capytown_esan/plot_odom.py` y guardadas en `capytown_esan/plots/resumen.csv`:

| Bag                | Topics                    | Msgs `/odom` | Duración | Distancia | Despl. neto | Desvío lateral máx. | Giro acumulado |
| ------------------ | ------------------------- | ------------ | -------- | --------- | ----------- | ------------------- | -------------- |
| `rc0_recta`        | `/odom`, `/tf`, `/cmd_vel` | 1972         | 177.36 s | 0.9845 m  | 0.9844 m    | 0.23 cm             | +0.84°         |
| `rc0_giro`         | `/odom`, `/tf`, `/cmd_vel` | 722          | 64.88 s  | 0.1117 m  | 0.0099 m    | 1.03 cm             | +363.25°       |
| `rc0_val_recta1`   | `/odom`, `/tf`, `/cmd_vel` | 1093         | 98.30 s  | 0.9894 m  | 0.9892 m    | 1.95 cm             | −2.27°         |
| `rc0_val_recta2`   | `/odom`, `/tf`, `/cmd_vel` | 1419         | 127.66 s | 0.9874 m  | 0.9873 m    | 0.93 cm             | +0.39°         |
| `rc0_val_giro`     | `/odom`, `/tf`, `/cmd_vel` | 420          | 37.75 s  | 0.1475 m  | 0.0048 m    | 0.64 cm             | +361.89°       |

Comandos registrados en `/cmd_vel`:

| Bag              | Movimiento registrado                                       |
| ---------------- | ----------------------------------------------------------- |
| `rc0_recta`      | 0.10 m/s durante 10.511 s; luego parada                     |
| `rc0_giro`       | 0.5 rad/s durante 39.15 s; luego parada                     |
| `rc0_val_recta1` | solo el mensaje de parada (la distancia se toma de `/odom`) |
| `rc0_val_recta2` | 0.10 m/s durante 10.507 s; luego parada                     |
| `rc0_val_giro`   | 0.3 rad/s durante 24.19 s; luego parada                     |

Las duraciones de los bags incluyen tiempo en reposo; las duraciones de la tabla de comandos salen de `/cmd_vel`.

## Verificación del sistema (Parte A)

Datos tomados de `setup_log.txt` (Thu Jul 16 06:45:25 UTC 2026):

- `ROS_DOMAIN_ID=20`, `ROS_DISTRO=humble`, `ROS_LOCALHOST_ONLY=0`.
- Nodos visibles: `/YB_Car_Node` y `/odom_corrector`.
- Topics: `/battery`, `/beep`, `/cmd_vel`, `/imu`, `/odom`, `/odom_raw`, `/odom_reset`, `/scan`, `/servo_s1`, `/servo_s2`, `/tf`, entre otros.
- `/odom` publica a **≈11.1 Hz** (ventanas medidas: 11.33, 11.10, 11.20, 11.08, 11.15 y 11.07 Hz).
- `grafo.png` contiene el grafo de nodos/topics (rqt_graph).
- El `setup_log.txt` no incluye la frecuencia de `/imu`; solo se registró la de `/odom`.

## Scripts y herramientas

### `scripts/calcular_recorrido.py`

Publica `0.10 m/s`, espera **10.5 s**, envía la parada y lee la posición final de `/odom` para calcular la distancia neta. Se usó en la corrida final de recta y en las validaciones. Las corridas 1–3 de calibración se hicieron con 10 s. El texto de consola del script todavía dice “10 segundos”, pero el tiempo efectivo es 10.5 s.

### `scripts/calibrar_radio.py`

Calcula el error relativo y el factor de corrección del radio:

```text
error = |d_real − d_odom| / d_real · 100
factor = d_real / d_odom
r_nuevo = r_actual · factor
```

Uso previsto (ejemplo con la corrida 4):

```bash
python3 calibrar_radio.py 0.9860 0.9844 --radio-actual-mm 31.3029273082
```

### `scripts/calibrate_beff.py`

Gira a `0.5 rad/s` hasta que `/odom` acumule el objetivo (360° por defecto), pide el ángulo físico medido y calcula `b_eff_despues = b_eff_antes · 360° / medido`. Registra cada iteración en `calibration_log.csv`.

```bash
ros2 run capytown_esan calibrate_beff --ros-args \
  -p b_eff:=0.2501 -p target_angle_deg:=360.0 -p angular_vel:=0.5
```

### `capytown_esan/plot_odom.py`

Lee el `storage_identifier` del `metadata.yaml` de cada bag (`sqlite3` o `mcap`), procesa `/odom` y `/cmd_vel`, calcula distancia, desplazamiento neto, desvío lateral, giro acumulado y duración, y genera las gráficas x-y y θ(t). Salida: 10 PNG (2 por bag) y `plots/resumen.csv`.

Desde la raíz del repositorio:

```bash
cd capytown_esan
python3 plot_odom.py --all --dir ../bags --out plots
```

El `--out` es relativo a la carpeta del script; por eso se ejecuta desde `capytown_esan/`.

## Reproducir una prueba

```bash
export ROS_DOMAIN_ID=20
ros2 launch capytown_esan bringup.launch.py

# Recta: 0.10 m/s durante 10.5 s y freno
ros2 bag record /odom /cmd_vel /tf -o rc0_recta
python3 scripts/calcular_recorrido.py

# Giro de calibración
ros2 bag record /odom /cmd_vel /tf -o rc0_giro
ros2 run capytown_esan calibrate_beff --ros-args \
  -p b_eff:=0.2501 -p target_angle_deg:=360.0 -p angular_vel:=0.5

# Gráficas
cd capytown_esan
python3 plot_odom.py --all --dir ../bags --out plots
```

## Estructura del repositorio

```text
.
├── bags/
│   ├── rc0_recta/
│   ├── rc0_giro/
│   ├── rc0_val_recta1/
│   ├── rc0_val_recta2/
│   └── rc0_val_giro/
├── capytown_esan/
│   ├── plot_odom.py
│   └── plots/                 # 10 PNG
├── config/
│   └── wheel_params.yaml
├── scripts/
│   ├── calcular_recorrido.py
│   ├── calibrar_radio.py
│   └── calibrate_beff.py
├── calibration_log.csv
├── linea_recta.csv
├── validacion.csv
├── setup_log.txt
├── grafo.png
├── informe.pdf
└── README.md
```

## Entregables

- `config/wheel_params.yaml` con `r = 0.0313029273082 m` y `b_eff = 0.2501 m`.
- `linea_recta.csv` con 4 corridas (la última con 10.5 s).
- `calibration_log.csv` con 5 iteraciones de giro.
- `validacion.csv` con 2 rectas y 1 giro en A4.
- `setup_log.txt` y `grafo.png`.
- `bags/` con los 5 bags de ROS 2.
- `capytown_esan/plot_odom.py` y `capytown_esan/plots/` con las gráficas.
- `informe.pdf` de 1 página.
- `README.md` con integrantes, roles, sección, variantes, dominio y parámetros finales.

