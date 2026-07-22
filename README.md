# [Nombre del repositorio]

Firmware, simulaciones y datasets asociados a la tesis doctoral *Técnicas de gestión eficiente de sistemas Energy Harvesting en entornos IoT orientadas a la neutralidad energética*.

Se incluyen tres algoritmos de gestión energética (**TB**, **G-PD** y **GRP-LAR**), implementados en firmware para nodo IoT y replicados en Python para simulación, junto con los datasets experimentales utilizados para su evaluación.

---

## Contenido

| Directorio | Descripción |
|---|---|
| `firmware/` | Implementación embebida de los tres algoritmos |
| `simulation/` | Scripts Python de simulación y análisis |
| `datasets/` | Datos experimentales y de irradiancia |
| `results/` | Figuras y salidas generadas |
| `docs/` | Documentación adicional |

---

## Algoritmos

| Sigla | Nombre completo | Descripción breve |
|---|---|---|
| **TB** | [Time-Based / completar] | [Una o dos líneas: criterio de decisión, entradas, salida] |
| **G-PD** | [Greedy - ... / completar] | [ídem] |
| **GRP-LAR** | [ídem] | [ídem] |

---

## Estructura del repositorio

```
.
├── firmware/
│   ├── tb/
│   │   ├── tb.ino
│   │   └── config.h
│   ├── g_pd/
│   └── grp_lar/
│   └── common/              # drivers compartidos (INA219, RTC, LoRa)
│
├── simulation/
│   ├── sim_tb.py
│   ├── sim_g_pd.py
│   ├── sim_grp_lar.py
│   ├── common/              # modelos de batería, panel, carga
│   └── requirements.txt
│
├── datasets/
│   ├── raw/
│   ├── processed/
│   └── README.md            # descripción de columnas, unidades, periodo
│
├── results/
├── LICENSE
└── README.md
```

---

## Hardware

| Componente | Modelo | Notas |
|---|---|---|
| Microcontrolador | [Adafruit Feather M0 LoRa / RP2040] | |
| Panel solar | [modelo, Voc, Isc, Pmax] | |
| Batería | [química, capacidad] | |
| Sensado de corriente | INA219 ×3 | panel / batería / carga |
| Gauge de batería | LC709203F | |
| RTC | [modelo] | interrupción de despertado |

---

## Firmware

### Requisitos

- Arduino IDE [versión] o PlatformIO [versión]
- Placa: [core y versión]
- Librerías: [listar con versiones]

### Compilación y carga

```bash
# ejemplo
arduino-cli compile --fqbn <fqbn> firmware/tb
arduino-cli upload  --fqbn <fqbn> -p <puerto> firmware/tb
```

### Configuración

Parámetros ajustables en `config.h` (número de slots, duración de slot, planes de consumo, umbrales).

---

## Simulaciones

### Instalación

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r simulation/requirements.txt
```

### Ejecución

```bash
python simulation/sim_tb.py --dataset datasets/processed/<archivo>.csv --out results/
```

| Argumento | Descripción | Por defecto |
|---|---|---|
| `--dataset` | Ruta al CSV de entrada | — |
| `--out` | Directorio de salida | `results/` |

---

## Datasets

| Dataset | Ubicación | Periodo | Frecuencia de muestreo | Variables |
|---|---|---|---|---|
| [nombre] | [sitio] | [fechas] | [p. ej. 1 min] | [I, V, P, T, ...] |

Formato, unidades y criterios de limpieza documentados en `datasets/README.md`.

---

## Reproducción de los resultados de la tesis

| Figura / Tabla | Script | Dataset |
|---|---|---|
| Fig. [n] | `simulation/[script].py` | `datasets/processed/[archivo]` |

---

## Cita

```bibtex
@phdthesis{kuzman2026,
  author = {Kuzman, Melisa},
  title  = {Técnicas de gestión eficiente de sistemas Energy Harvesting en entornos IoT orientadas a la neutralidad energética},
  school = {Universidad Nacional de Mar del Plata, Facultad de Ingeniería},
  year   = {2026}
}
```

---

## Licencia

[MIT / CC BY 4.0 / ...] — ver `LICENSE`.

Sugerencia habitual: código bajo MIT o Apache-2.0, datasets bajo CC BY 4.0.

---

## Contacto

Melisa Kuzman — [correo]
