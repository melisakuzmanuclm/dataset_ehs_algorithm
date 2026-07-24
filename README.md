# Políticas de gestión energética para sistemas Energy Harvesting: firmware, simulaciones y datasets

Firmware, simulaciones y datasets asociados a la tesis doctoral *Técnicas de gestión eficiente de sistemas Energy Harvesting en entornos IoT orientadas a la neutralidad energética*.

Se incluyen tres algoritmos de gestión energética (**TB**, **G-PD** y **GRP-LAR**), implementados en firmware para nodo IoT y replicados en Python para simulación, junto con los datasets experimentales utilizados para su evaluación.

---

## Contenido

| Directorio | Descripción |
|---|---|
| `firmware/` | Implementación embebida de los tres algoritmos |
| `simulation/` | Scripts Python de simulación y análisis |
| `datasets/` | Datos experimentales |


---

## Algoritmos

| Sigla | Nombre completo | Descripción breve |
|---|---|---|
| **TB** | Threshold Battery | Decisión tomadas por umbrales |
| **G-PD** | Greedy - PD | Política Greedy con planificación del día previo |
| **GRP-LAR** | Greedy - RP-GLAR | Política Greedy con replanificación GLAR |

---

## Hardware

| Componente | Modelo | Notas |
|---|---|---|
| Microcontrolador | Adafruit Feather M0 LoRa | |
| Panel solar | Potencia variable | |
| Batería | Li-Po | Capacidad variable|
| Sensado de corriente | INA219 ×3 | panel / batería / carga |
| Gauge de batería | LC709203F | |
| RTC | PCF8523 | interrupción de despertado |

---

## Firmware

### Requisitos

- Arduino IDE o PlatformIO
- Placa: Adafruit Cortex M0 RF65

---
