# Datasets

Experimental data for the thesis *Técnicas de gestión eficiente de sistemas Energy Harvesting en entornos IoT orientadas a la neutralidad energética*.

The data is split into two groups:

- **`production_data/`**: production and characterization measurements of the panel and the node, independent of any algorithm.
- **`algorithm_results/`**: execution logs of the energy-management algorithms (TB, G-PD and GRP-LAR) running on the IoT node.

> **Format note:** the files do not share a common format. Before loading any of them, check its table below for the **field separator** (`;` or tab) and the **decimal separator** (point or comma).

```
datasets/
├── README.md
├── production_data/
│   ├── Datos_nodo_ESI.csv          continuous characterization (~1 min)
│   └── PV_production_dataset.csv   per-slot production (Voc, Isc, Pmax, temp) FF = 0.75
└── algorithm_results/
    ├── 260710_GLAR_550.CSV         GRP-LAR run (node EH04)
    ├── 260710_PD_550.CSV           G-PD run    (node EH05)
    └── UMBRALES_ARCHIVO1_260601.CSV TB run     (node N01)
```

---

## Overview

| File | Group | Algorithm | Node | Period | Records | Separator | Decimal |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `production_data/Datos_nodo_ESI.csv` | Production | (none) | ESI node | 2026-04-28 → 2026-05-18 | 30,017 | tab | `,` |
| `production_data/PV_production_dataset.csv` | Production | (none) | panel | 2026-07-24 → 2026-10-16 | 4,031 | `;` | `,` |
| `algorithm_results/260710_GLAR_550.CSV` | Results | GRP-LAR | EH04 | 2026-07-10 → 2026-07-17 | 338 | `;` | `.` |
| `algorithm_results/260710_PD_550.CSV` | Results | G-PD | EH05 | 2026-07-10 → 2026-07-17 | 337 | `;` | `.` |
| `algorithm_results/UMBRALES_ARCHIVO1_260601.CSV` | Results | TB (thresholds) | N01 | 2026-06-01 → 2026-06-09 | 384 | tab | `.` |

Common conventions:

- `slot` / `SLOT`: half-hour slot index within the day (0 to 47; e.g. 13:00 = slot 26).
- `PLAN` / `Plan`: active energy policy level (`P0` to `P3`).
- `N`: number of samples averaged within the slot.

Naming of the result files: `YYMMDD_<ALGORITHM>_<panel>.CSV`, where the date is the start of the run and `550` identifies the panel configuration used.

---

# production_data/

## `PV_production_dataset.csv`

Panel production characterization per half-hour slot, spanning ~3 months. This is the **net production** profile of the panel, with no algorithm involved.

Format: separator **`;`**, decimal **comma**, date as `D/M/YYYY`.

| Column | Description | Unit |
| --- | --- | --- |
| `date` | Record date (`D/M/YYYY`) | — |
| `slot` | Half-hour slot of the day (0 to 47) | — |
| `hora` | Slot start time (`HH:MM`) | — |
| `voc_V` | Panel open-circuit voltage | V |
| `isc_mA` | Panel short-circuit current | mA |
| `pmax_est_mW` | Estimated panel maximum power | mW |
| `temp_C` | Temperature | °C |

## `Datos_nodo_ESI.csv`

Continuous node measurements taken at the ESI (UCLM), with ~1-minute cadence: voltage, current and power of panel, battery and load.

Format: separator **tab**, decimal **comma**.

| Column | Description | Unit |
| --- | --- | --- |
| `FechaHora` | ISO 8601 timestamp | — |
| `Vp_V` | Panel voltage | V |
| `Ip_uA` / `Ip_mA` | Panel current (same magnitude, two scales) | µA / mA |
| `Pp_W` | Panel power | W |
| `Vc_V` | Load voltage | V |
| `Ic_uA` / `Ic_mA` | Load current | µA / mA |
| `Pc_W` | Load power | W |
| `Vb_V` | Battery voltage | V |
| `Ib_uA` / `Ib_mA` | Battery current | µA / mA |
| `Pb_W` | Battery power | W |
| `V_LC709203F` | Battery voltage from the LC709203F gauge | V |
| `SoC_LC709203F` | State of charge from the gauge | % |
| `Voc_V` | Panel open-circuit voltage | V |
| `Isc_mA` | Panel short-circuit current | mA |
| `Pmax_est_W` / `P_max_est_mW` | Estimated panel maximum power | W / mW |

---

# algorithm_results/

## `260710_GLAR_550.CSV` (GRP-LAR)

Execution log of the **GRP-LAR** algorithm (Greedy with GLAR replanning) on node EH04.

Format: separator **`;`**, decimal **point**.

| Column | Description | Unit |
| --- | --- | --- |
| `NODE` | Node identifier | — |
| `ID` | Record identifier / counter | — |
| `TIMESTAMP` | ISO 8601 timestamp | — |
| `SLOT` | Half-hour slot of the day (0 to 47) | — |
| `PLAN` | Active energy plan (`P0`–`P3`) | — |
| `N` | Samples averaged in the slot | — |
| `VP_V` | Panel voltage | V |
| `IP_mA` | Panel current | mA |
| `VC_V` | Load voltage | V |
| `IC_mA` | Load current | mA |
| `VBINA_V` | Battery voltage (INA219) | V |
| `IB_mA` | Battery current (INA219) | mA |
| `VBLC_V` | Battery voltage (LC709203F gauge) | V |
| `SOC_pct` | State of charge | % |
| `TEMP_C` | Temperature | °C |
| `VOC_V` | Panel open-circuit voltage | V |
| `ISC_mA` | Panel short-circuit current | mA |
| `VSHORT_V` | Voltage during the short-circuit test | V |
| `PMAX_EST_mW` | Estimated panel maximum power (Voc·Isc, ideal rectangle, no fill factor) | mW |
| `SOC_ULT_pct` | State of charge at slot close | % |
| `PLAN_ANT` | Plan before replanning | — |
| `PLAN_NVO` | New plan after replanning | — |
| `ERROR_mW` | Energy prediction error | mW |
| `PHAT_mW` | Predicted power (P estimate) | mW |

## `260710_PD_550.CSV` (G-PD)

Execution log of the **G-PD** algorithm (Greedy with previous-day planning) on node EH05. Shares the base columns with GRP-LAR, without the replanning columns (`PLAN_ANT`, `PLAN_NVO`, `ERROR_mW`, `PHAT_mW`).

Format: separator **`;`**, decimal **point**.

| Column | Description | Unit |
| --- | --- | --- |
| `NODE` | Node identifier | — |
| `ID` | Record identifier / counter | — |
| `TIMESTAMP` | ISO 8601 timestamp | — |
| `SLOT` | Half-hour slot of the day (0 to 47) | — |
| `PLAN` | Active energy plan (`P0`–`P3`) | — |
| `N` | Samples averaged in the slot | — |
| `VP_V` | Panel voltage | V |
| `IP_mA` | Panel current | mA |
| `VC_V` | Load voltage | V |
| `IC_mA` | Load current | mA |
| `VBINA_V` | Battery voltage (INA219) | V |
| `IB_mA` | Battery current (INA219) | mA |
| `VBLC_V` | Battery voltage (LC709203F gauge) | V |
| `SOC_pct` | State of charge | % |
| `TEMP_C` | Temperature | °C |
| `VOC_V` | Panel open-circuit voltage | V |
| `ISC_mA` | Panel short-circuit current | mA |
| `VSHORT_V` | Voltage during the short-circuit test | V |
| `PMAX_EST_mW` | Estimated panel maximum power (Voc·Isc, ideal rectangle, no fill factor) | mW |
| `SOC_ULT_pct` | State of charge at slot close | % |

## `UMBRALES_ARCHIVO1_260601.CSV` (TB / thresholds)

Execution log of the **TB (Threshold Battery)** algorithm, threshold-based decisions, on node N01.

Format: separator **tab**, decimal **point**.

> **Confirm units:** despite the `_mW` and `_mA` suffixes, the power and current values are large integers and appear to be expressed in **µW** and **µA** respectively (e.g. `Ip_mA = 89023` equals ~89 mA). Voltages (`V`) are in volts with decimals.

| Column | Description | Unit (per header) |
| --- | --- | --- |
| `ID` | Node identifier | — |
| `Packet` | Packet / record counter | — |
| `Fecha` | Date (YYYY-MM-DD) | — |
| `Hora` | Time (HH:MM:SS) | — |
| `Slot` | Half-hour slot of the day (0 to 47) | — |
| `Plan` | Active energy plan (`P0`–`P3`) | — |
| `Q` | Threshold-algorithm configuration parameter (confirm) | — |
| `N` | Samples averaged in the slot | — |
| `Pmax_est_mW` | Estimated maximum power (Voc·Isc, ideal rectangle, no fill factor) | mW (see note) |
| `Ppanel_mW` | Power actually harvested from the panel | mW (see note) |
| `Pnodo_mW` | Power consumed by the node | mW (see note) |
| `Pbatt_mW` | Battery power | mW (see note) |
| `Vp_V` | Panel voltage | V |
| `Ip_mA` | Panel current | mA (see note) |
| `Vc_V` | Load voltage | V |
| `Ic_mA` | Load current | mA (see note) |
| `Vb_V` | Battery voltage | V |
| `Ib_mA` | Battery current | mA (see note) |
| `Vlc_V` | Battery voltage (LC709203F gauge) | V |
| `SoC_pct` | State of charge | % |
| `Voc_V` | Panel open-circuit voltage | V |
| `Isc_mA` | Panel short-circuit current | mA (see note) |
| `Vshort_V` | Voltage during the short-circuit test | V |
| `Pmax_mW` | Measured panel maximum power | mW (see note) |
