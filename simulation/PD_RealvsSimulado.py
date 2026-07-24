"""PD - Greedy diario (nodo EH05 = Nodo B) - REAL vs SIMULADO, portando el firmware
planificador_greedy.ino. Mas simple que GLAR: NO hay LAR ni replanificacion.
  - Greedy de horizonte completo SOLO a medianoche (planificarGreedy).
  - Pronostico = pmaxPred = produccion REAL del dia anterior (overwrite directo,
    SIN EWMA). Arranca del array default del .ino.
  - PF0 reactivo: SOC_ULT (ultima medicion) <= SOC_MIN  o  VBLC_V < 3.5 V.
Las decisiones usan el SoC real medido (como el firmware: socIni = m.soc).
Datos: se concatenan los dos CSV (hubo un reinicio el 15/jul ~18:00).
"""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ===================== CONFIG =====================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(BASE_DIR, "figuras"); os.makedirs(FIG_DIR, exist_ok=True)
DATA_DIR = r"C:\Users\melis\OneDrive\Escritorio\Datos_Calculados_CASA\PD\17_7"
# Archivo unico ya fusionado (el reinicio del 15/jul quedo integrado; no pierde slots).
CSV_PATHS = [os.path.join(DATA_DIR, "260710_PD_550ma.CSV")]
N_DIAS = 7                              # recorta a 7 dias EXACTOS (7*48 = 336 slots)
# Metricas de coincidencia: elegis si contar o no el arranque parcial.
#   True  -> solo dias completos (48 slots); saca el 1er dia parcial (que "salio", no se eligio)
#   False -> todos los slots (incluye el arranque)
# (Las figuras y el CSV usan SIEMPRE los 7 dias completos.)
METRICAS_SOLO_COMPLETOS = False

# ---- Parametros del firmware (planificador_greedy.ino, EH05 = Nodo B) ----
N_SLOTS = 48
CONSUMO = [8.0, 9.4, 102.2, 140.2]         # CONSUMO_PLAN_MW  [P0,P1,P2,P3]
ETA_HARV = 0.71
ETA_CARGA = 0.95
BAT_MAH = 550
BAT_ENERGY = BAT_MAH * 3.7
MAX_CHARGE = BAT_MAH * 0.2 * 3.7
SOC_MIN, SOC_NEUTRAL_MAX, SOC_MAX = 20.0, 50.0, 100.0
# Margen del planificador: el greedy degrada hasta mantener socMin >= esto (SU modelo).
# = SOC_MIN (20) reproduce el firmware original (planifica al filo del 20%).
# > SOC_MIN (ej. 30) es el FIX propuesto: deja colchon para el exceso de consumo real.
# NO afecta el umbral de emergencia PF0 (ese sigue en SOC_MIN=20).
SOC_MARGEN_PLAN = 20.0
SLOT_H = 0.5
VBAT_EMERG = 3.50
MAX_ITER = 200

# Calidad real por plan (Nodo B). PF0 = nivel P0.
CALIDAD = {0: 5.7, 1: 6.7, 2: 72.9, 3: 100.0}

# pmaxPred default (perfil real medido, del .ino) [mW]
PMAXPRED_DEFAULT = np.array([
    0.020, 0.020, 0.020, 0.020,  0.020, 0.020, 0.020, 0.020,
    0.020, 0.020, 0.020, 0.030,  0.080, 0.300, 3.000, 8.000,
    14.000, 30.000, 45.000, 280.000,  330.000, 470.000, 555.000, 605.000,
    635.000, 655.000, 650.000, 690.000,  690.000, 630.000, 590.000, 585.000,
    555.000, 540.000, 335.000, 290.000,  260.000, 125.000, 35.000, 8.000,
    2.000, 0.500, 0.100, 0.050,  0.020, 0.020, 0.020, 0.020,
])

NUM_PLAN = {0: "P0", 1: "P1", 2: "P2", 3: "P3"}
PLAN_IDX = {"PF0": 0, "P0": 0, "P1": 1, "P2": 2, "P3": 3}
PLAN_LATEX = {0: r"$\phi_0$", 1: r"$\phi_1$", 2: r"$\phi_2$", 3: r"$\phi_3$"}
#C_REAL, C_SIM = "#ff7f0e", "#8c4a00"       # PD = naranja (coherente con las otras figuras)
C_REAL, C_SIM = "#000000", "#ff7f0e"

# Modelo energetico CALIBRADO solo para la figura de SoC (distinto del ETA_HARV=0.71
# del firmware, que es para planificar). El consumo base = always-on real no incluido
# en los costos de plan (MCU+sensores+UGC quiescente+gauge).
ETA_SOC = 0.81
P_BASE_SOC = 47.5      # mW; consumo base always-on, ajustado al SoC real continuo (MAE ~7%)
plt.rcParams.update({"font.size": 14, "axes.labelsize": 14, "legend.fontsize": 15})

# =================================================


# ---------------- Modelo energetico (simularSlot del .ino) ----------------
def simular_slot(soc, pmax, plan):
    pNeto = pmax * ETA_HARV - CONSUMO[plan]
    if pNeto > 0.0:
        pNeto *= ETA_CARGA
    if pNeto > MAX_CHARGE:
        pNeto = MAX_CHARGE
    socNext = soc + pNeto * SLOT_H / BAT_ENERGY * 100.0
    socNext = min(max(socNext, 0.0), SOC_MAX)
    return socNext, min(socNext, soc)


def soc_forward_cal(soc, pmax, plan):
    """SoC del gemelo CALIBRADO (ETA_SOC + consumo base). Solo para la figura de SoC."""
    pN = pmax * ETA_SOC - (CONSUMO[plan] + P_BASE_SOC)
    if pN > 0.0:
        pN *= ETA_CARGA
    if pN > MAX_CHARGE:
        pN = MAX_CHARGE
    return min(max(soc + pN * SLOT_H / BAT_ENERGY * 100.0, 0.0), SOC_MAX)


def spill_slot(soc, pmax, plan):
    """Spillage referido al nivel del panel (comparable con PMAX_EST - Vp*Ip) [mWh]."""
    pNeto = pmax * ETA_HARV - CONSUMO[plan]
    carga = 0.0
    if pNeto > 0.0:
        pNeto *= ETA_CARGA
        carga = min(pNeto, MAX_CHARGE)
        if soc + carga * SLOT_H / BAT_ENERGY * 100.0 > SOC_MAX:
            carga = (SOC_MAX - soc) / 100.0 * BAT_ENERGY / SLOT_H
    usado = CONSUMO[plan] + carga
    return max(0.0, pmax - usado / ETA_HARV) * SLOT_H


def simular_dia(socIni, sched, pmaxPred, socObj):
    soc, viol_soc = socIni, False
    for k in range(N_SLOTS):
        soc, smin = simular_slot(soc, pmaxPred[k], sched[k])
        if smin < SOC_MARGEN_PLAN:          # margen de planificacion (fix), no el umbral PF0
            viol_soc = True
    return viol_soc, (soc < socObj)


def greedy(sched, socIni, pmaxPred, socInicioDia):
    """planificarGreedy del .ino: todo P3, degrada el slot de peor margen."""
    socObj = min(socInicioDia, SOC_NEUTRAL_MAX)
    for k in range(N_SLOTS):
        sched[k] = 3
    for _ in range(MAX_ITER):
        vs, vn = simular_dia(socIni, sched, pmaxPred, socObj)
        if not vs and not vn:
            return
        peor, kp = 1e9, -1
        for k in range(N_SLOTS):
            if sched[k] == 0:
                continue
            m = pmaxPred[k] * ETA_HARV - CONSUMO[sched[k]]
            if m >= 0.0:
                continue
            if m < peor:
                peor, kp = m, k
        if kp < 0 and vn:
            for k in range(N_SLOTS):
                if sched[k] == 0:
                    continue
                m = pmaxPred[k] * ETA_HARV - CONSUMO[sched[k]]
                if m < peor:
                    peor, kp = m, k
        if kp < 0:
            return
        sched[kp] -= 1


# ---------------- Carga (dos archivos, reinicio) ----------------
def load_pd(paths):
    dfs = []
    for p in paths:
        x = pd.read_csv(p, sep=";")
        x.columns = x.columns.str.strip()
        dfs.append(x)
    d = pd.concat(dfs, ignore_index=True)
    d["PLAN"] = d["PLAN"].astype(str).str.strip()
    d["ts"] = pd.to_datetime(d["TIMESTAMP"])
    d["fecha"] = d["ts"].dt.date.astype(str)
    d = d[d["N"] > 3]
    for c in ["SOC_pct", "SOC_ULT_pct", "VBLC_V", "PMAX_EST_mW", "SLOT", "VP_V", "IP_mA"]:
        d[c] = pd.to_numeric(d[c], errors="coerce")
    d = d.sort_values("ts").drop_duplicates(subset=["fecha", "SLOT"], keep="first")
    d = d.head(N_DIAS * 48)                            # 7 dias exactos = 336 slots
    return d.reset_index(drop=True)


# ---------------- Replay del firmware (PD: solo medianoche, sin LAR) ----------------
def correr_gemelo(d):
    pmaxPred = PMAXPRED_DEFAULT.copy()
    filas = []
    soc_fw = None                          # SoC simulado hacia adelante (modelo firmware)
    forced_next = False
    for fecha, day in d.groupby("fecha", sort=True):
        day = day.sort_values("SLOT")
        rows = {int(r.SLOT): r for _, r in day.iterrows()}
        first_k = min(rows)
        if soc_fw is None:                     # UNICO SoC real usado: la condicion inicial
            soc_fw = float(rows[first_k]["SOC_pct"])
        socInicioDia = soc_fw                  # el greedy planifica con el SoC del SIM (no el real)

        # Dia sin medianoche en los datos (parcial, arranca > slot 0): la planificacion
        # de ese dia se hizo antes y NO es reconstruible -> usar el plan real ejecutado.
        usar_real = first_k > 0
        schedule = [0] * N_SLOTS
        if not usar_real:
            greedy(schedule, socInicioDia, pmaxPred, socInicioDia)  # pmaxPred = dia anterior

        for k in sorted(rows):
            r = rows[k]
            if usar_real:
                plan_exe = r["PLAN"]                       # tomar lo que el nodo ejecuto
            else:
                plan_exe = "PF0" if forced_next else NUM_PLAN[schedule[k]]

            lvl = PLAN_IDX[plan_exe]
            soc0 = soc_fw
            soc_fw = soc_forward_cal(soc_fw, float(r["PMAX_EST_mW"]), lvl)
            filas.append({
                "fecha": fecha, "slot": k, "plan_real": r["PLAN"], "plan_sim": plan_exe,
                "soc_real": float(r["SOC_pct"]), "soc_ult": float(r["SOC_ULT_pct"]),
                "soc_sim": (soc0 + soc_fw) / 2.0,          # medio del slot simulado
                "pmax_real": float(r["PMAX_EST_mW"]),
                "ppanel_real": float(r["VP_V"]) * float(r["IP_mA"]),
            })
            pmaxPred[k] = float(r["PMAX_EST_mW"])     # PD: pronostico del dia siguiente = real de hoy
            forced_next = (soc_fw <= SOC_MIN)         # PF0 sobre el SoC SIMULADO (no el real)

    o = pd.DataFrame(filas).reset_index(drop=True)
    o["slot_idx"] = np.arange(len(o))
    o["eje"] = o["plan_real"].map(PLAN_IDX)
    o["sim"] = o["plan_sim"].map(PLAN_IDX)
    return o


# ---------------- Metricas y figuras ----------------
def solo_completos(o):
    """Devuelve solo los dias COMPLETOS (48 slots) para las metricas (sin bordes parciales)."""
    return o[o.groupby("fecha")["slot"].transform("size") == 48].reset_index(drop=True)


def metricas(o):
    print("\n==============================")
    print("PD (Greedy diario) REAL vs SIMULADO (firmware portado)")
    print("==============================")
    print("Slots:", len(o), "| dias:", o["fecha"].nunique())
    print(f"Coincidencia plan ejecutado: {(o['eje'] == o['sim']).mean()*100:.1f}%")
    pf0r = int((o["plan_real"] == "PF0").sum()); pf0s = int((o["plan_sim"] == "PF0").sum())
    print(f"PF0 - real: {pf0r} | sim: {pf0s} | coinciden: {int(((o['plan_real']=='PF0')&(o['plan_sim']=='PF0')).sum())}")
    print("\n--- Calidad media [%] ---")
    print(f"  real: {o['eje'].map(CALIDAD).mean():.2f}   |   sim: {o['sim'].map(CALIDAD).mean():.2f}")
    print("\n--- Consumo a la carga [mWh] ---")
    print(f"  real: {sum(CONSUMO[i] for i in o['eje'])*SLOT_H:.0f}   |   sim: {sum(CONSUMO[i] for i in o['sim'])*SLOT_H:.0f}")
    disp = (o["pmax_real"] * SLOT_H).sum()
    sp_real = np.maximum(0.0, o["pmax_real"] - o["ppanel_real"]).mul(SLOT_H).sum()
    sp_sim = sum(spill_slot(sr, pm, pl) for sr, pm, pl in zip(o["soc_real"], o["pmax_real"], o["sim"]))
    print("\n--- Spillage [mWh] ---")
    print(f"  real (PMAX_EST - Vp*Ip): {sp_real:8.0f}  ({sp_real/disp*100:.1f}%)")
    print(f"  sim  (referido a panel): {sp_sim:8.0f}  ({sp_sim/disp*100:.1f}%)")

    # --- SoC minimo por dia (plataforma/sim vs real) y si cae en el mismo slot ---
    print("\n--- SoC mínimo por día (real vs sim) ---")
    mismo, nd = 0, 0
    for f, day in o.groupby("fecha"):
        rr = day.loc[day["soc_real"].idxmin()]
        ss = day.loc[day["soc_sim"].idxmin()]
        same = int(rr["slot"]) == int(ss["slot"])
        mismo += same; nd += 1
        print(f"  {f}: real {rr['soc_real']:5.1f}% (slot {int(rr['slot'])}) | "
              f"sim {ss['soc_sim']:5.1f}% (slot {int(ss['slot'])}) | mismo slot: {'sí' if same else 'no'}")
    smin_r = o.groupby("fecha")["soc_real"].min().mean()
    smin_s = o.groupby("fecha")["soc_sim"].min().mean()
    print(f"  min en el mismo slot: {mismo}/{nd} días | SoC_min medio: real {smin_r:.1f}%  sim {smin_s:.1f}%")


def guardar_vectores(o, path):
    """Exporta los vectores de plan y SoC (real y sim) a CSV para analisis externo."""
    cols = ["fecha", "slot", "slot_idx", "plan_real", "plan_sim",
            "soc_real", "soc_sim", "soc_ult", "pmax_real"]
    o[cols].to_csv(path, index=False, sep=";", decimal=",")
    print("Vectores CSV:", path)


def matriz(o, save):
    orden = ["P0", "P1", "P2", "P3"]
    m = pd.crosstab(o["eje"].map(NUM_PLAN), o["sim"].map(NUM_PLAN)).reindex(index=orden, columns=orden, fill_value=0)
    print("\n=== MATRIZ (real=filas, sim=columnas) ===")
    print(m.to_string()); print(f"Aciertos: {np.diag(m.values).sum()/m.values.sum()*100:.1f}%")
    fig, ax = plt.subplots(figsize=(5.2, 4.6))
    im = ax.imshow(m.values, cmap="Oranges")
    ax.set_xticks(range(4)); ax.set_xticklabels([PLAN_LATEX[i] for i in range(4)])
    ax.set_yticks(range(4)); ax.set_yticklabels([PLAN_LATEX[i] for i in range(4)])
    ax.set_xlabel("Plan simulado"); ax.set_ylabel("Plan nodo IoT")
    for i in range(4):
        for j in range(4):
            v = m.values[i, j]
            ax.text(j, i, str(v), ha="center", va="center", color="white" if v > m.values.max()/2 else "black")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04); fig.tight_layout()
    fig.savefig(save, bbox_inches="tight")
    fig.savefig(os.path.join(FIG_DIR, "_preview_pd_matriz.png"), dpi=110, bbox_inches="tight")


def plot_planes(o, save):
    fig, ax = plt.subplots(figsize=(13, 5))
    bordes = o.index[o["slot"] == 0].tolist()  # medianoche (00:00), cada 24 h
    for b in bordes:
        ax.axvline(o["slot_idx"].iloc[b] , color="red", ls=":", lw=1.2, alpha=0.8, zorder=1)
    ax.step(o["slot_idx"], o["eje"], where="post", color=C_REAL, lw=1.3, label="Plan nodo IoT")
    ax.step(o["slot_idx"], o["sim"], where="post",  color=C_SIM, lw=1.3, alpha=0.9,
            label="Plan simulado")
    ax.set_yticks([0, 1, 2, 3]); ax.set_yticklabels([PLAN_LATEX[i] for i in range(4)])
    ax.set_xlabel("N° de slot"); ax.set_ylabel("Plan")
    ax.margins(x=0); ax.grid(True, alpha=0.3)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(save, bbox_inches="tight")
    fig.savefig(os.path.join(FIG_DIR, "_preview_pd_planes.png"), dpi=110, bbox_inches="tight")


def plot_soc(o, save):
    fig, ax = plt.subplots(figsize=(13, 5))
    bordes = o.index[o["slot"] == 0].tolist()  # medianoche (00:00), cada 24 h
    for b in bordes:
        ax.axvline(o["slot_idx"].iloc[b], color="red", ls=":", lw=1.2, alpha=0.8, zorder=1)
    ax.plot(o["slot_idx"], o["soc_real"], color=C_REAL, lw=1.8, label="SoC real")
    ax.plot(o["slot_idx"], o["soc_sim"], color=C_SIM, lw=1.8, label="SoC simulado")
    ax.axhline(SOC_MIN, ls="--", color="gray", lw=1.3, label=f"SoC mín ({SOC_MIN:.0f}%)")
    ax.set_xlabel("N° de slot"); ax.set_ylabel("SoC [%]")
    ax.margins(x=0); ax.grid(True, alpha=0.3)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=3, frameon=False)
    fig.tight_layout()
    fig.savefig(save, bbox_inches="tight")
    fig.savefig(os.path.join(FIG_DIR, "_preview_pd_soc.png"), dpi=110, bbox_inches="tight")


if __name__ == "__main__":
    d = load_pd(CSV_PATHS)
    o = correr_gemelo(d)
    om = solo_completos(o) if METRICAS_SOLO_COMPLETOS else o   # segun el flag
    print(f"[metricas sobre {om['fecha'].nunique()} dias ({sorted(om['fecha'].unique())}) "
          f"| figuras/CSV sobre {len(o)} slots (7 dias)]")
    metricas(om)
    matriz(om, os.path.join(FIG_DIR, "pd_matriz_coincidencias.pdf"))
    plot_planes(o, os.path.join(FIG_DIR, "pd_planes_real_vs_sim.pdf"))     # figuras: 7 dias
    plot_soc(o, os.path.join(FIG_DIR, "pd_soc_real_vs_sim.pdf"))
    guardar_vectores(o, os.path.join(BASE_DIR, "pd_vectores.csv"))
    print("\nFiguras en:", FIG_DIR)
