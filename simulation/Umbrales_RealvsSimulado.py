import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ===================== CONFIGURACIÓN =====================

# Rutas relativas al propio script (portable Windows/Linux)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(BASE_DIR, "figuras")
os.makedirs(FIG_DIR, exist_ok=True)

CSV_PATH = os.path.join(BASE_DIR, "UMBRALES_ARCHIVO1_260601.CSV")
OUT_PATH = os.path.join(BASE_DIR, "comparacion_real_vs_simulado_umbrales.csv")
out_dir = os.path.dirname(OUT_PATH)

COL_PLAN_REAL = "Plan"
COL_SOC_REAL = "SoC_pct"
COL_PDISP = "Pmax_est_mW"     # OJO: en el CSV es Voc*Isc (rectangulo ideal, SIN FF)
COL_PPANEL = "Ppanel_mW"      # potencia realmente cosechada del panel
COL_VOC = "Voc_V"             # tension de circuito abierto (caracterizacion IV)
COL_ISC = "Isc_mA"            # corriente de cortocircuito (caracterizacion IV)

# Fill factor del panel (del codigo Arduino: const float fillFactor = 0.75f).
# El MPP real disponible = Voc * Isc * FILL_FACTOR (Pmax_est del CSV NO lo aplica).
FILL_FACTOR = 0.75

# Desfasaje del registro real respecto al simulado (medido: -1 = 1 slot).
# El dato guardado corresponde a la decision del slot anterior.
SHIFT_REAL_SLOTS = -1

SLOT_HOURS = 0.5

BAT_CAPACITY_MAH = 130
BAT_VOLTAGE_V = 3.7
BAT_ENERGY_MWH = BAT_CAPACITY_MAH * BAT_VOLTAGE_V

SOC_INITIAL = None
SOC_MIN = 20.0
SOC_MAX = 100.0

ETA_HARV = 0.70
ETA_BAT_CHARGE = 0.95

MAX_CHARGE_C_RATE = 0.2
MAX_CHARGE_CURRENT_MA = BAT_CAPACITY_MAH * MAX_CHARGE_C_RATE
MAX_CHARGE_POWER_MW = MAX_CHARGE_CURRENT_MA * BAT_VOLTAGE_V

PLANS = {
    "P0": {"power_mW": 13.40,  "quality": 5.4},
    "P1": {"power_mW": 25.49,  "quality": 7.2},
    "P2": {"power_mW": 66.93,  "quality": 76.4
           },
    "P3": {"power_mW": 112.03, "quality": 100},
}

THRESHOLDS = {
    "P0": 32,
    "P1": 40,
    "P2": 42,
}

PLAN_TO_NUM = {"P0": 0, "P1": 1, "P2": 2, "P3": 3}

plt.rcParams.update({
    "font.size": 14,
    "axes.titlesize": 16,
    "axes.labelsize": 15,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
    "legend.fontsize": 13,
})


# ===================== FUNCIONES =====================

def load_real_data(path):
    df = pd.read_csv(path, sep="\t")
    df.columns = df.columns.str.strip()

    for col in [COL_PLAN_REAL, COL_SOC_REAL, COL_PDISP, COL_PPANEL, COL_VOC, COL_ISC]:
        if col not in df.columns:
            print("Columnas disponibles:")
            print(df.columns.tolist())
            raise KeyError(f"No se encontró la columna: {col}")

    df[COL_SOC_REAL] = pd.to_numeric(df[COL_SOC_REAL], errors="coerce")
    df[COL_PDISP] = pd.to_numeric(df[COL_PDISP], errors="coerce") / 1000.0
    df[COL_PPANEL] = pd.to_numeric(df[COL_PPANEL], errors="coerce") / 1000.0
    df[COL_VOC] = pd.to_numeric(df[COL_VOC], errors="coerce")            # V
    df[COL_ISC] = pd.to_numeric(df[COL_ISC], errors="coerce") / 1000.0   # uA -> mA

    df = df.dropna(subset=[COL_SOC_REAL, COL_PDISP, COL_PLAN_REAL]).copy()
    df = df.reset_index(drop=True)
    df["slot_index"] = np.arange(len(df))

    return df


def choose_plan_threshold(soc):
    if soc < THRESHOLDS["P0"]:
        return "P0"
    if soc < THRESHOLDS["P1"]:
        return "P1"
    if soc < THRESHOLDS["P2"]:
        return "P2"
    return "P3"


def simulate_thresholds_from_real_pdisp(df):
    soc = float(df[COL_SOC_REAL].iloc[0]) if SOC_INITIAL is None else SOC_INITIAL
    results = []

    for _, row in df.iterrows():
        pmax_raw_mW = float(row[COL_PDISP])   # Pmax_est del CSV = Voc*Isc (sin FF)
        ppanel_real_mW = float(row[COL_PPANEL])
        soc_real = float(row[COL_SOC_REAL])
        plan_real = str(row[COL_PLAN_REAL]).strip()

        plan_sim = choose_plan_threshold(soc)
        pcons_mW = PLANS[plan_sim]["power_mW"]
        quality = PLANS[plan_sim]["quality"]

        # --- Potencia disponible en el MPP real (comun a real y simulado) ---
        # = Voc*Isc*FF. Es previa a la UGC/conversion, por eso la usan ambos.
        voc = float(row[COL_VOC])
        isc_mA = float(row[COL_ISC])
        p_mpp_real_mW = voc * isc_mA * FILL_FACTOR
        pdisp_mW = p_mpp_real_mW              # el "mismo Pmax" que usa la simulacion

        # --- Metricas del sistema REAL (nodo A) ---
        # Calidad real: plan realmente ejecutado, misma escala que el simulado.
        quality_real = PLANS[plan_real]["quality"] if plan_real in PLANS else np.nan
        # Spillage real: disponible en el MPP pero no cosechado (bateria llena/limitada).
        spillage_real_mW = max(0.0, p_mpp_real_mW - ppanel_real_mW)
        spillage_real_mWh = spillage_real_mW * SLOT_HOURS

        # Simulacion: parte del mismo P_max (con FF) y aplica solo ETA (conversion).
        p_panel_mW = pdisp_mW * ETA_HARV
        p_excedente_mW = p_panel_mW - pcons_mW

        e_panel_mWh = p_panel_mW * SLOT_HOURS
        e_cons_mWh = pcons_mW * SLOT_HOURS

        if p_excedente_mW >= 0:
            p_entra_bat_mW = min(p_excedente_mW, MAX_CHARGE_POWER_MW)

            spillage_power_mW = max(0, p_excedente_mW - MAX_CHARGE_POWER_MW)
            spillage_mWh = spillage_power_mW * SLOT_HOURS

            e_entra_bat_mWh = p_entra_bat_mW * SLOT_HOURS * ETA_BAT_CHARGE
            e_sale_bat_mWh = 0.0
        else:
            e_entra_bat_mWh = 0.0
            e_sale_bat_mWh = abs(p_excedente_mW) * SLOT_HOURS
            spillage_mWh = 0.0

        soc_next_raw = soc + ((e_entra_bat_mWh - e_sale_bat_mWh) / BAT_ENERGY_MWH) * 100.0
        soc_next = min(max(soc_next_raw, 0.0), SOC_MAX)

        deficit_mWh = 0.0
        if soc_next_raw < SOC_MIN:
            deficit_mWh = ((SOC_MIN - soc_next_raw) / 100.0) * BAT_ENERGY_MWH

        results.append({
            "slot_index": int(row["slot_index"]),
            "Pmax_raw_mW": pmax_raw_mW,     # Voc*Isc (sin FF), referencia
            "Pdisp_mW": pdisp_mW,           # Voc*Isc*FF (el usado por real y sim)
            "Plan_real": plan_real,
            "Plan_sim": plan_sim,
            "Plan_real_num": PLAN_TO_NUM.get(plan_real, np.nan),
            "Plan_sim_num": PLAN_TO_NUM.get(plan_sim, np.nan),
            "SoC_real": soc_real,
            "SoC_sim": soc,
            "SoC_sim_next": soc_next,
            "Pcons_sim_mW": pcons_mW,
            "quality_sim": quality,
            "E_panel_mWh": e_panel_mWh,
            "E_cons_mWh": e_cons_mWh,
            "E_entra_bat_mWh": e_entra_bat_mWh,
            "E_sale_bat_mWh": e_sale_bat_mWh,
            "deficit_mWh": deficit_mWh,
            "match_plan": plan_real == plan_sim,
            "spillage_mWh": spillage_mWh,
            # --- reales ---
            "Ppanel_real_mW": ppanel_real_mW,
            "P_mpp_real_mW": p_mpp_real_mW,
            "quality_real": quality_real,
            "spillage_real_mWh": spillage_real_mWh,
        })

        soc = soc_next

    res = pd.DataFrame(results)

    # Comparación sin alinear
    res["SoC_error"] = res["SoC_sim"] - res["SoC_real"]
    res["SoC_abs_error"] = res["SoC_error"].abs()

    # Comparación alineada: el real guardado va desfasado SHIFT_REAL_SLOTS slots
    # respecto al simulado (medido: -1 es el mejor -> coincidencia y MAE optimos).
    res["SoC_real_aligned"] = res["SoC_real"].shift(SHIFT_REAL_SLOTS)
    res["Plan_real_aligned"] = res["Plan_real"].shift(SHIFT_REAL_SLOTS)
    res["Plan_real_aligned_num"] = res["Plan_real_aligned"].map(PLAN_TO_NUM)

    res["SoC_error_aligned"] = res["SoC_sim"] - res["SoC_real_aligned"]
    res["SoC_abs_error_aligned"] = res["SoC_error_aligned"].abs()
    res["match_plan_aligned"] = res["Plan_real_aligned"] == res["Plan_sim"]

    res["quality_weighted"] = res["quality_sim"] * SLOT_HOURS
    res["quality_acc"] = res["quality_weighted"].cumsum()

    # Calidad real (plan ejecutado real), misma escala -> comparable
    res["quality_real_weighted"] = res["quality_real"] * SLOT_HOURS
    res["quality_real_acc"] = res["quality_real_weighted"].cumsum()

    # Spillage acumulado (real y simulado)
    res["spillage_acc"] = res["spillage_mWh"].cumsum()
    res["spillage_real_acc"] = res["spillage_real_mWh"].cumsum()

    return res


def print_summary(res):
    print("\n==============================")
    print("COMPARACIÓN REAL vs SIMULADO - UMBRALES")
    print("==============================")

    print("Slots comparados:", len(res))
    print("\n--- Sin alinear ---")
    print("Coincidencia planes [%]:", res["match_plan"].mean() * 100)
    print("MAE SoC [%]:", res["SoC_abs_error"].mean())
    print("RMSE SoC [%]:", np.sqrt((res["SoC_error"] ** 2).mean()))

    print("\n--- Alineado un slot ---")
    print("Coincidencia planes [%]:", res["match_plan_aligned"].mean() * 100)
    print("MAE SoC [%]:", res["SoC_abs_error_aligned"].mean())
    print("RMSE SoC [%]:", np.sqrt((res["SoC_error_aligned"] ** 2).mean()))

    print("\nSoC real inicial:", res["SoC_real"].iloc[0])
    print("SoC real final:", res["SoC_real"].iloc[-1])
    print("SoC sim final:", res["SoC_sim_next"].iloc[-1])

    print("\n--- Calidad ---")
    print("Calidad REAL total     :", res["quality_real_weighted"].sum())
    print("Calidad simulada total :", res["quality_weighted"].sum())
    print("Calidad REAL media [%] :", res["quality_real"].mean())
    print("Calidad sim  media [%] :", res["quality_sim"].mean())

    print("\n--- Spillage ---")
    sp_real = res["spillage_real_mWh"].sum()
    sp_sim = res["spillage_mWh"].sum()
    e_disp = (res["P_mpp_real_mW"] * SLOT_HOURS).sum()   # disponible en MPP real (Voc*Isc*FF)
    print(f"(disponible real MPP = Voc*Isc*FF, FF={FILL_FACTOR})")
    print("Spillage REAL total [mWh]     :", sp_real)
    print("Spillage simulado total [mWh] :", sp_sim)
    if e_disp > 0:
        print("Spillage REAL [% del disponible]:", sp_real / e_disp * 100.0)

    print("\nDéficit total simulado [mWh]:", res["deficit_mWh"].sum())

    print("\nPlanes reales:")
    print(res["Plan_real"].value_counts().sort_index())

    print("\nPlanes simulados:")
    print(res["Plan_sim"].value_counts().sort_index())

    print("\nPlan real alineado vs Plan simulado:")
    print(pd.crosstab(res["Plan_real_aligned"], res["Plan_sim"]))


def plot_soc_comparison(res):
    plt.figure(figsize=(13, 5))

    plt.plot(
    res["slot_index"],
    res["SoC_real"],
    label="SoC real guardado",
    linewidth=1.5,
    color="#1f77b4"
)
    plt.plot(res["slot_index"], res["SoC_sim"], "--", label="SoC simulado", linewidth=1.5, color="#8b23cf")

    plt.axhline(SOC_MIN, linestyle=":", linewidth=1, color="gray", label="SoC mínimo")

    plt.xlabel("N° de slot")
    plt.ylabel("SoC [%]")
    plt.title("SoC real guardado vs SoC simulado")
    plt.margins(x=0)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    #plt.savefig("/home/melisa/Documentos/Python/Doctorado/Capitulo Algoritmos/Algoritmos_simulados/figura_SoC_TB.pdf", bbox_inches="tight")
    plt.show()



def plot_soc_comparison_aligned(res):
    plt.figure(figsize=(13, 5))

    plt.plot(res["slot_index"], res["SoC_real_aligned"], label="SoC real", linewidth=2, color="#1f77b4")
    plt.plot(res["slot_index"], res["SoC_sim"], "--", label="SoC simulado", linewidth=2, color="#e21212")

    plt.axhline(SOC_MIN, linestyle=":", linewidth=2, color="gray", label="SoC mínimo")

    plt.xlabel("N° de slot")
    plt.ylabel("SoC [%]")
    plt.margins(x=0)
    #plt.title("SoC real vs SoC simulado")
    plt.grid(True, alpha=0.3)
    plt.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 1.02),
    ncol=3,
    frameon=False
    )
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "figura_SoC_TB.pdf"), bbox_inches="tight")
    plt.show()


def plot_plan_comparison_aligned(res):
    plt.figure(figsize=(13, 4))
    plt.margins(x=0)
    plt.step(
        res["slot_index"],
        res["Plan_real_aligned_num"],
        where="post",
        label="Planificación real",
        linewidth=2, 
        color = "#1f77b4"
    )

    plt.step(
        res["slot_index"],
        res["Plan_sim_num"],
        where="post",
        linestyle="--",
        label="Plan simulado",
        linewidth=2,
        alpha=0.8,
        color="#e21212"
    )

    plt.yticks([0, 1, 2, 3],
           [r"$\phi_0$", r"$\phi_1$", r"$\phi_2$", r"$\phi_3$"])
    plt.xlabel("N° de slot")
    plt.ylabel("Plan")
    #plt.title("Planificación real vs simulada")
    plt.grid(True, alpha=0.3)
    plt.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 1.02),
    ncol=2,
    frameon=False
    )
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "figura_plan_TB.pdf"), bbox_inches="tight")
    plt.show()

def plot_quality_acc(res):
    plt.figure(figsize=(13, 4))

    plt.plot(res["slot_index"], res["quality_real_acc"],
             label="Calidad acumulada REAL", linewidth=2, color="#1f77b4")
    plt.plot(res["slot_index"], res["quality_acc"], "--",
             label="Calidad acumulada simulada", linewidth=2, color="#e21212")

    plt.xlabel("N° de slot")
    plt.ylabel("Calidad acumulada")
    plt.margins(x=0)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "figura_calidad_TB.pdf"), bbox_inches="tight")
    plt.show()

def plot_spillage(res):
    plt.figure(figsize=(13, 4))

    plt.plot(res["slot_index"], res["spillage_real_acc"],
             label="Spillage acumulado REAL", linewidth=2, color="#1f77b4")
    plt.plot(res["slot_index"], res["spillage_acc"], "--",
             label="Spillage acumulado simulado", linewidth=2, color="#e21212")

    plt.xlabel("N° de slot")
    plt.ylabel("Spillage acumulado [mWh]")
    plt.margins(x=0)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "figura_spillage_TB.pdf"), bbox_inches="tight")
    plt.show()

def plot_spillage_slot(res):
    """Spillage real por slot (barras), por si querés ver la distribución."""
    plt.figure(figsize=(13, 4))
    plt.bar(res["slot_index"], res["spillage_real_mWh"], label="Spillage real por slot")
    plt.xlabel("N° de slot")
    plt.ylabel("Spillage [mWh]")
    plt.margins(x=0)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_soc_error_aligned(res):
    plt.figure(figsize=(13, 4))

    plt.plot(
        res["slot_index"],
        res["SoC_error_aligned"],
        label="Error SoC alineado = simulado - real"
    )

    plt.axhline(0, color="black", linewidth=1)

    plt.xlabel("Slot de 30 min")
    plt.ylabel("Error SoC [%]")
    plt.title("Error de SoC alineado")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_power_balance(res):

    plt.figure(figsize=(14,5))

    plt.plot(
        res["slot_index"],
        res["Pmax_real_mW"],
        label="Pmax real",
        linewidth=2
    )

    plt.plot(
        res["slot_index"],
        res["Pnodo_real_mW"],
        label="Pnodo real",
        linewidth=2
    )

    plt.plot(
        res["slot_index"],
        np.abs(res["Pbatt_real_mW"]),
        label="|Pbatt real|",
        linewidth=2
    )

    plt.xlabel("Slot")
    plt.ylabel("Potencia [mW]")
    plt.title("Potencias medidas")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

# ===================== MAIN =====================

df_real = load_real_data(CSV_PATH)

res = simulate_thresholds_from_real_pdisp(df_real)

res["Pmax_real_mW"] = pd.to_numeric(
    df_real["Pmax_est_mW"],
    errors="coerce"
) / 1000.0

res["Pnodo_real_mW"] = pd.to_numeric(
    df_real["Pnodo_mW"],
    errors="coerce"
) / 1000.0

res["Pbatt_real_mW"] = pd.to_numeric(
    df_real["Pbatt_mW"],
    errors="coerce"
) / 1000.0

print_summary(res)

res.to_csv(OUT_PATH, index=False)
    
#plot_soc_comparison(res)
plot_soc_comparison_aligned(res)
plot_plan_comparison_aligned(res)

#plot_power_balance(res)
#plot_soc_error_aligned(res)
plot_quality_acc(res)       # calidad acumulada REAL vs simulada
plot_spillage(res)          # spillage acumulado REAL vs simulado
#plot_spillage_slot(res)    # spillage real por slot (barras)

print("\nArchivo generado:")
print(OUT_PATH)
print("Spillage REAL total [mWh]     :", res["spillage_real_mWh"].sum())
print("Spillage simulado total [mWh] :", res["spillage_mWh"].sum())
print("Calidad REAL total            :", res["quality_real_weighted"].sum())
print("Calidad simulada total        :", res["quality_weighted"].sum())