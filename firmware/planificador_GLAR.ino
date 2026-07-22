// ============================================================
// planificador_GLAR.ino
// Nodo EH3 - Planificador energético Greedy + LAR por slots de 30 min
// Hardware: Adafruit Feather M0 LoRa (SAMD21) + PCF8523 + SH1107
//           + INA219 x3 + LC709203F + DS18B20 + SD
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <RadioLib.h>
#include <ArduinoLowPower.h>
#include <Adafruit_INA219.h>
#include <Adafruit_LC709203F.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "tipos_planificador.h"  // enums y structs antes de los prototipos auto-generados

// ====================== IDENTIFICACIÓN ======================
const char* NODE_ID = "EH04";          // identificador único del nodo

// ====================== PINES ======================
#define DS18B20_PIN      A1            // sensor de temperatura
#define RTC_INT_PIN      A2            // interrupción del RTC para despertar el MCU
#define LED_PIN          LED_BUILTIN

#define LORA_CS          8
#define LORA_RST         4
#define LORA_IRQ         3
#define LORA_FREQ        433.0         // frecuencia LoRa [MHz]

#define INA_PANEL_ADDR   0x40          // INA219 del panel solar
#define INA_CARGA_ADDR   0x41          // INA219 de la carga/nodo
#define INA_BAT_ADDR     0x45          // INA219 de la batería

#define SD_CS            10
#define RELAY_SET_PIN    11
#define RELAY_UNSET_PIN  6
#define MOSFET_PIN       12

// ====================== BATERÍA (elegir UNA opción) ======================
// Opciones disponibles: 150, 550, 680, 800 mAh.
// Cambiando SOLO este número se ajustan BAT_ENERGY_MWH, MAX_CHARGE_MW y el APA del gauge.
#define BATERIA_MAH   150

#if   BATERIA_MAH == 150
  #define BAT_APA  0x0A
#elif BATERIA_MAH == 550
  #define BAT_APA  0x1B
#elif BATERIA_MAH == 680
  #define BAT_APA  0x1F
#elif BATERIA_MAH == 800
  #define BAT_APA  0x22
#else
  #error "BATERIA_MAH debe ser 150, 550, 680 u 800"
#endif

// ====================== MODELO ENERGÉTICO ======================
const float BAT_ENERGY_MWH     = BATERIA_MAH * 3.7f;        // capacidad total [mWh]
const float ETA_HARV           = 0.71f;   // eficiencia real de cosecha (GLAR, con UGC)
const float ETA_CARGA          = 0.95f;   // eficiencia de carga de la batería
const float MAX_CHARGE_MW      = BATERIA_MAH * 0.2f * 3.7f; // límite de carga C/5 [mW]
const float SOC_MIN            = 20.0f;   // SoC mínimo permitido [%]
const float SOC_NEUTRAL_MAX    = 50.0f;   // tope del objetivo de neutralidad [%]
                                           // = SOC_MIN + 12h de autonomia en P0 sin produccion
const float SOC_MAX            = 100.0f;  // SoC máximo [%]
const float SLOT_HORAS         = 0.5f;    // duración del slot [h]
const float VBAT_EMERGENCIA    = 3.50f;   // tensión de batería crítica: PF0 + salteo de Voc/Isc [V]
const float VPANEL_MIN_MEDICION = 2.0f;   // VP mínimo para que valga la pena medir Voc/Isc [V]

// Consumo promedio de cada plan durante un slot de 30 min [mW]
// Incluye MCU, sensores, radio y display, ponderado por tiempo activo/sleep
// P0=bajo consumo, P1=muestreo 1min, P2=P1+LoRa RX, P3=P2+OLED
const float CONSUMO_PLAN_MW[4] = {6.7f, 9.0f, 95.7f, 125.2f};  // GLAR real medido (P0,P1,P2,P3) [mW]

// ====================== PLANIFICACIÓN ======================
#define N_SLOTS          48    // slots por día (48 × 30 min = 24 h)
#define MAX_ITER_GREEDY  200   // iteraciones máximas del Greedy (144 es el máximo teórico)
#define MAX_SLEEP_SEC    120   // tope de seguridad para el sleep: coincide con P0 (2 min)

// ====================== PARÁMETROS GLAR ======================
const float ALPHA_EWMA = 0.828f;  // factor de suavizado EWMA (calibrado experimentalmente)
const float BETA_LAR   = 0.673f;  // ganancia de corrección auto-regresiva LAR

// Intervalo de muestreo según plan
#define SAMPLE_P0_SEC    120   // P0: muestreo cada 2 minutos
#define SAMPLE_SEC       60    // P1/P2/P3: muestreo cada 1 minuto

// ====================== PLANES ======================
// enum PlanEnergetico definido en tipos_planificador.h

// Nombre del plan como cadena (para Serial y SD)
const char* nombrePlan(PlanEnergetico p) {
  switch (p) {
    case PLAN_P0: return "P0";
    case PLAN_P1: return "P1";
    case PLAN_P2: return "P2";
    case PLAN_P3: return "P3";
    default:      return "PX";
  }
}

// Calidad de servicio del plan (métrica para log)
uint8_t calidadPlan(PlanEnergetico p) {
  switch (p) {
    case PLAN_P0: return 10;
    case PLAN_P1: return 20;
    case PLAN_P2: return 60;
    case PLAN_P3: return 100;
    default:      return 0;
  }
}

// ====================== VECTOR DE PREDICCIÓN ======================
// Pmax_est [mW] de un día conocido como predicción inicial.
// Valores < 6 mW zeroed (ruido nocturno).
// Se sobreescribe slot a slot con mediciones reales del día en curso.
// Al llegar a medianoche, contiene el día completo anterior → predicción del día nuevo.
// Valores iniciales: perfil real medido en este nodo (casa, jun-2026) [mW].
// Mañana sin sol directo hasta ~09:30 (slot 19); pico ~690 mW al mediodia.
// Se persiste en SD (PLAN.CSV linea 3) y se refina con EWMA; este array es solo
// el arranque del primer dia o tras perder el backup de SD.
float pmaxPred[N_SLOTS] = {
  0.020f,    0.020f,    0.020f,    0.020f,    // 00:00 - 01:30
  0.020f,    0.020f,    0.020f,    0.020f,    // 02:00 - 03:30
  0.020f,    0.020f,    0.020f,    0.030f,    // 04:00 - 05:30
  0.080f,    0.300f,    3.000f,    8.000f,    // 06:00 - 07:30
  14.000f,   30.000f,   45.000f,   280.000f,  // 08:00 - 09:30
  330.000f,  470.000f,  555.000f,  605.000f,  // 10:00 - 11:30
  635.000f,  655.000f,  650.000f,  690.000f,  // 12:00 - 13:30
  690.000f,  630.000f,  590.000f,  585.000f,  // 14:00 - 15:30
  555.000f,  540.000f,  335.000f,  290.000f,  // 16:00 - 17:30
  260.000f,  125.000f,  35.000f,   8.000f,    // 18:00 - 19:30
  2.000f,    0.500f,    0.100f,    0.050f,    // 20:00 - 21:30
  0.020f,    0.020f,    0.020f,    0.020f     // 22:00 - 23:30
};

// Mediciones reales de Pmax por slot durante el dia en curso [mW]
// Se usa a medianoche para actualizar pmaxPred con EWMA.
// -1.0f = sin medicion (slot no ejecutado aun o primer arranque parcial)
float pmaxMedido[N_SLOTS];

// Plan asignado por el Greedy a cada slot (0=P0 .. 3=P3)
// Se inicializa en P0 hasta que el Greedy calcule o se recupere de SD
uint8_t schedule[N_SLOTS];

// ====================== VARIABLES DE TRAZABILIDAD LAR ======================
// Se actualizan en cada replanificacion por slot; se guardan en SD
float           errorLAR  = 0.0f;   // error del slot anterior: e_d(i-1) = pReal - pPred
float           pHatLAR   = 0.0f;   // prediccion corregida LAR para el slot siguiente
PlanEnergetico  planAntes = PLAN_P0; // plan asignado ANTES de replanificar
PlanEnergetico  planNuevo = PLAN_P0; // plan asignado DESPUES de replanificar

// Energía desperdiciada por slot en la simulación [mWh]
float spillageSlot[N_SLOTS];

// SoC simulado al final de cada slot en la planificación [%]
float socSlot[N_SLOTS];

// ====================== OBJETOS HARDWARE ======================
RTC_PCF8523         rtc;
Adafruit_LC709203F  lc;
Adafruit_SSD1306    display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_INA219     inaPanel(INA_PANEL_ADDR);
Adafruit_INA219     inaCarga(INA_CARGA_ADDR);
Adafruit_INA219     inaBat(INA_BAT_ADDR);
SX1276              radio = new Module(LORA_CS, LORA_IRQ, LORA_RST);
OneWire             oneWire(DS18B20_PIN);
DallasTemperature   sensors(&oneWire);
File                dataFile;

// ====================== FLAGS DE HARDWARE ======================
bool rtcOK      = false;
bool lcOK       = false;
bool oledOK     = false;
bool loraOK     = false;
bool sdOK       = false;
bool inaPanelOK = false;
bool inaCargaOK = false;
bool inaBatOK   = false;

// ====================== VARIABLES GLOBALES ======================
volatile bool wakeRTC     = false;     // flag de interrupción del RTC
uint8_t       slotActual  = 255;       // slot en curso (255 = no inicializado)
DateTime      inicioSlotRTC;           // hora de inicio del slot en curso
AcumuladorSlot acc;                    // acumulador del slot en curso (declarado aqui para que enviarLoRa lo vea)
bool          planForzado = false;     // true si el plan actual fue forzado a P0 por emergencia de SoC
PlanEnergetico planActual = PLAN_P0;   // plan activo en el slot corriente
float         socInicioDia = 50.0f;    // SoC al arrancar el día: referencia de neutralidad SoC(0)

// Etiqueta a mostrar/registrar: distingue P0 planificado de P0 forzado por emergencia de SoC
const char* etiquetaPlan() {
  if (planForzado && planActual == PLAN_P0) return "PF0";
  return nombrePlan(planActual);
}
uint32_t      packetID    = 0;         // ID de paquete, incrementa cada slot
uint32_t      lastSampleUnix = 0;      // unix del último muestreo
uint32_t      lastLoRaUnix   = 0;      // unix del último TX LoRa
bool          relayEstado    = false;
char          filename[13];            // nombre del archivo CSV de datos

// ====================== MEDICIONES ======================
// struct Mediciones definido en tipos_planificador.h

Mediciones m;
float temperatura = 0.0f;
float Voc         = NAN;
float Isc         = NAN;
float Vshort      = NAN;
float Pmax_est    = NAN;
const float fillFactor = 0.75f;  // factor de forma del panel

// ====================== ESTRUCTURAS DEL PLANIFICADOR ======================
// ResultadoSlot y ResultadoDia definidos en tipos_planificador.h

// ====================== ISR ======================
void despertarRTC() {
  wakeRTC = true;  // solo levantar la bandera; el trabajo se hace en el loop
}

// ====================== SIMULACIÓN ENERGÉTICA ======================

// Simula el balance energético de un slot de 30 min usando promedio por slot
// Entradas: SoC inicial [%], Pmax predicha [mW], plan asignado
// Salida:   ResultadoSlot con SoC final, mínimo, spillage y flag de violación
ResultadoSlot simularSlot(float socIni, float pmaxPred_val, PlanEnergetico plan) {

  float pPanel = pmaxPred_val * ETA_HARV;         // potencia disponible tras harvesting
  float pNeto  = pPanel - CONSUMO_PLAN_MW[plan];  // balance: generación - consumo del plan

  // Lo que no consume el nodo va a la batería con la pérdida de carga correspondiente
  if (pNeto > 0.0f) pNeto *= ETA_CARGA;

  // Spillage por límite de C-rate: no cargar más rápido de MAX_CHARGE_MW
  float spillage_crate = 0.0f;
  if (pNeto > MAX_CHARGE_MW) {
    spillage_crate = (pNeto - MAX_CHARGE_MW) * SLOT_HORAS;  // energía que no puede entrar [mWh]
    pNeto = MAX_CHARGE_MW;                                    // recortar al límite
  }

  // Calcular nuevo SoC con el balance neto del slot
  float socNext = socIni + (pNeto * SLOT_HORAS / BAT_ENERGY_MWH * 100.0f);

  // Spillage por batería llena: no superar SOC_MAX
  float spillage_bat = 0.0f;
  if (socNext > SOC_MAX) {
    float eReal  = (SOC_MAX - socIni) / 100.0f * BAT_ENERGY_MWH;  // energía que realmente entró
    spillage_bat = pNeto * SLOT_HORAS - eReal;                      // excedente no almacenado
    socNext      = SOC_MAX;
  }

  // Evitar SoC negativo
  if (socNext < 0.0f) socNext = 0.0f;

  // Con promedio por slot, el mínimo es el extremo inferior del balance
  float socMin = (socNext < socIni) ? socNext : socIni;

  ResultadoSlot r;
  r.soc_next     = socNext;
  r.soc_min      = socMin;
  r.spillage_mWh = spillage_crate + spillage_bat;
  r.violation    = (socMin < SOC_MIN);
  return r;
}

// Simula el día completo con el schedule dado
// Llena socSlot[] y spillageSlot[] slot a slot
// Retorna ResultadoDia distinguiendo el tipo de violación
// slotInicio: primer slot a simular (0 para el dia completo, >0 para replanificacion parcial)
// socObjetivo: SoC minimo que debe tener el horizonte simulado al finalizar (neutralidad)
ResultadoDia simularDia(float socIni, uint8_t sched[], float socObjetivo, uint8_t slotInicio = 0) {
  float soc = socIni;
  ResultadoDia res = {false, false};

  for (uint8_t k = slotInicio; k < N_SLOTS; k++) {
    ResultadoSlot r = simularSlot(soc, pmaxPred[k], (PlanEnergetico)sched[k]);

    socSlot[k]      = r.soc_next;      // SoC simulado al final del slot k
    spillageSlot[k] = r.spillage_mWh;  // energía desperdiciada en el slot k

    if (r.violation) res.violacionSoC = true;

    soc = r.soc_next;
  }

  // Neutralidad: el horizonte simulado debe terminar con al menos el objetivo
  if (soc < socObjetivo) res.violacionNeutral = true;

  return res;
}

// ====================== GREEDY ======================

// Degrada el plan un nivel (P3→P2→P1→P0); en P0 no hace nada
PlanEnergetico degradarPlan(PlanEnergetico p) {
  if (p == PLAN_P0) return PLAN_P0;
  return (PlanEnergetico)(p - 1);
}

// Algoritmo Greedy: arranca con todos los slots en P3 y degrada iterativamente
// el slot con peor margen energético hasta que el día sea factible.
// Modifica schedule[] directamente.
// Retorna true si encontró un plan factible, false si operó en mejor esfuerzo.
bool planificarGreedy(float socIni) {

  // Objetivo de neutralidad: no exigir más SoC final que SOC_NEUTRAL_MAX
  float socObjetivo = (socIni > SOC_NEUTRAL_MAX) ? SOC_NEUTRAL_MAX : socIni;

  // Inicializar con el plan máximo: se irá degradando según necesidad
  for (uint8_t k = 0; k < N_SLOTS; k++) schedule[k] = PLAN_P3;

  for (uint16_t iter = 0; iter < MAX_ITER_GREEDY; iter++) {

    ResultadoDia res = simularDia(socIni, schedule, socObjetivo);

    // Sin violaciones: plan factible, terminar
    if (!res.violacionSoC && !res.violacionNeutral) {
      Serial.print("Greedy OK en "); Serial.print(iter); Serial.println(" iter");
      return true;
    }

    // Buscar el slot con peor margen que todavía se pueda degradar.
    // Fase 1: solo slots en déficit (margen < 0), para no aumentar spillage
    // degradando horarios donde ya sobra energía.
    float  peorMargen = 1e9f;  // inicializar con valor muy alto
    int8_t kPeor      = -1;

    for (uint8_t k = 0; k < N_SLOTS; k++) {
      if (schedule[k] == PLAN_P0) continue;  // ya en mínimo, no degradable

      // Margen = energía harvested - consumo del plan actual en ese slot
      float margen = pmaxPred[k] * ETA_HARV - CONSUMO_PLAN_MW[schedule[k]];
      if (margen >= 0.0f) continue;  // slot con superávit: no degradar en fase 1

      if (margen < peorMargen) {
        peorMargen = margen;
        kPeor      = k;
      }
    }

    // Fase 2: degradar slots de superávit SOLO para corregir neutralidad final.
    // Para una violacion de SoC, degradar superavit no sirve (la bateria carga
    // igual) y solo tira trabajo util: si hay sol, hay que intentar funcionar.
    if (kPeor < 0 && res.violacionNeutral) {
      for (uint8_t k = 0; k < N_SLOTS; k++) {
        if (schedule[k] == PLAN_P0) continue;

        float margen = pmaxPred[k] * ETA_HARV - CONSUMO_PLAN_MW[schedule[k]];

        if (margen < peorMargen) {
          peorMargen = margen;
          kPeor      = k;
        }
      }
    }

    // Si no hay ningún slot degradable, no se puede mejorar más
    if (kPeor < 0) {
      Serial.println("Greedy: sin slots degradables");
      return false;
    }

    // Degradar el slot con peor margen un nivel
    schedule[kPeor] = (uint8_t)degradarPlan((PlanEnergetico)schedule[kPeor]);
  }

  Serial.println("Greedy: max iter alcanzadas");
  return false;
}

// Variante del Greedy que replantifica solo desde slotInicio hasta el final del dia.
// Los slots ya ejecutados (0..slotInicio-1) se respetan tal cual estan en schedule[].
// pmaxPred[slotInicio] debe contener ya la prediccion corregida LAR antes de llamar.
bool planificarGreedyDesde(uint8_t slotInicio, float socIni) {

  // Objetivo de neutralidad anclado al SoC de INICIO DEL DÍA, SoC(0), no al SoC
  // actual: el horizonte arranca en socIni (correcto), pero el piso de neutralidad
  // debe seguir siendo min(SoC(0), 50) y no aflojar a medida que la batería baja.
  float socObjetivo = (socInicioDia > SOC_NEUTRAL_MAX) ? SOC_NEUTRAL_MAX : socInicioDia;

  // Inicializar los slots restantes con plan maximo
  for (uint8_t k = slotInicio; k < N_SLOTS; k++) schedule[k] = PLAN_P3;

  for (uint16_t iter = 0; iter < MAX_ITER_GREEDY; iter++) {

    ResultadoDia res = simularDia(socIni, schedule, socObjetivo, slotInicio);

    if (!res.violacionSoC && !res.violacionNeutral) {
      Serial.print("Greedy LAR OK en "); Serial.print(iter); Serial.println(" iter");
      return true;
    }

    // Buscar slot degradable con peor margen, solo entre los restantes.
    // Fase 1: solo slots en déficit (margen < 0)
    float  peorMargen = 1e9f;
    int8_t kPeor      = -1;

    for (uint8_t k = slotInicio; k < N_SLOTS; k++) {
      if (schedule[k] == PLAN_P0) continue;

      float margen = pmaxPred[k] * ETA_HARV - CONSUMO_PLAN_MW[schedule[k]];
      if (margen >= 0.0f) continue;  // slot con superávit: no degradar en fase 1

      if (margen < peorMargen) {
        peorMargen = margen;
        kPeor      = k;
      }
    }

    // Fase 2: degradar slots de superávit SOLO para corregir neutralidad final.
    // Para una violacion de SoC, degradar superavit no sirve (la bateria carga
    // igual) y solo tira trabajo util: si hay sol, hay que intentar funcionar.
    if (kPeor < 0 && res.violacionNeutral) {
      for (uint8_t k = slotInicio; k < N_SLOTS; k++) {
        if (schedule[k] == PLAN_P0) continue;

        float margen = pmaxPred[k] * ETA_HARV - CONSUMO_PLAN_MW[schedule[k]];

        if (margen < peorMargen) {
          peorMargen = margen;
          kPeor      = k;
        }
      }
    }

    if (kPeor < 0) {
      Serial.println("Greedy LAR: sin slots degradables");
      return false;
    }

    schedule[kPeor] = (uint8_t)degradarPlan((PlanEnergetico)schedule[kPeor]);
  }

  Serial.println("Greedy LAR: max iter alcanzadas");
  return false;
}

// ====================== SD: PLAN.CSV ======================

// Guarda schedule[] y packetID en PLAN.CSV como backup ante reset o pérdida de batería
void guardarPlanSD() {
  if (!sdOK) return;

  digitalWrite(LORA_CS, HIGH);
  delay(2);

  // Abrir siempre en modo escritura y truncar: solo interesa el plan del día actual
  File f = SD.open("PLAN.CSV", O_WRITE | O_CREAT | O_TRUNC);

  if (!f) {
    Serial.println("Error escribiendo PLAN.CSV");
    digitalWrite(SD_CS, HIGH);
    return;
  }

  // Línea 1: schedule[48] separado por comas, valores 0-3
  for (uint8_t k = 0; k < N_SLOTS; k++) {
    f.print(schedule[k]);
    if (k < N_SLOTS - 1) f.print(",");
  }
  f.println();

  // Línea 2: packetID para recuperar continuidad de numeración al reiniciar
  f.println(packetID);

  // Línea 3: pmaxPred[48] (aprendizaje EWMA) para que sobreviva a resets.
  // Sin esto, cada reinicio vuelve al default optimista y reinicia la convergencia.
  for (uint8_t k = 0; k < N_SLOTS; k++) {
    f.print(pmaxPred[k], 3);
    if (k < N_SLOTS - 1) f.print(",");
  }
  f.println();

  f.close();
  digitalWrite(SD_CS, HIGH);

  Serial.println("PLAN.CSV guardado");
}

// Recupera schedule[] y packetID desde PLAN.CSV al reiniciar
// Retorna true si el archivo existía y se pudo leer correctamente
bool recuperarPlanSD() {
  if (!sdOK) return false;

  digitalWrite(LORA_CS, HIGH);
  delay(2);

  if (!SD.exists("PLAN.CSV")) {
    Serial.println("PLAN.CSV no encontrado");
    digitalWrite(SD_CS, HIGH);
    return false;
  }

  File f = SD.open("PLAN.CSV", FILE_READ);

  if (!f) {
    Serial.println("Error leyendo PLAN.CSV");
    digitalWrite(SD_CS, HIGH);
    return false;
  }

  // Parsear línea 1: 48 enteros separados por comas
  for (uint8_t k = 0; k < N_SLOTS; k++) {
    schedule[k] = (uint8_t)f.parseInt();
  }

  // Parsear línea 2: packetID
  packetID = (uint32_t)f.parseInt();

  // Consumir el salto de línea de la línea 2 antes de evaluar si hay línea 3
  while (f.available() && (f.peek() == '\n' || f.peek() == '\r')) f.read();

  // Línea 3 (opcional): pmaxPred[48], el aprendizaje EWMA. Backward-compatible:
  // si el PLAN.CSV es viejo (2 líneas), available()==0 y se conservan los defaults.
  if (f.available() > 0) {
    float tmp[N_SLOTS];
    bool ok = true;
    for (uint8_t k = 0; k < N_SLOTS; k++) {
      if (k > 0 && f.available() <= 0) { ok = false; break; }  // línea 3 truncada
      tmp[k] = f.parseFloat();
    }
    if (ok) {
      for (uint8_t k = 0; k < N_SLOTS; k++) pmaxPred[k] = tmp[k];
      Serial.println("EWMA recuperado de SD");
    } else {
      Serial.println("EWMA en SD incompleto; usando default");
    }
  }

  f.close();
  digitalWrite(SD_CS, HIGH);

  Serial.print("Plan recuperado. PacketID=");
  Serial.println(packetID);
  return true;
}

// ====================== UTILIDADES ======================

float sanitizar(float x) {
  if (isnan(x) || isinf(x)) return 0.0f;
  return x;
}

float limpiarCorriente(float i) {
  if (abs(i) < 0.5f) return 0.0f;  // filtrar ruido por debajo de 0.5 mA
  return i;
}

float limpiarCorrienteBat(float i) {
  if (abs(i) < 0.2f) return 0.0f;  // umbral más bajo para la batería
  return i;
}

// Retorna el número de slot (0-47) correspondiente al momento actual del RTC
uint8_t obtenerSlotRTC(DateTime now) {
  return now.hour() * 2 + (now.minute() >= 30 ? 1 : 0);
}

// Retorna el DateTime del próximo corte de slot (XX:00 o XX:30)
DateTime obtenerProximoCorteSlot(DateTime now) {
  if (now.minute() < 30) {
    return DateTime(now.year(), now.month(), now.day(), now.hour(), 30, 0);
  } else {
    DateTime nextHour = now + TimeSpan(0, 1, 0, 0);
    return DateTime(nextHour.year(), nextHour.month(), nextHour.day(), nextHour.hour(), 0, 0);
  }
}

// Retorna el intervalo de muestreo en segundos según el plan activo
uint32_t intervaloMuestreoSec(PlanEnergetico p) {
  if (p == PLAN_P0) return SAMPLE_P0_SEC;  // P0: 120 s
  return SAMPLE_SEC;                        // P1/P2/P3: 60 s
}

void imprimirFechaHora(DateTime t) {
  Serial.print(t.year());   Serial.print("-");
  Serial.print(t.month());  Serial.print("-");
  Serial.print(t.day());    Serial.print(" ");
  Serial.print(t.hour());   Serial.print(":");
  Serial.print(t.minute()); Serial.print(":");
  Serial.println(t.second());
}

// ====================== RTC ======================

// Configura el countdown timer del PCF8523 para despertar al MCU
// MAX_SLEEP_SEC actúa de safety net: si falla la lógica, el RTC despierta igual
void configurarTimerRTC(uint16_t segundos) {
  if (!rtcOK) return;
  if (segundos < 1)             segundos = 1;
  if (segundos > MAX_SLEEP_SEC) segundos = MAX_SLEEP_SEC;  // tope de seguridad
  rtc.deconfigureAllTimers();
  rtc.enableCountdownTimer(PCF8523_FrequencySecond, (uint8_t)segundos);
}

void iniciarRTC() {
  rtcOK = rtc.begin();

  if (!rtcOK) {
    Serial.println("RTC no detectado");
    return;
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC sin hora, ajustando a compilacion");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.start();
  rtc.deconfigureAllTimers();

  pinMode(RTC_INT_PIN, INPUT_PULLUP);

  // Asociar la ISR al pin de interrupción del RTC para despertar el MCU
  LowPower.attachInterruptWakeup(RTC_INT_PIN, despertarRTC, FALLING);

  Serial.print("RTC OK: ");
  imprimirFechaHora(rtc.now());
}

// ====================== LORA ======================

void iniciarLoRa() {
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  SPI.begin();

  int state = radio.begin(LORA_FREQ, 125.0, 7, 5, 0x12, 20, 8);

  if (state == RADIOLIB_ERR_NONE) {
    loraOK = true;
    radio.setCRC(true);
    radio.sleep();
    Serial.println("LoRa OK");
  } else {
    loraOK = false;
    Serial.print("LoRa ERROR: "); Serial.println(state);
  }
}

void dormirLoRa() {
  if (!loraOK) return;
  radio.sleep();
  digitalWrite(LORA_CS, HIGH);
}

// Pone la radio en modo recepción continua (P2 y P3)
void iniciarRecepcionLoRa() {
  if (!loraOK) return;
  digitalWrite(SD_CS,   HIGH);
  digitalWrite(LORA_CS, HIGH);
  delay(2);
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa RX ERROR: "); Serial.println(state);
  }
  digitalWrite(LORA_CS, HIGH);
}

// Transmite una muestra por LoRa con el plan activo incluido en el payload
void enviarLoRa(DateTime now) {
  if (!loraOK) return;

  char payload[220];

  // Formato: NODE;packetID;timestamp;plan;VP;IP;VC;IC;VBINA;IB;VBLC;SOC;TEMP;VOC;ISC;VSHORT;PMAX
  snprintf(
    payload, sizeof(payload),
    "%s;%lu;%04d-%02d-%02dT%02d:%02d:%02d;%u;%s;%s;%s;"
    "%.3f;%.1f;%.3f;%.1f;%.3f;%.1f;%.3f;%.1f;%.2f;%.3f;%.1f;%.3f;%.1f;%.1f",
    NODE_ID,
    (unsigned long)packetID,
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second(),
    slotActual,
    etiquetaPlan(),
    nombrePlan(planAntes),
    nombrePlan(planNuevo),
    m.vPanel, m.iPanel,
    m.vCarga, m.iCarga,
    m.vBatINA, m.iBat,
    m.vBatLC, m.soc,
    temperatura,
    Voc, Isc, Vshort, Pmax_est,
    acc.socUltimo
  );

  radio.standby();

  Serial.print("LoRa TX: "); Serial.println(payload);

  int state = radio.transmit(payload);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa TX OK");
  } else {
    Serial.print("LoRa TX ERROR: "); Serial.println(state);
  }

  // Volver al modo correcto según el plan activo
  if (planActual == PLAN_P2 || planActual == PLAN_P3) {
    iniciarRecepcionLoRa();  // P2/P3: retomar RX continuo
  } else {
    dormirLoRa();            // P0/P1: volver a sleep
  }
}

// ====================== ACUMULADOR ======================

// Resetea el acumulador al inicio de cada slot
void resetAcumulador() {
  acc = AcumuladorSlot();  // pone todo en cero y n=0
}

// Suma la muestra actual al acumulador
void acumularMuestra() {
  acc.vPanel  += sanitizar(m.vPanel);
  acc.iPanel  += sanitizar(m.iPanel);
  acc.pPanel  += sanitizar(m.pPanel);
  acc.vCarga  += sanitizar(m.vCarga);
  acc.iCarga  += sanitizar(m.iCarga);
  acc.pCarga  += sanitizar(m.pCarga);
  acc.vBatINA += sanitizar(m.vBatINA);
  acc.iBat    += sanitizar(m.iBat);
  acc.pBat    += sanitizar(m.pBat);
  acc.vBatLC  += sanitizar(m.vBatLC);
  acc.soc     += sanitizar(m.soc);
  acc.voc     += sanitizar(Voc);
  acc.isc     += sanitizar(Isc);
  acc.vshort  += sanitizar(Vshort);
  acc.pmax    += sanitizar(Pmax_est);
  acc.temp    += temperatura;
  acc.socUltimo = sanitizar(m.soc);  // se sobreescribe: al final queda el ultimo valor
  acc.n++;
}

// Guarda el promedio del slot en SD al finalizar el slot
void guardarPromedioSlot(uint8_t slot, DateTime inicio) {
  if (!sdOK || acc.n == 0) return;

  float n = (float)acc.n;

  // Aparcar LoRa antes de usar el bus SPI para la SD
  Serial.println("CK1a: radio.sleep...");
  if (loraOK) { radio.sleep(); delay(5); }
  Serial.println("CK1b: radio.sleep OK");
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS,   HIGH);
  delay(5);

  Serial.println("CK1c: SD.open...");
  dataFile = SD.open(filename, FILE_WRITE);
  Serial.println("CK1d: SD.open OK");

  if (!dataFile) {
    Serial.println("Error abriendo CSV de datos");
    digitalWrite(SD_CS, HIGH);
    if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) iniciarRecepcionLoRa();
    return;
  }

  // Una fila por slot con los promedios de todas las muestras
  dataFile.print(NODE_ID);                               dataFile.print(";");
  dataFile.print((unsigned long)packetID);               dataFile.print(";");

  char ts[20];
  sprintf(ts, "%04d-%02d-%02dT%02d:%02d:%02d",
    inicio.year(), inicio.month(), inicio.day(),
    inicio.hour(), inicio.minute(), inicio.second());
  dataFile.print(ts);                                    dataFile.print(";");

  dataFile.print(slot);                                  dataFile.print(";");  // numero de slot 0-47
  dataFile.print(etiquetaPlan());                        dataFile.print(";");
  dataFile.print(acc.n);                                 dataFile.print(";");  // muestras acumuladas

  dataFile.print(acc.vPanel  / n, 3);                   dataFile.print(";");
  dataFile.print(acc.iPanel  / n, 1);                   dataFile.print(";");
  dataFile.print(acc.vCarga  / n, 3);                   dataFile.print(";");
  dataFile.print(acc.iCarga  / n, 1);                   dataFile.print(";");
  dataFile.print(acc.vBatINA / n, 3);                   dataFile.print(";");
  dataFile.print(acc.iBat    / n, 1);                   dataFile.print(";");
  dataFile.print(acc.vBatLC  / n, 3);                   dataFile.print(";");
  dataFile.print(acc.soc     / n, 1);                   dataFile.print(";");
  dataFile.print(acc.temp    / n, 2);                   dataFile.print(";");
  dataFile.print(acc.voc     / n, 3);                   dataFile.print(";");
  dataFile.print(acc.isc     / n, 1);                   dataFile.print(";");
  dataFile.print(acc.vshort  / n, 3);                   dataFile.print(";");
  dataFile.print(acc.pmax    / n, 1);                   dataFile.print(";");
  dataFile.print(acc.socUltimo, 1);                     dataFile.print(";");
  dataFile.print(nombrePlan(planAntes));                 dataFile.print(";");
  dataFile.print(nombrePlan(planNuevo));                 dataFile.print(";");
  dataFile.print(errorLAR, 3);                          dataFile.print(";");
  dataFile.println(pHatLAR, 3);                         // println pone el \n final

  Serial.println("CK1e: dataFile.close...");
  dataFile.close();
  digitalWrite(SD_CS, HIGH);
  Serial.println("CK1f: dataFile.close OK");

  if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) {
    Serial.println("CK1g: iniciarRecepcionLoRa...");
    iniciarRecepcionLoRa();
    Serial.println("CK1h: iniciarRecepcionLoRa OK");
  }

  Serial.print("Promedio slot guardado. N="); Serial.println(acc.n);
}

// ====================== SD: DATOS ======================

void iniciarSD() {
  if (loraOK) { radio.sleep(); delay(5); }
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS,   HIGH);
  delay(5);

  sdOK = SD.begin(SD_CS);
  Serial.print("SD: "); Serial.println(sdOK ? "OK" : "ERROR");

  if (sdOK) crearArchivoCSV();

  digitalWrite(SD_CS, HIGH);
}

void crearArchivoCSV() {
  DateTime now = rtc.now();

  // Nombre de archivo diario: YYMMDD.CSV
  sprintf(filename, "%02d%02d%02d.CSV", now.year() % 100, now.month(), now.day());

  digitalWrite(LORA_CS, HIGH);
  delay(2);

  bool existe = SD.exists(filename);
  dataFile    = SD.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error creando CSV de datos");
    sdOK = false;
    return;
  }

  // Escribir encabezado solo si el archivo es nuevo o está vacío
  if (!existe || dataFile.size() == 0) {
    dataFile.println(
      "NODE;ID;TIMESTAMP;SLOT;PLAN;N;"
      "VP_V;IP_mA;VC_V;IC_mA;VBINA_V;IB_mA;VBLC_V;SOC_pct;"
      "TEMP_C;VOC_V;ISC_mA;VSHORT_V;PMAX_EST_mW;SOC_ULT_pct;"
      "PLAN_ANT;PLAN_NVO;ERROR_mW;PHAT_mW"
    );
  }

  dataFile.close();
  digitalWrite(SD_CS, HIGH);

  Serial.print("CSV datos: "); Serial.println(filename);
}

void guardarSD(DateTime now) {
  if (!sdOK) return;

  // Aparcar LoRa antes de tomar el bus SPI para la SD
  if (loraOK) { radio.sleep(); delay(5); }
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS,   HIGH);
  delay(5);

  dataFile = SD.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error abriendo CSV de datos");
    digitalWrite(SD_CS, HIGH);
    // Reanudar LoRa si corresponde
    if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) iniciarRecepcionLoRa();
    return;
  }

  // Una fila por muestra; incluye el plan activo para trazabilidad
  // Se usa print() campo a campo para evitar truncamiento del buffer de printf en SAMD21
  dataFile.print(NODE_ID);                               dataFile.print(";");
  dataFile.print((unsigned long)packetID);               dataFile.print(";");

  // Timestamp manual para evitar dependencia de sprintf con buffers largos
  char ts[20];
  sprintf(ts, "%04d-%02d-%02dT%02d:%02d:%02d",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());
  dataFile.print(ts);                                    dataFile.print(";");

  dataFile.print(nombrePlan(planActual));                dataFile.print(";");

  dataFile.print(sanitizar(m.vPanel),  3);              dataFile.print(";");
  dataFile.print(sanitizar(m.iPanel),  1);              dataFile.print(";");
  dataFile.print(sanitizar(m.vCarga),  3);              dataFile.print(";");
  dataFile.print(sanitizar(m.iCarga),  1);              dataFile.print(";");
  dataFile.print(sanitizar(m.vBatINA), 3);              dataFile.print(";");
  dataFile.print(sanitizar(m.iBat),    1);              dataFile.print(";");
  dataFile.print(sanitizar(m.vBatLC),  3);              dataFile.print(";");
  dataFile.print(sanitizar(m.soc),     1);              dataFile.print(";");
  dataFile.print(temperatura,          2);              dataFile.print(";");
  dataFile.print(sanitizar(Voc),       3);              dataFile.print(";");
  dataFile.print(sanitizar(Isc),       1);              dataFile.print(";");
  dataFile.print(sanitizar(Vshort),    3);              dataFile.print(";");
  dataFile.println(sanitizar(Pmax_est), 1);             // println pone el \n final

  dataFile.close();
  digitalWrite(SD_CS, HIGH);

  // Reanudar modo LoRa correspondiente al plan
  if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) iniciarRecepcionLoRa();

  Serial.println("SD OK");
}

// Registra entradas a emergencia PF0 en EVENTOS.CSV con timestamp exacto,
// para poder calcular la energía consumida en cada tramo del slot
// Formato: NODE;TIMESTAMP;SLOT;MOTIVO;SOC;VBAT;PLAN_PREVIO
void guardarEvento(DateTime now, uint8_t slot, const char* motivo,
                   float soc, float vbat, PlanEnergetico planPrevio) {
  if (!sdOK) return;

  if (loraOK) { radio.sleep(); delay(5); }
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS,   HIGH);
  delay(5);

  File f = SD.open("EVENTOS.CSV", FILE_WRITE);
  if (f) {
    char ts[20];
    sprintf(ts, "%04d-%02d-%02dT%02d:%02d:%02d",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());
    f.print(NODE_ID);                 f.print(";");
    f.print(ts);                      f.print(";");
    f.print(slot);                    f.print(";");
    f.print(motivo);                  f.print(";");
    f.print(sanitizar(soc), 1);       f.print(";");
    f.print(sanitizar(vbat), 3);      f.print(";");
    f.println(nombrePlan(planPrevio));
    f.close();
    Serial.println("EVENTOS.CSV guardado");
  } else {
    Serial.println("Error escribiendo EVENTOS.CSV");
  }
  digitalWrite(SD_CS, HIGH);

  if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) iniciarRecepcionLoRa();
}

// ====================== SENSORES ======================

void iniciarSensores() {
  inaPanelOK = inaPanel.begin();
  inaCargaOK = inaCarga.begin();
  inaBatOK   = inaBat.begin();

  if (inaPanelOK) inaPanel.setCalibration_32V_2A();
  if (inaCargaOK) inaCarga.setCalibration_32V_2A();
  if (inaBatOK)   inaBat.setCalibration_32V_2A();

  Serial.print("INA PANEL: "); Serial.println(inaPanelOK ? "OK" : "FAIL");
  Serial.print("INA CARGA: "); Serial.println(inaCargaOK ? "OK" : "FAIL");
  Serial.print("INA BAT:   "); Serial.println(inaBatOK   ? "OK" : "FAIL");

  lcOK = lc.begin();
  if (lcOK) {
    lc.setPackSize((lc709203_adjustment_t)BAT_APA);  // APA según BATERIA_MAH (definido arriba)
    Serial.println("LC709203F OK");
  } else {
    Serial.println("LC709203F ERROR");
  }

  sensors.begin();
  Serial.println("DS18B20 OK");
}

void dormirINAs() {
  if (inaPanelOK) inaPanel.powerSave(true);
  if (inaCargaOK) inaCarga.powerSave(true);
  if (inaBatOK)   inaBat.powerSave(true);
}

void despertarINAs() {
  if (inaPanelOK) {
    inaPanel.powerSave(false);
    inaPanel.setCalibration_32V_2A();  // re-escribir calibración: powerSave no la restaura
  }
  if (inaCargaOK) {
    inaCarga.powerSave(false);
    inaCarga.setCalibration_32V_2A();
  }
  if (inaBatOK) {
    inaBat.powerSave(false);
    inaBat.setCalibration_32V_2A();
  }

  delay(25);
  // Nota: el LC709203F no tiene powerSave, no necesita reinicialización aquí
}

void leerTemperatura() {
  sensors.requestTemperatures();
  temperatura = sensors.getTempCByIndex(0);
  Serial.print("Temp: "); Serial.print(temperatura); Serial.println(" C");
}

// ============ ESTIMADOR DE SoC (fusión gauge + tensión) ============
// Fusiona el SoC del LC709203F con un SoC derivado de la tensión (curva OCV del
// LiPo). El gauge a veces se congela o miente en la zona plana; la tensión lo
// corrige. El peso de la tensión sube en reposo (fiable) y baja bajo carga (droop).

struct PuntoOCV { float v; float soc; };
// Curva OCV genérica de LiPo 1S, en reposo (tensión -> SoC%), de mayor a menor
const PuntoOCV OCV_TABLA[] = {
  {4.20f,100.0f}, {4.06f,90.0f}, {3.98f,80.0f}, {3.92f,70.0f}, {3.87f,60.0f},
  {3.82f,50.0f},  {3.79f,40.0f}, {3.77f,30.0f}, {3.74f,20.0f}, {3.68f,10.0f},
  {3.45f,5.0f},   {3.00f,0.0f}
};
const uint8_t OCV_N = sizeof(OCV_TABLA) / sizeof(OCV_TABLA[0]);

const float    IB_REST       = 5.0f;    // |IB| por debajo -> reposo, tensión fiable [mA]
const float    IB_LOAD       = 35.0f;   // |IB| por encima -> carga alta (P3 real ~35mA), tensión no fiable [mA]
const float    W_VOLT_REST   = 0.70f;   // peso de la tensión en reposo
const float    W_VOLT_LOAD   = 0.15f;   // peso de la tensión bajo carga
const uint16_t GAUGE_STUCK_N = 12;      // muestras idénticas para sospechar congelamiento
const float    VBINA_MOVIO   = 0.015f;  // V que se movió la tensión mientras el SoC no [V]

float    socGaugePrev   = -1.0f;
float    vbinaAncla     = 0.0f;
uint16_t gaugeQuietoN   = 0;
bool     gaugeCongelado = false;

// Interpola SoC a partir de la tensión usando la tabla OCV
float socDesdeTension(float v) {
  if (isnan(v)) return NAN;
  if (v >= OCV_TABLA[0].v)         return 100.0f;
  if (v <= OCV_TABLA[OCV_N - 1].v) return 0.0f;
  for (uint8_t i = 0; i < OCV_N - 1; i++) {
    if (v <= OCV_TABLA[i].v && v > OCV_TABLA[i + 1].v) {
      float f = (v - OCV_TABLA[i + 1].v) / (OCV_TABLA[i].v - OCV_TABLA[i + 1].v);
      return OCV_TABLA[i + 1].soc + f * (OCV_TABLA[i].soc - OCV_TABLA[i + 1].soc);
    }
  }
  return NAN;
}

// SoC fusionado a partir del gauge, la tensión y la corriente de batería
float estimarSoC(float socGauge, float vbina, float ibat) {
  float socV = socDesdeTension(vbina);

  // Detección de gauge congelado: SoC clavado mientras la tensión se mueve
  if (!isnan(socGauge)) {
    if (socGaugePrev >= 0.0f && fabs(socGauge - socGaugePrev) < 0.05f) {
      if (gaugeQuietoN == 0) vbinaAncla = vbina;   // anclar al empezar a estar quieto
      gaugeQuietoN++;
      if (gaugeQuietoN >= GAUGE_STUCK_N && !isnan(vbina) &&
          fabs(vbina - vbinaAncla) > VBINA_MOVIO) {
        gaugeCongelado = true;
      }
    } else {
      gaugeQuietoN = 0;
      gaugeCongelado = false;
    }
    socGaugePrev = socGauge;
  }

  if (isnan(socV))     return socGauge;   // sin tensión útil -> gauge
  if (isnan(socGauge)) return socV;       // sin gauge -> tensión
  if (gaugeCongelado)  return socV;       // gauge muerto -> solo tensión

  // Peso de la tensión según la corriente (droop bajo carga)
  float ai = fabs(ibat);
  float wV;
  if (ai <= IB_REST)      wV = W_VOLT_REST;
  else if (ai >= IB_LOAD) wV = W_VOLT_LOAD;
  else wV = W_VOLT_REST + (W_VOLT_LOAD - W_VOLT_REST) * (ai - IB_REST) / (IB_LOAD - IB_REST);

  float soc = wV * socV + (1.0f - wV) * socGauge;
  if (soc < 0.0f)   soc = 0.0f;
  if (soc > 100.0f) soc = 100.0f;
  return soc;
}

void leerSensores() {
  const int N = 5;  // número de muestras a promediar por lectura

  float sumVp = 0, sumIp = 0, sumPp = 0;
  float sumVc = 0, sumIc = 0, sumPc = 0;
  float sumVb = 0, sumIb = 0, sumPb = 0;
  float sumVlc = 0, sumSoc = 0;

  for (int i = 0; i < N; i++) {
    if (inaPanelOK) {
      sumVp += sanitizar(inaPanel.getBusVoltage_V());
      sumIp += sanitizar(inaPanel.getCurrent_mA());
      sumPp += sanitizar(inaPanel.getPower_mW());
    }
    if (inaCargaOK) {
      sumVc += sanitizar(inaCarga.getBusVoltage_V());
      sumIc += sanitizar(inaCarga.getCurrent_mA());
      sumPc += sanitizar(inaCarga.getPower_mW());
    }
    if (inaBatOK) {
      sumVb += sanitizar(inaBat.getBusVoltage_V());
      sumIb += sanitizar(inaBat.getCurrent_mA());
      sumPb += sanitizar(inaBat.getPower_mW());
    }
    if (lcOK) {
      sumVlc += sanitizar(lc.cellVoltage());
      sumSoc += sanitizar(lc.cellPercent());
    }
    delay(10);
  }

  m.vPanel  = inaPanelOK ? sanitizar(sumVp / N) : NAN;
  m.iPanel  = inaPanelOK ? limpiarCorriente(sanitizar(sumIp / N)) : NAN;
  m.pPanel  = inaPanelOK ? sanitizar(sumPp / N) : NAN;

  m.vCarga  = inaCargaOK ? sanitizar(sumVc / N) : NAN;
  m.iCarga  = inaCargaOK ? limpiarCorriente(sanitizar(sumIc / N)) : NAN;
  m.pCarga  = inaCargaOK ? sanitizar(sumPc / N) : NAN;

  m.vBatINA = inaBatOK ? sanitizar(sumVb / N) : NAN;
  m.iBat    = inaBatOK ? limpiarCorrienteBat(sanitizar(sumIb / N)) : NAN;
  m.pBat    = inaBatOK ? sanitizar(sumPb / N) : NAN;

  m.vBatLC  = lcOK ? sanitizar(sumVlc / N) : NAN;

  // SoC fusionado: gauge (LC709203F) + tensión (OCV del INA de batería).
  // Corrige congelamientos y errores del gauge apoyándose en la tensión.
  float socGauge = lcOK ? sanitizar(sumSoc / N) : NAN;
  m.soc = estimarSoC(socGauge, m.vBatINA, m.iBat);

  Serial.print("SoC gauge="); Serial.print(socGauge, 1);
  Serial.print(" tension="); Serial.print(socDesdeTension(m.vBatINA), 1);
  Serial.print(" fusion="); Serial.print(m.soc, 1);
  if (gaugeCongelado) Serial.print(" [GAUGE CONGELADO]");
  Serial.println();

  leerTemperatura();
}

void relaySet() {
  if (relayEstado) return;              // ya está en SET, no pulsar de nuevo
  digitalWrite(RELAY_UNSET_PIN, LOW);  // asegurar que UNSET esté apagado antes de activar SET
  digitalWrite(RELAY_SET_PIN, HIGH);
  delay(50);
  digitalWrite(RELAY_SET_PIN, LOW);
  relayEstado = true;
  Serial.println("Relay: SET");
}

void relayUnset() {
  if (!relayEstado) return;            // ya está en UNSET, no pulsar de nuevo
  digitalWrite(RELAY_SET_PIN, LOW);   // asegurar que SET esté apagado antes de activar UNSET
  digitalWrite(RELAY_UNSET_PIN, HIGH);
  delay(50);
  digitalWrite(RELAY_UNSET_PIN, LOW);
  relayEstado = false;
  Serial.println("Relay: UNSET");
}

// Mide Voc e Isc del panel desconectando y cortocircuitando brevemente
void medirVocIsc() {
  if (!inaPanelOK) {
    Voc = NAN; Isc = NAN; Vshort = NAN; Pmax_est = NAN;
    return;
  }

  // Gate 1: panel sin tensión útil -> no hay nada que medir, y no vale la pena
  // conmutar el relay (la posición de medición desconecta el panel del cargador)
  float vPanelAhora = sanitizar(inaPanel.getBusVoltage_V());
  if (vPanelAhora < VPANEL_MIN_MEDICION) {
    Voc = vPanelAhora; Isc = 0.0f; Vshort = 0.0f;
    Pmax_est = 0.0f;  // producción nula real: el EWMA debe aprenderla como 0
    Serial.print("Voc/Isc salteado: VP="); Serial.print(vPanelAhora, 2); Serial.println(" V");
    return;
  }

  // Gate 2: batería crítica -> no desconectar el cargador justo ahora.
  // Pmax_est = NAN: acá sí puede haber producción, el EWMA debe ignorar el slot
  if (!isnan(m.vBatINA) && m.vBatINA > 0.5f && m.vBatINA < VBAT_EMERGENCIA) {
    Voc = NAN; Isc = NAN; Vshort = NAN; Pmax_est = NAN;
    Serial.println("Voc/Isc salteado: bateria critica");
    return;
  }

  const int N = 5;
  float sumaVoc = 0, sumaIsc = 0, sumaVshort = 0;

  digitalWrite(MOSFET_PIN, LOW);
  delay(20);
  relayUnset();   // desconectar carga → medir Voc
  delay(150);

  for (int i = 0; i < N; i++) {
    sumaVoc += sanitizar(inaPanel.getBusVoltage_V());
    delay(5);
  }
  Voc = sanitizar(sumaVoc / N);

  digitalWrite(MOSFET_PIN, HIGH);  // cortocircuitar → medir Isc
  delay(50);

  for (int i = 0; i < N; i++) {
    sumaVshort += sanitizar(inaPanel.getBusVoltage_V());
    sumaIsc    += abs(sanitizar(inaPanel.getCurrent_mA()));
    delay(5);
  }

  digitalWrite(MOSFET_PIN, LOW);
  Vshort   = sanitizar(sumaVshort / N);
  Isc      = sanitizar(sumaIsc / N);

  relaySet();   // reconectar carga
  delay(50);

  Pmax_est = sanitizar(Voc * Isc * fillFactor);  // potencia máxima estimada del panel [mW]

  Serial.print("Voc: ");       Serial.print(Voc, 3);
  Serial.print(" V | Isc: ");  Serial.print(Isc, 2);
  Serial.print(" mA | Pmax: ");Serial.print(Pmax_est, 1);
  Serial.println(" mW");
}

// ====================== OLED (SSD1306 128x32 FeatherWing) ======================

void iniciarOLED() {
  delay(250);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if (!oledOK) { Serial.println("OLED ERROR"); return; }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Nodo EH3");
  display.println("Iniciando...");
  display.display();
  Serial.println("OLED OK");
}

void apagarOLED() {
  if (!oledOK) return;
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);  // apagar panel para ahorrar energía (P0/P1/P2)
}

void encenderOLED() {
  if (!oledOK) return;
  display.ssd1306_command(SSD1306_DISPLAYON);
}

void actualizarOLED(DateTime now) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setCursor(0, 0);

  // Línea 1: nodo, plan activo y slot
  display.print("EH3 | "); display.print(nombrePlan(planActual));
  display.print(" S:"); display.println(slotActual);

  // Línea 2: hora actual
  display.print("Hora: ");
  if (now.hour()   < 10) display.print("0"); display.print(now.hour());   display.print(":");
  if (now.minute() < 10) display.print("0"); display.print(now.minute()); display.print(":");
  if (now.second() < 10) display.print("0"); display.println(now.second());

  // Línea 3: SoC y tensión de batería
  display.print("SoC: "); display.print(m.soc, 0);
  display.print("% Vb:"); display.println(m.vBatLC, 2);

  // Línea 4: Pmax estimada y potencia real del panel
  display.print("Pm:"); display.print(Pmax_est, 0);
  display.print(" Pp:"); display.println(m.pPanel, 0);

  display.display();
}

// ====================== GESTIÓN DE SLOTS ======================

// Configura el hardware según el plan del slot (OLED, INAs, LoRa)
void aplicarHardwarePlan(PlanEnergetico plan) {
  switch (plan) {

    case PLAN_P0:
      // Todo apagado, sleep largo; los INAs se despiertan solo al muestrear
      apagarOLED();
      dormirINAs();
      dormirLoRa();
      break;

    case PLAN_P1:
      // Sin OLED; LoRa duerme entre TX; INAs se despiertan solo al muestrear
      apagarOLED();
      dormirINAs();
      dormirLoRa();
      break;

    case PLAN_P2:
      // Sin OLED; LoRa en RX continuo; INAs se despiertan solo al muestrear
      apagarOLED();
      dormirINAs();
      iniciarRecepcionLoRa();
      break;

    case PLAN_P3:
      // OLED encendido; LoRa en RX continuo; INAs se despiertan solo al muestrear
      encenderOLED();
      dormirINAs();
      iniciarRecepcionLoRa();
      break;
  }
}

// Inicializa el nuevo slot: selecciona plan, configura hardware, prepara timers
void iniciarNuevoSlot(DateTime now) {
  packetID++;  // cada slot es un nuevo paquete
  inicioSlotRTC = now;  // marca el inicio de este slot, para timestamps de guardado

  // Leer el plan asignado por el Greedy para este slot
  planActual = (PlanEnergetico)schedule[slotActual];

  // Configurar hardware según el plan
  aplicarHardwarePlan(planActual);

  // Forzar que la primera muestra ocurra de inmediato
  uint32_t t0    = now.unixtime();
  lastSampleUnix = t0 - intervaloMuestreoSec(planActual);
  lastLoRaUnix   = t0 - 60;

  Serial.print("Slot "); Serial.print(slotActual);
  Serial.print(" | PKT="); Serial.print(packetID);
  Serial.print(" | Plan="); Serial.print(etiquetaPlan());
  Serial.print(" | Q="); Serial.print(calidadPlan(planActual));
  Serial.print(" | SoC="); Serial.print(m.soc, 1); Serial.println("%");
}

// Replanificacion LAR al cierre de cada slot.
// slotCerrado: slot que acaba de terminar (del que tenemos medicion real)
// slotSiguiente: slot que va a comenzar
void replanificarLAR(uint8_t slotCerrado, uint8_t slotSiguiente) {

  // --- Error del slot cerrado: e_d(i-1) = pReal - pPred ---
  float pReal = pmaxMedido[slotCerrado];   // medicion real guardada al cierre
  float pPred = pmaxPred[slotCerrado];     // prediccion EWMA para ese slot
  float error = (pReal >= 0.0f) ? (pReal - pPred) : 0.0f;  // 0 si no hay medicion

  // --- Prediccion corregida LAR para el slot siguiente ---
  float pHat = pmaxPred[slotSiguiente] + BETA_LAR * error;
  if (pHat < 0.0f) pHat = 0.0f;  // la produccion no puede ser negativa

  // --- Guardar para trazabilidad en SD ---
  errorLAR = error;
  pHatLAR  = pHat;

  // --- Plan antes de replanificar ---
  planAntes = (PlanEnergetico)schedule[slotSiguiente];

  // --- Sustituir temporalmente pmaxPred[slotSiguiente] con la prediccion corregida ---
  float pPredOriginal         = pmaxPred[slotSiguiente];
  pmaxPred[slotSiguiente]     = pHat;

  // --- Re-correr Greedy desde slotSiguiente con SoC real actual ---
  float socActual = (lcOK && !isnan(m.soc)) ? m.soc : acc.socUltimo;
  planificarGreedyDesde(slotSiguiente, socActual);

  // --- Restaurar pmaxPred (el EWMA no se altera por la correccion LAR) ---
  pmaxPred[slotSiguiente] = pPredOriginal;

  // --- Plan despues de replanificar ---
  planNuevo = (PlanEnergetico)schedule[slotSiguiente];

  Serial.print("LAR | e=");     Serial.print(error, 2);
  Serial.print(" pHat=");       Serial.print(pHat, 2);
  Serial.print(" | ");          Serial.print(nombrePlan(planAntes));
  Serial.print(" -> ");         Serial.println(nombrePlan(planNuevo));
}

// Corre el planificador GLAR al inicio del nuevo día (slot 0 = 00:00)
void correrPlanificadorMedianoche() {
  Serial.println("=== PLANIFICADOR GLAR (medianoche) ===");

  // --- 1. Actualizar pmaxPred con EWMA usando mediciones del dia que termina ---
  for (uint8_t i = 0; i < N_SLOTS; i++) {
    if (pmaxMedido[i] >= 0.0f) {  // -1.0f es centinela "sin medicion"
      pmaxPred[i] = ALPHA_EWMA * pmaxMedido[i] + (1.0f - ALPHA_EWMA) * pmaxPred[i];
    }
    // si no hay medicion real, pmaxPred[i] se mantiene (warmup o dia incompleto)
  }
  Serial.println("EWMA actualizado");

  // --- 2. Resetear mediciones del dia que comienza ---
  for (uint8_t i = 0; i < N_SLOTS; i++) pmaxMedido[i] = -1.0f;

  // --- 3. SoC real como punto de partida ---
  float socIni = (lcOK && !isnan(m.soc)) ? m.soc : 50.0f;
  socInicioDia = socIni;        // fijar la referencia SoC(0) del día (neutralidad del LAR)
  Serial.print("SoC ini: "); Serial.print(socIni, 1); Serial.println("%");

  // --- 4. Greedy completo con pmaxPred actualizado ---
  bool ok = planificarGreedy(socIni);
  Serial.println(ok ? "Plan: factible" : "Plan: mejor esfuerzo");

  Serial.print("Schedule: ");
  for (uint8_t k = 0; k < N_SLOTS; k++) {
    Serial.print(schedule[k]);
    if (k < N_SLOTS - 1) Serial.print(",");
  }
  Serial.println();

  guardarPlanSD();
}

// ====================== SLEEP ======================

void dormirHastaProximoEvento() {
  // P2 y P3 mantienen el MCU activo para sostener LoRa RX continuo
  if (planActual == PLAN_P2 || planActual == PLAN_P3) {
    delay(200);
    return;
  }

  DateTime now        = rtc.now();
  uint32_t nowUnix    = now.unixtime();
  uint32_t proxSlot   = obtenerProximoCorteSlot(now).unixtime();
  uint32_t proxMuestra= lastSampleUnix + intervaloMuestreoSec(planActual);

  // Próximo evento: lo que ocurra primero entre muestra y cambio de slot
  uint32_t nextEvent  = min(proxMuestra, proxSlot);

  // P1 también considera el próximo TX LoRa
  if (planActual == PLAN_P1) {
    uint32_t proxLoRa = lastLoRaUnix + 60;
    nextEvent = min(nextEvent, proxLoRa);
  }

  if (nextEvent <= nowUnix) return;  // ya es momento de actuar, no dormir

  uint32_t sleepSec = nextEvent - nowUnix;

  // Tope de seguridad: vale tanto para el timer del RTC como para LowPower.sleep
  // En P0, sleepSec coincide con el intervalo real (120 s = MAX_SLEEP_SEC)
  if (sleepSec > MAX_SLEEP_SEC) sleepSec = MAX_SLEEP_SEC;
  configurarTimerRTC((uint16_t)sleepSec);

  wakeRTC = false;
  delay(50);

  Serial.print("Sleep: "); Serial.print(sleepSec); Serial.println(" s");
  Serial.flush();

  // Despertar por RTC interno del SAMD21 (no depende del cableado de RTC_INT_PIN).
  // Si el INT del PCF8523 está conectado en A2, despierta por lo que llegue primero.
  LowPower.sleep(sleepSec * 1000UL);

  // Baliza post-sleep: el USB serial muere tras el standby, así que el LED
  // es la única señal visible. 3 parpadeos cortos = el MCU despertó.
  Serial.println("CK_WAKE: volvio del sleep");
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }

  // Reiniciar bus I2C tras el sleep para garantizar estabilidad del SAMD21
  Wire.end();
  delay(10);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  // 1 parpadeo largo = I2C reiniciado; lo próximo que corre es rtc.now() en loop()
  Serial.println("CK_WAKE2: I2C reiniciado");
  digitalWrite(LED_PIN, HIGH); delay(500);
  digitalWrite(LED_PIN, LOW);
}

// ====================== SETUP ======================

void setup() {
  // Configurar todos los pines de salida antes de iniciar periféricos
  pinMode(RELAY_SET_PIN,   OUTPUT); digitalWrite(RELAY_SET_PIN,   LOW);
  pinMode(RELAY_UNSET_PIN, OUTPUT); digitalWrite(RELAY_UNSET_PIN, LOW);
  pinMode(MOSFET_PIN,      OUTPUT); digitalWrite(MOSFET_PIN,      LOW);
  pinMode(RTC_INT_PIN,     INPUT_PULLUP);
  pinMode(LED_PIN,         OUTPUT); digitalWrite(LED_PIN,         LOW);
  pinMode(SD_CS,           OUTPUT); digitalWrite(SD_CS,           HIGH);
  pinMode(LORA_CS,         OUTPUT); digitalWrite(LORA_CS,         HIGH);

  SPI.begin();
  Serial.begin(115200);
  delay(1500);

  Serial.println("=== Nodo EH3 - Planificador GLAR ===");

  // Inicializar pmaxMedido con centinela -1 (sin medicion aun)
  for (uint8_t i = 0; i < N_SLOTS; i++) pmaxMedido[i] = -1.0f;

  Wire.begin();
  Wire.setClock(100000);

  iniciarOLED();
  iniciarRTC();
  iniciarLoRa();
  iniciarSD();
  iniciarSensores();

  relaySet();  // dejar relay activo al arrancar

  // Inicializar schedule en P0 como fallback seguro
  for (uint8_t k = 0; k < N_SLOTS; k++) schedule[k] = PLAN_P0;

  // Intentar recuperar el plan del día desde SD (ante reset o fallo de batería)
  if (!recuperarPlanSD()) {
    // Sin backup: calcular un plan inicial con el vector de predicción default
    // Así el nodo arranca con un plan coherente desde el primer slot
    Serial.println("Sin backup. Calculando plan inicial con prediccion default...");
    correrPlanificadorMedianoche();
  }

  // Lectura inicial de sensores para tener m.soc válido antes de iniciarNuevoSlot
  DateTime now = rtc.now();
  despertarINAs();
  medirVocIsc();
  leerSensores();
  dormirINAs();

  // Referencia de neutralidad del día: inicializar con el SoC real del arranque.
  // Sin esto, el primer día (o tras un reset con backup) quedaría forzada al default 50%.
  socInicioDia = (lcOK && !isnan(m.soc)) ? m.soc : 50.0f;

  slotActual = obtenerSlotRTC(now);
  resetAcumulador();     // limpiar acumulador antes de arrancar el primer slot
  iniciarNuevoSlot(now);

  Serial.println("Setup completo");
}

// ====================== LOOP ======================

void loop() {
  DateTime now      = rtc.now();

  // Validar DateTime completo antes de usarlo
  // El RTC puede devolver basura justo al despertar del sleep
  if (now.year() < 2024 || now.year() > 2035 ||
      now.hour() > 23   || now.minute() > 59) {
    delay(100);
    now = rtc.now();  // un solo reintento
  }

  // Calcular slot y proteger contra desbordamiento del array schedule[48]
  uint8_t slotNuevo = obtenerSlotRTC(now);
  if (slotNuevo >= N_SLOTS) slotNuevo = slotActual;  // ignorar slot inválido

  // Detectar cambio de slot
  if (slotNuevo != slotActual) {

    // Guardar promedio del slot que acaba de terminar
    Serial.println("CK1: guardarPromedioSlot...");
    guardarPromedioSlot(slotActual, inicioSlotRTC);
    Serial.println("CK2: guardarPromedioSlot OK");

    // Guardar medicion real del slot cerrado para EWMA de medianoche
    if (!isnan(Pmax_est) && Pmax_est >= 0.0f) {
      pmaxMedido[slotActual] = Pmax_est;
    }

    // Si el nuevo slot es el 0 (medianoche): actualizar EWMA y planificar el dia completo
    Serial.println("CK3: replan...");
    if (slotNuevo == 0) {
      correrPlanificadorMedianoche();
    } else {
      // Replanificacion LAR para los slots restantes del dia
      replanificarLAR(slotActual, slotNuevo);
    }
    Serial.println("CK4: replan OK");

    // Chequeo de emergencia al cierre del slot: SoC <= SOC_MIN o tensión de
    // batería crítica -> forzar P0 ("PF0") en el que comienza, pisando el plan
    bool emergSoC  = (acc.socUltimo <= SOC_MIN);
    bool emergVbat = (!isnan(m.vBatINA) && m.vBatINA > 0.5f && m.vBatINA < VBAT_EMERGENCIA);
    if (emergSoC || emergVbat) {
      PlanEnergetico planPlanificado = (PlanEnergetico)schedule[slotNuevo];
      schedule[slotNuevo] = PLAN_P0;
      planForzado = true;
      Serial.print("ALERTA: SoC=");
      Serial.print(acc.socUltimo, 1);
      Serial.print("% | Vbat=");
      Serial.print(m.vBatLC, 3);
      Serial.println(" V al cierre del slot -> forzando PF0 en el siguiente");
      guardarEvento(now, slotNuevo,
                    emergVbat ? (emergSoC ? "SOC+VBAT" : "VBAT") : "SOC",
                    acc.socUltimo, m.vBatLC, planPlanificado);
    } else {
      planForzado = false;
    }
    Serial.println("CK5: PF0 check OK");

    slotActual = slotNuevo;
    resetAcumulador();       // limpiar acumulador para el nuevo slot
    Serial.println("CK6: iniciarNuevoSlot...");
    iniciarNuevoSlot(now);
    Serial.println("CK7: iniciarNuevoSlot OK");
    now = rtc.now();
  }

  uint32_t nowUnix = now.unixtime();

  // Tomar muestra si corresponde según el intervalo del plan activo
  if (nowUnix - lastSampleUnix >= intervaloMuestreoSec(planActual)) {
    despertarINAs();
    medirVocIsc();
    leerSensores();
    dormirINAs();

    if (planActual == PLAN_P3) actualizarOLED(now);  // actualizar OLED solo en P3

    acumularMuestra();   // acumular para el promedio del slot, no guardar aún
    lastSampleUnix = nowUnix;

    // Emergencia intra-slot: si la batería entró en zona crítica, no esperar
    // al cierre del slot para frenar; el brownout es cuestión de minutos
    if (planActual != PLAN_P0) {
      bool emergSoC  = (!isnan(m.soc) && m.soc > 0.0f && m.soc <= SOC_MIN);
      bool emergVbat = (!isnan(m.vBatINA) && m.vBatINA > 0.5f && m.vBatINA < VBAT_EMERGENCIA);
      if (emergSoC || emergVbat) {
        PlanEnergetico planPrevio = planActual;
        schedule[slotActual] = PLAN_P0;
        planActual  = PLAN_P0;
        planForzado = true;
        aplicarHardwarePlan(PLAN_P0);
        Serial.print("EMERGENCIA intra-slot: SoC="); Serial.print(m.soc, 1);
        Serial.print("% | Vbat="); Serial.print(m.vBatLC, 3);
        Serial.println(" V -> PF0 inmediato");
        guardarEvento(now, slotActual,
                      emergVbat ? (emergSoC ? "SOC+VBAT" : "VBAT") : "SOC",
                      m.soc, m.vBatLC, planPrevio);
      }
    }
  }

  // TX LoRa cada 60 s para P1, P2 y P3 (P0 no transmite)
  if (planActual != PLAN_P0 && nowUnix - lastLoRaUnix >= 60) {
    enviarLoRa(now);
    lastLoRaUnix = nowUnix;
  }

  dormirHastaProximoEvento();
}
