#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RadioLib.h>

#include <Adafruit_INA219.h>
#include <Adafruit_LC709203F.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoLowPower.h>

// ====================== CONFIG ======================
#define NODE_ID "N01"

#define RELAY_SET_PIN    11
#define RELAY_UNSET_PIN  6
#define MOSFET_PIN       12

#define SD_CS            10
#define LORA_CS          8
#define LORA_RST         4
#define LORA_IRQ         3

#define INA_PANEL_ADDR   0x40
#define INA_CARGA_ADDR   0x41
#define INA_BAT_ADDR     0x45

#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    32

#define LORA_FREQ        433.0

#define MOSFET_PIN 12

float Voc = NAN;
float Isc = NAN;
float Vshort = NAN;
float Pmax_est = NAN;

const float fillFactor = 0.75;

// Panel nuevo: 6 V, 0.75 W, 150 mA.
// FF queda en 1.00 para no subestimar Pmax hasta calibrar con datos reales.
const float fillFactor = 1.00;
const float PANEL_V_NOMINAL_V = 6.0;
const float PANEL_IMAX_NOMINAL_MA = 150.0;
const float PANEL_PMAX_NOMINAL_MW = 750.0;
const bool LIMITAR_PMAX_PANEL = false;  // Cambiar a true si luego se quiere cortar a 750 mW.

// Frecuencias lógicas de operación.
const uint32_t SAMPLE_INTERVAL_P0_SEC = 120;
const uint32_t SAMPLE_INTERVAL_SEC    = 60;
const uint32_t LORA_INTERVAL_SEC      = 60;

// Seguridad ante RTC loco: nunca dormir más de 30 min.
const uint32_t MAX_SLEEP_SEC = 1800;


// ====================== OBJETOS ======================
Adafruit_INA219 inaPanel(INA_PANEL_ADDR);
Adafruit_INA219 inaCarga(INA_CARGA_ADDR);
Adafruit_INA219 inaBat(INA_BAT_ADDR);

Adafruit_LC709203F lc;
RTC_PCF8523 rtc;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

SX1276 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST);

File dataFile;
char filename[13];

// ====================== FLAGS ======================
bool sdOK = false;
bool oledOK = false;
bool rtcOK = false;
bool lcOK = false;
bool loraOK = false;

bool inaPanelOK = false;
bool inaCargaOK = false;
bool inaBatOK = false;

// ====================== CONTADORES ======================
// packetID identifica el slot. Aumenta al iniciar un slot nuevo.
// Si el slot no se guarda en SD, queda un salto útil para detectar pérdida.
uint32_t packetID = 0;

// txID identifica cada transmisión LoRa.
uint32_t txID = 0;

// Control temporal.
uint32_t lastSampleUnix = 0;
uint32_t lastLoRaUnix = 0;

// ====================== PLANES ======================
enum PlanEnergetico {
  PLAN_P0 = 0,
  PLAN_P1,
  PLAN_P2,
  PLAN_P3
};

PlanEnergetico planActual = PLAN_P0;
PlanEnergetico planSlot = PLAN_P0;

uint8_t slotActual = 255;
DateTime inicioSlotRTC;

// ====================== VARIABLES MEDIDAS ======================
// Tensiones en V, corrientes en mA, potencias en mW.
float Vp = 0, Ip = 0, Pp = 0;        // Panel: potencia entregada real.
float Vc = 0, Ic = 0, Pc = 0;        // Carga/nodo.
float Vb = 0, Ib = 0, Pb = 0;        // Batería.
float Vlc = 0, SoC = 0;              // Fuel gauge.

float Voc = 0, Isc = 0, Vshort = 0;
float Pmax_est = 0;                  // Potencia generable estimada, en mW.

// ====================== ACUMULADOR ======================
struct AcumuladorSlot {
  float Vp = 0, Ip = 0, Pp = 0;
  float Vc = 0, Ic = 0, Pc = 0;
  float Vb = 0, Ib = 0, Pb = 0;
  float Vlc = 0, SoC = 0;
  float Voc = 0, Isc = 0, Vshort = 0, Pmax = 0;
  uint16_t n = 0;
};

AcumuladorSlot acc;

// ====================== UTILIDADES ======================
float sanitizar(float x) {
  if (isnan(x) || isinf(x)) return 0.0;
  return x;
}

uint32_t intervaloMuestreoSec(PlanEnergetico p) {
  if (p == PLAN_P0) return SAMPLE_INTERVAL_P0_SEC;
  return SAMPLE_INTERVAL_SEC;
}


void medirVocIsc() {
  const int N = 5;

  float sumaVoc = 0;
  float sumaIsc = 0;
  float sumaVshort = 0;

  if (!inaPanelOK) {
    Voc = NAN;
    Isc = NAN;
    Vshort = NAN;
    Pmax_est = NAN;
    Serial.println("No se mide Voc/Isc: INA panel no disponible");
    return;
  }

  digitalWrite(MOSFET_PIN, LOW);
  delay(20);

  relayUnset();
  delay(150);

  for (int i = 0; i < N; i++) {
    sumaVoc += sanitizar(inaPanel.getBusVoltage_V());
    delay(5);
  }

  Voc = sanitizar(sumaVoc / N);

  digitalWrite(MOSFET_PIN, HIGH);
  delay(50);

  for (int i = 0; i < N; i++) {
    sumaVshort += sanitizar(inaPanel.getBusVoltage_V());
    sumaIsc += abs(sanitizar(inaPanel.getCurrent_mA()));
    delay(5);
  }

  digitalWrite(MOSFET_PIN, LOW);

  Vshort = sanitizar(sumaVshort / N);
  Isc = sanitizar(sumaIsc / N);

  relaySet();
  delay(50);

  Pmax_est = sanitizar(Voc * Isc * fillFactor);

  Serial.print("Voc: "); Serial.print(Voc, 3);
  Serial.print(" V | Isc: "); Serial.print(Isc, 2);
  Serial.print(" mA | Vshort: "); Serial.print(Vshort, 3);
  Serial.print(" V | Pmax_est: "); Serial.print(Pmax_est, 1);
  Serial.println(" mW");
}
// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Inicio planificador por umbrales de SoC");

  Wire.begin();
  Wire.setClock(100000);
  SPI.begin();

  pinMode(SD_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);

  pinMode(RELAY_SET_PIN, OUTPUT);
  pinMode(RELAY_UNSET_PIN, OUTPUT);
  pinMode(MOSFET_PIN, OUTPUT);

  digitalWrite(RELAY_SET_PIN, LOW);
  digitalWrite(RELAY_UNSET_PIN, LOW);
  digitalWrite(MOSFET_PIN, LOW);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  relaySet();

  iniciarSensores();
  iniciarOLED();
  iniciarRTC();
  iniciarSD();
  iniciarLoRa();

  dormirINAs();
  apagarOLED();

  DateTime now = rtc.now();
  slotActual = obtenerSlotRTC(now);
  iniciarNuevoSlot(now);

  Serial.println("Setup terminado");
}

// ====================== LOOP ======================
void loop() {
  DateTime now = rtc.now();
  uint8_t slotNuevo = obtenerSlotRTC(now);

  if (slotNuevo != slotActual) {
    finalizarSlot(now);

    slotActual = slotNuevo;
    iniciarNuevoSlot(now);

    now = rtc.now();
  }

  uint32_t nowUnix = now.unixtime();

  if (nowUnix - lastSampleUnix >= intervaloMuestreoSec(planActual)) {
    tomarMuestraPlan();
    lastSampleUnix = nowUnix;
  }

  if (planActual != PLAN_P0 && nowUnix - lastLoRaUnix >= LORA_INTERVAL_SEC) {
    enviarResumenLoRa(now);
    lastLoRaUnix = nowUnix;
  }

  dormirHastaProximoEvento();
}

// ====================== INICIALIZACION ======================
void iniciarSensores() {
  inaPanelOK = inaPanel.begin();
  inaCargaOK = inaCarga.begin();
  inaBatOK   = inaBat.begin();

  Serial.print("INA panel: ");
  Serial.println(inaPanelOK ? "OK" : "ERROR");

  Serial.print("INA carga: ");
  Serial.println(inaCargaOK ? "OK" : "ERROR");

  Serial.print("INA bat: ");
  Serial.println(inaBatOK ? "OK" : "ERROR");

  lcOK = lc.begin();

  if (lcOK) {
    lc.setPackSize((lc709203_adjustment_t)0x09);
    Serial.println("LC709203F OK");
    Serial.println("APA custom = 0x09 (~130 mAh)");
  } else {
    Serial.println("LC709203F ERROR");
  }
}

void iniciarOLED() {
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.print("OLED: ");
  Serial.println(oledOK ? "OK" : "ERROR");

  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Sistema listo");
    display.display();
    delay(1000);
  }
}

void iniciarRTC() {
  rtcOK = rtc.begin();

  if (!rtcOK) {
    Serial.println("ERROR RTC");
    while (1) delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC no inicializado, ajustando...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("RTC OK");
}

void iniciarSD() {
  digitalWrite(LORA_CS, HIGH);
  delay(2);

  sdOK = SD.begin(SD_CS);
  Serial.print("SD: ");
  Serial.println(sdOK ? "OK" : "ERROR");

  if (sdOK) {
    crearArchivo();
  }

  digitalWrite(SD_CS, HIGH);
}

void iniciarLoRa() {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  delay(2);

  int state = radio.begin(
    LORA_FREQ,
    125.0,
    7,
    5,
    0x12,
    20,
    8
  );

  if (state == RADIOLIB_ERR_NONE) {
    loraOK = true;
    radio.setCRC(true);
    radio.sleep();
    Serial.println("LoRa OK");
  } else {
    loraOK = false;
    Serial.print("LoRa ERROR: ");
    Serial.println(state);
  }

  digitalWrite(LORA_CS, HIGH);
}

// ====================== PLANIFICACION ======================
uint8_t obtenerSlotRTC(DateTime now) {
  return now.hour() * 2 + (now.minute() >= 30 ? 1 : 0);
}

PlanEnergetico seleccionarPlan(float soc) {
  if (soc < 40.0) return PLAN_P0;
  if (soc < 60.0) return PLAN_P1;
  if (soc < 80.0) return PLAN_P2;
  return PLAN_P3;
}

const char* nombrePlan(PlanEnergetico p) {
  switch (p) {
    case PLAN_P0: return "P0";
    case PLAN_P1: return "P1";
    case PLAN_P2: return "P2";
    case PLAN_P3: return "P3";
    default: return "PX";
  }
}

uint8_t calidadPlan(PlanEnergetico p) {
  switch (p) {
    case PLAN_P0: return 10;
    case PLAN_P1: return 20;
    case PLAN_P2: return 60;
    case PLAN_P3: return 100;
    default: return 0;
  }
}

void iniciarNuevoSlot(DateTime now) {
  inicioSlotRTC = now;
  resetAcumulador();

  packetID++;

  despertarINAs();
  medirVocIsc();
  leerSensores();
  dormirINAs();

  planActual = seleccionarPlan(SoC);
  planSlot = planActual;

  uint32_t t0 = now.unixtime();
  lastSampleUnix = t0 - intervaloMuestreoSec(planActual);
  lastLoRaUnix = t0 - LORA_INTERVAL_SEC;

  if (planActual == PLAN_P2 || planActual == PLAN_P3) {
    iniciarRecepcionContinuaLoRa();
  } else {
    dormirLoRa();
  }

  if (planActual == PLAN_P3) {
    mostrarPantalla();
  } else {
    apagarOLED();
  }

  Serial.print("Nuevo slot ");
  Serial.print(slotActual);
  Serial.print(" | PKT=");
  Serial.print(packetID);
  Serial.print(" | SoC=");
  Serial.print(SoC, 1);
  Serial.print("% | Plan=");
  Serial.println(nombrePlan(planActual));
}

void finalizarSlot(DateTime now) {
  guardarPromedioSlot(now);

  Serial.print("Fin slot ");
  Serial.print(slotActual);
  Serial.print(" | PKT=");
  Serial.print(packetID);
  Serial.print(" | muestras=");
  Serial.println(acc.n);
}

void tomarMuestraPlan() {
  despertarINAs();

  leerSensores();
  medirVocIsc();

  acumularMuestra();

  dormirINAs();

  if (planActual == PLAN_P3) {
    mostrarPantalla();
  } else {
    apagarOLED();
  }
}

// ====================== SLEEP ======================
void dormirHastaProximoEvento() {
  if (planActual == PLAN_P2 || planActual == PLAN_P3) {
    delay(200);
    return;
  }

  DateTime now = rtc.now();
  uint32_t nowUnix = now.unixtime();
  uint32_t proximoSlotUnix = obtenerProximoCorteSlot(now).unixtime();

  uint32_t proximaMuestra = lastSampleUnix + intervaloMuestreoSec(planActual);
  uint32_t nextEventUnix = min(proximaMuestra, proximoSlotUnix);

  if (planActual == PLAN_P1) {
    uint32_t proximoLoRa = lastLoRaUnix + LORA_INTERVAL_SEC;
    nextEventUnix = min(nextEventUnix, proximoLoRa);
  }

  if (nextEventUnix <= nowUnix) return;

  uint32_t sleepSec = nextEventUnix - nowUnix;

  if (sleepSec < 1) return;

  if (sleepSec > MAX_SLEEP_SEC) {
    sleepSec = 30;
  }

  dormirLoRa();

  Serial.print("Sleep sec: ");
  Serial.println(sleepSec);
  Serial.flush();

  LowPower.sleep(sleepSec * 1000UL);
}

DateTime obtenerProximoCorteSlot(DateTime now) {
  if (now.minute() < 30) {
    return DateTime(now.year(), now.month(), now.day(), now.hour(), 30, 0);
  } else {
    DateTime nextHour = now + TimeSpan(0, 1, 0, 0);
    return DateTime(nextHour.year(), nextHour.month(), nextHour.day(), nextHour.hour(), 0, 0);
  }
}

// ====================== INA POWER SAVE ======================
void dormirINAs() {
  if (inaPanelOK) inaPanel.powerSave(true);
  if (inaCargaOK) inaCarga.powerSave(true);
  if (inaBatOK)   inaBat.powerSave(true);
}

void despertarINAs() {
  if (inaPanelOK) inaPanel.powerSave(false);
  if (inaCargaOK) inaCarga.powerSave(false);
  if (inaBatOK)   inaBat.powerSave(false);

  delay(25);
}

// ====================== ACUMULADOR ======================
void resetAcumulador() {
  acc = AcumuladorSlot();
}

void acumularMuestra() {
  acc.Vp += Vp;
  acc.Ip += Ip;
  acc.Pp += Pp;

  acc.Vc += Vc;
  acc.Ic += Ic;
  acc.Pc += Pc;

  acc.Vb += Vb;
  acc.Ib += Ib;
  acc.Pb += Pb;

  acc.Vlc += Vlc;
  acc.SoC += SoC;

  acc.Voc += Voc;
  acc.Isc += Isc;
  acc.Vshort += Vshort;
  acc.Pmax += Pmax_est;

  acc.n++;
}

// ====================== PROMEDIOS ======================
bool obtenerPromediosParciales(
  float &avgPmax,
  float &avgPpanel,
  float &avgPnodo,
  float &avgPbatt,
  float &avgSoC,
  float &avgVb
) {
  if (acc.n == 0) return false;

  float n = acc.n;

  avgPmax   = acc.Pmax / n;
  avgPpanel = acc.Pp / n;
  avgPnodo  = acc.Pc / n;
  avgPbatt  = acc.Pb / n;
  avgSoC    = acc.SoC / n;
  avgVb     = acc.Vb / n;

  return true;
}

// ====================== RELAY ======================
void relaySet() {
  digitalWrite(RELAY_SET_PIN, HIGH);
  delay(15);
  digitalWrite(RELAY_SET_PIN, LOW);
}

void relayUnset() {
  digitalWrite(RELAY_UNSET_PIN, HIGH);
  delay(15);
  digitalWrite(RELAY_UNSET_PIN, LOW);
}

// ====================== MEDICIONES ======================
void leerSensores() {
  const int N = 5;

  float sumVp = 0, sumIp = 0, sumPp = 0;
  float sumVc = 0, sumIc = 0, sumPc = 0;
  float sumVb = 0, sumIb = 0, sumPb = 0;
  float sumVlc = 0, sumSoC = 0;

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
      sumSoC += sanitizar(lc.cellPercent());
    }

    delay(10);
  }

  if (inaPanelOK) {
    Vp = sanitizar(sumVp / N);
    Ip = sanitizar(sumIp / N);
    Pp = sanitizar(sumPp / N);
  } else {
    Vp = 0;
    Ip = 0;
    Pp = 0;
  }

  if (inaCargaOK) {
    Vc = sanitizar(sumVc / N);
    Ic = sanitizar(sumIc / N);
    Pc = sanitizar(sumPc / N);
  } else {
    Vc = 0;
    Ic = 0;
    Pc = 0;
  }

  if (inaBatOK) {
    Vb = sanitizar(sumVb / N);
    Ib = sanitizar(sumIb / N);
    Pb = sanitizar(sumPb / N);
  } else {
    Vb = 0;
    Ib = 0;
    Pb = 0;
  }

  if (lcOK) {
    Vlc = sanitizar(sumVlc / N);
    SoC = sanitizar(sumSoC / N);
  } else {
    Vlc = 0;
    SoC = 0;
  }
}

void medirVocIsc() {
  const int N = 5;

  float sumaVoc = 0;
  float sumaIsc = 0;
  float sumaVshort = 0;

  if (!inaPanelOK) {
    Voc = 0;
    Isc = 0;
    Vshort = 0;
    Pmax_est = 0;
    Serial.println("No se mide Voc/Isc: INA panel no disponible");
    return;
  }

  digitalWrite(MOSFET_PIN, LOW);
  delay(20);

  relayUnset();
  delay(150);

  for (int i = 0; i < N; i++) {
    sumaVoc += sanitizar(inaPanel.getBusVoltage_V());
    delay(5);
  }

  Voc = sanitizar(sumaVoc / N);

  digitalWrite(MOSFET_PIN, HIGH);
  delay(50);

  for (int i = 0; i < N; i++) {
    sumaVshort += sanitizar(inaPanel.getBusVoltage_V());
    sumaIsc += abs(sanitizar(inaPanel.getCurrent_mA()));
    delay(5);
  }

  digitalWrite(MOSFET_PIN, LOW);

  Vshort = sanitizar(sumaVshort / N);
  Isc = sanitizar(sumaIsc / N);

  relaySet();
  delay(50);

  // Releer panel en estado normal.
  Vp = sanitizar(inaPanel.getBusVoltage_V());
  Ip = sanitizar(inaPanel.getCurrent_mA());
  Pp = sanitizar(Vp * Ip);  // V * mA = mW.

  Pmax_est = sanitizar(Voc * Isc * fillFactor);  // V * mA = mW.

  if (LIMITAR_PMAX_PANEL && Pmax_est > PANEL_PMAX_NOMINAL_MW) {
    Pmax_est = PANEL_PMAX_NOMINAL_MW;
  }

  Serial.print("Voc: "); Serial.print(Voc, 3);
  Serial.print(" V | Isc: "); Serial.print(Isc, 2);
  Serial.print(" mA | Vshort: "); Serial.print(Vshort, 3);
  Serial.print(" V | Pmax: "); Serial.print(Pmax_est, 1);
  Serial.println(" mW");
}

// ====================== OLED ======================
void mostrarPantalla() {
  if (!oledOK) return;

  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Plan:");
  display.print(nombrePlan(planActual));
  display.print(" S:");
  display.print(slotActual);

  display.setCursor(0, 10);
  display.print("SoC:");
  display.print(SoC, 0);
  display.print("% Vb:");
  display.print(Vb, 2);

  display.setCursor(0, 20);
  display.print("Pm:");
  display.print(Pmax_est, 0);
  display.print(" Pp:");
  display.print(Pp, 0);

  display.display();
}

void apagarOLED() {
  if (!oledOK) return;

  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

// ====================== LORA ======================
void dormirLoRa() {
  if (!loraOK) return;

  radio.sleep();
  digitalWrite(LORA_CS, HIGH);
}

void iniciarRecepcionContinuaLoRa() {
  if (!loraOK) return;

  digitalWrite(SD_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  delay(2);

  int state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa RX ERROR: ");
    Serial.println(state);
  }

  digitalWrite(LORA_CS, HIGH);
}

void enviarResumenLoRa(DateTime now) {
  if (!loraOK) return;
  if (acc.n == 0) return;

  float avgPmax, avgPpanel, avgPnodo, avgPbatt, avgSoC, avgVb;

  if (!obtenerPromediosParciales(avgPmax, avgPpanel, avgPnodo, avgPbatt, avgSoC, avgVb)) {
    return;
  }

  txID++;

  char payload[180];

  snprintf(
    payload,
    sizeof(payload),
    "%s;PKT=%lu;TX=%lu;S=%u;%02d:%02d;%s;N=%u;Pm=%.0f;Pp=%.0f;Pc=%.0f;Pb=%.0f;SoC=%.0f",
    NODE_ID,
    (unsigned long)packetID,
    (unsigned long)txID,
    slotActual,
    now.hour(),
    now.minute(),
    nombrePlan(planActual),
    acc.n,
    avgPmax,
    avgPpanel,
    avgPnodo,
    avgPbatt,
    avgSoC
  );

  digitalWrite(SD_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  delay(2);

  radio.standby();

  Serial.print("LoRa TX: ");
  Serial.println(payload);

  int state = radio.transmit(payload);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa TX OK");
  } else {
    Serial.print("LoRa TX ERROR: ");
    Serial.println(state);
  }

  if (planActual == PLAN_P2 || planActual == PLAN_P3) {
    iniciarRecepcionContinuaLoRa();
  } else {
    dormirLoRa();
  }

  digitalWrite(LORA_CS, HIGH);
}

// ====================== SD ======================
void crearArchivo() {
  DateTime now = rtc.now();

  // Archivo diario: YYMMDD.CSV
  sprintf(filename, "%02d%02d%02d.CSV",
          now.year() % 100, now.month(), now.day());

  digitalWrite(LORA_CS, HIGH);
  delay(2);

  bool existe = SD.exists(filename);

  dataFile = SD.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("ERROR creando/abriendo CSV");
    sdOK = false;
    return;
  }

  if (!existe || dataFile.size() == 0) {
    dataFile.println(
      "ID;Packet;Fecha;Hora;Slot;Plan;Q;N;"
      "Pmax_est_mW;Ppanel_mW;Pnodo_mW;Pbatt_mW;"
      "Vp_V;Ip_mA;"
      "Vc_V;Ic_mA;"
      "Vb_V;Ib_mA;"
      "Vlc_V;SoC_pct;"
      "Voc_V;Isc_mA;Vshort_V;Pmax_mW"
    );
  }

  dataFile.close();
  digitalWrite(SD_CS, HIGH);

  Serial.print("Archivo abierto: ");
  Serial.println(filename);
}

void guardarPromedioSlot(DateTime now) {
  if (!sdOK) return;
  if (acc.n == 0) return;

  float n = acc.n;

  float avgVp = acc.Vp / n;
  float avgIp = acc.Ip / n;
  float avgPp = acc.Pp / n;

  float avgVc = acc.Vc / n;
  float avgIc = acc.Ic / n;
  float avgPc = acc.Pc / n;

  float avgVb = acc.Vb / n;
  float avgIb = acc.Ib / n;
  float avgPb = acc.Pb / n;

  float avgVlc = acc.Vlc / n;
  float avgSoC = acc.SoC / n;

  float avgVoc = acc.Voc / n;
  float avgIsc = acc.Isc / n;
  float avgVshort = acc.Vshort / n;
  float avgPmax = acc.Pmax / n;

  // Sacar LoRa del bus SPI antes de usar SD.
  // Sacar LoRa del bus SPI antes de usar SD
  if (loraOK) {
    radio.standby();
    delay(5);
    radio.sleep();
    delay(5);
  }

  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  delay(5);

  dataFile = SD.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("ERROR abriendo CSV");
    digitalWrite(SD_CS, HIGH);

    if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) {
      iniciarRecepcionContinuaLoRa();
    }

    return;
  }

  char fecha[11];
  char hora[9];

  sprintf(fecha, "%04d-%02d-%02d",
          inicioSlotRTC.year(), inicioSlotRTC.month(), inicioSlotRTC.day());

  sprintf(hora, "%02d:%02d:%02d",
          inicioSlotRTC.hour(), inicioSlotRTC.minute(), inicioSlotRTC.second());

  dataFile.print(NODE_ID); dataFile.print(";");
  dataFile.print(packetID); dataFile.print(";");
  dataFile.print(fecha); dataFile.print(";");
  dataFile.print(hora); dataFile.print(";");
  dataFile.print(slotActual); dataFile.print(";");
  dataFile.print(nombrePlan(planSlot)); dataFile.print(";");
  dataFile.print(calidadPlan(planSlot)); dataFile.print(";");
  dataFile.print(acc.n); dataFile.print(";");

  dataFile.print(avgPmax, 3); dataFile.print(";");  // Pmax_est_mW
  dataFile.print(avgPp, 3); dataFile.print(";");    // Ppanel_mW
  dataFile.print(avgPc, 3); dataFile.print(";");    // Pnodo_mW
  dataFile.print(avgPb, 3); dataFile.print(";");    // Pbatt_mW

  dataFile.print(avgVp, 4); dataFile.print(";");
  dataFile.print(avgIp, 3); dataFile.print(";");

  dataFile.print(avgVc, 4); dataFile.print(";");
  dataFile.print(avgIc, 3); dataFile.print(";");

  dataFile.print(avgVb, 4); dataFile.print(";");
  dataFile.print(avgIb, 3); dataFile.print(";");

  dataFile.print(avgVlc, 4); dataFile.print(";");
  dataFile.print(avgSoC, 2); dataFile.print(";");

  dataFile.print(avgVoc, 4); dataFile.print(";");
  dataFile.print(avgIsc, 3); dataFile.print(";");
  dataFile.print(avgVshort, 4); dataFile.print(";");
  dataFile.print(avgPmax, 3);

  dataFile.println();
  dataFile.close();

  digitalWrite(SD_CS, HIGH);

  if (loraOK && (planActual == PLAN_P2 || planActual == PLAN_P3)) {
    iniciarRecepcionContinuaLoRa();
  }

  Serial.println("Promedio final de slot guardado en SD");
}