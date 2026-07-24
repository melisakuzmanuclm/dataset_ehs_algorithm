#ifndef TIPOS_PLANIFICADOR_H
#define TIPOS_PLANIFICADOR_H

// ====================== PLANES ======================
enum PlanEnergetico {
  PLAN_P0 = 0,  // bajo consumo: sleep largo, sin LoRa, sin OLED
  PLAN_P1,      // muestreo normal, LoRa TX periódico, sin OLED
  PLAN_P2,      // muestreo normal, LoRa RX continuo, sin OLED
  PLAN_P3       // máxima calidad: LoRa RX continuo + OLED encendido
};

// ====================== RESULTADO DE SIMULACIÓN ======================

// Resultado de simular un slot individual
struct ResultadoSlot {
  float soc_next;      // SoC al final del slot [%]
  float soc_min;       // SoC mínimo visto en el slot [%]
  float spillage_mWh;  // energía desperdiciada por C-rate o batería llena [mWh]
  bool  violation;     // true si soc_min < SOC_MIN
};

// Resultado de simular un día completo
struct ResultadoDia {
  bool violacionSoC;      // al menos un slot bajó del SOC_MIN
  bool violacionNeutral;  // soc_final < soc_ini: no se cumple la neutralidad energética
};

// ====================== MEDICIONES ======================
struct Mediciones {
  float vPanel, iPanel, pPanel;
  float vCarga, iCarga, pCarga;
  float vBatINA, iBat, pBat;
  float vBatLC, soc;
};

// ====================== ACUMULADOR DE SLOT ======================
// Acumula muestras durante el slot para guardar el promedio al final
struct AcumuladorSlot {
  float vPanel, iPanel, pPanel;
  float vCarga, iCarga, pCarga;
  float vBatINA, iBat, pBat;
  float vBatLC, soc;
  float voc, isc, vshort, pmax;
  float temp;
  float socUltimo;  // ultimo valor de SoC muestreado (no promedio)
  uint16_t n;  // cantidad de muestras acumuladas
};

#endif
