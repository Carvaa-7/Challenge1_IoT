/*
 * ========================================================================
 * HydroSentry - Challenge #2 - Firmware completo integrado
 * ========================================================================
 * Módulo 1: Lectura de sensores en tarea FreeRTOS (core 0)
 * Módulo 2: Lógica de fusión (riesgo base + Índice de Estrés Evaporativo)
 * Módulo 3: Evapotranspiración (Hargreaves-Samani + validación por balance
 *           de masa contra el descenso real medido por el HC-SR04)
 * Módulo 4: Servidor web embebido (modo Station, WLAN local, autenticación
 *           básica, histórico en buffer circular, botón de silenciar alarma)
 * Módulo 5: Alarma física (LEDs tipo semáforo + buzzer no bloqueante) y
 *           LCD 16x2 sin botones (ciclo automático de pantallas)
 *
 * DECISIÓN DE ARQUITECTURA IMPORTANTE:
 * El loop() principal NUNCA usa delay() bloqueante. Todo se programa con
 * millis() (cooperative multitasking), porque el servidor web necesita
 * atender peticiones (server.handleClient()) constantemente. Un delay()
 * largo en el loop() congelaría el tablero web cada vez que se ejecuta.
 * Esto es una decisión de ingeniería que se documenta en la wiki.
 *
 * Librerías necesarias (Gestor de Bibliotecas de Arduino IDE):
 *   - Adafruit BME280 Library (+ Adafruit Unified Sensor, Adafruit BusIO)
 *   - BH1750 (Christopher Laws)
 *   - HX711 (bogde)
 *   - LiquidCrystal (la de Arduino, NO la I2C -> el LCD que tienes es de
 *     16 pines en paralelo, sin backpack I2C)
 *   - WebServer, WiFi, Wire, time.h -> incluidas en el core de ESP32, no
 *     hace falta instalarlas aparte.
 * ========================================================================
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <HX711.h>
#include <LiquidCrystal.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <math.h>

// ========================================================================
// SECCIÓN 0 — CONFIGURACIÓN QUE DEBES AJUSTAR TÚ (marcado con TODO)
// ========================================================================

// ---- WiFi: la WLAN local a la que se conecta el ESP32 (modo Station) ----
const char* WIFI_SSID     = "TODO_NOMBRE_DE_TU_WLAN";      // TODO: pon el SSID real
const char* WIFI_PASSWORD = "TODO_CLAVE_DE_TU_WLAN";       // TODO: pon la clave real

// ---- Autenticación básica del tablero web (acceso restringido) ----
const char* WEB_USER = "hydrosentry";          // TODO: cámbialo si quieres
const char* WEB_PASS = "chia2026";             // TODO: cámbialo si quieres

// ---- Zona horaria (Bogotá, UTC-5) y NTP para tener hora real ----
const long  GMT_OFFSET_SEC = -5 * 3600;
const int   DAYLIGHT_OFFSET_SEC = 0;
const char* NTP_SERVER = "pool.ntp.org";

// ---- Si la WLAN NO tiene salida a internet, no habrá NTP. En ese caso
// se usa este día juliano fijo como respaldo para el cálculo de Ra
// (afecta solo la precisión del modelo de evaporación, no la fusión
// principal ni la alarma). Ajusta al día real si vas a probar offline. ----
const int DIA_JULIANO_FALLBACK = 258; // aprox. 15-sep; ajustar el día de la prueba

// ---- Latitud de Chía, Cundinamarca (punto crítico elegido) ----
const float LATITUD_GRADOS = 4.86;

// ---- Geometría real del tanque (de la especificación ya cerrada) ----
const float TANQUE_DIAMETRO_CM = 14.0;
const float TANQUE_AREA_CM2 = PI * pow(TANQUE_DIAMETRO_CM / 2.0, 2); // ~153.9 cm²

// ---- Calibración del HC-SR04 según montaje del puente ----
const float SENSOR_ALTURA_CM = 10.54;  // altura del sensor sobre la base del tanque
const float NIVEL_MAX_CM     = 5.44;   // límite operativo (80%) para proteger la celda

// ---- Calibración del HX711: PENDIENTE, recalibrar con el montaje 3D final ----
// !!! PROVISIONAL - NO CONFIABLE TODAVIA !!!
// Calibrado el 20-sep con 12 puntos y regresión lineal, pero el error salió
// muy alto (promedio 73.8g, máximo 148g sobre un rango de ~1000g = hasta 15%
// de error). Las lecturas crudas mostraron saltos de signo (positivo a
// negativo) con pesos positivos, lo que indica torsión mecánica en la celda,
// no solo ruido: revisar que quede perfectamente horizontal, el extremo fijo
// realmente inmóvil, y el objeto siempre centrado en el mismo punto antes de
// repetir la calibración con 04_calibracion_galga.ino.
const float FACTOR_CALIBRACION_HX711 = 6780.9158;
const float PESO_MAX_TANQUE_G = 1000.0;       // peso del tanque lleno al 80% (~1000mL)

// ========================================================================
// SECCIÓN 1 — PINES
// ========================================================================
#define I2C_SDA 21
#define I2C_SCL 22

#define HCSR04_TRIG 5
#define HCSR04_ECHO 18

#define HX711_DOUT 4
#define HX711_SCK  2

#define PIN_BUZZER   27
#define PIN_LED_VERDE   32   // Normal
#define PIN_LED_AMARILLO 33  // Preventivo
#define PIN_LED_NARANJA  25  // Alto
#define PIN_LED_ROJO     26  // Crítico

#define PIN_BOMBA 14 // pin IN del módulo relé que enciende/apaga la bombita sumergible.
                     // OJO: si tu módulo relé es "activo en bajo" (muy común), invierte
                     // la lógica HIGH/LOW en manejarApiBomba() y en el apagado de setup().
                     // Pruébalo primero con un sketch simple antes de confiar en esto.

// LCD 16x2 en modo paralelo (SIN backpack I2C):
//   LiquidCrystal(RS, E, D4, D5, D6, D7)
// Conexiones fijas fuera de estos pines: VSS->GND, VDD->5V, RW->GND,
// VEE->cursor del potenciometro de contraste (10k entre 5V y GND),
// LEDA->5V (con resistencia ~220ohm si el LCD no la trae), LEDK->GND.
#define LCD_RS 15
#define LCD_E  13
#define LCD_D4 12
#define LCD_D5 23
#define LCD_D6 19
#define LCD_D7 17

// ========================================================================
// SECCIÓN 2 — OBJETOS GLOBALES
// ========================================================================
Adafruit_BME280 bme;
BH1750 lightMeter;
HX711 scale;
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
WebServer server(80);

// ========================================================================
// SECCIÓN 3 — ESTRUCTURA COMPARTIDA DE SENSORES (protegida por mutex)
// ========================================================================
struct SensorData {
  float temperatura;
  float humedad;
  float presion;
  float lux;
  float nivel_cm;
  float nivel_pct;
  float peso_g;
  bool  bme_ok;
  bool  bh1750_ok;
  bool  hcsr04_ok;
  bool  hx711_ok;
  unsigned long timestamp_ms;
};

SensorData datos;
SemaphoreHandle_t mutexDatos;

// ========================================================================
// SECCIÓN 4 — MÓDULO 2: LÓGICA DE FUSIÓN
// ========================================================================
enum EstadoRiesgo { RIESGO_NORMAL = 0, RIESGO_PREVENTIVO = 1, RIESGO_ALTO = 2, RIESGO_CRITICO = 3 };

struct ResultadoFusion {
  float riesgo_nivel;
  float riesgo_peso;
  float riesgo_base;
  bool  discrepancia_sensores;
  float IEE;
  bool  escalado;
  float riesgo_final;
  EstadoRiesgo estado;
};

const float UMBRAL_PREVENTIVO = 25.0;
const float UMBRAL_ALTO       = 50.0;
const float UMBRAL_CRITICO    = 75.0;
const float UMBRAL_DISCREPANCIA = 20.0;

const float TEMP_MIN = 15.0, TEMP_MAX = 30.0;
const float HR_MIN   = 30.0, HR_MAX   = 100.0;
const float LUX_MIN  = 0.0,  LUX_MAX  = 100000.0;

const float PESO_TEMP = 0.4;
const float PESO_HR   = 0.3;
const float PESO_LUX  = 0.3;
const float UMBRAL_IEE_CRITICO = 0.7;

float normalizar(float valor, float minimo, float maximo) {
  float n = (valor - minimo) / (maximo - minimo);
  return constrain(n, 0.0f, 1.0f);
}

ResultadoFusion calcularFusion(const SensorData &d) {
  ResultadoFusion r;

  r.riesgo_nivel = 100.0 - d.nivel_pct;
  float peso_pct = constrain((d.peso_g / PESO_MAX_TANQUE_G) * 100.0f, 0.0f, 100.0f);
  r.riesgo_peso  = 100.0 - peso_pct;

  float diferencia = fabs(r.riesgo_nivel - r.riesgo_peso);
  r.discrepancia_sensores = (diferencia > UMBRAL_DISCREPANCIA);

  if (r.discrepancia_sensores) {
    r.riesgo_base = max(r.riesgo_nivel, r.riesgo_peso); // conservador: el peor caso
  } else {
    r.riesgo_base = (r.riesgo_nivel + r.riesgo_peso) / 2.0;
  }

  float n_temp = normalizar(d.temperatura, TEMP_MIN, TEMP_MAX);
  float n_hr   = 1.0 - normalizar(d.humedad, HR_MIN, HR_MAX);
  float n_lux  = normalizar(d.lux, LUX_MIN, LUX_MAX);
  r.IEE = PESO_TEMP * n_temp + PESO_HR * n_hr + PESO_LUX * n_lux;

  r.escalado = false;
  r.riesgo_final = r.riesgo_base;
  if (r.IEE > UMBRAL_IEE_CRITICO && r.riesgo_base >= UMBRAL_PREVENTIVO) {
    r.riesgo_final = r.riesgo_base + 25.0;
    r.escalado = true;
  }
  r.riesgo_final = constrain(r.riesgo_final, 0.0f, 100.0f);

  if (r.riesgo_final < UMBRAL_PREVENTIVO) r.estado = RIESGO_NORMAL;
  else if (r.riesgo_final < UMBRAL_ALTO) r.estado = RIESGO_PREVENTIVO;
  else if (r.riesgo_final < UMBRAL_CRITICO) r.estado = RIESGO_ALTO;
  else r.estado = RIESGO_CRITICO;

  return r;
}

const char* nombreEstado(EstadoRiesgo e) {
  switch (e) {
    case RIESGO_NORMAL:     return "NORMAL";
    case RIESGO_PREVENTIVO: return "PREVENTIVO";
    case RIESGO_ALTO:       return "ALTO";
    case RIESGO_CRITICO:    return "CRITICO";
  }
  return "DESCONOCIDO";
}

// ========================================================================
// SECCIÓN 5 — MÓDULO 3: EVAPOTRANSPIRACIÓN (Hargreaves-Samani)
// ========================================================================

// Ventana de acumulación para Tmin/Tmax/Tmedia. En producción real debería
// ser 24h; para pruebas de banco se deja configurable y corto para poder
// demostrarlo en la sustentación sin esperar un día completo. Se documenta
// esta decisión explícitamente en la wiki como simplificación de banco de
// pruebas, con la fórmula preparada para 24h en despliegue real.
const unsigned long VENTANA_EVAP_MS = 60UL * 60UL * 1000UL; // 1 hora

struct VentanaEvaporacion {
  float t_min;
  float t_max;
  float t_suma;
  unsigned long muestras;
  float nivel_inicio_cm;
  unsigned long inicio_ms;
  bool bomba_activada_en_ventana; // si se activó la bomba, se invalida el balance de masa
  bool valida;
};

VentanaEvaporacion ventana;

struct ResultadoEvaporacion {
  float ET0_mm_dia;      // evapotranspiración de referencia calculada (Hargreaves-Samani)
  float evap_calc_ml;    // volumen esperado perdido en la ventana, según el modelo
  float evap_medida_ml;  // volumen realmente perdido, según el HC-SR04
  float error_pct;       // diferencia entre modelo y medición real (validación cruzada)
  bool  disponible;      // true si ya se completó al menos una ventana
};

ResultadoEvaporacion ultimoResultadoEvap;

void reiniciarVentanaEvaporacion(float nivel_actual_cm) {
  ventana.t_min = 1000;
  ventana.t_max = -1000;
  ventana.t_suma = 0;
  ventana.muestras = 0;
  ventana.nivel_inicio_cm = nivel_actual_cm;
  ventana.inicio_ms = millis();
  ventana.bomba_activada_en_ventana = false;
  ventana.valida = true;
}

void actualizarVentanaEvaporacion(float temperatura, bool bombaActivaAhora) {
  if (temperatura < ventana.t_min) ventana.t_min = temperatura;
  if (temperatura > ventana.t_max) ventana.t_max = temperatura;
  ventana.t_suma += temperatura;
  ventana.muestras++;
  if (bombaActivaAhora) ventana.bomba_activada_en_ventana = true;
}

// Día juliano a partir de la hora real (NTP) o del respaldo fijo si no hay internet
int obtenerDiaJuliano() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 100)) {
    return timeinfo.tm_yday + 1;
  }
  return DIA_JULIANO_FALLBACK;
}

// Radiación extraterrestre Ra (FAO-56), en mm/día equivalentes de evaporación
float calcularRa(float latitud_grados, int diaJuliano) {
  const float Gsc = 0.0820; // MJ/m2/min
  float phi = latitud_grados * PI / 180.0;
  float dr = 1.0 + 0.033 * cos((2.0 * PI / 365.0) * diaJuliano);
  float delta = 0.409 * sin((2.0 * PI / 365.0) * diaJuliano - 1.39);
  float ws = acos(constrain(-tan(phi) * tan(delta), -1.0f, 1.0f));

  float Ra_MJ = (24.0 * 60.0 / PI) * Gsc * dr *
                (ws * sin(phi) * sin(delta) + cos(phi) * cos(delta) * sin(ws));
  return Ra_MJ * 0.408; // conversión de MJ/m2/día a mm/día equivalentes
}

// Modelo de Hargreaves-Samani (1985)
float calcularET0(float t_media, float t_max, float t_min, float Ra_mm) {
  float rango = t_max - t_min;
  if (rango < 0) rango = 0;
  return 0.0023 * (t_media + 17.8) * sqrt(rango) * Ra_mm;
}

// Cierra la ventana, calcula ET0, y hace la validación cruzada contra el
// descenso real de nivel medido por el HC-SR04 (balance de masa)
void procesarVentanaCompleta(float nivel_actual_cm) {
  if (ventana.muestras == 0) return;

  float t_media = ventana.t_suma / ventana.muestras;
  int dia = obtenerDiaJuliano();
  float Ra_mm = calcularRa(LATITUD_GRADOS, dia);
  float ET0 = calcularET0(t_media, ventana.t_max, ventana.t_min, Ra_mm);

  float horas_ventana = (millis() - ventana.inicio_ms) / 3600000.0;
  float dias_ventana = horas_ventana / 24.0;
  float area_m2 = TANQUE_AREA_CM2 / 10000.0;
  float evap_calc_L = ET0 * area_m2 * dias_ventana;
  float evap_calc_mL = evap_calc_L * 1000.0;

  float delta_nivel_cm = nivel_actual_cm - ventana.nivel_inicio_cm; // negativo si bajó
  float evap_medida_mL = 0;
  if (delta_nivel_cm < 0) {
    evap_medida_mL = fabs(delta_nivel_cm) * TANQUE_AREA_CM2; // cm3 = mL
  }

  ultimoResultadoEvap.ET0_mm_dia = ET0;
  ultimoResultadoEvap.evap_calc_ml = evap_calc_mL;
  ultimoResultadoEvap.evap_medida_ml = evap_medida_mL;
  ultimoResultadoEvap.disponible = !ventana.bomba_activada_en_ventana; // si hubo bomba, no es válida la comparación

  if (evap_calc_mL > 0.01) {
    ultimoResultadoEvap.error_pct = fabs(evap_medida_mL - evap_calc_mL) / evap_calc_mL * 100.0;
  } else {
    ultimoResultadoEvap.error_pct = 0;
  }

  Serial.printf("[EVAPORACION] ET0=%.3fmm/dia | calc=%.1fmL | medida=%.1fmL | error=%.1f%%%s\n",
                ET0, evap_calc_mL, evap_medida_mL, ultimoResultadoEvap.error_pct,
                ventana.valida ? "" : " [VENTANA INVALIDADA POR BOMBA]");

  reiniciarVentanaEvaporacion(nivel_actual_cm);
}

// ========================================================================
// SECCIÓN 6 — HISTÓRICO (buffer circular para el tablero web)
// ========================================================================
struct MuestraHistorica {
  unsigned long ts_ms;
  float riesgo_final;
  float nivel_pct;
  float peso_g;
  float temperatura;
  uint8_t estado;
};

const int HISTORICO_TAM = 60; // ~5 minutos de historia a 5s por muestra
MuestraHistorica historico[HISTORICO_TAM];
int historicoIndice = 0;
int historicoCuenta = 0;

void registrarHistorico(const SensorData &d, const ResultadoFusion &f) {
  historico[historicoIndice].ts_ms = millis();
  historico[historicoIndice].riesgo_final = f.riesgo_final;
  historico[historicoIndice].nivel_pct = d.nivel_pct;
  historico[historicoIndice].peso_g = d.peso_g;
  historico[historicoIndice].temperatura = d.temperatura;
  historico[historicoIndice].estado = (uint8_t)f.estado;

  historicoIndice = (historicoIndice + 1) % HISTORICO_TAM;
  if (historicoCuenta < HISTORICO_TAM) historicoCuenta++;
}

// ========================================================================
// SECCIÓN 7 — ALARMA (LEDs + buzzer no bloqueante) y control de silencio
// ========================================================================
bool silenciado = false;
unsigned long silenciadoHasta = 0;
const unsigned long DURACION_SILENCIO_MS = 10UL * 60UL * 1000UL; // 10 minutos

unsigned long ultimoCambioBuzzer = 0;
bool buzzerEncendido = false;

void actualizarAlarma(EstadoRiesgo estado) {
  // LEDs tipo semáforo: solo se enciende el que corresponde al estado actual
  digitalWrite(PIN_LED_VERDE,    estado == RIESGO_NORMAL     ? HIGH : LOW);
  digitalWrite(PIN_LED_AMARILLO, estado == RIESGO_PREVENTIVO ? HIGH : LOW);
  digitalWrite(PIN_LED_NARANJA,  estado == RIESGO_ALTO       ? HIGH : LOW);
  digitalWrite(PIN_LED_ROJO,     estado == RIESGO_CRITICO    ? HIGH : LOW);

  // Si el silencio expiró, se reactiva el sonido automáticamente
  if (silenciado && millis() > silenciadoHasta) {
    silenciado = false;
  }

  if (silenciado || estado == RIESGO_NORMAL) {
    noTone(PIN_BUZZER);
    buzzerEncendido = false;
    return;
  }

  // Patrón de beep no bloqueante: intervalo distinto según severidad
  unsigned long intervalo;
  if (estado == RIESGO_PREVENTIVO) intervalo = 4000; // beep corto cada 4s
  else if (estado == RIESGO_ALTO)  intervalo = 1000; // cada 1s
  else                              intervalo = 300;  // casi continuo (crítico)

  unsigned long ahora = millis();
  if (ahora - ultimoCambioBuzzer >= intervalo) {
    ultimoCambioBuzzer = ahora;
    buzzerEncendido = !buzzerEncendido;
    if (buzzerEncendido) tone(PIN_BUZZER, 2000);
    else noTone(PIN_BUZZER);
  }
}

// ========================================================================
// SECCIÓN 8 — LCD (ciclo automático, sin botones)
// ========================================================================
int pantallaLCDActual = 0;
const unsigned long INTERVALO_LCD_MS = 3000;

void actualizarLCD(const SensorData &d, const ResultadoFusion &f) {
  lcd.clear();

  // Salto directo a la pantalla de riesgo si el estado es grave, sin
  // esperar el ciclo normal (prioridad de visibilidad en campo)
  int pantalla = pantallaLCDActual;
  if (f.estado == RIESGO_ALTO || f.estado == RIESGO_CRITICO) pantalla = 2;

  switch (pantalla) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.printf("Nivel: %5.1f%%", d.nivel_pct);
      lcd.setCursor(0, 1);
      lcd.printf("Peso:  %5.0fg", d.peso_g);
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.printf("T:%4.1fC H:%4.1f%%", d.temperatura, d.humedad);
      lcd.setCursor(0, 1);
      lcd.printf("P:%4.0fhPa L:%4.0f", d.presion, d.lux);
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.printf("Riesgo: %5.1f%%", f.riesgo_final);
      lcd.setCursor(0, 1);
      lcd.print(nombreEstado(f.estado));
      if (f.escalado) lcd.print(" (clima)");
      break;
  }

  pantallaLCDActual = (pantallaLCDActual + 1) % 3;
}

// ========================================================================
// SECCIÓN 9 — MÓDULO 1: TAREA DE SENSORES (core 0)
// ========================================================================
bool bombaActiva = false; // estado actual de la bomba (controlada por módulo relé en PIN_BOMBA), manejado desde el tablero web

float leerDistanciaHCSR04() {
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG, LOW);
  long duracion = pulseIn(HCSR04_ECHO, HIGH, 30000);
  if (duracion == 0) return -1;
  return duracion * 0.0343 / 2.0;
}

void tareaSensores(void *parametro) {
  const TickType_t periodo = pdMS_TO_TICKS(1000);
  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;
    float l = lightMeter.readLightLevel();
    float dist = leerDistanciaHCSR04();
    float peso = datos.hx711_ok ? scale.get_units(3) : 0.0;

    float dist_lleno = SENSOR_ALTURA_CM - NIVEL_MAX_CM;
    float dist_vacio = SENSOR_ALTURA_CM;
    float nivel_pct = 100.0 * (dist_vacio - dist) / (dist_vacio - dist_lleno);
    nivel_pct = constrain(nivel_pct, 0.0f, 100.0f);

    if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(200)) == pdTRUE) {
      datos.temperatura = t;
      datos.humedad = h;
      datos.presion = p;
      datos.lux = l;
      datos.nivel_cm = dist;
      datos.nivel_pct = nivel_pct;
      datos.peso_g = peso;
      datos.hcsr04_ok = (dist > 0);
      datos.timestamp_ms = millis();
      xSemaphoreGive(mutexDatos);
    }

    vTaskDelayUntil(&ultimoDespertar, periodo);
  }
}

// ========================================================================
// SECCIÓN 10 — MÓDULO 4: SERVIDOR WEB
// ========================================================================
bool autenticado() {
  if (!server.authenticate(WEB_USER, WEB_PASS)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

const char PAGINA_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="es"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HydroSentry</title>
<style>
body{font-family:Arial,sans-serif;background:#0e1a14;color:#eaf5ee;margin:0;padding:16px;}
h1{font-size:20px;margin-bottom:4px;}
.tarjeta{background:#16261d;border-radius:10px;padding:14px;margin:10px 0;}
.riesgo{font-size:32px;font-weight:bold;}
.NORMAL{color:#4caf50;} .PREVENTIVO{color:#ffca28;} .ALTO{color:#ff9800;} .CRITICO{color:#f44336;}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
.dato{background:#1c2e22;border-radius:8px;padding:8px;text-align:center;}
.dato span{display:block;font-size:11px;color:#9fb3a6;}
button{background:#c62828;color:white;border:none;padding:10px 16px;border-radius:8px;font-size:15px;width:100%;margin-top:8px;}
button:disabled{background:#555;}
.fila{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
.btnBombaOn{background:#2e7d32;}
.btnBombaOff{background:#555;}
small{color:#9fb3a6;}
</style></head><body>
<h1>HydroSentry - Chia</h1>
<small id="ts">Cargando...</small>
<div class="tarjeta">
  <div class="riesgo" id="riesgoTxt">--</div>
  <div id="estadoTxt" class="riesgo">--</div>
</div>
<div class="grid">
  <div class="dato"><span>Nivel</span><b id="nivel">--</b></div>
  <div class="dato"><span>Peso</span><b id="peso">--</b></div>
  <div class="dato"><span>Temp</span><b id="temp">--</b></div>
  <div class="dato"><span>Humedad</span><b id="hum">--</b></div>
  <div class="dato"><span>Luz</span><b id="lux">--</b></div>
  <div class="dato"><span>ET0 (evap.)</span><b id="et0">--</b></div>
</div>
<button id="btnSilenciar" onclick="silenciar()">Silenciar alarma (10 min)</button>
<p style="margin:14px 0 4px;font-size:13px;color:#9fb3a6;">Bomba de recirculacion: <b id="bombaTxt">--</b></p>
<div class="fila">
  <button class="btnBombaOn" onclick="controlarBomba('on')">Encender bomba</button>
  <button class="btnBombaOff" onclick="controlarBomba('off')">Apagar bomba</button>
</div>
<script>
async function actualizar(){
  try{
    const r = await fetch('/api/data');
    const d = await r.json();
    document.getElementById('ts').innerText = 'Ultima lectura: ' + d.ts;
    document.getElementById('riesgoTxt').innerText = d.riesgo.toFixed(1) + '%';
    const e = document.getElementById('estadoTxt');
    e.innerText = d.estado; e.className = 'riesgo ' + d.estado;
    document.getElementById('nivel').innerText = d.nivel.toFixed(1) + '%';
    document.getElementById('peso').innerText = d.peso.toFixed(0) + 'g';
    document.getElementById('temp').innerText = d.temp.toFixed(1) + 'C';
    document.getElementById('hum').innerText = d.hum.toFixed(1) + '%';
    document.getElementById('lux').innerText = d.lux.toFixed(0) + 'lx';
    document.getElementById('et0').innerText = d.et0.toFixed(2) + 'mm/dia';
    document.getElementById('bombaTxt').innerText = d.bomba ? 'ENCENDIDA' : 'apagada';
  }catch(e){ document.getElementById('ts').innerText = 'Sin conexion con el dispositivo'; }
}
async function silenciar(){
  const b = document.getElementById('btnSilenciar');
  b.disabled = true; b.innerText = 'Silenciado';
  await fetch('/api/silenciar', {method:'POST'});
  setTimeout(()=>{ b.disabled=false; b.innerText='Silenciar alarma (10 min)'; }, 60000);
}
async function controlarBomba(accion){
  await fetch('/api/bomba?accion=' + accion, {method:'POST'});
  actualizar();
}
actualizar();
setInterval(actualizar, 3000);
</script></body></html>
)HTML";

void manejarRaiz() {
  if (!autenticado()) return;
  server.send_P(200, "text/html", PAGINA_HTML);
}

void manejarApiData() {
  if (!autenticado()) return;

  SensorData d;
  ResultadoFusion f;
  if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(200)) == pdTRUE) {
    d = datos;
    xSemaphoreGive(mutexDatos);
  }
  f = calcularFusion(d);

  char ts[24] = "sin-hora";
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 50)) {
    strftime(ts, sizeof(ts), "%H:%M:%S", &timeinfo);
  }

  char buffer[460];
  snprintf(buffer, sizeof(buffer),
    "{\"ts\":\"%s\",\"riesgo\":%.1f,\"estado\":\"%s\",\"escalado\":%s,"
    "\"nivel\":%.1f,\"peso\":%.1f,\"temp\":%.2f,\"hum\":%.2f,\"presion\":%.2f,"
    "\"lux\":%.1f,\"et0\":%.3f,\"evap_calc_ml\":%.1f,\"evap_medida_ml\":%.1f,"
    "\"bomba\":%s,"
    "\"bme_ok\":%s,\"bh1750_ok\":%s,\"hcsr04_ok\":%s,\"hx711_ok\":%s}",
    ts, f.riesgo_final, nombreEstado(f.estado), f.escalado ? "true" : "false",
    d.nivel_pct, d.peso_g, d.temperatura, d.humedad, d.presion,
    d.lux, ultimoResultadoEvap.ET0_mm_dia, ultimoResultadoEvap.evap_calc_ml,
    ultimoResultadoEvap.evap_medida_ml,
    bombaActiva ? "true" : "false",
    d.bme_ok ? "true" : "false", d.bh1750_ok ? "true" : "false",
    d.hcsr04_ok ? "true" : "false", d.hx711_ok ? "true" : "false");

  server.send(200, "application/json", buffer);
}

void manejarApiHistorico() {
  if (!autenticado()) return;

  String json;
  json.reserve(HISTORICO_TAM * 60);
  json = "[";
  int inicio = (historicoCuenta < HISTORICO_TAM) ? 0 : historicoIndice;
  for (int i = 0; i < historicoCuenta; i++) {
    int idx = (inicio + i) % HISTORICO_TAM;
    char item[100];
    snprintf(item, sizeof(item), "%s{\"t\":%lu,\"r\":%.1f,\"n\":%.1f,\"p\":%.0f,\"e\":%d}",
             i > 0 ? "," : "",
             historico[idx].ts_ms, historico[idx].riesgo_final,
             historico[idx].nivel_pct, historico[idx].peso_g,
             historico[idx].estado);
    json += item;
  }
  json += "]";
  server.send(200, "application/json", json);
}

void manejarApiSilenciar() {
  if (!autenticado()) return;
  silenciado = true;
  silenciadoHasta = millis() + DURACION_SILENCIO_MS;
  Serial.println("[WEB] Alarma silenciada por 10 minutos desde el tablero.");
  server.send(200, "application/json", "{\"ok\":true}");
}

// Control manual de la bomba desde el tablero web (POST /api/bomba?accion=on|off).
// No hay activación automática por riesgo: se deja como acción del operador,
// para evitar que el sistema mueva agua sin supervisión durante las pruebas.
void manejarApiBomba() {
  if (!autenticado()) return;

  if (server.hasArg("accion")) {
    String accion = server.arg("accion");
    if (accion == "on") {
      digitalWrite(PIN_BOMBA, HIGH);
      bombaActiva = true;
      Serial.println("[WEB] Bomba ENCENDIDA manualmente desde el tablero.");
    } else if (accion == "off") {
      digitalWrite(PIN_BOMBA, LOW);
      bombaActiva = false;
      Serial.println("[WEB] Bomba APAGADA manualmente desde el tablero.");
    } else {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"accion invalida\"}");
      return;
    }
  }

  char buffer[50];
  snprintf(buffer, sizeof(buffer), "{\"ok\":true,\"bomba\":%s}", bombaActiva ? "true" : "false");
  server.send(200, "application/json", buffer);
}

void configurarServidorWeb() {
  server.on("/", HTTP_GET, manejarRaiz);
  server.on("/api/data", HTTP_GET, manejarApiData);
  server.on("/api/bomba", HTTP_POST, manejarApiBomba);
  server.on("/api/historico", HTTP_GET, manejarApiHistorico);
  server.on("/api/silenciar", HTTP_POST, manejarApiSilenciar);
  server.begin();
  Serial.println("Servidor web iniciado en puerto 80.");
}

// ---- Reconexión WiFi no bloqueante (robustez ante caídas de la WLAN) ----
unsigned long ultimoIntentoWiFi = 0;
const unsigned long INTERVALO_RECONEXION_MS = 10000;

void gestionarWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long ahora = millis();
    if (ahora - ultimoIntentoWiFi > INTERVALO_RECONEXION_MS) {
      ultimoIntentoWiFi = ahora;
      Serial.println("WiFi desconectado. Reintentando...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

// ========================================================================
// SECCIÓN 11 — SETUP
// ========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_NARANJA, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BOMBA, OUTPUT);
  digitalWrite(PIN_BOMBA, LOW); // bomba apagada al arrancar (si tu relé es activo-bajo, cambia esto a HIGH)

  Wire.begin(I2C_SDA, I2C_SCL);

  lcd.begin(16, 2); // LCD paralelo: no tiene init()/backlight() por software,
                     // el backlight se controla solo con el cableado (LEDA/LEDK)
  lcd.setCursor(0, 0);
  lcd.print("HydroSentry");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando...");

  datos.bme_ok = bme.begin(0x76);
  if (!datos.bme_ok) Serial.println("ADVERTENCIA: BME280 no responde en 0x76");

  datos.bh1750_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23);
  if (!datos.bh1750_ok) Serial.println("ADVERTENCIA: BH1750 no responde en 0x23");

  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);

  scale.begin(HX711_DOUT, HX711_SCK);
  datos.hx711_ok = scale.wait_ready_timeout(2000);
  if (datos.hx711_ok) {
    scale.set_scale(FACTOR_CALIBRACION_HX711);
    scale.tare();
  } else {
    Serial.println("ADVERTENCIA: HX711 no responde");
  }

  mutexDatos = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(tareaSensores, "TareaSensores", 4096, NULL, 1, NULL, 0);

  // --- WiFi modo Station ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando a WiFi");
  unsigned long inicioConexion = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicioConexion < 15000) {
    delay(300); // única espera bloqueante del programa: solo ocurre una vez, en el arranque
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  } else {
    Serial.println();
    Serial.println("ADVERTENCIA: no se pudo conectar al WiFi. El tablero web no estara disponible hasta reconectar.");
  }

  configurarServidorWeb();

  // Esperamos una primera lectura de sensores antes de iniciar la ventana de evaporación
  delay(1200);
  float nivelInicial;
  if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(200)) == pdTRUE) {
    nivelInicial = datos.nivel_cm;
    xSemaphoreGive(mutexDatos);
  }
  reiniciarVentanaEvaporacion(nivelInicial);

  Serial.println("Sistema HydroSentry iniciado por completo.");
}

// ========================================================================
// SECCIÓN 12 — LOOP (no bloqueante, cooperativo)
// ========================================================================
unsigned long ultimaFusion = 0;
unsigned long ultimoLCD = 0;
unsigned long ultimoHistorico = 0;
unsigned long ultimaVentanaEvap = 0;

const unsigned long INTERVALO_FUSION_MS = 2000;
const unsigned long INTERVALO_HISTORICO_MS = 5000;

// Se guarda fuera del loop para que el buzzer/LEDs puedan refrescarse en
// cada vuelta del loop (por ejemplo, justo tras silenciar desde la web),
// sin tener que esperar al siguiente cálculo de fusión cada 2s.
EstadoRiesgo ultimoEstadoConocido = RIESGO_NORMAL;

void loop() {
  server.handleClient();   // siempre de primero: nunca se bloquea la web
  gestionarWiFi();

  unsigned long ahora = millis();

  if (ahora - ultimaFusion >= INTERVALO_FUSION_MS) {
    ultimaFusion = ahora;

    SensorData copiaLocal;
    if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(100)) == pdTRUE) {
      copiaLocal = datos;
      xSemaphoreGive(mutexDatos);

      ResultadoFusion resultado = calcularFusion(copiaLocal);
      ultimoEstadoConocido = resultado.estado;

      Serial.printf("T=%.2fC HR=%.2f%% P=%.2fhPa Lux=%.1f Nivel=%.1f%% Peso=%.1fg | Riesgo=%.1f%% [%s]%s\n",
                     copiaLocal.temperatura, copiaLocal.humedad, copiaLocal.presion,
                     copiaLocal.lux, copiaLocal.nivel_pct, copiaLocal.peso_g,
                     resultado.riesgo_final, nombreEstado(resultado.estado),
                     resultado.escalado ? " ESCALADO" : "");

      actualizarAlarma(resultado.estado);
      actualizarVentanaEvaporacion(copiaLocal.temperatura, bombaActiva);

      if (ahora - ultimoHistorico >= INTERVALO_HISTORICO_MS) {
        ultimoHistorico = ahora;
        registrarHistorico(copiaLocal, resultado);
      }

      if (ahora - ultimaVentanaEvap >= VENTANA_EVAP_MS) {
        ultimaVentanaEvap = ahora;
        procesarVentanaCompleta(copiaLocal.nivel_cm);
      }

      if (ahora - ultimoLCD >= INTERVALO_LCD_MS) {
        ultimoLCD = ahora;
        actualizarLCD(copiaLocal, resultado);
      }
    }
  } else {
    // Aun sin recalcular fusión, mantenemos la alarma actualizada por si
    // el usuario silenció desde el tablero (necesita revisarse seguido,
    // no solo cada 2s, para que el botón responda rápido)
    actualizarAlarma(ultimoEstadoConocido);
  }
}
