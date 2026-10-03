/*
 * HydroSentry - Challenge #2
 * Módulo 1: Lectura de sensores en tarea FreeRTOS (core 0),
 * separada del hilo principal (core 1), cumpliendo el requisito
 * de "medición desde ISR o hilo diferente al hilo principal".
 *
 * Sensores: BME280 (temp/humedad/presión), BH1750 (luz),
 * HC-SR04 (nivel), HX711 + celda de carga (peso).
 *
 * Librerías necesarias (Gestor de Bibliotecas):
 *   - Adafruit BME280 Library (+ Adafruit Unified Sensor, Adafruit BusIO)
 *   - BH1750 (Christopher Laws)
 *   - HX711 (bogde)
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <HX711.h>

// ---------- Pines ----------
#define I2C_SDA 21
#define I2C_SCL 22

#define HCSR04_TRIG 5
#define HCSR04_ECHO 18

#define HX711_DOUT 4
#define HX711_SCK  2

// ---------- Objetos de sensores ----------
Adafruit_BME280 bme;
BH1750 lightMeter;
HX711 scale;

// ---------- Estructura de datos compartida entre tareas ----------
// Se protege con un mutex porque la tarea de sensores (core 0) escribe
// y el loop principal / servidor web (core 1) lee al mismo tiempo.
struct SensorData {
  float temperatura;
  float humedad;
  float presion;
  float lux;
  float nivel_cm;      // distancia medida por el HC-SR04
  float nivel_pct;     // % de llenado del tanque
  float peso_g;        // peso leído por la celda de carga
  bool  bme_ok;
  bool  bh1750_ok;
  bool  hx711_ok;
  unsigned long timestamp_ms;
};

SensorData datos;
SemaphoreHandle_t mutexDatos;

// ---------- Calibración del tanque (del documento de especificación) ----------
// Tanque pequeño: 14cm diámetro x 8cm alto, sensor montado a 10.5cm de la base.
const float SENSOR_ALTURA_CM   = 10.5;  // altura del HC-SR04 sobre la base del tanque
const float NIVEL_MAX_CM       = 6.5;   // límite operativo (80%) para proteger la celda de carga
const float FACTOR_CALIBRACION_HX711 = 24.57; // CALIBRADO 15/09: con 100mL de agua (tanque vacío -> +100g reales)

// ---------- Prototipos ----------
void tareaSensores(void *parametro);
float leerDistanciaHCSR04();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  // BME280
  datos.bme_ok = bme.begin(0x76);
  if (!datos.bme_ok) Serial.println("ADVERTENCIA: BME280 no responde en 0x76");

  // BH1750
  datos.bh1750_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23);
  if (!datos.bh1750_ok) Serial.println("ADVERTENCIA: BH1750 no responde en 0x23");

  // HC-SR04
  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);

  // HX711
  scale.begin(HX711_DOUT, HX711_SCK);
  datos.hx711_ok = scale.wait_ready_timeout(2000);
  if (datos.hx711_ok) {
    scale.set_scale(FACTOR_CALIBRACION_HX711);
    scale.tare(); // pone en cero con el tanque vacío - IMPORTANTE: taren con el tanque vacío puesto encima
  } else {
    Serial.println("ADVERTENCIA: HX711 no responde");
  }

  // Mutex para proteger la estructura compartida
  mutexDatos = xSemaphoreCreateMutex();

  // Tarea de sensores anclada al core 0 (loop principal queda libre en core 1)
  xTaskCreatePinnedToCore(
    tareaSensores,     // función
    "TareaSensores",   // nombre
    4096,              // tamaño de pila (stack)
    NULL,              // parámetro
    1,                 // prioridad
    NULL,              // handle (no lo necesitamos aquí)
    0                  // core 0
  );

  Serial.println("Sistema iniciado. Tarea de sensores corriendo en core 0.");
}

void loop() {
  // El core 1 queda libre para LCD, servidor web, lógica de fusión, etc.
  // (esos módulos se agregan en los siguientes pasos)

  // Ejemplo de lectura segura de los datos desde el core 1:
  if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.printf("T=%.2f°C  HR=%.2f%%  P=%.2fhPa  Lux=%.2f  Nivel=%.1f%%  Peso=%.1fg\n",
                   datos.temperatura, datos.humedad, datos.presion,
                   datos.lux, datos.nivel_pct, datos.peso_g);
    xSemaphoreGive(mutexDatos);
  }

  delay(2000);
}

// =====================================================================
// TAREA DE SENSORES - corre en el core 0, independiente del loop()
// =====================================================================
void tareaSensores(void *parametro) {
  const TickType_t periodo = pdMS_TO_TICKS(1000); // lee cada 1 segundo
  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;
    float l = lightMeter.readLightLevel();
    float dist = leerDistanciaHCSR04();
    float peso = datos.hx711_ok ? scale.get_units(3) : 0.0; // promedio de 3 lecturas

    // Nivel: distancia sensor->agua. Menor distancia = más lleno.
    // dist_min (tanque lleno al 80%) = SENSOR_ALTURA_CM - NIVEL_MAX_CM
    float dist_lleno = SENSOR_ALTURA_CM - NIVEL_MAX_CM;
    float dist_vacio = SENSOR_ALTURA_CM; // agua a nivel 0
    float nivel_pct = 100.0 * (dist_vacio - dist) / (dist_vacio - dist_lleno);
    nivel_pct = constrain(nivel_pct, 0.0, 100.0);

    // Escribimos en la estructura compartida, protegidos por mutex
    if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(200)) == pdTRUE) {
      datos.temperatura = t;
      datos.humedad = h;
      datos.presion = p;
      datos.lux = l;
      datos.nivel_cm = dist;
      datos.nivel_pct = nivel_pct;
      datos.peso_g = peso;
      datos.timestamp_ms = millis();
      xSemaphoreGive(mutexDatos);
    }

    // vTaskDelayUntil mantiene el período exacto de 1s, sin acumular drift
    vTaskDelayUntil(&ultimoDespertar, periodo);
  }
}

// =====================================================================
// Lectura de distancia HC-SR04 (bloqueante corta, ~30ms máx, va en la
// tarea de sensores, nunca en una ISR real)
// =====================================================================
float leerDistanciaHCSR04() {
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG, LOW);

  long duracion = pulseIn(HCSR04_ECHO, HIGH, 30000); // timeout 30ms (~5m máx)
  if (duracion == 0) return -1; // sin eco, sensor desconectado o fuera de rango

  float distancia_cm = duracion * 0.0343 / 2.0;
  return distancia_cm;
}
