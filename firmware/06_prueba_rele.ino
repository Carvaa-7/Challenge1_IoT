/*
 * ========================================================================
 * HydroSentry - Prueba del módulo relé (bomba) + verificación general
 * ========================================================================
 * Este sketch NO es el firmware final. Sirve para:
 *   1. Confirmar si tu módulo relé es "activo en alto" (se activa con
 *      HIGH) o "activo en bajo" (se activa con LOW) — dato que falta
 *      confirmar antes de dar por cerrado el firmware completo.
 *   2. De paso, imprime por el Monitor Serial TODAS las variables de los
 *      sensores (temperatura, humedad, presión, luz, nivel, peso) más el
 *      estado del relé, así puedes verificar de una sola vez que todo lo
 *      que va ANTES del relé en el firmware completo (sensores, fusión)
 *      sigue leyendo bien con la organización de pines nueva.
 *
 * Procedimiento para la prueba del relé:
 *   1. Sube este sketch y abre el Monitor Serial a 115200 baudios.
 *   2. El relé va a alternar solo, cada 3 segundos, entre los dos
 *      estados (ver mensajes en el monitor).
 *   3. Observa el módulo relé: casi todos tienen un LED indicador propio
 *      y hacen un "clic" audible al activarse.
 *   4. Anota en cuál de los dos mensajes ("Probando HIGH" o "Probando
 *      LOW") el relé SÍ se activa (LED encendido + clic). Ese es tu
 *      caso real:
 *        - Si se activa en "Probando HIGH"  -> tu relé es ACTIVO EN ALTO
 *          (el firmware completo ya está bien tal cual, HIGH=encendido).
 *        - Si se activa en "Probando LOW"   -> tu relé es ACTIVO EN BAJO
 *          (avísame y en 2 líneas invierto la lógica en el firmware
 *          completo: manejarApiBomba() y el apagado inicial en setup()).
 * ========================================================================
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <HX711.h>

// ---- Mismos pines del firmware completo (lado izquierdo) ----
#define I2C_SDA 25
#define I2C_SCL 26

#define HCSR04_TRIG 27
#define HCSR04_ECHO 34

#define HX711_DOUT 35
#define HX711_SCK  14

#define PIN_BOMBA 13

// ---- Calibración ya confirmada ----
const float SENSOR_ALTURA_CM = 10.54;
const float NIVEL_MAX_CM     = 5.44;
const float FACTOR_CALIBRACION_HX711 = 634.3359;

Adafruit_BME280 bme;
BH1750 lightMeter;
HX711 scale;

bool bme_ok, bh1750_ok, hx711_ok;

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== HydroSentry - Prueba de rele + verificacion de sensores ===");

  pinMode(PIN_BOMBA, OUTPUT);
  digitalWrite(PIN_BOMBA, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  bme_ok = bme.begin(0x76);
  if (!bme_ok) Serial.println("ADVERTENCIA: BME280 no responde en 0x76");

  bh1750_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23);
  if (!bh1750_ok) Serial.println("ADVERTENCIA: BH1750 no responde en 0x23");

  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);

  scale.begin(HX711_DOUT, HX711_SCK);
  hx711_ok = scale.wait_ready_timeout(2000);
  if (hx711_ok) {
    scale.set_scale(FACTOR_CALIBRACION_HX711);
    scale.tare();
  } else {
    Serial.println("ADVERTENCIA: HX711 no responde");
  }

  Serial.println("Listo. El rele va a alternar cada 3s. Observa el modulo fisico.");
  Serial.println();
}

unsigned long ultimoCambio = 0;
bool estadoPrueba = false; // false = probando LOW, true = probando HIGH
const unsigned long INTERVALO_MS = 3000;

void loop() {
  unsigned long ahora = millis();

  // --- Alternar el pin del rele cada 3s, mostrando bien claro cuál estado se prueba ---
  if (ahora - ultimoCambio >= INTERVALO_MS) {
    ultimoCambio = ahora;
    estadoPrueba = !estadoPrueba;
    digitalWrite(PIN_BOMBA, estadoPrueba ? HIGH : LOW);
    Serial.println(estadoPrueba
      ? ">>> Probando HIGH en PIN_BOMBA (GPIO13) -- revisa si el rele hizo clic / encendio su LED"
      : ">>> Probando LOW en PIN_BOMBA (GPIO13)  -- revisa si el rele hizo clic / encendio su LED");
  }

  // --- Verificación de todas las variables de sensores (lo que va antes del rele) ---
  float t = bme_ok ? bme.readTemperature() : NAN;
  float h = bme_ok ? bme.readHumidity() : NAN;
  float p = bme_ok ? bme.readPressure() / 100.0F : NAN;
  float l = bh1750_ok ? lightMeter.readLightLevel() : NAN;
  float dist = leerDistanciaHCSR04();
  float peso = hx711_ok ? scale.get_units(3) : NAN;

  float dist_lleno = SENSOR_ALTURA_CM - NIVEL_MAX_CM;
  float dist_vacio = SENSOR_ALTURA_CM;
  float nivel_pct = 100.0 * (dist_vacio - dist) / (dist_vacio - dist_lleno);
  nivel_pct = constrain(nivel_pct, 0.0f, 100.0f);

  Serial.printf("T=%.2fC HR=%.2f%% P=%.2fhPa Lux=%.1f Dist=%.2fcm Nivel=%.1f%% Peso=%.1fg | Rele(GPIO13)=%s\n",
                t, h, p, l, dist, nivel_pct, peso, estadoPrueba ? "HIGH" : "LOW");

  delay(1000);
}
