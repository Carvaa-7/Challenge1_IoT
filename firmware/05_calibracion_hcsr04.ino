/*
 * ========================================================================
 * HydroSentry - Calibración del sensor de nivel HC-SR04
 * ========================================================================
 * Este sketch NO es el firmware final. Sirve para:
 *   1. Verificar que el sensor mide distancias correctamente y de forma
 *      estable (varias lecturas seguidas no deberían saltar mucho).
 *   2. Medir la distancia real del sensor al fondo del tanque (tanque
 *      vacío) y al nivel máximo permitido (80% de llenado).
 *   3. Obtener los dos valores que van al firmware principal:
 *        SENSOR_ALTURA_CM  -> distancia sensor-fondo (tanque vacío)
 *        NIVEL_MAX_CM      -> altura de agua máxima permitida (80%)
 *
 * Procedimiento:
 *   1. Sube este sketch, abre el Monitor Serial a 115200 baudios.
 *   2. Con el tanque VACIO, anota el promedio que muestra "Distancia".
 *      Ese número es tu SENSOR_ALTURA_CM.
 *   3. Llena el tanque hasta la altura de agua que calculaste como el
 *      80% (en tu caso, con tanque de 8cm de alto, serían 6.4-6.5cm de
 *      columna de agua). Puedes medir esa altura de agua con una regla
 *      metida en el tanque, por fuera del cono del sensor.
 *   4. Anota la distancia que marca el sensor en ese punto.
 *      NIVEL_MAX_CM = SENSOR_ALTURA_CM - esa_distancia
 *   5. Revisa también que la distancia mínima medible (zona muerta,
 *      normalmente ~2cm) no interfiera: SENSOR_ALTURA_CM - NIVEL_MAX_CM
 *      debe ser MAYOR a 2cm, si no el sensor no podrá "ver" el agua
 *      cerca del máximo.
 * ========================================================================
 */

#define HCSR04_TRIG 5
#define HCSR04_ECHO 18

float leerDistanciaCM() {
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG, LOW);

  long duracion = pulseIn(HCSR04_ECHO, HIGH, 30000); // timeout 30ms (~5m max)
  if (duracion == 0) return -1; // no hubo eco (fuera de rango o cable mal)

  return duracion * 0.0343 / 2.0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);

  Serial.println();
  Serial.println("=== Calibracion HC-SR04 - HydroSentry ===");
  Serial.println("Recuerda: divisor de voltaje en ECHO (1k/2k) antes de conectar al ESP32.");
  Serial.println("Tomando 10 lecturas cada segundo, con su promedio y desviacion...");
  Serial.println();
}

void loop() {
  const int N = 10;
  float lecturas[N];
  float suma = 0;
  int validas = 0;

  for (int i = 0; i < N; i++) {
    float d = leerDistanciaCM();
    if (d > 0) {
      lecturas[validas] = d;
      suma += d;
      validas++;
    }
    delay(60); // HC-SR04 necesita ~60ms entre disparos
  }

  if (validas == 0) {
    Serial.println("SIN ECO - revisa cableado (TRIG=GPIO5, ECHO=GPIO18 via divisor) o que no haya algo pegado al sensor.");
  } else {
    float promedio = suma / validas;
    float varianza = 0;
    for (int i = 0; i < validas; i++) {
      varianza += pow(lecturas[i] - promedio, 2);
    }
    float desviacion = sqrt(varianza / validas);

    Serial.printf("Distancia: %.2f cm  (n=%d validas/%d, desv.std=%.2f cm)\n",
                  promedio, validas, N, desviacion);

    if (desviacion > 0.5) {
      Serial.println("  -> AVISO: lecturas inestables (>0.5cm de variacion). Revisa que el sensor");
      Serial.println("     este bien fijo, apuntando perpendicular al agua, sin nada vibrando cerca.");
    }
    if (promedio < 2.5) {
      Serial.println("  -> AVISO: estas muy cerca de la zona muerta del sensor (~2cm). Si el agua");
      Serial.println("     puede subir mas de esto, el sensor dejara de medir bien cerca del maximo.");
    }
  }

  delay(1000);
}
