/*
 * ========================================================================
 * HydroSentry - Caracterización de la celda de carga (galga)
 * ========================================================================
 * Este sketch NO es el firmware final. Es una herramienta de un solo uso
 * para encontrar el factor de calibración real del HX711 con TU celda
 * específica, usando varios pesos de referencia pesados en la gramera.
 *
 * Procedimiento:
 *   1. Sube este sketch y abre el Monitor Serial a 115200 baudios,
 *      con "Nueva línea" seleccionado (para que Enter funcione).
 *   2. Deja el platillo/plataforma vacío sobre la celda.
 *   3. Escribe:  t   y presiona Enter -> hace la tara (pone en cero).
 *   4. Pesa un objeto en la gramera (anota el peso exacto, ej: 187.3).
 *   5. Pon ESE MISMO objeto sobre la plataforma de la celda.
 *   6. Escribe el peso que marcó la gramera (ej: 187.3) y Enter.
 *      El sketch guarda el par (peso real, lectura cruda).
 *   7. Quita el objeto, pon otro de peso distinto, repite el paso 4-6.
 *      Repite con AL MENOS 5 objetos distintos, repartidos en el rango
 *      0-1000g (ej: 50g, 200g, 400g, 650g, 900g).
 *   8. Cuando tengas todos los puntos, escribe:  c   y Enter.
 *      El sketch calcula el factor de calibración y el error real.
 *   9. Copia el valor de "Factor de calibracion" al firmware principal,
 *      en la constante FACTOR_CALIBRACION_HX711.
 *
 * Comandos disponibles en el Monitor Serial:
 *   t       -> tara (hacer SIEMPRE primero, con el platillo vacío)
 *   r       -> muestra una lectura cruda actual (para verificar estabilidad)
 *   <numero>-> registra un punto de calibración con ese peso en gramos
 *   c       -> calcula la regresión y el factor de calibración final
 *   reset   -> borra todos los puntos capturados y empieza de nuevo
 * ========================================================================
 */

#include <HX711.h>

#define HX711_DOUT 27
#define HX711_SCK  14

HX711 scale;

const int MAX_PUNTOS = 12;
float pesosReales[MAX_PUNTOS];
long  lecturasCrudas[MAX_PUNTOS];
int   numPuntos = 0;

long leerCrudoPromedio(int muestras) {
  return scale.get_value(muestras); // ya viene con la tara restada
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  scale.begin(HX711_DOUT, HX711_SCK);

  Serial.println();
  Serial.println("=== Caracterizacion de la celda de carga HydroSentry ===");
  if (scale.wait_ready_timeout(2000)) {
    Serial.println("HX711 detectado correctamente.");
  } else {
    Serial.println("ADVERTENCIA: HX711 no responde. Revisa el cableado (DT=GPIO4, SCK=GPIO2).");
  }
  Serial.println("Con el platillo VACIO, escribe: t   y presiona Enter para tarar.");
  Serial.println("Comandos: t=tarar | r=lectura cruda | <peso>=registrar punto | c=calcular | reset=borrar todo");
}

void calcularRegresion() {
  if (numPuntos < 2) {
    Serial.println("Necesitas minimo 2 puntos (lo ideal es 5 o mas) antes de calcular.");
    return;
  }

  double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  for (int i = 0; i < numPuntos; i++) {
    double x = pesosReales[i];
    double y = (double)lecturasCrudas[i];
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumXX += x * x;
  }
  int n = numPuntos;
  double denominador = (n * sumXX - sumX * sumX);
  if (fabs(denominador) < 1e-9) {
    Serial.println("ERROR: todos los pesos son iguales, no se puede calcular una recta.");
    return;
  }

  double pendiente   = (n * sumXY - sumX * sumY) / denominador; // crudo por gramo
  double intercepto  = (sumY - pendiente * sumX) / n;

  // El HX711 usa: peso = crudo / factor  =>  factor = crudo / peso = pendiente
  double factorCalibracion = pendiente;

  // Evaluamos el error real de cada punto usando la recta encontrada
  double sumaErrorAbs = 0, errorMaximo = 0;
  Serial.println();
  Serial.println("Punto | Peso real (g) | Crudo | Peso estimado (g) | Error (g)");
  for (int i = 0; i < numPuntos; i++) {
    double pesoEstimado = ((double)lecturasCrudas[i] - intercepto) / pendiente;
    double error = fabs(pesoEstimado - pesosReales[i]);
    sumaErrorAbs += error;
    if (error > errorMaximo) errorMaximo = error;
    Serial.printf("  %2d  |    %7.1f    | %6ld |      %7.1f      |   %5.1f\n",
                  i + 1, pesosReales[i], lecturasCrudas[i], pesoEstimado, error);
  }
  double errorPromedio = sumaErrorAbs / numPuntos;

  Serial.println();
  Serial.println("=== RESULTADO DE LA CALIBRACION ===");
  Serial.printf("Factor de calibracion (usar en el firmware): %.4f\n", factorCalibracion);
  Serial.printf("Intercepto (deberia ser cercano a 0): %.2f\n", intercepto);
  Serial.printf("Error promedio: %.2f g\n", errorPromedio);
  Serial.printf("Error maximo: %.2f g\n", errorMaximo);
  Serial.println();
  if (errorPromedio > 15) {
    Serial.println("AVISO: el error promedio es alto (>15g). Revisa que la celda esté");
    Serial.println("bien fijada (un extremo fijo, el otro libre, sin tocar nada más) y");
    Serial.println("que no haya vibraciones o viento moviendo la plataforma al medir.");
  } else {
    Serial.println("Calibracion aceptable. Copia el factor de calibracion al firmware:");
    Serial.println("  const float FACTOR_CALIBRACION_HX711 = <el numero de arriba>;");
  }
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();

    if (linea.length() == 0) return;

    if (linea == "t") {
      scale.tare();
      Serial.println("Tara hecha. El platillo vacio ahora es el cero de referencia.");
    } else if (linea == "r") {
      Serial.print("Lectura cruda actual (promedio de 10): ");
      Serial.println(leerCrudoPromedio(10));
    } else if (linea == "c") {
      calcularRegresion();
    } else if (linea == "reset") {
      numPuntos = 0;
      Serial.println("Puntos de calibracion borrados. Empieza de nuevo.");
    } else {
      float peso = linea.toFloat();
      if (peso > 0 && numPuntos < MAX_PUNTOS) {
        long crudo = leerCrudoPromedio(15); // promedio de 15 lecturas para reducir ruido
        pesosReales[numPuntos] = peso;
        lecturasCrudas[numPuntos] = crudo;
        numPuntos++;
        Serial.printf("Punto %d/%d registrado: peso=%.1fg  lectura_cruda=%ld\n",
                      numPuntos, MAX_PUNTOS, peso, crudo);
        if (numPuntos >= 5) {
          Serial.println("Ya tienes 5+ puntos. Puedes seguir agregando o escribir 'c' para calcular.");
        }
      } else if (numPuntos >= MAX_PUNTOS) {
        Serial.println("Ya alcanzaste el maximo de puntos. Escribe 'c' para calcular o 'reset' para reiniciar.");
      } else {
        Serial.println("Comando no reconocido. Usa: t | r | c | reset | o un peso en gramos (ej: 250.5)");
      }
    }
  }
}
