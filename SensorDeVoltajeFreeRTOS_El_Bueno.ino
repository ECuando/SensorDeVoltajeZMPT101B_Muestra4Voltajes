/*     
 *     Código para utilizar el medidor de voltaje ZMPT101B con el microcontrolador ESP32 y 
 *     el sistema FreeRTOS mostrando la lectura de voltaje en el puerto serie 
 *     actualizandola cada 5 segundos. 
 *     
 *     Se calcula la desviación estándar:
 *      
 *     1. Se calcula la media de las lecturas tomadas del sensor.
 *     2. Luego, se calcula la varianza de las lecturas. La varianza es la media de los cuadrados de las desviaciones de cada dato respecto a la media.
 *     3. Finalmente, se obtiene la desviación estándar, que es la raíz cuadrada de la varianza.
 * 
 *     Una vez que se tiene la desviación estándar, se utiliza la siguiente fórmula para calcular el voltaje:
 *     voltaje = intercepción de recta + pendiente * desviación estándar
 *     
 *     Donde la intercepción de recta y la pendiente son parámetros que 
 *     deben ser ajustados mediante calibración previa del sensor. 
 *     En este código, se asume que estos valores ya han sido ajustados 
 *     previamente mediante pruebas de calibración. Luego, el resultado 
 *     de esta ecuación se multiplica por un factor de calibración 
 *     específico para el sensor utilizado.
 */

//Bibliotecas de FreeRTOS y comunicación serial
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Definición de constantes Tarea 1-Sensor de Voltaje 1.
const int PinADCSensor1 = 25; //Pin ADC1_8 del ESP32.
const float FrecuenciaDeRed = 60.0; //Frecuencia de la señal de la red de energía pública.
const float LongitudDeOndaPromedio = 40.0; // Promedio de la señal
const float IntercepcionDeRecta = -0.04; 
const float Pendiente = 0.0405;
const unsigned long Periodo = 5000;

// Definición de constantes Tarea 2-Sensor de Voltaje 2.
const int PinADCSensor2 = 33; //Pin ADC1_8 del ESP32.
const float FrecuenciaDeRed2 = 60.0; //Frecuencia de la señal de la red de energía pública.
const float LongitudDeOndaPromedio2 = 40.0; // Promedio de la señal
const float IntercepcionDeRecta2 = -0.04; 
const float Pendiente2 = 0.0405;
const unsigned long Periodo2 = 5000;

// Definición de constantes Tarea 3-Sensor de Voltaje 3.
const int PinADCSensor3 = 34; //Pin ADC1_8 del ESP32.
const float FrecuenciaDeRed3 = 60.0; //Frecuencia de la señal de la red de energía pública.
const float LongitudDeOndaPromedio3 = 40.0; // Promedio de la señal
const float IntercepcionDeRecta3 = -0.04; 
const float Pendiente3 = 0.0405;
const unsigned long Periodo3 = 5000;

// Definición de constantes Tarea 4-Sensor de Voltaje 4.
const int PinADCSensor4 = 35; //Pin ADC1_8 del ESP32.
const float FrecuenciaDeRed4 = 60.0; //Frecuencia de la señal de la red de energía pública.
const float LongitudDeOndaPromedio4 = 40.0; // Promedio de la señal
const float IntercepcionDeRecta4 = -0.04; 
const float Pendiente4 = 0.0405;
const unsigned long Periodo4 = 5000;

//Se declaran algunas variables globales que se utilizarán en la tarea del Sensor de Voltaje 1.
float Volts;
float SumatoriaLecturas;
float SumatoriaLecturasAlCuadrado;
int NumeroDeLecturas;
unsigned long TiempoAnterior;

//Se declaran algunas variables globales que se utilizarán en la tarea del Sensor de Voltaje 2.
float Volts2;
float SumatoriaLecturas2;
float SumatoriaLecturasAlCuadrado2;
int NumeroDeLecturas2;
unsigned long TiempoAnterior2;

//Se declaran algunas variables globales que se utilizarán en la tarea del Sensor de Voltaje 3.
float Volts3;
float SumatoriaLecturas3;
float SumatoriaLecturasAlCuadrado3;
int NumeroDeLecturas3;
unsigned long TiempoAnterior3;

//Se declaran algunas variables globales que se utilizarán en la tarea del Sensor de Voltaje 4.
float Volts4;
float SumatoriaLecturas4;
float SumatoriaLecturasAlCuadrado4;
int NumeroDeLecturas4;
unsigned long TiempoAnterior4;

// Prototipo de las tareas
void LecturaSensor(void* Parametro);
void LecturaSensor2(void* Parametro2); 
void LecturaSensor3(void* Parametro3); 
void LecturaSensor4(void* Parametro4); 

///////////////////////////TaskHandle_t LecturaSensor4;
//Inicializa la comunicación serial
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

// Se y crea la tarea "LecturaSensor" con una prioridad de 1 y un tamaño de pila de 2048 bytes.
  xTaskCreate(LecturaSensor, "LecturaSensor", 2048, NULL, 4, NULL);
  xTaskCreate(LecturaSensor2, "LecturaSensor2",2048, NULL, 4, NULL);
  xTaskCreate(LecturaSensor3, "LecturaSensor3",2048, NULL, 4, NULL);
  xTaskCreate(LecturaSensor4, "LecturaSensor4",2048, NULL, 4, NULL);

 //////////////////////////////////  xTaskCreatePinnedToCore(LecturaSensor4, "LecturaSensor4", 10000, NULL, 1, LecturaSensor4, 1);
}

// El bucle principal no hace nada en este código.
void loop() {
}

//  Esta parte del código define la tarea principal del programa que se encarga de leer las 
//  señales del sensor conectado al pin analógico del microcontrolador.

void LecturaSensor(void* Parametro) {
  pinMode(PinADCSensor1, INPUT); //Se establece el Pin ADC como entrada.

//Bucle infinito para leer la señal analógica del sensor.
  while (true) {
    int ValorDeSensor = analogRead(PinADCSensor1);
    SumatoriaLecturas += ValorDeSensor;
    SumatoriaLecturasAlCuadrado += pow(ValorDeSensor, 2);
    NumeroDeLecturas++;

    if ((unsigned long)(millis() - TiempoAnterior) >= Periodo) {
      float Media = SumatoriaLecturas / NumeroDeLecturas;
      float Varianza = (SumatoriaLecturasAlCuadrado / NumeroDeLecturas) - pow(Media, 2);
      float DesviacionEstandar = sqrt(Varianza);

      Volts = IntercepcionDeRecta + Pendiente * DesviacionEstandar;
      Volts *= 40.3231;

      Serial.print("\tVoltaje Sensor 1: ");
      Serial.print(Volts);
      Serial.println("V AC"); // Muestra el mensaje "Voltage" en el monitor serial
      vTaskDelay(1);
      
      SumatoriaLecturas = 0.0;
      SumatoriaLecturasAlCuadrado = 0.0;
      NumeroDeLecturas = 0;
      TiempoAnterior = millis();
    }

    vTaskDelay(1);
  }
}



void LecturaSensor2(void* Parametro2) {
  pinMode(PinADCSensor2, INPUT); //Se establece el Pin ADC como entrada.

//Bucle infinito para leer la señal analógica del sensor.
  while (true) {
    int ValorDeSensor2 = analogRead(PinADCSensor2);
    SumatoriaLecturas2 += ValorDeSensor2;
    SumatoriaLecturasAlCuadrado2 += pow(ValorDeSensor2, 2);
    NumeroDeLecturas2++;

    if ((unsigned long)(millis() - TiempoAnterior2) >= Periodo2) {
      float Media2 = SumatoriaLecturas2 / NumeroDeLecturas2;
      float Varianza2 = (SumatoriaLecturasAlCuadrado2 / NumeroDeLecturas2) - pow(Media2, 2);
      float DesviacionEstandar2 = sqrt(Varianza2);

      Volts2 = IntercepcionDeRecta2 + Pendiente2 * DesviacionEstandar2;
      Volts2 *= 40.3231;

      Serial.print("\tVoltaje Sensor 2: ");
      Serial.print(Volts2);
      Serial.println("V AC"); // Muestra el mensaje "Voltage" en el monitor serial
     // Serial.println("-------------------------------------------------------");
     
      
      SumatoriaLecturas2 = 0.0;
      SumatoriaLecturasAlCuadrado2 = 0.0;
      NumeroDeLecturas2 = 0;
      TiempoAnterior2 = millis();
    }

    vTaskDelay(1);
 
  }
}


void LecturaSensor3(void* Parametro3) {
  pinMode(PinADCSensor3, INPUT); //Se establece el Pin ADC como entrada.

//Bucle infinito para leer la señal analógica del sensor.
  while (true) {
    int ValorDeSensor3 = analogRead(PinADCSensor3);
    SumatoriaLecturas3 += ValorDeSensor3;
    SumatoriaLecturasAlCuadrado3 += pow(ValorDeSensor3, 2);
    NumeroDeLecturas3++;

    if ((unsigned long)(millis() - TiempoAnterior3) >= Periodo3) {
      float Media3 = SumatoriaLecturas3 / NumeroDeLecturas3;
      float Varianza3 = (SumatoriaLecturasAlCuadrado3 / NumeroDeLecturas3) - pow(Media3, 2);
      float DesviacionEstandar3 = sqrt(Varianza3);

      Volts3 = IntercepcionDeRecta3 + Pendiente3 * DesviacionEstandar3;
      Volts3 *= 40.3231;

      Serial.print("\tVoltaje Sensor 3: ");
      Serial.print(Volts3);
      Serial.println("V AC"); // Muestra el mensaje "Voltage" en el monitor serial
     // Serial.println("-------------------------------------------------------");
    
      
      SumatoriaLecturas3 = 0.0;
      SumatoriaLecturasAlCuadrado3 = 0.0;
      NumeroDeLecturas3 = 0;
      TiempoAnterior3 = millis();
    }

    vTaskDelay(1);
    
  }
}


void LecturaSensor4(void* Parametro4) {
  pinMode(PinADCSensor4, INPUT); //Se establece el Pin ADC como entrada.

//Bucle infinito para leer la señal analógica del sensor.
  while (true) {
    int ValorDeSensor4 = analogRead(PinADCSensor4);
    SumatoriaLecturas4 += ValorDeSensor4;
    SumatoriaLecturasAlCuadrado4 += pow(ValorDeSensor4, 2);
    NumeroDeLecturas4++;

    if ((unsigned long)(millis() - TiempoAnterior4) >= Periodo4) {
      float Media4 = SumatoriaLecturas4 / NumeroDeLecturas4;
      float Varianza4 = (SumatoriaLecturasAlCuadrado4 / NumeroDeLecturas4) - pow(Media4, 2);
      float DesviacionEstandar4 = sqrt(Varianza4);

      Volts4 = IntercepcionDeRecta4 + Pendiente4 * DesviacionEstandar4;
      Volts4 *= 40.3231;

      Serial.print("\tVoltaje Sensor 4: ");
      Serial.print(Volts4);
      Serial.println("V AC"); // Muestra el mensaje "Voltage" en el monitor serial
      Serial.println("-------------------------------------------------------");
     
      
      SumatoriaLecturas4 = 0.0;
      SumatoriaLecturasAlCuadrado4 = 0.0;
      NumeroDeLecturas4 = 0;
      TiempoAnterior4 = millis();
    }

    vTaskDelay(1);
  }
}
