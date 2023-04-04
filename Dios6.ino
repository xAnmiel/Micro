#include <esp32-hal-ledc.h>


// Constantes de dirección
#define NORTH 1
#define EAST  2
#define SOUTH 4
#define WEST  8

// Pines de control del motor
int motor1Pin1 = 2;
int motor1Pin2 = 4;
int motor1Speed = 5;
int motor2Pin1 = 27;
int motor2Pin2 = 33;
int motor2Speed = 18;

// Pines del sensor
const int analogInPin = A1;
const int OutPin = 13;
const int slotSensorPin = 22;

// Variables globales
int sensorValue = 0;
volatile int slotSensorPulses = 0;
int currentDirection = NORTH; // Inicialmente apuntando al norte

// Función de interrupción del sensor ranurado
void slotSensorInterrupt() {
  slotSensorPulses++;
}

void setup() {
  // Configurar pines de control del motor
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2Speed, OUTPUT);

  // Configurar PWM para controlar la velocidad del motor
  ledcSetup(0, 5000, 8); // Canal 0, frecuencia 5000 Hz, resolución de 8 bits
  ledcAttachPin(motor1Speed, 0);

  // Inicializar el monitor serie
  Serial.begin(9600);

  // Configurar los pines del sensor
  pinMode(analogInPin, INPUT);
  pinMode(OutPin, OUTPUT);

  // Configurar el pin del sensor ranurado como entrada y establecer la interrupción
  pinMode(slotSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(slotSensorPin), slotSensorInterrupt, RISING);

  // Definir la dirección inicial basada en la posición de inicio
  int startPosition = 0; // Supongamos que startPosition es 0 para este ejemplo

  switch (startPosition) {
    case 0:
      currentDirection = EAST;
      break;
    case 1:
      currentDirection = SOUTH;
      break;
    case 2:
      currentDirection = NORTH;
      break;
    case 3:
      currentDirection = WEST;
      break;
  }
}

void loop() {
  // Leer el valor del sensor fototransistor
  sensorValue = analogRead(analogInPin);

  // Lógica para controlar los motores
  // Aquí es donde debe colocar la lógica para controlar los motores usando
  // el algoritmo de inundación y los datos del sensor ranurado

  // Encender el LED si el valor del fototransistor es mayor a 900
  if (sensorValue > 900) {
    digitalWrite(OutPin, HIGH);
  } else {
    digitalWrite(OutPin, LOW);
  }

  // Imprimir información en el monitor serie
  Serial.print("Sensor fototransistor = ");
  Serial.println(sensorValue);
Serial.print("Pulsos del sensor ranurado = ");
Serial.println(slotSensorPulses);
void updateDireccion (int turn){
  
  }

// Aquí es donde debes colocar la lógica para controlar los motores utilizando
// el algoritmo de inundación y los datos del sensor ranurado.
//
// La variable 'currentDirection' almacena la dirección actual del robot.
// La variable 'slotSensorPulses' almacena la cantidad de pulsos detectados por el sensor ranurado.
//
// Puedes utilizar estas variables junto con el algoritmo de inundación para controlar
// los motores y navegar por el laberinto.
//
// Por ejemplo, podrías mover el robot hacia adelante y contar los pulsos del sensor ranurado
// hasta que alcances un cierto número de celdas, luego cambiar la dirección según el algoritmo de inundación.

// Agregar un retraso antes de la siguiente iteración del bucle
delay(100);
}
