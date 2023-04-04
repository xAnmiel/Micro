#include <ESP32PWM.h>

// Definimos los pines de los motores
#define motor1A 2
#define motor1B 4
#define motor2A 15
#define motor2B 13

// Definimos el pin del sensor de velocidad
#define sensorVelocidad 18

// Definimos las variables que almacenarán la velocidad de cada motor
int velocidadMotor1 = 0;
int velocidadMotor2 = 0;

// Función para mover el motor 1 hacia adelante
void moverMotor1Adelante(int velocidad) {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  ledcWrite(0, velocidad);
}

// Función para mover el motor 1 hacia atrás
void moverMotor1Atras(int velocidad) {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  ledcWrite(0, velocidad);
}

// Función para detener el motor 1
void detenerMotor1() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  ledcWrite(0, 0);
}

// Función para mover el motor 2 hacia adelante
void moverMotor2Adelante(int velocidad) {
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  ledcWrite(1, velocidad);
}

// Función para mover el motor 2 hacia atrás
void moverMotor2Atras(int velocidad) {
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
  ledcWrite(1, velocidad);
}

// Función para detener el motor 2
void detenerMotor2() {
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
  ledcWrite(1, 0);
}

void setup() {
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(sensorVelocidad, INPUT);
  
  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);
  ledcAttachPin(motor1A, 0);
  ledcAttachPin(motor2A, 1);
}

void loop() {
  // Leemos la velocidad del sensor
  velocidadMotor1 = pulseIn(sensorVelocidad, HIGH);
  velocidadMotor2 = pulseIn(sensorVelocidad, HIGH);
  
  // Convertimos la velocidad a un valor entre 0 y 255
  velocidadMotor1 = map(velocidadMotor1, 0, 1000, 0, 255);
  velocidadMotor2 = map(velocidadMotor2, 0, 1000, 0, 255);
  
  // Movemos los motores según la velocidad leída
  if (velocidadMotor1 > 0) {
    moverMotor1Adelante(velocidadMotor1);
  } else
