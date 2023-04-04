#include <analogWrite.h>
// Declaración de variables
int motor1Pin1 = 2; // Conectado al pin 1 del puente H para controlar el motor 1
int motor1Pin2 = 4; // Conectado al pin 2 del puente H para controlar el motor 1
int motor1Speed = 5; // Conectado al pin Enable del puente H para controlar la velocidad del motor 1
int motor2Pin1 = 27; // Conectado al pin 3 del puente H para controlar el motor 2
int motor2Pin2 = 33; // Conectado al pin 4 del puente H para controlar el motor 2
int motor2Speed = 18; // Conectado al pin Enable del puente H para controlar la velocidad del motor 2

void setup() {
  // Configurar pines como salida
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
}

void loop() {
  // Girar motor 1 hacia adelante a velocidad máxima
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
    analogWrite(motor1Speed, 255);
  
  // Girar motor 2 hacia atrás a velocidad media
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(motor2Speed, 255);
  
  // Esperar 2 segundos
  delay(2000);
  
  // Detener ambos motores
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor1Speed, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor2Speed, LOW);
  
  // Esperar 1 segundo
  delay(1000);
}
void girarMotor(char dir, int vel, int pulsos){
  analogWrite(PWM, vel);
  
}
