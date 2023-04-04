// Declaración de variables
int motor1Pin1 = 3; // Conectado al pin 1 del puente H para controlar el motor 1
int motor1Pin2 = 5; // Conectado al pin 2 del puente H para controlar el motor 1
int motor1Speed = 6; // Conectado al pin Enable del puente H para controlar la velocidad del motor 1
int motor2Pin1 = 9; // Conectado al pin 3 del puente H para controlar el motor 2
int motor2Pin2 = 10; // Conectado al pin 4 del puente H para controlar el motor 2
int motor2Speed = 11; // Conectado al pin Enable del puente H para controlar la velocidad del motor 2

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
  analogWrite(motor2Speed, 127);
  
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
