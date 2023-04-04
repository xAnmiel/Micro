const int analogInPin = A1; // Pin analogico 1 para la lectura del Fototransistor
const int OutPin = 13;      // Pin para el led indicador


int sensorValue = 0;        // Inicializamos el valor del sensor

void setup() {
  // Inicializamos la comunicacion serial a 9600 bps:
  Serial.begin(9600); 
  pinMode(OutPin, OUTPUT);
}

void loop() {
  // leemos el pin para y asignamos el valor a la variable.
  sensorValue = analogRead(analogInPin);            
  
  // Si el valor obtenido es mayor a 900 se activa el LED
  if(sensorValue > 20)
  {
    Serial.print("El valor es: ");
    Serial.println(sensorValue);
    digitalWrite(OutPin, HIGH);
  }
  else
  {
    digitalWrite(OutPin, LOW);
    Serial.print("El valor es: ");
    Serial.println(sensorValue);
  }

  // Imprimimos el valor en el monitor.
  Serial.print("sensor = " );                       
  Serial.println(sensorValue);     

  delay(100);                     
}
