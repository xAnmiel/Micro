#include <esp32-hal-ledc.h>
#include <queue.h>
#include <Arduino.h>
#include <limits>

// Constantes de dirección
#define NORTH 1
#define EAST  2
#define SOUTH 4
#define WEST  8

// Dimensiones del laberinto
#define MAZE_WIDTH 12
#define MAZE_HEIGHT 7

// Pines de control del motor
int motor1Pin1 = 2;
int motor1Pin2 = 4;
int motor1Speed = 5;
int motor2Pin1 = 27;
int motor2Pin2 = 33;
int motor2Speed = 18;
int dx[] = {0, 1, 0, -1};
int dy[] = {1, 0, -1, 0};

// Pines del sensor
const int analogInPin = 34;
const int OutPin = 13;
const int slotSensorPin = 22;

// Variables globales
int sensorValue = 0;
volatile int slotSensorPulses = 0;
int currentDirection = NORTH; // Inicialmente apuntando al norte

// Variables globales para el algoritmo flood-fill
int floodFillArray[MAZE_WIDTH][MAZE_HEIGHT];
int currentPositionX = 0;
int currentPositionY = 0;





// Implementación de funciones para controlar el robot
void moveForward() {
  // Aquí debes implementar la lógica para mover el robot hacia adelante
}

void turnLeft() {

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  ledcWrite(0, 128); // Velocidad del motor 1
  ledcWrite(1, 128); // Velocidad del motor 2

  delay(800); // Duración del giro (esto puede variar según el robot)

  // Detener los motores
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  // Actualizar la dirección actual del robot
  currentDirection = (currentDirection << 1) % 15;
  if (currentDirection == 0) {
    currentDirection = NORTH;
  }
}

void turnRight() {

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  ledcWrite(0, 128); // Velocidad del motor 1
  ledcWrite(1, 128); // Velocidad del motor 2

  delay(800); // Duración del giro (esto puede variar según el robot)

  // Detener los motores
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  // Actualizar la dirección actual del robot
  currentDirection = (currentDirection >> 1) % 15;
  if (currentDirection == 0) {
    currentDirection = WEST;
  }
}


void updateCurrentDirection(int turn) {
  switch (turn) {
    case NORTH:
      currentDirection = (currentDirection == WEST) ? NORTH : currentDirection << 1;
      break;
    case EAST:
      currentDirection = (currentDirection == NORTH) ? EAST : currentDirection << 1;
      break;
    case SOUTH:
      currentDirection = (currentDirection == EAST) ? SOUTH : currentDirection << 1;
      break;
    case WEST:
      currentDirection = (currentDirection == SOUTH) ? WEST : currentDirection << 1;
      break;
  }
}

// Función para calcular la posición objetivo
void calculateGoalPosition(int &goalX, int &goalY) {
  // Calcular la posición objetivo en función del tamaño del laberinto
  goalX = (MAZE_WIDTH / 2) - 2;
  goalY = (MAZE_HEIGHT / 2) - 2;
}


void updateFloodFillArray() {
  // Inicializar la cola para el algoritmo flood-fill
  std::queue<std::pair<int, int>> floodFillQueue;

  // Obtener la posición del objetivo
  int goalX, goalY;
  calculateGoalPosition(goalX, goalY);

  // Establecer la distancia del objetivo a 0 y agregarlo a la cola
  floodFillArray[goalX][goalY] = 0;
  floodFillQueue.push(make_pair(goalX, goalY));

  // Procesar las celdas en la cola
  while (!floodFillQueue.empty()) {
    int x = floodFillQueue.front().first;
    int y = floodFillQueue.front().second;
    floodFillQueue.pop();

    int distance = floodFillArray[x][y];

    // Procesar las celdas adyacentes en cada dirección
    for (int dir = NORTH; dir <= WEST; dir <<= 1) {
      int newX = x;
      int newY = y;

      // Actualizar las coordenadas de la celda adyacente
      switch (dir) {
        case NORTH:
          newY++;
          break;
        case EAST:
          newX++;
          break;
        case SOUTH:
          newY--;
          break;
        case WEST:
          newX--;
          break;
      }

      // Comprobar si la celda adyacente está dentro del laberinto
      if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT) {
        // Comprobar si la distancia en la celda adyacente puede ser actualizada
        if (floodFillArray[newX][newY] > distance + 1) {
          floodFillArray[newX][newY] = distance + 1;
          floodFillQueue.push(make_pair(newX, newY));
        }
      }
    }
  }
}

// Implementación del algoritmo flood-fill
void floodFill() {
  updateFloodFillArray();
}


bool isMazeSolved() {
  int goalX, goalY;
  calculateGoalPosition(goalX, goalY);

  // Devuelve true si el robot ha llegado a la posición objetivo
  return currentPositionX == goalX && currentPositionY == goalY;
}

void updateCurrentPosition(int newX, int newY) {
  currentPositionX = newX;
  currentPositionY = newY;
}

void navigateToNextCell() {
  int minValue = std::numeric_limits<int>::max();
  int nextX = currentPositionX;
  int nextY = currentPositionY;

  // Verificar las celdas adyacentes en las cuatro direcciones: arriba, abajo, izquierda y derecha
  for (int i = 0; i < 4; i++) {
    int newX = currentPositionX + dx[i];
    int newY = currentPositionY + dy[i];

    // Comprobar si la nueva posición está dentro de los límites del laberinto
    if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT) {
      // Comprobar si la celda adyacente tiene un valor menor que el mínimo actual
      if (floodFillArray[newX][newY] < minValue) {
        minValue = floodFillArray[newX][newY];
        nextX = newX;
        nextY = newY;
      }
    }
  }

  // Actualizar la posición actual del robot
  updateCurrentPosition(nextX, nextY);
}



void updateDistances() {
  int goalX, goalY;
  calculateGoalPosition(goalX, goalY);

  for (int i = 0; i < MAZE_WIDTH; i++) {
    for (int j = 0; j < MAZE_HEIGHT; j++) {
      floodFillArray[i][j] = abs(goalX - i) + abs(goalY - j);
    }
  }
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void updateFloodFillArray(int x, int y, int value) {
  floodFillArray[x][y] = value;

  // Verificar las celdas adyacentes en las cuatro direcciones: arriba, abajo, izquierda y derecha
  for (int i = 0; i < 4; i++) {
    int newX = x + dx[i];
    int newY = y + dy[i];

    // Comprobar si la nueva posición está dentro de los límites del laberinto
    if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE) {
      // Comprobar si la celda adyacente tiene un valor mayor que el valor actual + 1
      if (floodFillArray[newX][newY] > value + 1) {
        // Actualizar la celda adyacente con el valor actual + 1 y continuar propagando
        updateFloodFillArray(newX, newY, value + 1);
      }
    }
  }
}

int getOptimalDirection(int x, int y) {
  int minValue = std::numeric_limits<int>::max();
  int optimalDirection = -1;

  // Verificar las celdas adyacentes en las cuatro direcciones: arriba, abajo, izquierda y derecha
  for (int i = 0; i < 4; i++) {
    int newX = x + dx[i];
    int newY = y + dy[i];

    // Comprobar si la nueva posición está dentro de los límites del laberinto
    if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE) {
      // Comprobar si la celda adyacente tiene un valor menor que el mínimo actual
      if (floodFillArray[newX][newY] < minValue) {
        minValue = floodFillArray[newX][newY];
        optimalDirection = i;
      }
    }
  }

  return optimalDirection;
}



bool shouldTurnRight() {
  int rightDirection = (currentDirection >> 1) % 15;
  if (rightDirection == 0) {
    rightDirection = WEST;
  }

  // Comprobar si la celda a la derecha tiene una distancia menor que la actual
  int nextX = currentPositionX;
  int nextY = currentPositionY;
  switch (rightDirection) {
    case NORTH:
      nextY++;
      break;
    case EAST:
      nextX++;
      break;
    case SOUTH:
      nextY--;
      break;
    case WEST:
      nextX--;
      break;
  }

  return (floodFillArray[nextX][nextY] < floodFillArray[currentPositionX][currentPositionY]);
}

bool shouldTurnLeft() {
  int leftDirection = (currentDirection << 1) % 15;
  if (leftDirection == 0) {
    leftDirection = NORTH;
  }

  // Comprobar si la celda a la izquierda tiene una distancia menor que la actual
  int nextX = currentPositionX;
  int nextY = currentPositionY;
  switch (leftDirection) {
    case NORTH:
      nextY++;
      break;
    case EAST:
      nextX++;
      break;
    case SOUTH:
      nextY--;
      break;
    case WEST:
      nextX--;
      break;
  }

  return (floodFillArray[nextX][nextY] < floodFillArray[currentPositionX][currentPositionY]);
}


def moveForward(current_pos, direction):
    x, y = current_pos
    if direction == 'N':
        y += 1
    elif direction == 'S':
        y -= 1
    elif direction == 'E':
        x += 1
    elif direction == 'O':
        x -= 1
    return (x, y)




// Función de interrupción del sensor ranurado
void slotSensorInterrupt() {
  slotSensorPulses++;
}

// Funciones para controlar el robot
void moveForward();
void turnLeft();
void turnRight();
void updateCurrentDirection(int turn);

// Implementación del algoritmo flood-fill
void floodFill();
bool isMazeSolved();
void updateFloodFillArray();
void navigateToNextCell();

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

  // Inicializar el array flood-fill con valores máximos
  for (int i = 0; i < MAZE_WIDTH; i++) {
    for (int j = 0; j < MAZE_HEIGHT; j++) {
      floodFillArray[i][j] = MAZE_WIDTH * MAZE_HEIGHT;
    }
  }
  updateFloodFillArray();

  // Definir la posición inicial basada en la posición de inicio
  int startPosition = 0; // Cambia esto para cambiar la posición de inicio (0-3)

  switch (startPosition) {
    case 0:
      currentPositionX = 0;
      currentPositionY = 0;
      currentDirection = EAST;
      break;
    case 1:
      currentPositionX = MAZE_WIDTH - 1;
      currentPositionY = 0;
      currentDirection = WEST;
      break;
    case 2:
     
      currentPositionX = 0;
      currentPositionY = MAZE_HEIGHT - 1;
      currentDirection = EAST;
      break;
    case 3:
      currentPositionX = MAZE_WIDTH - 1;
      currentPositionY = MAZE_HEIGHT - 1;
      currentDirection = WEST;
      break;
  }

  floodFillArray[currentPositionX][currentPositionY] = 0; // Posición inicial
  floodFill();
}

void loop() {
  int currentCellPulses = 0;
  // Leer el valor del sensor fototransistor
  sensorValue = analogRead(analogInPin);

  // Lógica para controlar los motores
  updateDistances();
  navigateToNextCell();

  // Algoritmo de inundación básico
  floodFill();
  updateFloodFillArray();
  
  // Control de motores
  // Mover hacia adelante hasta que el sensor ranurado detecte un cambio de celda
  if (slotSensorPulses < currentCellPulses) {
    moveForward();
  } else {
    slotSensorPulses = 0;
    currentCellPulses = 0;
    
    // Cambiar de dirección según el algoritmo de inundación
    if (shouldTurnRight()) {
      turnRight();
      updateCurrentDirection(1);
    } else if (shouldTurnLeft()) {
      turnLeft();
      updateCurrentDirection(-1);
    }
  }

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

  // Agregar un retraso antes de la siguiente iteración del bucle
  delay(100);
}

  // Actualizar la posición actual del robot
  updateCurrentPosition(optimalDirection);
  currentDirection = optimalDirection;

  // Incrementar los pulsos del sensor ranurado
  slotSensorPulses++;

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


  delay(100);
}
