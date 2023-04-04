#include <esp32-hal-ledc.h>
#include <QueueArray.h>
#include <Arduino.h>
#include <limits>
#include <vector>
using namespace std;

// Constantes de dirección
#define NORTH 1
#define EAST 2
#define SOUTH 4
#define WEST 8

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
const int MAZE_WIDTH = 12;
const int MAZE_HEIGHT = 7;
const int DX[] = {0, 1, 0, -1}; // Corrección: agregar constantes DX y DY
const int DY[] = {-1, 0, 1, 0};
// Pines del sensor
const int analogInPin = 34;
const int OutPin = 13;
const int slotSensorPin = 22;
int goalX = 0;
int goalY = 0;
char mazeGrid[MAZE_WIDTH][MAZE_HEIGHT];
const int NAVIGATION_MODE = 0; // Corrección: agregar constantes NAVIGATION_MODE y MAPPING_MODE
const int MAPPING_MODE = 1;
// Variables globales
int sensorValue = 0;
volatile int slotSensorPulses = 0;
int currentDirection = NORTH;  // Inicialmente apuntando al norte
int currentX = 0;
int currentY = 0;
int currentMode = MAPPING_MODE; 

// Variables globales para el algoritmo flood-fill
int floodFillArray[MAZE_WIDTH][MAZE_HEIGHT];
int currentPositionX = 0;
int currentPositionY = 0;


const int in1 = 35;
const int in2 = 36;
const int in3 = 37;
const int in4 = 38;


void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  ledcWrite(0, 128);  // Velocidad del motor 1
  ledcWrite(1, 128);  // Velocidad del motor 2

  delay(1000);  // Duración del movimiento hacia adelante (esto puede variar según el robot)

  // Detener los motores
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  // Actualizar posición actual en función de la dirección
  switch (currentDirection) {
    case NORTH:
      currentPositionY++;
      break;
    case EAST:
      currentPositionX++;
      break;
    case SOUTH:
      currentPositionY--;
      break;
    case WEST:
      currentPositionX--;
      break;
  }
}

void turnLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  ledcWrite(0, 128);  // Velocidad del motor 1
  ledcWrite(1, 128);  // Velocidad del motor 2

  delay(800);  // Duración del giro (esto puede variar según el robot)

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
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  ledcWrite(0, 128);  // Velocidad del motor 1
  ledcWrite(1, 128);  // Velocidad del motor 2

  delay(800);  // Duración del giro (esto puede variar según el robot)

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

bool isValid(int x, int y, vector<vector<char>>& grid);
bool hasWall(int x, int y, vector<vector<char>>& grid);
pair<int, int> updateCurrentPosition(int x, int y, int dx, int dy);
void updateFloodFillArray(int x, int y, vector<vector<char>>& grid, vector<vector<int>>& floodFillArray, int value, vector<vector<char>>& mazeGrid);
void navigateToNextCell(int x, int y, vector<vector<char>>& grid, vector<vector<int>>& floodFillArray, vector<vector<bool>>& visited);
vector<vector<int>> flood_fill(vector<vector<char>>& grid, int startX, int startY);

// Función para calcular la posición objetivo
void calculateGoalPosition(int &goalX, int &goalY) {
  // Calcular la posición objetivo en función del tamaño del laberinto
  goalX = (MAZE_WIDTH / 2) - 2;
  goalY = (MAZE_HEIGHT / 2) - 2;
}


void updateFloodFillArray(int mazeGrid[][MAZE_HEIGHT]) {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      if (isValid(x, y, mazeGrid)) {
        int newX, newY;
        for (int d = 0; d < 4; d++) {
          newX = x + DX[d];
          newY = y + DY[d];
          if (isValid(newX, newY, mazeGrid) && !visited[newX][newY] && !hasWall(x, y, d, mazeGrid)) { // Corrección: agregar mazeGrid como argumento
            floodFillArray[newX][newY] = min(floodFillArray[newX][newY], floodFillArray[x][y] + 1);
          }
        }
      }
    }
  }
}


// Implementación del algoritmo flood-fill
void floodFill() {
  int mazeGrid[MAZE_WIDTH][MAZE_HEIGHT];
  updateFloodFillArray(mazeGrid);
}


bool isValid(int x, int y, vector<vector<char>>& grid) {
    if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
        return false;
    }
    return true;
}

bool hasWall(int x, int y, vector<vector<char>>& grid) {
    if (grid[x][y] == '#') {
        return true;
    }
    return false;
}

pair<int, int> updateCurrentPosition(int x, int y, int dx, int dy) {
    x += dx;
    y += dy;
    return make_pair(x, y);
}

void updateFloodFillArray(int x, int y, vector<vector<char>>& grid, vector<vector<int>>& floodFillArray, int value, vector<vector<char>>& mazeGrid) {
    if (isValid(x, y, grid) && !hasWall(x, y, grid)) {
        floodFillArray[x][y] = value;
    }
}

void navigateToNextCell(int x, int y, vector<vector<char>>& grid, vector<vector<int>>& floodFillArray, vector<vector<bool>>& visited) {
    vector<pair<int, int>> neighbors = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    for (auto neighbor : neighbors) {
        int newX, newY;
        tie(newX, newY) = updateCurrentPosition(x, y, neighbor.first, neighbor.second);

        if (isValid(newX, newY, grid) && !hasWall(newX, newY, grid) && !visited[newX][newY]) {
            updateFloodFillArray(newX, newY, grid, floodFillArray, floodFillArray[x][y] + 1, mazeGrid);
            visited[newX][newY] = true;
            navigateToNextCell(newX, newY, grid, floodFillArray, visited);
        }
    }
}

vector<vector<int>> flood_fill(vector<vector<char>>& grid, int startX, int startY) {
    int rows = grid.size();
    int cols = grid[0].size();
    vector<vector<int>> floodFillArray(rows, vector<int>(cols, -1));

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            if (grid[x][y] == '#') {
                floodFillArray[x][y] = -1;
            } else {
                floodFillArray[x][y] = 0;
            }
        }
    }

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    visited[startX][startY] = true;

    navigateToNextCell(startX, startY, grid, floodFillArray, visited);
    return floodFillArray;
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

void updateFloodFillArray(int x, int y, int value, vector<vector<char>>& mazeGrid) {
  floodFillArray[x][y] = value;

  // Verificar las celdas adyacentes en las cuatro direcciones: arriba, abajo, izquierda y derecha
  for (int i = 0; i < 4; i++) {
    int newX = x + dx[i];
    int newY = y + dy[i];

    // Comprobar si la nueva posición está dentro de los límites del laberinto
    if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT) {
      // Comprobar si la celda adyacente tiene un valor mayor que el valor actual + 1
      if (floodFillArray[newX][newY] > value + 1) {
        // Actualizar la celda adyacente con el valor actual + 1 y continuar propagando
        updateFloodFillArray(newX, newY, value + 1, mazeGrid);
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
    if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT) {
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
void updateFloodFillArray(const vector<vector<char>>& mazeGrid);
void navigateToNextCell();

void setup() {
  int mazeGrid[MAZE_WIDTH][MAZE_HEIGHT];
  // Configurar pines de control del motor
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2Speed, OUTPUT);

  // Configurar PWM para controlar la velocidad del motor
  ledcSetup(0, 5000, 8);  // Canal 0, frecuencia 5000 Hz, resolución de 8 bits
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
  updateFloodFillArray(mazeGrid);

  // Definir la posición inicial basada en la posición de inicio
  int startPosition = 0;  // Cambia esto para cambiar la posición de inicio (0-3)

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

  floodFillArray[currentPositionX][currentPositionY] = 0;  // Posición inicial
  floodFill();
}

void loop() {
  int mazeGrid[MAZE_WIDTH][MAZE_HEIGHT];
  int currentCellPulses = 0;
  // Leer el valor del sensor fototransistor
  sensorValue = analogRead(analogInPin);

  // Lógica para controlar los motores
  updateDistances();
  navigateToNextCell();

  // Algoritmo de inundación básico
  floodFill();
  updateFloodFillArray(mazeGrid);

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

  // Actualizar la posición actual del robot
if (currentMode == NAVIGATION_MODE) {
  int optimalDirection = getOptimalDirection();
  int newX, newY;
  tie(newX, newY) = updateCurrentPosition(currentX, currentY, DX[optimalDirection], DY[optimalDirection]); // Corrección: llamar a updateCurrentPosition() de forma diferente
  updateCurrentPosition(newX, newY);

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



