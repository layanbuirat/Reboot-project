#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>  // Include the MPU6050 library


// Motor pins
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
#define ENA 26
#define ENB 25
#define ROBOT_RADIUS 6

// IR sensor pins
#define IR_LEFT 34
#define IR_RIGHT 35

// Encoder pins
#define ENCA_A 4
#define ENCA_B 18
#define ENCB_A 19
#define ENCB_B 23

// LiDAR
VL53L0X lidar;

MPU6050 mpu;

#include <queue> // Include the queue library for flood-fill

// Structure to represent a cell in the maze
struct Cell {
  int x;
  int y;
};



// Encoder variables
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;

// Function prototypes for encoder interrupts
void IRAM_ATTR encoderAInterrupt();
void IRAM_ATTR encoderBInterrupt();
void IRAM_ATTR encoderCInterrupt();
void IRAM_ATTR encoderDInterrupt();

const int mazeSize = 8;
int maze[mazeSize][mazeSize];

void initializeMaze() {
  // Set the goal cells to weight 0
  maze[7][7] = 0;
  maze[7][8] = 0;
  maze[8][7] = 0;
  maze[8][8] = 0;

  // Calculate Manhattan distance for all other cells
  for (int row = 0; row < mazeSize; row++) {
    for (int col = 0; col < mazeSize; col++) {
      if (!((row == 7 && col == 7) || (row == 7 && col == 8) || 
            (row == 8 && col == 7) || (row == 8 && col == 8))) {
        // Calculate minimum Manhattan distance to the closest goal cell
        int distToGoal1 = abs(row - 7) + abs(col - 7);
        int distToGoal2 = abs(row - 7) + abs(col - 8);
        int distToGoal3 = abs(row - 8) + abs(col - 7);
        int distToGoal4 = abs(row - 8) + abs(col - 8);
        
        maze[row][col] = min(min(distToGoal1, distToGoal2), 
                             min(distToGoal3, distToGoal4));
      }
    }
  }
}


void printMaze() {
  for (int row = 0; row < mazeSize; row++) {
    for (int col = 0; col < mazeSize; col++) {
      Serial.print(maze[row][col]);
      Serial.print("\t"); // Use tab for better spacing
    }
    Serial.println(); // New line after each row
  }
}

// Robot's current position in the maze
struct Position {
  int x;  // Column
  int y;  // Row
};

Position currentPosition = {0, 0}; // Starting position at (0,0)

// Constants for wheel circumference and cell size
const float WHEEL_CIRCUMFERENCE = 12.56;  // Wheel circumference in cm (for 4 cm diameter)
const int PULSES_PER_REV = 204;           // Encoder pulses per wheel revolution
const int CELL_SIZE = 20;                 // Cell size in cm

const float DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;  // 0.0616 cm per pulse
const int PULSES_PER_CELL = CELL_SIZE / DISTANCE_PER_PULSE;  // ~325 pulses per cell

// Encoder pulse counters
volatile long leftWheelPulses = 0;
volatile long rightWheelPulses = 0;

// Encoder interrupt handlers
void IRAM_ATTR encoderAInterrupt() {
  leftWheelPulses++;
}

void IRAM_ATTR encoderBInterrupt() {
  leftWheelPulses--;  // Decrease pulses for backward direction
}

void IRAM_ATTR encoderCInterrupt() {
  rightWheelPulses++;
}

void IRAM_ATTR encoderDInterrupt() {
  rightWheelPulses--;  // Decrease pulses for backward direction
}

// Function to update the robot's position in the maze
void updatePosition() {
  static long lastLeftPulses = 0;
  static long lastRightPulses = 0;

  // Calculate the change in pulses since the last update
  long deltaLeft = leftWheelPulses - lastLeftPulses;
  long deltaRight = rightWheelPulses - lastRightPulses;

  lastLeftPulses = leftWheelPulses;
  lastRightPulses = rightWheelPulses;

  // If the pulses cover one cell, move to the next cell
  if (deltaLeft >= PULSES_PER_CELL && deltaRight >= PULSES_PER_CELL) {
    leftWheelPulses -= PULSES_PER_CELL;
    rightWheelPulses -= PULSES_PER_CELL;

    // Print the new position
    Serial.print("Moved to Cell: (");
    Serial.print(currentPosition.x);
    Serial.print(", ");
    Serial.print(currentPosition.y);
    Serial.println(")");
  }
}

// Free cells array (up to 3 possible free cells: left, right, front)
int freeCells[4][2]; // To store row and column of free cells
int freeCellCount = 0;

void readSensors() {
  freeCellCount = 0; // Reset the free cell count

  // Add the current cell to the freeCells array
  freeCells[freeCellCount][0] = currentPosition.y;
  freeCells[freeCellCount][1] = currentPosition.x;
  freeCellCount++;

  // Read LiDAR distance
  int distance = lidar.readRangeSingleMillimeters() / 10;

  if (lidar.timeoutOccurred()) {
    Serial.println("LiDAR Timeout!");
    return;
  }
  Serial.print("LiDAR Distance (cm): ");
  Serial.println(distance);

  // Read IR sensors (line detection or obstacle detection)
  int irLeft = digitalRead(IR_LEFT);
  int irRight = digitalRead(IR_RIGHT);

  Serial.print("IR Left: ");
  Serial.println(irLeft);
  Serial.print("IR Right: ");
  Serial.println(irRight);

  // Check the left cell
  if (irLeft == 1) { // Left cell is free
    int leftRow = currentPosition.y;
    int leftCol = currentPosition.x - 1;
    if (leftCol >= 0) { // Ensure it's within the maze
      freeCells[freeCellCount][0] = leftRow;
      freeCells[freeCellCount][1] = leftCol;
      freeCellCount++;
    }
  }

  // Check the right cell
  if (irRight == 1) { // Right cell is free
    int rightRow = currentPosition.y;
    int rightCol = currentPosition.x + 1;
    if (rightCol < mazeSize) { // Ensure it's within the maze
      freeCells[freeCellCount][0] = rightRow;
      freeCells[freeCellCount][1] = rightCol;
      freeCellCount++;
    }
  }

  // Check the front cell
  if (distance > 9) { // Front cell is free
    int frontRow = currentPosition.y + 1; // Move up in the maze
    int frontCol = currentPosition.x;
    if (frontRow >= 0) { // Ensure it's within the maze
      freeCells[freeCellCount][0] = frontRow;
      freeCells[freeCellCount][1] = frontCol;
      freeCellCount++;
    }
  }

  }




Cell findMinWeightCell(int freeCells[][2], int freeCellCount, int& minWeight, Cell currentCell) {
  Cell minCell;
  minCell.x = currentCell.x;
  minCell.y = currentCell.y;
  minWeight = maze[currentCell.y][currentCell.x];

  for (int i = 0; i < freeCellCount; i++) {
    int cellX = freeCells[i][1];
    int cellY = freeCells[i][0];
    int cellWeight = maze[cellY][cellX];

    if (cellWeight < minWeight) {
      minWeight = cellWeight;
      minCell.x = cellX;
      minCell.y = cellY;
    }
  }

  return minCell;
}


// Function to implement the flood-fill algorithm
void floodFill(Cell currentCell, int minFreeCellWeight) {
  // Update the current cell's weight to be 1 greater than the minimum free cell weight
  maze[currentCell.y][currentCell.x] = minFreeCellWeight + 1;

  // Optional: Print the updated weight for debugging
  Serial.print("Updated current cell weight to: ");
  Serial.println(maze[currentCell.y][currentCell.x]);
}

Cell makeDecision() {
  int minFreeCellWeight;
  Cell nextPosition; // Declare nextPosition
  Cell currentPosition1;
  Cell minCell = findMinWeightCell(freeCells, freeCellCount, minFreeCellWeight, currentPosition1); // Pass currentPosition

  // Check if the current cell has the minimum weight
  if (minCell.x == currentPosition1.x && minCell.y == currentPosition1.y) {
    // Call floodFill to update the current cell's weight
    floodFill(minCell, minFreeCellWeight);
    nextPosition.x = minCell.x;
    nextPosition.y = minCell.y;
  } else {
    // Move to the cell with the minimum weight
    nextPosition.x = minCell.x;
    nextPosition.y = minCell.y;
    Serial.print("Moving to Cell: (");
    Serial.print(nextPosition.x);
    Serial.print(", ");
    Serial.print(nextPosition.y);
    Serial.println(")");
  }

  // Return the next position
  return nextPosition;
}

void moveMotor(Cell targetCell) {
  // Calculate the direction to move
  int deltaX = targetCell.x - currentPosition.x;
  int deltaY = targetCell.y - currentPosition.y;

  // Move in the horizontal direction first
  if (deltaX > 0) {  // Move right
    turnRight();
    delay(600);  // Move exactly one cell
  } else if (deltaX < 0) {  // Move left
    turnLeft();
    delay(600);  // Move exactly one cell
  }

  // Stop motors before changing direction
 // stopMotors();

  // Move in the vertical direction
  if (deltaY > 0) {  // Move down
    moveForward();
    delay(600);  // Move exactly one cell
  } else if (deltaY < 0) {  // Move up
    moveBackward();
    delay(600);  // Move exactly one cell
  }

  // Stop the motors again after reaching the cell
  //stopMotors();

  // Update the current position after reaching the target
  currentPosition.x = targetCell.x;
  currentPosition.y = targetCell.y;

  // Print the updated position
  Serial.print("Robot moved to: (");
  Serial.print(currentPosition.x);
  Serial.print(", ");
  Serial.print(currentPosition.y);
  Serial.println(")");
}


void setup() {
  Serial.begin(115200);
  initializeMaze();
  
  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR sensors setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Encoder setup
  pinMode(ENCA_A, INPUT);
  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_A, INPUT);
  pinMode(ENCB_B, INPUT);

  // Attach interrupts for both channels of both encoders
  attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), encoderBInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderCInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), encoderDInterrupt, CHANGE);
  
  // Initialize I2C communication for MPU6050
  Wire.begin(21, 22);
  mpu.initialize(); // Initialize the MPU6050 sensor
  lidar.init();
  lidar.setTimeout(800);
}

void loop() {  
 
  if(maze[currentPosition.y][currentPosition.x]==0){
       stopMotors();
        return ;
  }
  

 readSensors();

 // Call makeDecision to determine the next cell to move to
  Cell targetCell = makeDecision();

  // Move the robot to the target cell
  moveMotor(targetCell);

  // Optional: Add a delay to avoid rapid movements or unnecessary processing
 // delay(500);
  
  }


// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 58); // left wheel
  analogWrite(ENB, 67);  // right wheel
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 58);  //left wheel
  analogWrite(ENB,67);   // right wheel
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 58);  // left wheel
  analogWrite(ENB, 67);  // right wheel
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 58);  // left wheel
  analogWrite(ENB, 67);  // right wheel
}
