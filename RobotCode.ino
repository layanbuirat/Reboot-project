const int MAX_SIZE = 16;  // Maximum number of cells (adjustable)
int maze[MAX_SIZE][MAX_SIZE];  // Maze structure
int numCells;  // Number of cells (e.g., 8×8, 16×16)
float cellSize;  // Size of each cell in cm

void setup() {
    Serial.begin(115200);
    initializeMaze();
    printMaze();
}

void loop() {
    // The maze will be displayed once in setup()
}

void initializeMaze() {
    Serial.println("Enter the number of cells (e.g., 8 for 8x8):");

    while (Serial.available() == 0) {} // Wait for input
    numCells = Serial.parseInt();
    if (numCells <= 0 || numCells > MAX_SIZE) {
        Serial.println("Invalid size! Defaulting to 8x8.");
        numCells = 8;
    }

    Serial.println("Enter the physical size of each cell in cm (e.g., 20):");
    while (Serial.available() == 0) {} // Wait for input
    cellSize = Serial.parseFloat();
    if (cellSize <= 0) {
        Serial.println("Invalid size! Defaulting to 20 cm.");
        cellSize = 20;
    }

    // Generate a simple maze structure
    for (int i = 0; i < numCells; i++) {
        for (int j = 0; j < numCells; j++) {
            maze[i][j] = (i == 0 || j == 0 || i == numCells - 1 || j == numCells - 1) ? 1 : 0;  // Border walls
        }
    }

    Serial.print("Maze size: ");
    Serial.print(numCells);
    Serial.print(" x ");
    Serial.print(numCells);
    Serial.print(" cells | Cell size: ");
    Serial.print(cellSize);
    Serial.println(" cm");
    Serial.print("Total maze dimensions: ");
    Serial.print(numCells * cellSize);
    Serial.print(" cm x ");
    Serial.print(numCells * cellSize);
    Serial.println(" cm");
}

void printMaze() {
    Serial.println("\nGenerated Maze:");
    for (int i = 0; i < numCells; i++) {
        for (int j = 0; j < numCells; j++) {
            Serial.print(maze[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }
}
