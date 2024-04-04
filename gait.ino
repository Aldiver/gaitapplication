#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

const int LEFT_FSR_PIN_1 = A0;
const int LEFT_FSR_PIN_2 = A1;
const int LEFT_PIEZO_PIN_1 = A2;
const int LEFT_PIEZO_PIN_2 = A3;
const int RIGHT_FSR_PIN_1 = A4;
const int RIGHT_FSR_PIN_2 = A5;
const int RIGHT_PIEZO_PIN_1 = A6;
const int RIGHT_PIEZO_PIN_2 = A7;

Adafruit_ADXL345_Unified accelLeft = Adafruit_ADXL345_Unified(0x1D); // Left accelerometer address
Adafruit_ADXL345_Unified accelRight = Adafruit_ADXL345_Unified(0x53); // Right accelerometer address

const int CALIBRATION_TIME = 5000; // 5 seconds
const int SAMPLE_INTERVAL = 100; // 100 milliseconds
const int MINIMUM_LAND_DIFFERENCE = 20; // Adjust as needed
const int MINIMUM_LIFT_DIFFERENCE = 20; // Adjust as needed
const float STEP_INTERVAL = 1000; // 1 second
const float MAX_STEP_INTERVAL_ERROR = 0.2; // 20%

File dataFile;

// Struct for sensor data and calibration values
struct SensorData {
  int FSR1;
  int FSR2;
  int Piezo1;
  int Piezo2;
};

// Struct for both feet sensor data and calibration values
struct FootSensorData {
  SensorData left;
  SensorData right;
};

FootSensorData liftedCalibration;
FootSensorData landedCalibration;
FootSensorData currentSensorData;

// State machine states
enum State {
  LIFTED_LEFT_FOOT,
  LANDED_LEFT_FOOT,
  LIFTED_RIGHT_FOOT,
  LANDED_RIGHT_FOOT
};

// Variables to track state and timing
State currentState;
unsigned long liftTimestamp;
unsigned long landTimestamp;
bool samplingFlag = false; // Flag to indicate data sampling

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  
  if (!SD.begin(4)) {
    Serial.println("SD initialization failed!");
    return;
  }

  if (!accelLeft.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor for left foot, check wiring!");
    while (1);
  }

  if (!accelRight.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor for right foot, check wiring!");
    while (1);
  }
  
  calibrateFoot(LEFT_FSR_PIN_1, LEFT_FSR_PIN_2, LEFT_PIEZO_PIN_1, LEFT_PIEZO_PIN_2, &liftedCalibration);
  calibrateFoot(LEFT_FSR_PIN_1, LEFT_FSR_PIN_2, LEFT_PIEZO_PIN_1, LEFT_PIEZO_PIN_2, &landedCalibration);
  calibrateFoot(RIGHT_FSR_PIN_1, RIGHT_FSR_PIN_2, RIGHT_PIEZO_PIN_1, RIGHT_PIEZO_PIN_2, &liftedCalibration);
  calibrateFoot(RIGHT_FSR_PIN_1, RIGHT_FSR_PIN_2, RIGHT_PIEZO_PIN_1, RIGHT_PIEZO_PIN_2, &landedCalibration);
  
  currentState = LIFTED_LEFT_FOOT;
}

void loop() {
  // Read sensor data for both feet
  readSensorData(&currentSensorData.left, LEFT_FSR_PIN_1, LEFT_FSR_PIN_2, LEFT_PIEZO_PIN_1, LEFT_PIEZO_PIN_2);
  readSensorData(&currentSensorData.right, RIGHT_FSR_PIN_1, RIGHT_FSR_PIN_2, RIGHT_PIEZO_PIN_1, RIGHT_PIEZO_PIN_2);
  
  // Update state machine based on sensor readings
  updateStateMachine(currentSensorData);

  // If both feet are in opposite states (i.e., left foot lifted, right foot landed or vice versa),
  // start data sampling
  if ((currentState == LIFTED_LEFT_FOOT && currentState == LANDED_RIGHT_FOOT) ||
      (currentState == LANDED_LEFT_FOOT && currentState == LIFTED_RIGHT_FOOT)) {
    startDataSampling();
  }
  
  // If sampling flag is set, continuously record sensor data with timestamps
  if (samplingFlag) {
    recordSensorData();
  }
  
  // Check if state machine returns to initial state, indicating end of data sampling
  if ((currentState == LIFTED_LEFT_FOOT && currentState == LANDED_RIGHT_FOOT) ||
      (currentState == LANDED_LEFT_FOOT && currentState == LIFTED_RIGHT_FOOT)) {
    stopDataSampling();
  }
}

void calibrateFoot(int fsrPin1, int fsrPin2, int piezoPin1, int piezoPin2, FootSensorData* calibrationData) {
  int sumLiftedFSR1 = 0, sumLiftedFSR2 = 0, sumLiftedPiezo1 = 0, sumLiftedPiezo2 = 0;
  int sumLandedFSR1 = 0, sumLandedFSR2 = 0, sumLandedPiezo1 = 0, sumLandedPiezo2 = 0;
  unsigned long startMillis = millis();

  // Calibration for lifted values
  while (millis() - startMillis < CALIBRATION_TIME) {
    sumLiftedFSR1 += analogRead(fsrPin1);
    sumLiftedFSR2 += analogRead(fsrPin2);
    sumLiftedPiezo1 += analogRead(piezoPin1);
    sumLiftedPiezo2 += analogRead(piezoPin2);
    delay(SAMPLE_INTERVAL);
  }
  calibrationData->left.FSR1 = sumLiftedFSR1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->left.FSR2 = sumLiftedFSR2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->left.Piezo1 = sumLiftedPiezo1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->left.Piezo2 = sumLiftedPiezo2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);

  // Delay before calibrating landed values
  delay(CALIBRATION_TIME);

  startMillis = millis();

  // Calibration for landed values
  while (millis() - startMillis < CALIBRATION_TIME) {
    sumLandedFSR1 += analogRead(fsrPin1);
    sumLandedFSR2 += analogRead(fsrPin2);
    sumLandedPiezo1 += analogRead(piezoPin1);
    sumLandedPiezo2 += analogRead(piezoPin2);
    delay(SAMPLE_INTERVAL);
  }
  calibrationData->right.FSR1 = sumLandedFSR1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->right.FSR2 = sumLandedFSR2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->right.Piezo1 = sumLandedPiezo1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->right.Piezo2 = sumLandedPiezo2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
}

void readSensorData(SensorData* data, int fsrPin1, int fsrPin2, int piezoPin1, int piezoPin2) {
  data->FSR1 = analogRead(fsrPin1);
  data->FSR2 = analogRead(fsrPin2);
  data->Piezo1 = analogRead(piezoPin1);
  data->Piezo2 = analogRead(piezoPin2);
}

void updateStateMachine(FootSensorData sensorData) {
  int leftFSR_diff = abs(liftedCalibration.left.FSR1 - sensorData.left.FSR1) + abs(liftedCalibration.left.FSR2 - sensorData.left.FSR2);
  int leftPiezo_diff = abs(landedCalibration.left.Piezo1 - sensorData.left.Piezo1) + abs(landedCalibration.left.Piezo2 - sensorData.left.Piezo2);
  
  int rightFSR_diff = abs(liftedCalibration.right.FSR1 - sensorData.right.FSR1) + abs(liftedCalibration.right.FSR2 - sensorData.right.FSR2);
  int rightPiezo_diff = abs(landedCalibration.right.Piezo1 - sensorData.right.Piezo1) + abs(landedCalibration.right.Piezo2 - sensorData.right.Piezo2);

  // Check left foot state
  if (leftFSR_diff > MINIMUM_LAND_DIFFERENCE && leftPiezo_diff > MINIMUM_LAND_DIFFERENCE) {
    currentState = LANDED_LEFT_FOOT;
  } else if (leftFSR_diff > MINIMUM_LIFT_DIFFERENCE && leftPiezo_diff > MINIMUM_LIFT_DIFFERENCE) {
    currentState = LIFTED_LEFT_FOOT;
  }

  // Check right foot state
  if (rightFSR_diff > MINIMUM_LAND_DIFFERENCE && rightPiezo_diff > MINIMUM_LAND_DIFFERENCE) {
    currentState = LANDED_RIGHT_FOOT;
  } else if (rightFSR_diff > MINIMUM_LIFT_DIFFERENCE && rightPiezo_diff > MINIMUM_LIFT_DIFFERENCE) {
    currentState = LIFTED_RIGHT_FOOT;
  }
}

void startDataSampling() {
  samplingFlag = true;
  // Start recording data to SD card with timestamp
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Start Sampling - Timestamp: ");
    dataFile.println(millis());
    dataFile.close();
  }
}

void stopDataSampling() {
  samplingFlag = false;
  // End recording data to SD card with timestamp
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("End Sampling - Timestamp: ");
    dataFile.println(millis());
    dataFile.close();
  }
}

void recordSensorData() {
  // Record sensor data along with timestamp to SD card
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (dataFile) {
    // Record timestamp
    dataFile.print("Timestamp: ");
    dataFile.println(millis());
    // Record left foot sensor data
    dataFile.print("Left FSR1: ");
    dataFile.println(currentSensorData.left.FSR1);
    dataFile.print("Left FSR2: ");
    dataFile.println(currentSensorData.left.FSR2);
    dataFile.print("Left Piezo1: ");
    dataFile.println(currentSensorData.left.Piezo1);
    dataFile.print("Left Piezo2: ");
    dataFile.println(currentSensorData.left.Piezo2);
    // Record right foot sensor data
    dataFile.print("Right FSR1: ");
    dataFile.println(currentSensorData.right.FSR1);
    dataFile.print("Right FSR2: ");
    dataFile.println(currentSensorData.right.FSR2);
    dataFile.print("Right Piezo1: ");
    dataFile.println(currentSensorData.right.Piezo1);
    dataFile.print("Right Piezo2: ");
    dataFile.println(currentSensorData.right.Piezo2);
    dataFile.println(); // Add newline for better readability
    dataFile.close();
  }
}
