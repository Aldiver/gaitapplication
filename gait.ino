#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h>

const int LEFT_FSR_PIN_1 = A0;
const int LEFT_FSR_PIN_2 = A1;
const int LEFT_PIEZO_PIN_1 = A2;
const int LEFT_PIEZO_PIN_2 = A3;
const int RIGHT_FSR_PIN_1 = A4;
const int RIGHT_FSR_PIN_2 = A5;
const int RIGHT_PIEZO_PIN_1 = A6;
const int RIGHT_PIEZO_PIN_2 = A7;

Adafruit_MPU6050 mpuLeft;
Adafruit_MPU6050 mpuRight;

const int CALIBRATION_TIME = 5000; // 5 seconds
const int SAMPLE_INTERVAL = 100;   // 100 milliseconds
const int MINIMUM_LAND_DIFFERENCE = 20; // Adjust as needed
const int MINIMUM_LIFT_DIFFERENCE = 20; // Adjust as needed

File dataFile;

struct SensorData {
  unsigned long timestamp;
  int FSR1;
  int FSR2;
  int Piezo1;
  int Piezo2;
  float AccX;
  float AccY;
  float AccZ;
};

struct FootSensorData {
  SensorData lifted;
  SensorData landed;
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

State currentState;
bool samplingFlag = false;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  if (!SD.begin(4)) {
    Serial.println("SD initialization failed!");
    return;
  }

  // Initialize the first MPU6050 sensor
  if (!mpuLeft.begin()) {
    Serial.println("Failed to find left MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Left MPU6050 Found!");

  // Initialize the second MPU6050 sensor
  if (!mpuRight.begin()) {
    Serial.println("Failed to find right MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Right MPU6050 Found!");

  // Set configurations for the first sensor
  mpuLeft.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpuLeft.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set configurations for the second sensor
  mpuRight.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpuRight.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateFoot(LEFT_FSR_PIN_1, LEFT_FSR_PIN_2, LEFT_PIEZO_PIN_1, LEFT_PIEZO_PIN_2, &liftedCalibration);
  calibrateFoot(LEFT_FSR_PIN_1, LEFT_FSR_PIN_2, LEFT_PIEZO_PIN_1, LEFT_PIEZO_PIN_2, &landedCalibration);
  calibrateFoot(RIGHT_FSR_PIN_1, RIGHT_FSR_PIN_2, RIGHT_PIEZO_PIN_1, RIGHT_PIEZO_PIN_2, &liftedCalibration);
  calibrateFoot(RIGHT_FSR_PIN_1, RIGHT_FSR_PIN_2, RIGHT_PIEZO_PIN_1, RIGHT_PIEZO_PIN_2, &landedCalibration);

  currentState = LIFTED_LEFT_FOOT;

  Serial.println("");
  delay(100);
}

void loop() {
  readSensorData(&currentSensorData.lifted, LEFT_FSR_PIN_1, LEFT_FSR_PIN_2, LEFT_PIEZO_PIN_1, LEFT_PIEZO_PIN_2, &mpuLeft);
  readSensorData(&currentSensorData.landed, RIGHT_FSR_PIN_1, RIGHT_FSR_PIN_2, RIGHT_PIEZO_PIN_1, RIGHT_PIEZO_PIN_2, &mpuRight);

  updateStateMachine(currentSensorData);

  if ((currentState == LIFTED_LEFT_FOOT && currentState == LANDED_RIGHT_FOOT) ||
      (currentState == LANDED_LEFT_FOOT && currentState == LIFTED_RIGHT_FOOT)) {
    startDataSampling();
  }

  if (samplingFlag) {
    recordSensorData();
  }

  if ((currentState == LIFTED_LEFT_FOOT && currentState == LANDED_RIGHT_FOOT) ||
      (currentState == LANDED_LEFT_FOOT && currentState == LIFTED_RIGHT_FOOT)) {
    stopDataSampling();
  }
}

void calibrateFoot(int fsrPin1, int fsrPin2, int piezoPin1, int piezoPin2, FootSensorData* calibrationData) {
  int sumLiftedFSR1 = 0, sumLiftedFSR2 = 0, sumLiftedPiezo1 = 0, sumLiftedPiezo2 = 0;
  int sumLandedFSR1 = 0, sumLandedFSR2 = 0, sumLandedPiezo1 = 0, sumLandedPiezo2 = 0;
  unsigned long startMillis = millis();

  while (millis() - startMillis < CALIBRATION_TIME) {
    sumLiftedFSR1 += analogRead(fsrPin1);
    sumLiftedFSR2 += analogRead(fsrPin2);
    sumLiftedPiezo1 += analogRead(piezoPin1);
    sumLiftedPiezo2 += analogRead(piezoPin2);
    delay(SAMPLE_INTERVAL);
  }
  calibrationData->lifted.FSR1 = sumLiftedFSR1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->lifted.FSR2 = sumLiftedFSR2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->lifted.Piezo1 = sumLiftedPiezo1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->lifted.Piezo2 = sumLiftedPiezo2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);

  delay(CALIBRATION_TIME);

  startMillis = millis();

  while (millis() - startMillis < CALIBRATION_TIME) {
    sumLandedFSR1 += analogRead(fsrPin1);
    sumLandedFSR2 += analogRead(fsrPin2);
    sumLandedPiezo1 += analogRead(piezoPin1);
    sumLandedPiezo2 += analogRead(piezoPin2);
    delay(SAMPLE_INTERVAL);
  }
  calibrationData->landed.FSR1 = sumLandedFSR1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->landed.FSR2 = sumLandedFSR2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->landed.Piezo1 = sumLandedPiezo1 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
  calibrationData->landed.Piezo2 = sumLandedPiezo2 / (CALIBRATION_TIME / SAMPLE_INTERVAL);
}

void readSensorData(SensorData* data, int fsrPin1, int fsrPin2, int piezoPin1, int piezoPin2, Adafruit_MPU6050* mpu) {
  sensors_event_t a, g, temp;
  mpu->getEvent(&a, &g, &temp);

  data->FSR1 = analogRead(fsrPin1);
  data->FSR2 = analogRead(fsrPin2);
  data->Piezo1 = analogRead(piezoPin1);
  data->Piezo2 = analogRead(piezoPin2);
  data->AccX = a.acceleration.x;
  data->AccY = a.acceleration.y;
  data->AccZ = a.acceleration.z;
}

void updateStateMachine(FootSensorData sensorData) {
  int leftFSR_diff = abs(liftedCalibration.lifted.FSR1 - sensorData.lifted.FSR1) + abs(liftedCalibration.lifted.FSR2 - sensorData.lifted.FSR2);
  int leftPiezo_diff = abs(landedCalibration.landed.Piezo1 - sensorData.landed.Piezo1) + abs(landedCalibration.landed.Piezo2 - sensorData.landed.Piezo2);
  
  int rightFSR_diff = abs(liftedCalibration.lifted.FSR1 - sensorData.lifted.FSR1) + abs(liftedCalibration.lifted.FSR2 - sensorData.lifted.FSR2);
  int rightPiezo_diff = abs(landedCalibration.landed.Piezo1 - sensorData.landed.Piezo1) + abs(landedCalibration.landed.Piezo2 - sensorData.landed.Piezo2);

  if (leftFSR_diff > MINIMUM_LAND_DIFFERENCE && leftPiezo_diff > MINIMUM_LAND_DIFFERENCE) {
    currentState = LANDED_LEFT_FOOT;
  } else if (leftFSR_diff > MINIMUM_LIFT_DIFFERENCE && leftPiezo_diff > MINIMUM_LIFT_DIFFERENCE) {
    currentState = LIFTED_LEFT_FOOT;
  }

  if (rightFSR_diff > MINIMUM_LAND_DIFFERENCE && rightPiezo_diff > MINIMUM_LAND_DIFFERENCE) {
    currentState = LANDED_RIGHT_FOOT;
  } else if (rightFSR_diff > MINIMUM_LIFT_DIFFERENCE && rightPiezo_diff > MINIMUM_LIFT_DIFFERENCE) {
    currentState = LIFTED_RIGHT_FOOT;
  }
}

void startDataSampling() {
  samplingFlag = true;
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Start Sampling - Timestamp: ");
    dataFile.println(millis());
    dataFile.close();
  }
}

void stopDataSampling() {
  samplingFlag = false;
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("End Sampling - Timestamp: ");
    dataFile.println(millis());
    dataFile.close();
  }
}

void recordSensorData() {
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Timestamp: ");
    dataFile.println(currentSensorData.landed.timestamp);
    dataFile.print("Left FSR1: ");
    dataFile.println(currentSensorData.lifted.FSR1);
    dataFile.print("Left FSR2: ");
    dataFile.println(currentSensorData.lifted.FSR2);
    dataFile.print("Left Piezo1: ");
    dataFile.println(currentSensorData.landed.Piezo1);
    dataFile.print("Left Piezo2: ");
    dataFile.println(currentSensorData.landed.Piezo2);
    dataFile.print("Left AccX: ");
    dataFile.println(currentSensorData.lifted.AccX);
    dataFile.print("Left AccY: ");
    dataFile.println(currentSensorData.lifted.AccY);
    dataFile.print("Left AccZ: ");
    dataFile.println(currentSensorData.lifted.AccZ);
    dataFile.print("Right FSR1: ");
    dataFile.println(currentSensorData.lifted.FSR1);
    dataFile.print("Right FSR2: ");
    dataFile.println(currentSensorData.lifted.FSR2);
    dataFile.print("Right Piezo1: ");
    dataFile.println(currentSensorData.landed.Piezo1);
    dataFile.print("Right Piezo2: ");
    dataFile.println(currentSensorData.landed.Piezo2);
    dataFile.print("Right AccX: ");
    dataFile.println(currentSensorData.lifted.AccX);
    dataFile.print("Right AccY: ");
    dataFile.println(currentSensorData.lifted.AccY);
    dataFile.print("Right AccZ: ");
    dataFile.println(currentSensorData.lifted.AccZ);
    dataFile.println();
    dataFile.close();
  }
}
