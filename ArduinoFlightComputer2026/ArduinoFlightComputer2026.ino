#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_MKRENV.h>
#include <SD.h>
#include <SPI.h>

Adafruit_MPU6050 mpu;
File logFile;

unsigned long lastFlush = 0;

String folderName = "RKT2026";

String getNextFlightFilename() {
  if (!SD.exists(folderName)) {
    SD.mkdir(folderName);
  }

  int flightNum = 1;
  String filename;

  while (true) {
    filename = "/" + folderName + "/FLIGHT" + String(flightNum) + ".csv";
    if (!SD.exists(filename)) {
      return filename;
    }
    flightNum++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Initializing sensors...");

  // MPU6050 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 OK");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // MKR ENV SHIELD 
  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }
  Serial.println("MKR ENV Shield OK");

  // SD CARD
  while (!SD.begin()) {
    Serial.println("SD init failed, retrying...");
    delay(500);
  }
  Serial.println("SD card OK");

  // Create next flight file
  String flightFile = getNextFlightFilename();
  Serial.print("Logging to: ");
  Serial.println(flightFile);

  logFile = SD.open(flightFile, FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open flight log file!");
    while (1);
  }

  // Clean CSV header
  logFile.println("millis,ax,ay,az,gx,gy,gz,pitch,roll,temperature,pressure");
  logFile.flush();

  Serial.println("Logging started...");
}

void loop() {
  // MPU6050 READINGS
  sensors_event_t accel, gyro, tempEvent;
  mpu.getEvent(&accel, &gyro, &tempEvent);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // ENV SHIELD READINGS
  float temperature = ENV.readTemperature();
  float pressure    = ENV.readPressure();

  // WRITE TO SD (CSV)
  unsigned long t = millis();

  logFile.print(t); logFile.print(",");
  logFile.print(ax); logFile.print(",");
  logFile.print(ay); logFile.print(",");
  logFile.print(az); logFile.print(",");
  logFile.print(gx); logFile.print(",");
  logFile.print(gy); logFile.print(",");
  logFile.print(gz); logFile.print(",");
  logFile.print(pitch); logFile.print(",");
  logFile.print(roll); logFile.print(",");
  logFile.print(temperature); logFile.print(",");
  logFile.println(pressure);   

  // Flush every 250ms
  if (millis() - lastFlush > 250) {
    logFile.flush();
    lastFlush = millis();
  }
}
