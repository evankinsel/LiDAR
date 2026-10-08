#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <math.h>

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint8_t BNO055_ADDRESS = 0x28;
constexpr uint8_t SAMPLES_PER_POINT = 5;
constexpr uint16_t MIN_VALID_MM = 30;
constexpr uint16_t MAX_VALID_MM = 1800;
constexpr uint16_t SAMPLE_INTERVAL_MS = 100;

Adafruit_VL53L0X lox;
Adafruit_BNO055 bno(55, BNO055_ADDRESS, &Wire);

VL53L0X_RangingMeasurementData_t measure;

bool readDistance(float &distanceMm) {
  uint32_t totalMm = 0;
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < SAMPLES_PER_POINT; i++) {
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4 &&
        measure.RangeMilliMeter >= MIN_VALID_MM &&
        measure.RangeMilliMeter <= MAX_VALID_MM) {
      totalMm += measure.RangeMilliMeter;
      validCount++;
    }

    delay(10);
  }

  if (validCount == 0) {
    return false;
  }

  distanceMm = static_cast<float>(totalMm) / validCount;
  return true;
}

void printPoint() {
  float distanceMm = 0.0f;
  const imu::Quaternion q = bno.getQuat();

  if (!readDistance(distanceMm)) {
    Serial.print("INVALID,");
    Serial.println(millis());
    return;
  }

  const float qw = q.w();
  const float qx = q.x();
  const float qy = q.y();
  const float qz = q.z();

  const float directionX = 1.0f - 2.0f * (qy * qy + qz * qz);
  const float directionY = 2.0f * (qx * qy + qw * qz);
  const float directionZ = 2.0f * (qx * qz - qw * qy);

  const float xMm = distanceMm * directionX;
  const float yMm = distanceMm * directionY;
  const float zMm = distanceMm * directionZ;

  uint8_t systemCalibration = 0;
  uint8_t gyroCalibration = 0;
  uint8_t accelCalibration = 0;
  uint8_t magnetometerCalibration = 0;

  bno.getCalibration(
    &systemCalibration,
    &gyroCalibration,
    &accelCalibration,
    &magnetometerCalibration
  );

  Serial.print("POINT,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(distanceMm, 2);
  Serial.print(',');
  Serial.print(qw, 5);
  Serial.print(',');
  Serial.print(qx, 5);
  Serial.print(',');
  Serial.print(qy, 5);
  Serial.print(',');
  Serial.print(qz, 5);
  Serial.print(',');
  Serial.print(xMm, 2);
  Serial.print(',');
  Serial.print(yMm, 2);
  Serial.print(',');
  Serial.print(zMm, 2);
  Serial.print(',');
  Serial.print(systemCalibration);
  Serial.print(',');
  Serial.print(gyroCalibration);
  Serial.print(',');
  Serial.print(accelCalibration);
  Serial.print(',');
  Serial.println(magnetometerCalibration);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(500);

  if (!lox.begin()) {
    Serial.println("ERROR,VL53L0X_not_found");

    while (true) {
      delay(1000);
    }
  }

  if (!bno.begin()) {
    Serial.println("ERROR,BNO055_not_found");

    while (true) {
      delay(1000);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("3D_LIDAR_READY");
  Serial.println("POINT,time_ms,distance_mm,qw,qx,qy,qz,x_mm,y_mm,z_mm,sys,gyro,accel,mag");
}

void loop() {
  printPoint();
  delay(SAMPLE_INTERVAL_MS);
}
