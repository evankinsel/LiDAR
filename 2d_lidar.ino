#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr int SERVO_PIN = 18;

constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;

constexpr int MIN_ANGLE_DEG = 20;
constexpr int MAX_ANGLE_DEG = 160;
constexpr int ANGLE_STEP_DEG = 2;
constexpr uint16_t SERVO_SETTLE_MS = 45;
constexpr uint16_t BETWEEN_SWEEPS_MS = 250;
constexpr uint16_t MIN_VALID_MM = 30;
constexpr uint16_t MAX_VALID_MM = 1800;

constexpr float PI_F = 3.14159265359f;
constexpr float DEG_TO_RAD = PI_F / 180.0f;

Adafruit_VL53L0X lox;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo scannerServo;

VL53L0X_RangingMeasurementData_t measure;

int currentAngle = MIN_ANGLE_DEG;
int angleDirection = 1;

void showFatalError(const char *message) {
  Serial.println(message);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("2D LIDAR ERROR");
  display.println(message);
  display.display();
}

void drawReading(bool valid, uint16_t distanceMm, float xMm, float yMm) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("2D LIDAR SCANNER");
  display.print("Angle: ");
  display.print(currentAngle);
  display.println(" deg");

  if (valid) {
    display.print("Range: ");
    display.print(distanceMm);
    display.println(" mm");
    display.print("X: ");
    display.print(xMm, 0);
    display.print("  Y: ");
    display.println(yMm, 0);
  } else {
    display.println("Range: invalid");
    display.print("Status: ");
    display.println(measure.RangeStatus);
  }

  display.display();
}

void advanceScanAngle() {
  currentAngle += angleDirection * ANGLE_STEP_DEG;

  if (currentAngle >= MAX_ANGLE_DEG) {
    currentAngle = MAX_ANGLE_DEG;
    angleDirection = -1;
    delay(BETWEEN_SWEEPS_MS);
  } else if (currentAngle <= MIN_ANGLE_DEG) {
    currentAngle = MIN_ANGLE_DEG;
    angleDirection = 1;
    delay(BETWEEN_SWEEPS_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 initialization failed.");
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Starting 2D LIDAR...");
  display.display();

  if (!lox.begin()) {
    showFatalError("VL53L0X not found");
    while (true) {
      delay(1000);
    }
  }

  scannerServo.setPeriodHertz(50);
  scannerServo.attach(SERVO_PIN, 500, 2400);
  scannerServo.write(currentAngle);
  delay(500);

  Serial.println("format: SCAN,angle_deg,distance_mm,x_mm,y_mm");
}

void loop() {
  scannerServo.write(currentAngle);
  delay(SERVO_SETTLE_MS);

  lox.rangingTest(&measure, false);

  const uint16_t distanceMm = measure.RangeMilliMeter;
  const bool validMeasurement =
      measure.RangeStatus != 4 &&
      distanceMm >= MIN_VALID_MM &&
      distanceMm <= MAX_VALID_MM;

  if (validMeasurement) {
    const float radians = currentAngle * DEG_TO_RAD;
    const float xMm = distanceMm * cosf(radians);
    const float yMm = distanceMm * sinf(radians);

    Serial.print("SCAN,");
    Serial.print(currentAngle);
    Serial.print(',');
    Serial.print(distanceMm);
    Serial.print(',');
    Serial.print(xMm, 2);
    Serial.print(',');
    Serial.println(yMm, 2);

    drawReading(true, distanceMm, xMm, yMm);
  } else {
    Serial.print("INVALID,");
    Serial.print(currentAngle);
    Serial.print(",status,");
    Serial.println(measure.RangeStatus);
    drawReading(false, distanceMm, 0.0f, 0.0f);
  }

  advanceScanAngle();
}
