/*
 * Spray Can Freeze Experiment
 * Full Sensor Integration - Fahrenheit
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_BME280.h>
#include <HX711.h>
#include <max6675.h>
#include <ESP32Servo.h>

// ============== PIN DEFINITIONS ==============
#define I2C_SDA 21
#define I2C_SCL 22
#define TC_CLK   18
#define TC_CS    5
#define TC_MISO  19
#define SD_CS    4
#define HX711_DT  32
#define HX711_SCK 33
#define MIC_PIN 27
#define SERVO_PIN    13
#define SERVO_RELEASE 0
#define SERVO_PRESS   45

// ============== SENSOR OBJECTS ==============
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BME280 bme;
HX711 scale;
MAX6675 thermocouple(TC_CLK, TC_CS, TC_MISO);
Servo triggerServo;

#define MPU_ADDR 0x69

// ============== CONFIGURATION ==============
const float HX711_CALIBRATION = 420.0;
const int SAMPLE_RATE_MS = 50;

// ============== DATA STRUCTURE ==============
struct Reading {
  unsigned long time_ms;
  float mass_g;
  float dm_dt;
  float t_nozzle_f;
  float t_surface_f;
  float t_ir_ambient_f;
  float t_ambient_f;
  float humidity_pct;
  float pressure_hpa;
  float accel_mag;
  float mic_rms;
  bool valve_open;
};

// ============== GLOBALS ==============
bool isLogging = false;
bool valveOpen = false;
float lastMass = 0;
unsigned long lastSampleTime = 0;
unsigned long logStartTime = 0;

// ============== HELPER ==============
float toF(float c) { return c * 9.0 / 5.0 + 32.0; }

float readAccelMagnitude() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  return sqrt((float)ax*ax + ay*ay + az*az) / 16384.0;
}

float readMicRMS() {
  long sum = 0;
  int n = 0;
  unsigned long start = millis();
  while (millis() - start < 10) {
    int raw = analogRead(MIC_PIN);
    int centered = raw - 2048;
    sum += (long)centered * centered;
    n++;
  }
  return sqrt((float)sum / n);
}

Reading readSensors() {
  Reading r;
  r.time_ms = millis() - logStartTime;
  r.valve_open = valveOpen;
  if (scale.is_ready()) {
    r.mass_g = scale.get_units(1);
    r.dm_dt = (lastMass - r.mass_g) / (SAMPLE_RATE_MS / 1000.0);
    lastMass = r.mass_g;
  } else {
    r.mass_g = 0;
    r.dm_dt = 0;
  }
  r.t_nozzle_f = toF(thermocouple.readCelsius());
  r.t_surface_f = toF(mlx.readObjectTempC());
  r.t_ir_ambient_f = toF(mlx.readAmbientTempC());
  r.t_ambient_f = toF(bme.readTemperature());
  r.humidity_pct = bme.readHumidity();
  r.pressure_hpa = bme.readPressure() / 100.0;
  r.accel_mag = readAccelMagnitude();
  r.mic_rms = readMicRMS();
  return r;
}

void printLive(Reading &r) {
  Serial.print(r.time_ms); Serial.print(",");
  Serial.print(r.mass_g, 1); Serial.print(",");
  Serial.print(r.dm_dt, 2); Serial.print(",");
  Serial.print(r.t_nozzle_f, 1); Serial.print(",");
  Serial.print(r.t_surface_f, 1); Serial.print(",");
  Serial.print(r.t_ambient_f, 1); Serial.print(",");
  Serial.print(r.humidity_pct, 0); Serial.print(",");
  Serial.print(r.pressure_hpa, 0); Serial.print(",");
  Serial.print(r.accel_mag, 2); Serial.print(",");
  Serial.print(r.mic_rms, 0); Serial.print(",");
  Serial.println(r.valve_open ? "SPRAY" : "off");
}

void openValve() {
  valveOpen = true;
  triggerServo.write(SERVO_PRESS);
  Serial.println("\n>>> VALVE OPEN <<<\n");
}

void closeValve() {
  valveOpen = false;
  triggerServo.write(SERVO_RELEASE);
  Serial.println("\n>>> valve closed <<<\n");
}

void startLogging() {
  isLogging = true;
  logStartTime = millis();
  lastMass = scale.get_units(5);
  Serial.println("\n>>> LOGGING STARTED <<<");
  Serial.println("time_ms,mass_g,flow,t_noz,t_surf,t_amb,hum,pres,accel,mic,valve");
}

void stopLogging() {
  if (isLogging) {
    Serial.println(">>> LOGGING STOPPED <<<");
    isLogging = false;
  }
}

void printHelp() {
  Serial.println("\n========================================");
  Serial.println("            COMMANDS");
  Serial.println("========================================");
  Serial.println("  start / s     Start logging");
  Serial.println("  stop  / x     Stop logging");
  Serial.println("  press / p     Open valve (spray)");
  Serial.println("  release / r   Close valve");
  Serial.println("  pulse         1-second spray");
  Serial.println("  pulse 500     Custom pulse (ms)");
  Serial.println("  tare / t      Zero the scale");
  Serial.println("  read          Single sensor reading");
  Serial.println("  servo         Test servo movement");
  Serial.println("  setpress 45   Test servo angle");
  Serial.println("  help / h      Show this menu");
  Serial.println("========================================\n");
}

void handleCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd == "start" || cmd == "s") startLogging();
  else if (cmd == "stop" || cmd == "x") stopLogging();
  else if (cmd == "press" || cmd == "p") openValve();
  else if (cmd == "release" || cmd == "r") closeValve();
  else if (cmd == "tare" || cmd == "t") {
    Serial.println("Taring scale...");
    scale.tare();
    lastMass = 0;
    Serial.println("Done - scale zeroed");
  }
  else if (cmd == "read") {
    Reading r = readSensors();
    Serial.println("\n=== SENSOR READING ===");
    Serial.print("Mass:        "); Serial.print(r.mass_g, 2); Serial.println(" g");
    Serial.print("Nozzle temp: "); Serial.print(r.t_nozzle_f, 1); Serial.println(" F");
    Serial.print("Surface temp:"); Serial.print(r.t_surface_f, 1); Serial.println(" F");
    Serial.print("Ambient temp:"); Serial.print(r.t_ambient_f, 1); Serial.println(" F");
    Serial.print("Humidity:    "); Serial.print(r.humidity_pct, 1); Serial.println(" %");
    Serial.print("Pressure:    "); Serial.print(r.pressure_hpa, 1); Serial.println(" hPa");
    Serial.print("Accel:       "); Serial.println(r.accel_mag, 3);
    Serial.print("Mic RMS:     "); Serial.println(r.mic_rms, 0);
    Serial.println("======================\n");
  }
  else if (cmd.startsWith("pulse ")) {
    int ms = cmd.substring(6).toInt();
    if (ms > 0 && ms < 30000) {
      Serial.print("Pulsing for "); Serial.print(ms); Serial.println(" ms");
      openValve(); delay(ms); closeValve();
    }
  }
  else if (cmd == "pulse") {
    Serial.println("Pulsing 1 second...");
    openValve(); delay(1000); closeValve();
  }
  else if (cmd == "servo") {
    Serial.println("Testing servo: press -> release");
    triggerServo.write(SERVO_PRESS); delay(1000);
    triggerServo.write(SERVO_RELEASE);
    Serial.println("Done");
  }
  else if (cmd.startsWith("setpress ")) {
    int angle = cmd.substring(9).toInt();
    if (angle >= 0 && angle <= 180) {
      Serial.print("Testing servo angle: "); Serial.println(angle);
      triggerServo.write(angle);
    }
  }
  else if (cmd == "help" || cmd == "h") printHelp();
  else Serial.println("Unknown command. Type 'help'.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("   SPRAY CAN FREEZE EXPERIMENT");
  Serial.println("========================================\n");
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Initializing sensors...");
  if (mlx.begin()) Serial.println("  [OK] MLX90614");
  else Serial.println("  [FAIL] MLX90614");
  if (bme.begin(0x76) || bme.begin(0x77)) Serial.println("  [OK] BME280");
  else Serial.println("  [FAIL] BME280");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  if (Wire.endTransmission() == 0) Serial.println("  [OK] MPU6050");
  else Serial.println("  [FAIL] MPU6050");
  delay(500);
  if (!isnan(thermocouple.readCelsius())) Serial.println("  [OK] MAX6675");
  else Serial.println("  [FAIL] MAX6675");
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(HX711_CALIBRATION);
  Serial.println("  [OK] HX711");
  Serial.println("  [OK] MAX9814");
  triggerServo.attach(SERVO_PIN);
  triggerServo.write(SERVO_RELEASE);
  Serial.println("  [OK] Servo");
  Serial.println("\nReady! Type 'help' for commands.\n");
}

void loop() {
  handleCommands();
  if (isLogging && (millis() - lastSampleTime >= SAMPLE_RATE_MS)) {
    lastSampleTime = millis();
    Reading r = readSensors();
    printLive(r);
  }
}