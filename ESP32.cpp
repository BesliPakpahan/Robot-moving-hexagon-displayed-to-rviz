#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WiFiUdp.h>

MPU6050 mpu;

const float wheel_base = 0.17;
int16_t gx, gy, gz;
float gx_offset = 0, gz_offset = 0;

float yaw = 0.0;
#define GYRO_THRESHOLD 0.5
unsigned long prevTime;
static float y;
float targetYaw = 0;

// PID - Parameter dari kode kedua
float kp = 60, ki = 0.0, kd = 10; //bagus
// float kp = 40, ki = 0.0, kd = 30;
float integral = 0, prevError = 0;

// Motor
const int CW[2] = {4, 19};
const int CCW[2] = {2, 18};
int enA = 5;
int enB = 15;

// WiFi & UDP
const char* ssid = "wonus";
const char* password = "12345678";
const char* udpAddress = "192.168.241.207";
const int udpPort = 4210;
const int localPort = 4211;

WiFiUDP udp;
char incomingPacket[255];
bool mode_enam = false;
bool isCompleted = false;
unsigned long lastUdpSendTime = 0;
const unsigned long udpInterval = 100;

void calibrateGyro() {
  Serial.println("Kalibrasi gyro...");
  const int numSamples = 500;
  float sum_gx = 0, sum_gz = 0;
  for (int i = 0; i < numSamples; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum_gx += gx;
    sum_gz += gz;
    delay(2);
  }
  gx_offset = sum_gx / numSamples;
  gz_offset = sum_gz / numSamples;
  Serial.println("Kalibrasi selesai!");
}

void setMotor(int cwPin, int ccwPin, float pwmVal) {
  pwmVal = constrain(pwmVal, -100, 100);
  if (pwmVal > 0) {
    analogWrite(cwPin, pwmVal);
    analogWrite(ccwPin, 0);
  } else if (pwmVal < 0) {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, -pwmVal);
  } else {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, 0);
  }
}

void stopMotors() {
  setMotor(CW[0], CCW[0], 0);
  setMotor(CW[1], CCW[1], 0);
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
}

float pidT(float target, float curr) {
  float error = curr - target;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  
  // Hanya menggunakan komponen Proportional
  return kp * error;
}

void differential_K(float Vx, float omega) {
  if (isCompleted) {
    stopMotors();
    return;
  }

  // y += 0.01;
  y += 0.009;
  if (y >= 7) {
    y = 7;
    isCompleted = true;
    stopMotors();
    return;
  }

  float control = pidT(omega, yaw);
  float V_left = Vx + (control * wheel_base / 2);
  float V_right = Vx - (control * wheel_base / 2);

  setMotor(CW[0], CCW[0], V_left);
  setMotor(CW[1], CCW[1], V_right);
}

void sendUdpData() {
  unsigned long curr = millis();
  if (curr - lastUdpSendTime < udpInterval) return;
  lastUdpSendTime = curr;

  String msg = "ANGLE:" + String(yaw) + ",MOVING:" + (isCompleted ? "0" : "1");
  udp.beginPacket(udpAddress, udpPort);
  udp.print(msg);
  udp.endPacket();
  Serial.println("UDP Sent: " + msg);
}

void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 gagal");
    while (1);
  }

  calibrateGyro();

  for (int i = 0; i < 2; i++) {
    pinMode(CW[i], OUTPUT);
    pinMode(CCW[i], OUTPUT);
  }
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

  prevTime = millis();
  mode_enam = true;
  isCompleted = false;
  y = 0;
}

void loop() {
  unsigned long curr = millis();
  float dt = (curr - prevTime) / 1000.0;
  prevTime = curr;

  mpu.getRotation(&gx, &gy, &gz);
  float gz_dps = (gz - gz_offset) / 131.0;
  if (abs(gz_dps) > GYRO_THRESHOLD) {
    yaw += gz_dps * dt;
  }
  if (yaw < 0) yaw += 360;
  else if (yaw >= 360) yaw -= 360;

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
      if (strcmp(incomingPacket, "START") == 0) {
        mode_enam = true;
        isCompleted = false;
        y = 0;
      } else if (strcmp(incomingPacket, "STOP") == 0) {
        mode_enam = false;
        stopMotors();
      } else if (strcmp(incomingPacket, "RESET") == 0) {
        ESP.restart();
      }
    }
  }

  if (mode_enam && !isCompleted) {
    if (y >= 0 && y < 1) differential_K(80, 0);
    else if (y >= 1 && y < 2) differential_K(80, 60);
    else if (y >= 2 && y < 3) differential_K(80, 120);
    else if (y >= 3 && y < 4) differential_K(80, 180);
    else if (y >= 4 && y < 5) differential_K(80, 240);
    else if (y >= 5 && y < 6) differential_K(80, 300);
    else if (y >= 6 && y < 7) differential_K(80, 360);
  }

  sendUdpData();
  delay(10);
}
