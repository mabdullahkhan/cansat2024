#include <Servo.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP380.h> // Library for BMP380 altitude sensor

// Define pins for ultrasonic sensor
#define TRIGGER_PIN  D1
#define ECHO_PIN     D2

// Define pins for servos
#define SERVO_1_PIN  D3
#define SERVO_2_PIN  D4
#define SERVO_3_PIN  D5

// Define buzzer pin
#define BUZZER_PIN D8

// Define altitude sensor pin
#define ALTITUDE_SENSOR_SDA D6
#define ALTITUDE_SENSOR_SCL D7

// Define thresholds
#define DISTANCE_THRESHOLD 100 // in cm
#define ALTITUDE_THRESHOLD 100 // in meters

// Create instances
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
Servo servo1;
Servo servo2;
Servo servo3;

// MPU6050 setup
MPU6050 mpu;
int16_t ax, ay, az;

// BMP380 setup
Adafruit_BMP380 bmp;

// Flag to indicate if the cansat has landed
bool landed = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize servos
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  servo3.attach(SERVO_3_PIN);
  
  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  
  // Initialize BMP380
  if (!bmp.begin()) {
    Serial.println("Could not find BMP380 sensor, check wiring!");
    while (1);
  }
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // Read ultrasonic sensor
  long distance = ultrasonic.read();
  
  // Read MPU6050 data
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Check conditions for servo 1
  if (distance > DISTANCE_THRESHOLD && az < 0) {
    // Rotate servo 1
    servo1.write(90); // Assuming 90 is the desired angle
  }
  
  // Check conditions for servo 2
  if (altitudeCheck() > ALTITUDE_THRESHOLD) {
    // Rotate servo 2
    servo2.write(90); // Assuming 90 is the desired angle
  }
  
  // Delay for 50 milliseconds
  delay(50);
  
  // Rotate servo 3
  servo3.write(90); // Assuming 90 is the desired angle
  
  // Check if cansat has landed
  if (!landed && altitudeCheck() == 0) {
    // If cansat has landed, turn on the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    landed = true;
  }
  
  // Read air pressure
  float pressure = bmp.readPressure();
  
  // Print air pressure
  Serial.print("Air Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
}

float altitudeCheck() {
  // Read altitude from BMP380
  float altitude = bmp.readAltitude();
  
  // Convert altitude from meters to centimeters
  altitude *= 100;
  
  // Return the altitude value
  return altitude;
}
