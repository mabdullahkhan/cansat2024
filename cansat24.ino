#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <DHT.h>

#define DHTPIN 2         // Pin where the DHT11 is connected
#define DHTTYPE DHT11    // DHT 11
#define SEALEVELPRESSURE_HPA (1013.25) // Standard sea level pressure in hPa

DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP280 bmp;
MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!bmp.begin()) {
    Serial.println("Could not find BMP280 sensor!");
    while (1);
  }

  dht.begin();
}

void loop() {
  // Read temperature from BMP280
  float temperature = bmp.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  // Read pressure from BMP280
  float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  // Read altitude from BMP280
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");

  // Read acceleration and gyroscope from MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.println("Accelerometer readings:");
  Serial.print("X: "); Serial.print(ax); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ay); Serial.print("  ");
  Serial.print("Z: "); Serial.println(az);

  Serial.println("Gyroscope readings:");
  Serial.print("X: "); Serial.print(gx); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gy); Serial.print("  ");
  Serial.print("Z: "); Serial.println(gz);

  // Read humidity from DHT11
  float humidity = dht.readHumidity();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  delay(1000); // Adjust delay as needed
}
