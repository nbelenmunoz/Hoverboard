#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Crea una instancia del sensor BNO055
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  Serial.println("Calibración del BNO055");

  // Inicia el sensor BNO055
  if (!bno.begin()) {
    Serial.print("¡No se encontró el sensor BNO055! Por favor verifica tus conexiones.");
    while (1);
  }

  // Establece la velocidad de actualización del BNO055
  bno.setExtCrystalUse(true);
}

void loop() {
  // Lee la orientación del BNO055
  imu::Quaternion quat = bno.getQuat();

  // Convierte la orientación en ángulos de Euler
  imu::Vector<3> euler = quat.toEuler();

  // Imprime el ángulo de inclinación en el eje X
  Serial.print("Ángulo de inclinación (eje X): ");
  Serial.print(euler.x(), 2);
  Serial.println(" grados");

  // Imprime el estado de calibración
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("Estado de calibración: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  // Espera 1 segundo antes de la siguiente lectura
  delay(1000);
}
