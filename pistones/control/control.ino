#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
  
// Conexión del sensor IMU BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Parámetros del controlador PID
double Kp = 60.0;
double Ki = 40.0;
double Kd = 10.0;

// Variables del controlador PID
double input, output, setpoint;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Configuración del controlador de motor L298N
const int in1 = 5;
const int in2 = 6;
const int enA = 9;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!bno.begin()) {
    Serial.print("¡No se encontró el sensor BNO055! Por favor, comprueba tus conexiones.");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Configuración del controlador PID
  setpoint = 0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(10);

  // Configuración del controlador de motor L298N
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
}

void loop() {
  // Leer el ángulo del sensor IMU
  sensors_event_t event;
  bno.getEvent(&event);
  input = event.orientation.x;

  // Ejecutar el controlador PID
  pid.Compute();
  controlMotor(output);
}

void controlMotor(double speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  }

  // Limitar la velocidad del motor a la velocidad máxima del actuador lineal
  double maxSpeed = 255.0 * (40.0 / 60.0); // (40 mm/s) / (60 s/min) = 0.83
  if (speed > maxSpeed) {
    speed = maxSpeed;
  }

  analogWrite(enA, speed);
}
