#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <L298N.h> // L298N es para un motor
#include <PID_v1.h>
  
// Conexión del sensor IMU BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Configuración del controlador de motor L298N
const int in1 = 5;
const int in2 = 6;
const int enA = 9;

//Conexión al L298N
L298N myMotor(enA, in1, in2);

// Parámetros del controlador PID
double Kp = 60.0;
double Ki = 40.0;
double Kd = 10.0;

// Variables del controlador PID
double input, output, setpoint;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

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
  input = event.orientation.z;
  Serial.print("Input: ");
  Serial.println(input);

  // Ejecutar el controlador PID
  pid.Compute();
  Serial.print("Output: ");
  Serial.println(output);
  controlMotor(output);
}

void controlMotor(double speed) {
  double maxSpeed = 255.0 * (40.0 / 60.0); // (40 mm/s) / (60 s/min) = 0.83
  speed = min(speed, maxSpeed);

  if (speed > 0) {
    myMotor.forward();
  } else {
    myMotor.backward();
  }

  Serial.print("Speed: ");
  Serial.println(speed);
  myMotor.setSpeed(speed);
  myMotor.run();
}
