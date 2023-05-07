#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <L298N.h> // L298N 
#include <PID_v1.h>

// Conexión del sensor IMU BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Conexión del sensor IMU FXOS8700 + FXAS21002
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

// Parámetros del controlador PID
double Kp = 60.0;
double Ki = 40.0;
double Kd = 10.0;

// Variables del controlador PID
double inputIzq, outputIzq, setpointIzq;
PID pidIzq(&inputIzq, &outputIzq, &setpointIzq, Kp, Ki, Kd, DIRECT);

double inputDer, outputDer, setpointDer;
PID pidDer(&inputDer, &outputDer, &setpointDer, Kp, Ki, Kd, DIRECT);

// Configuración del controlador de motor L298N para ambos motores
const int in1 = 5;
const int in2 = 6;
const int enA = 9;

const int in3 = 7;
const int in4 = 8;
const int enB = 10;

//Conexión al L298N
L298N motorIzq(enA, in1, in2);
L298N motorDer(enB, in3, in4);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializar el sensor BNO055
  if (!bno.begin()) {
    Serial.print("¡No se encontró el sensor BNO055! Por favor, comprueba tus conexiones.");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Inicializar el sensor FXOS8700 + FXAS21002
  if (!fxos.begin() || !fxas.begin()) {
    Serial.print("¡No se encontraron los sensores FXOS8700/FXAS21002! Por favor, comprueba tus conexiones.");
    while (1);
  }

  // Configuración del controlador PID
  setpointIzq = 0;
  pidIzq.SetMode(AUTOMATIC);
  pidIzq.SetOutputLimits(-255, 255);
  pidIzq.SetSampleTime(10);

  setpointDer = 0;
  pidDer.SetMode(AUTOMATIC);
  pidDer.SetOutputLimits(-255, 255);
  pidDer.SetSampleTime(10);

  // Configuración del controlador de motor L298N para ambos motores
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
}

void loop() {
  // Leer el ángulo del sensor IMU BNO055
  sensors_event_t event;
  bno.getEvent(&event);
  input = event.orientation.x;

  // Aquí puedes leer y procesar los datos del sensor FXOS8700 + FXAS21002, si es necesario
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t gyro_event;
  fxos.getEvent(&accel_event, &mag_event);
  fxas.getEvent(&gyro_event);

  // Procesa los datos del sensor FXOS8700 + FXAS21002 como sea necesario
  // Por ejemplo, puedes calcular una fusión de los datos de ambos sensores IMU y actualizar la variable 'input' con el resultado

  // Ejecutar el controlador PID
  pidIzq.Compute();
  pidDer.Compute();
  
  controlMotor(0, output); // Motor Izquierdo
  controlMotor(1, output); // Motor Derecho
}

void controlMotor(int motor, double speed) {
  if (speed < 0) {
    if (motor == 0) {
      motorIzq.backward();
    } else {
      motorDer.backward();
    }
    speed = -speed;
  } else {
    if (motor == 0) {
      motorIzq.forward();
    } else {
      motorDer.forward();
    }
  }

  double maxSpeed = 255.0 * (40.0 / 60.0); // (40 mm/s) / (60 s/min) = 0.83
  speed = min(speed, maxSpeed);

  if (motor == 0) {
    motorIzq.setSpeed(speed);
    motorIzq.run();
  } else {
    motorDer.setSpeed(speed);
    motorDer.run();
  }
}
