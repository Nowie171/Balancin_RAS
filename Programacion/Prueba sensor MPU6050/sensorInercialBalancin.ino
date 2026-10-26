#include <Wire.h>

/*

Módulo: Lectura del sensor IMU (MPU6050)
  ==========================================================

  Este código corresponde a la primera etapa del lazo de
  control (ver diapositiva "Visión General: Lazo Cerrado de
  Control"):

      Sensor IMU -> Arduino -> Driver de Motores -> Motores/Ruedas
         (este archivo cubre esta parte)

  Qué hace:
    1. Se comunica con el MPU6050 por I2C.
    2. Lee el acelerómetro y el giroscopio (registros crudos).
    3. Calcula el ángulo de inclinación (pitch) combinando
       ambas lecturas con un FILTRO COMPLEMENTARIO, que es
       rápido, estable y no necesita librerías externas.
    4. Expone una función getPitchAngle() que el resto del
       sistema (el bloque de control PID) puede usar como
       entrada para corregir el equilibrio del robot.

  Conexión del MPU6050 (I2C estándar en un Arduino UNO/Nano):
    VCC  -> 5V (o 3.3V según el módulo)
    GND  -> GND
    SCL  -> A5
    SDA  -> A4

  No requiere librerías adicionales: solo Wire.h (incluida
  en el IDE de Arduino).
  ==========================================================
*/



// ---------- Dirección I2C y registros del MPU6050 ----------
const int MPU_ADDR = 0x68;        // Dirección I2C por defecto (AD0 a GND)

const byte REG_PWR_MGMT_1   = 0x6B;
const byte REG_ACCEL_XOUT_H = 0x3B;
const byte REG_GYRO_XOUT_H  = 0x43;

// ---------- Factores de escala (configuración por defecto) ----------
// Acelerómetro: rango ±2g  -> sensibilidad 16384 LSB/g
// Giroscopio:   rango ±250°/s -> sensibilidad 131 LSB/(°/s)
const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;

// ---------- Variables de estado del filtro ----------
float pitchAngle = 0.0;     // Ángulo estimado (lo que usará el PID)
unsigned long lastTime = 0; // Para calcular dt entre lecturas

// Peso del filtro complementario: cuánto confiamos en el giroscopio
// frente al acelerómetro. 0.98 es un punto de partida típico;
// se puede afinar durante las pruebas (Fase 4 de la metodología).
const float ALPHA = 0.98;

// Offsets de calibración del giroscopio (se calculan en setup())
float gyroOffsetX = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Despertar el MPU6050 (por defecto arranca en modo "sleep")
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);

  calibrateGyro();

  // Ángulo inicial a partir del acelerómetro, para no partir de 0°
  // si el robot ya está inclinado al encender.
  pitchAngle = readAccelAngle();
  lastTime = micros();
}

void loop() {
  float pitch = getPitchAngle();

  // Salida de depuración: se puede ver en el Monitor Serie o en
  // el Plotter Serie mientras se inclina el robot a mano.
  Serial.print("Pitch (deg): ");
  Serial.println(pitch, 2);

  delay(10); // ~100 Hz de muestreo, suficiente para el lazo PID
}

/*
  Calibración simple del giroscopio: el robot debe permanecer
  quieto y nivelado durante este paso. Promedia varias lecturas
  en reposo para estimar el offset (deriva) del sensor en el
  eje que usamos para el pitch.
*/
void calibrateGyro() {
  const int muestras = 200;
  long suma = 0;

  for (int i = 0; i < muestras; i++) {
    int16_t gx, gy, gz;
    readRawGyro(gx, gy, gz);
    suma += gy;           // eje usado para el pitch (ver readGyroRate)
    delay(3);
  }

  gyroOffsetX = suma / (float)muestras;
}

/*
  Devuelve el ángulo de inclinación (pitch) actualizado, en grados.
  Esta es la función que el bloque de control (PID) debe llamar
  en cada ciclo para obtener el error de inclinación.
*/
float getPitchAngle() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0; // segundos
  lastTime = now;

  float accelAngle = readAccelAngle();
  float gyroRate = readGyroRate();

  // Filtro complementario:
  //  - Integra la velocidad angular del giroscopio (rápido, pero
  //    acumula deriva con el tiempo).
  //  - Lo corrige con el ángulo "absoluto" del acelerómetro (lento
  //    y ruidoso, pero no se desvía a largo plazo).
  pitchAngle = ALPHA * (pitchAngle + gyroRate * dt) + (1 - ALPHA) * accelAngle;

  return pitchAngle;
}

/*
  Calcula el ángulo de inclinación a partir del acelerómetro,
  usando los ejes Y y Z (ajustar según la orientación física
  del sensor sobre el chasis).
*/
float readAccelAngle() {
  int16_t ax, ay, az;
  readRawAccel(ax, ay, az);

  float accelY = ay / ACCEL_SENS;
  float accelZ = az / ACCEL_SENS;

  // atan2 evita divisiones por cero y da el ángulo en todo el rango
  return atan2(accelY, accelZ) * 180.0 / PI;
}

/*
  Devuelve la velocidad angular (°/s) ya corregida con el offset
  de calibración, lista para integrarse en el filtro.
*/
float readGyroRate() {
  int16_t gx, gy, gz;
  readRawGyro(gx, gy, gz);

  return (gy - gyroOffsetX) / GYRO_SENS;
}

// ---------- Lecturas crudas por I2C ----------

void readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}
