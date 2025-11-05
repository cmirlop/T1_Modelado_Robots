#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ---------- FSR ----------
const int pinFSR = A0;

// ---------- PCA9685 ----------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // I2C por defecto 0x40

// Canales para cada servo en el PCA9685
#define SERVO_BASE  0
#define SERVO_BRAZO 1
#define SERVO_CODO  2

// Ángulos actuales
int baseAngle = 0;
int brazoAngle = 90;
int codoAngle = 90;

// ---------- Escalado a kg (DIRECTO) ----------
// 0 puntos -> 0,2 kg
// 960 puntos -> 3,0 kg
const int   ADC_MIN = 0;
const int   ADC_MAX = 960;
const float KG_MIN  = 0.2f;
const float KG_MAX  = 3.0f;

// Filtro simple (media móvil) para estabilizar lectura
const uint8_t N_AVG = 8;
int bufAvg[N_AVG] = {0};
uint8_t idxAvg = 0;
bool bufLleno = false;

// ---------- Control de fuerza sobre el SERVO_BRAZO ----------
const float KG_TARGET      = 0.50f;   // umbral central
const float KG_BAND        = 0.05f;   // histéresis ±
const int   BRAZO_MIN      = 10;      // límite mecánico seguro (ajusta)
const int   BRAZO_MAX      = 170;     // límite mecánico seguro (ajusta)
const int   STEP_DEG       = 1;       // velocidad: grados por actualización
const uint16_t UPDATE_MS   = 25;      // periodo de actualización
const bool  INVERT_BRAZO_DIR = false; // true si el sentido te queda al revés

unsigned long tPrev = 0;

// Mapear ángulo a pulso (ajustar según tu servo)
int angleToPulse(int ang) {
  int pulsoMin = 150;  // 0°
  int pulsoMax = 600;  // 180°
  return map(ang, 0, 180, pulsoMin, pulsoMax);
}

// map float (lineal, directo)
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Convierte lectura ADC -> kg (0..960 -> 0,2..3,0 kg)
float rawToKg(int raw) {
  if (raw <= ADC_MIN) return KG_MIN;
  if (raw >= ADC_MAX) return KG_MAX;
  float kg = fmap((float)raw, (float)ADC_MIN, (float)ADC_MAX, KG_MIN, KG_MAX);
  if (kg < KG_MIN) kg = KG_MIN;
  if (kg > KG_MAX) kg = KG_MAX;
  return kg;
}

// Control del brazo según fuerza medida
void controlBrazoPorFuerza(float kg) {
  // Espera al siguiente tick
  unsigned long now = millis();
  if (now - tPrev < UPDATE_MS) return;
  tPrev = now;

  // Decide dirección con histéresis
  int dir = 0; // 0 = mantener, +1 = adelante, -1 = atrás
  if (kg < (KG_TARGET - KG_BAND))       dir = -1; // avanzar
  else if (kg > (KG_TARGET + KG_BAND))  dir = +1; // retroceder;

  // Permite invertir si tu montaje lo requiere
  if (INVERT_BRAZO_DIR) dir = dir;

  // Aplica paso
  if (dir != 0) {
    brazoAngle = constrain(brazoAngle + dir * STEP_DEG, BRAZO_MIN, BRAZO_MAX);
    moverServo(SERVO_BRAZO, brazoAngle);
  }
}

void setup() {
  Serial.begin(9600);

  // Inicializa PCA9685
  pwm.begin();
  pwm.setPWMFreq(60); // Frecuencia típica para servos
  delay(10);

  // Posiciones iniciales seguras
  moverServo(SERVO_BASE, baseAngle);
  moverServo(SERVO_BRAZO, brazoAngle);
  moverServo(SERVO_CODO, codoAngle);

  // Prellena el buffer de media
  int raw = analogRead(pinFSR);
  for (uint8_t i = 0; i < N_AVG; i++) bufAvg[i] = raw;
  bufLleno = true;
}

void loop() {
  // Lectura con media móvil
  int raw = analogRead(pinFSR);   // 0..1023
  bufAvg[idxAvg++] = raw;
  if (idxAvg >= N_AVG) { idxAvg = 0; bufLleno = true; }

  long suma = 0;
  uint8_t n = bufLleno ? N_AVG : idxAvg;
  for (uint8_t i = 0; i < n; i++) suma += bufAvg[i];
  int rawFiltrado = (int)(suma / n);

  // Escala a kg (0->0,2 ; 960->3,0)
  float kg = rawToKg(rawFiltrado);

  // Control del brazo
  controlBrazoPorFuerza(kg);

  // Monitor serie
  Serial.print("raw="); Serial.print(raw);
  Serial.print("  raw_filt="); Serial.print(rawFiltrado);
  Serial.print("  kg="); Serial.print(kg, 3);
  Serial.print("  brazoAngle="); Serial.println(brazoAngle);

  delay(5);
}

// Función para mover un servo
void moverServo(int canal, int angulo) {
  int pulso = angleToPulse(angulo);
  pwm.setPWM(canal, 0, pulso);
}
