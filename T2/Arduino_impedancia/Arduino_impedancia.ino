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
const int   ADC_MIN = 100;
const int   ADC_MAX = 960;
const float KG_MIN  = 0.2f;
const float KG_MAX  = 3.0f;

// ---------- Filtros ----------
const uint8_t N_AVG = 6;      // media móvil corta para suavizar ADC
int bufAvg[N_AVG] = {0};
uint8_t idxAvg = 0;
bool bufLleno = false;

// Filtro exponencial sobre fuerza ya escalada a kg
const float ALPHA_KG = 0.15f; // 0..1 (↑ suprime ruido, ↓ responde más rápido) 0.25
float kg_filt = 0.0f;

// ---------- Control de impedancia (sobre SERVO_BRAZO) ----------
const float KG_TARGET      = 0.5f;   // objetivo de fuerza (kg)
const float K_STIFF        = 6.0f;  // rigidez virtual [deg / kg] 12.0
const float B_DAMP         = 1.0f;   // amortiguación [deg / (kg/s)] 4.0
const float KG_DEADBAND    = 0.06f;  // banda muerta alrededor del objetivo 0.05

const int   BRAZO_MIN      = 10;     // límites mecánicos (ajusta a tu montaje)
const int   BRAZO_MAX      = 170;
const float STEP_LIMIT_DEG = 1.5f;   // límite de variación angular por actualización 3.0
const uint16_t UPDATE_MS   = 30;     // periodo de control
const bool  INVERT_BRAZO_DIR = false;// invierte si tu montaje va al revés

unsigned long tPrevCtrl = 0;
float kg_prev = 0.0f; // para derivada

// ---------------- Utilidades ----------------
int angleToPulse(int ang) {
  int pulsoMin = 150;  // 0°
  int pulsoMax = 600;  // 180°
  return map(ang, 0, 180, pulsoMin, pulsoMax);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float rawToKg(int raw) {
  if (raw <= ADC_MIN) return KG_MIN;
  if (raw >= ADC_MAX) return KG_MAX;
  float kg = fmap((float)raw, (float)ADC_MIN, (float)ADC_MAX, KG_MIN, KG_MAX);
  if (kg < KG_MIN) kg = KG_MIN;
  if (kg > KG_MAX) kg = KG_MAX;
  return kg;
}

void moverServo(int canal, int angulo) {
  int pulso = angleToPulse(angulo);
  pwm.setPWM(canal, 0, pulso);
}

// ----------- Bucle de control de impedancia -----------
void controlImpedanciaBrazo(float kg_meas) {
  unsigned long now = millis();
  if (now - tPrevCtrl < UPDATE_MS) return;

  float dt = (now - tPrevCtrl) / 1000.0f; // [s]
  tPrevCtrl = now;

  // Filtro exponencial sobre la fuerza (en kg)
  kg_filt = ALPHA_KG * kg_meas + (1.0f - ALPHA_KG) * kg_filt;

  // Error respecto a 1 kg
  float e = (KG_TARGET - kg_filt);

  // Banda muerta para evitar temblores alrededor del objetivo
  if (fabs(e) < KG_DEADBAND) e = 0.0f;

  // Derivada (dF/dt) estimada y suavizada ligeramente
  float dkg = (kg_filt - kg_prev) / max(dt, 1e-3f); // [kg/s]
  // (opcional) peina algo de ruido en la derivada:
  const float ALPHA_D = 0.3f;
  static float dkg_filt = 0.0f;
  dkg_filt = ALPHA_D * dkg + (1.0f - ALPHA_D) * dkg_filt;

  kg_prev = kg_filt;

  // Ley de control de impedancia: Δθ = K*e - B*dF/dt
  float dtheta = K_STIFF * e - B_DAMP * dkg_filt;

  if (INVERT_BRAZO_DIR) dtheta = -dtheta;

  // Limita paso y aplica
  if (dtheta >  STEP_LIMIT_DEG) dtheta =  -STEP_LIMIT_DEG;
  if (dtheta < -STEP_LIMIT_DEG) dtheta = STEP_LIMIT_DEG;

  // Actualiza ángulo y envía al servo
  float nuevo = (float)brazoAngle + dtheta;
  if (nuevo < BRAZO_MIN) nuevo = BRAZO_MIN;
  if (nuevo > BRAZO_MAX) nuevo = BRAZO_MAX;

  // Sólo manda pulso si cambia en al menos 0.5° para evitar tráfico/ruido
  if (fabs(nuevo - brazoAngle) >= 0.5f) {
    brazoAngle = (int)roundf(nuevo);
    moverServo(SERVO_BRAZO, brazoAngle);
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  moverServo(SERVO_BASE, baseAngle);
  moverServo(SERVO_BRAZO, brazoAngle);
  moverServo(SERVO_CODO, codoAngle);

  // Inicializa filtros
  int raw = analogRead(pinFSR);
  for (uint8_t i = 0; i < N_AVG; i++) bufAvg[i] = raw;
  bufLleno = true;

  kg_filt = rawToKg(raw); // arranque estable
  kg_prev = kg_filt;
  tPrevCtrl = millis();
}

void loop() {
  // Media móvil corta del ADC
  int raw = analogRead(pinFSR);
  bufAvg[idxAvg++] = raw;
  if (idxAvg >= N_AVG) { idxAvg = 0; bufLleno = true; }

  long suma = 0;
  uint8_t n = bufLleno ? N_AVG : idxAvg;
  for (uint8_t i = 0; i < n; i++) suma += bufAvg[i];
  int rawFiltrado = (int)(suma / n);

  // Escala a kg (0->0,2 ; 960->3,0)
  float kg = rawToKg(rawFiltrado);

  // Control de impedancia
  controlImpedanciaBrazo(kg);

  // Telemetría
  Serial.print("raw="); Serial.print(raw);
  Serial.print("  raw_filt="); Serial.print(rawFiltrado);
  Serial.print("  kg_filt="); Serial.print(kg_filt, 3);
  Serial.print("  brazoAngle="); Serial.println(brazoAngle);

  delay(3);
}
