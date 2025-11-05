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

// Guardaremos la posición inicial de brazo como "home"
int BRAZO_HOME = 90;

// ---------- Escalado a kg (DIRECTO) ----------
// 100 puntos -> ~0,2 kg   |  960 puntos -> 3,0 kg
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
const float ALPHA_KG = 0.20f; // 0..1 (↓ responde más lento, ↑ menos suavizado)
float kg_filt = 0.0f;

// ---------- Control (solo RETROCEDE > 0,5 kg) ----------
const float KG_TARGET        = 0.5f;   // umbral de fuerza (kg)
const float KG_DEADBAND      = 0.06f;  // banda muerta alrededor del objetivo

// Impedancia al RETROCEDER (cuando F > 0,5 kg):
const float K_STIFF_RET      = 4.0f;   // [deg/kg]
const float B_DAMP_RET       = 1.0f;   // [deg/(kg/s)]

// Vuelta suave a HOME cuando F ≤ 0,5 kg:
const float KP_HOME          = 0.8f;   // ↑ un poco para que sí avance a HOME
const float STEP_HOME_MAX    = 1.0f;   // límite de paso por ciclo a HOME [deg]
const float HOME_DEADBAND_DEG= 1.0f;   // banda muerta de posición en HOME

// Límites y timing
const int   BRAZO_MIN        = 10;      
const int   BRAZO_MAX        = 170;
const float STEP_LIMIT_DEG   = 1.5f;   // límite general de paso por ciclo
const uint16_t UPDATE_MS     = 40;     // periodo de control
const bool  INVERT_BRAZO_DIR = true;   // invierte sentido de “retroceder” si tu montaje lo requiere

unsigned long tPrevCtrl = 0;
unsigned long tLastSampleMs = 0;
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

void sendTelemetry(unsigned long timestampMs,
                   int raw,
                   int rawFiltrado,
                   float kgEscalado,
                   float kgFiltrado,
                   float kgReferencia,
                   int brazoPosDeg) {
  Serial.print("DATA,");
  Serial.print(timestampMs);
  Serial.print(',');
  Serial.print(raw);
  Serial.print(',');
  Serial.print(rawFiltrado);
  Serial.print(',');
  Serial.print(kgEscalado, 4);
  Serial.print(',');
  Serial.print(kgFiltrado, 4);
  Serial.print(',');
  Serial.print(kgReferencia, 4);
  Serial.print(',');
  Serial.println(brazoPosDeg);
}

// ----------- Bucle de control -----------
bool controlBrazoSoloRetroceso(float kg_meas) {
  unsigned long now = millis();
  if ((now - tPrevCtrl) < UPDATE_MS) return false;

  float dt = (now - tPrevCtrl) / 1000.0f; // [s]
  tPrevCtrl = now;

  // Filtrado exponencial de fuerza
  kg_filt = ALPHA_KG * kg_meas + (1.0f - ALPHA_KG) * kg_filt;

  // Derivada de fuerza (suavizada)
  float dkg = (kg_filt - kg_prev) / (dt > 1e-3f ? dt : 1e-3f); // [kg/s]
  const float ALPHA_D = 0.3f;
  static float dkg_filt = 0.0f;
  dkg_filt = ALPHA_D * dkg + (1.0f - ALPHA_D) * dkg_filt;
  kg_prev = kg_filt;

  float dtheta = 0.0f;

  // ¿Se supera el umbral?
  if (kg_filt > (KG_TARGET + KG_DEADBAND)) {
    // Error de fuerza solo cuando supera el umbral
    float eF = kg_filt - KG_TARGET; // >0

    // Solo amortiguamos cuando la fuerza va en aumento
    float dkg_pos = (dkg_filt > 0.0f) ? dkg_filt : 0.0f;

    // Impedancia de RETROCESO
    float dtheta_mag = K_STIFF_RET * eF + B_DAMP_RET * dkg_pos; // magnitud positiva
    if (dtheta_mag > STEP_LIMIT_DEG) dtheta_mag = STEP_LIMIT_DEG;

    // Signo de retroceder
    float retractSign = INVERT_BRAZO_DIR ? +1.0f : -1.0f;
    dtheta = retractSign * dtheta_mag;

  } else {
    // Mantener / volver a HOME
    float posErr = (float)BRAZO_HOME - (float)brazoAngle; // >0 si hay que ir hacia delante

    // Si ya estamos cerca de HOME, no hagas nada (evita temblores)
    if (fabs(posErr) < HOME_DEADBAND_DEG) {
      dtheta = 0.0f;
    } else {
      float step = KP_HOME * posErr;

      // Limita el paso de vuelta a home (¡corrección de signos!)
      if (step >  STEP_HOME_MAX) step =  STEP_HOME_MAX;
      if (step < -STEP_HOME_MAX) step = -STEP_HOME_MAX;

      // No avanzar por delante de HOME (solo regresar hacia él)
      if ((posErr <= 0.0f) && (step > 0.0f)) step = 0.0f;

      dtheta = step;
    }
  }

  // Clamp general de paso (¡corrección de signos!)
  if (dtheta >  STEP_LIMIT_DEG) dtheta =  STEP_LIMIT_DEG;
  if (dtheta < -STEP_LIMIT_DEG) dtheta = -STEP_LIMIT_DEG;

  // Aplica
  float nuevo = (float)brazoAngle + dtheta;
  if (nuevo < BRAZO_MIN) nuevo = BRAZO_MIN;
  if (nuevo > BRAZO_MAX) nuevo = BRAZO_MAX;

  if (fabs(nuevo - brazoAngle) >= 0.5f) {
    brazoAngle = (int)roundf(nuevo);
    moverServo(SERVO_BRAZO, brazoAngle);
  }

  tLastSampleMs = now;
  return true;
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("# Force control telemetry v1");
  Serial.println("# DATA,<ms>,<raw>,<raw_avg>,<kg>,<kg_filt>,<kg_ref>,<brazo_deg>");

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  moverServo(SERVO_BASE, baseAngle);
  moverServo(SERVO_BRAZO, brazoAngle);
  moverServo(SERVO_CODO, codoAngle);

  // Fijamos HOME en la posición inicial del brazo
  BRAZO_HOME = brazoAngle;

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

  // Escala a kg
  float kg = rawToKg(rawFiltrado);

  // Control “solo retrocede > 0,5 kg” y envío de telemetría estructurada
  if (controlBrazoSoloRetroceso(kg)) {
    sendTelemetry(tLastSampleMs,
                  raw,
                  rawFiltrado,
                  kg,
                  kg_filt,
                  KG_TARGET,
                  brazoAngle);
  }

  delay(3);
}
