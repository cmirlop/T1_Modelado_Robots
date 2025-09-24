#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Canales para cada servo en el PCA9685
#define SERVO_BASE 0
#define SERVO_BRAZO 1
#define SERVO_CODO 2

// Ángulos actuales
int baseAngle = 90;
int brazoAngle = 90;
int codoAngle = 90;

// Paso de incremento por pulsación
const int step = 5;

// Modo de operación
int modo = 0; // 0 = directa, 1 = inversa

// Mapear ángulo a pulso (ajustar según tu servo)
int angleToPulse(int ang) {
  int pulsoMin = 150;  // 0°
  int pulsoMax = 600;  // 180°
  return map(ang, 0, 180, pulsoMin, pulsoMax);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); // Frecuencia típica para servos

  // Inicializar servos en posición neutra
  moverServo(SERVO_BASE, baseAngle);
  moverServo(SERVO_BRAZO, brazoAngle);
  moverServo(SERVO_CODO, codoAngle);
}

void loop() {
  // Leer comandos desde el navegador
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "0") {
      modo = 0;
    } 
    else if (cmd == "1") {
      modo = 1;
    } 
    else if (cmd.startsWith("SET,") && modo == 1) {
      // Parsear los 3 valores recibidos
      cmd.remove(0, 4); // eliminar "SET,"
      int coma1 = cmd.indexOf(',');
      int coma2 = cmd.lastIndexOf(',');

      if (coma1 > 0 && coma2 > coma1) {
        int b = cmd.substring(0, coma1).toInt();
        int r = cmd.substring(coma1 + 1, coma2).toInt();
        int c = cmd.substring(coma2 + 1).toInt();

        //Activar funcion - Cinematica Inversa

        /*
        baseAngle = constrain(b, 0, 180);
        brazoAngle = constrain(r, 0, 180);
        codoAngle = constrain(c, 0, 180);

        moverServo(SERVO_BASE, baseAngle);
        moverServo(SERVO_BRAZO, brazoAngle);
        moverServo(SERVO_CODO, codoAngle);
        */
      }
    }
    else if (cmd == "Base+" && modo == 0) {
      baseAngle = constrain(baseAngle + step, 0, 180);
      moverServo(SERVO_BASE, baseAngle);
    } 
    else if (cmd == "Base-" && modo == 0) {
      baseAngle = constrain(baseAngle - step, 0, 180);
      moverServo(SERVO_BASE, baseAngle);
    } 
    else if (cmd == "Brazo+" && modo == 0) {
      brazoAngle = constrain(brazoAngle + step, 0, 180);
      moverServo(SERVO_BRAZO, brazoAngle);
    } 
    else if (cmd == "Brazo-" && modo == 0) {
      brazoAngle = constrain(brazoAngle - step, 0, 180);
      moverServo(SERVO_BRAZO, brazoAngle);
    } 
    else if (cmd == "Codo+" && modo == 0) {
      codoAngle = constrain(codoAngle + step, 0, 180);
      moverServo(SERVO_CODO, codoAngle);
    } 
    else if (cmd == "Codo-" && modo == 0) {
      codoAngle = constrain(codoAngle - step, 0, 180);
      moverServo(SERVO_CODO, codoAngle);
    }
  }

  // Enviar estado actual al navegador
  Serial.print(baseAngle);
  Serial.print(",");
  Serial.print(brazoAngle);
  Serial.print(",");
  Serial.println(codoAngle);

  delay(500); // refresco cada medio segundo
}

// Función para mover un servo
void moverServo(int canal, int angulo) {
  int pulso = angleToPulse(angulo);
  pwm.setPWM(canal, 0, pulso);
}
