#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Canales para cada servo en el PCA9685
#define SERVO_BASE 0
#define SERVO_BRAZO 1
#define SERVO_CODO 2


//Matriz DH cinematica directa
float Matriz_DH_CD[3][4]; //[alpha, a, d, theta]
const int N = 3;
const int N1 = 4;

//Matrices de cada angulo
//float A1[4][4],A2[4][4],A3[4][4];

//Matriz resultado
float res_CN[N1][N1];

// Ángulos actuales
int baseAngle = 0;
int brazoAngle = 0;
int codoAngle = 0;

//Distancia entre ejes, modificar a los reales
// de los brazos
float d1 = 0.075;
float d2 = 0.082;
float d3 = 0.050;

// --- Resultados de la cinemática inversa ---
float theta1, theta2, theta3;


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

/*
bool ik_RRR(float x, float y, float z) {
  // Variables locales temporales (en radianes)
  float theta12, theta22, theta32;

  // --- 1. Rotación base ---
  theta12 = atan2(y, x);

  // --- 2. Distancias en el plano XY y vertical ---
  float r = sqrt(x * x + y * y);
  float s = z - d1;

  // --- 3. Ley del coseno para el codo (theta3) ---
  // Forzamos rango [-π/2, +π/2]
  float D = (r * r + s * s - d2 * d2 - d3 * d3) / (2 * d2 * d3);
  if (fabs(D) > 1.0) {
    return false; // No hay solución geométrica
  }

  // Codo arriba
  theta32 = atan2(sqrt(1 - D * D), D)- (PI / 2.0);

  // --- 4. Calcular el hombro (theta2) ---
  theta22 = atan2(s, r) - atan2(d3 * sin(theta32), d2 + d3 * cos(theta32));

  // --- 5. Convertir a grados ---
  float t1 = theta12 * 180.0 / PI;
  float t2 = theta22 * 180.0 / PI;
  float t3 = theta32 * 180.0 / PI;

  // --- 6. Ajustes según tu mecánica ---

  // BASE: derecha=0°, izquierda=180°
  // atan2 da [-180,180]; convertimos a [0,180]
  if (t1 < 0) t1 = 360 + t1;   // ahora está en [0,360)
  if (t1 > 180) t1 = 360 - t1; // reflejamos para coincidir con tu montaje

  // BRAZO: 0° = horizontal, 100° = arriba
  // t2=0° significa brazo apuntando hacia el frente en horizontal
  // Si tu servo 0° es horizontal, lo dejamos así.
  if (t2 < 0) t2 = 0;
  if (t2 > 100) t2 = 100;  // limitamos al máximo físico

  // CODO: 0° = arriba, 90° = medio, 180° = abajo
  // Cinemáticamente, t3=0° = codo extendido
  // Pero tu servo está invertido → lo invertimos:
  //t3 = 180 - t3;
  //if (t3 < 0) t3 = 0;
  //if (t3 > 180) t3 = 180;
  t3 = (t3 + 90.0);
  // --- 7. Asignar resultados globales (ya en grados) ---
  theta1 = t1;
  theta2 = t2;
  theta3 = t3;

  return true;
}
*/

/*
bool ik_RRR(float x, float y, float z) {
  // longitudes (asegúrate están definidas globalmente)
  // float d1 = 0.065, d2 = 0.095, d3 = 0.055;

  // 1) Base
  float theta12 = atan2(y, x); // radianes

  // 2) Geometría
  float r = sqrt(x * x + y * y);
  float s = z - d1;

  // 3) Ley del coseno (D)
  float D = (r * r + s * s - d2 * d2 - d3 * d3) / (2.0f * d2 * d3);

  // Clamp numérico (evita rechazar por errores flotantes)
  if (D > 1.0f) D = 1.0f;
  if (D < -1.0f) D = -1.0f;

  // 4) Ángulo completo del codo (theta3_full) en [0, pi]
  float theta3_full = atan2( sqrt( fmaxf(0.0f, 1.0f - D*D) ), D ); // rad

  // Si deseas centrar en [-pi/2, +pi/2] para mapeo posterior:
  float theta32 = theta3_full - (PI / 2.0f); // rad, estará ~[-pi/2, +pi/2]

  // 5) Hombro: usar theta3_full (NO theta32 directamente)
  float num = d3 * sin(theta3_full);
  float den = d2 + d3 * cos(theta3_full);
  float theta22 = atan2(s, r) - atan2(num, den); // rad

  // 6) Convertir a grados
  float t1 = (theta12 * 180.0f / PI);   // base
  float t2 = theta22 * 180.0f / PI;   // hombro (geométrico)
  float t3 = theta32 * 180.0f / PI;   // codo en [-90, +90]

  // 7) Ajustes mecánicos
  // BASE: atan2 -> [-180,180] -> queremos [0,180] con 0=derecha, 180=izquierda
  if (t1 < 0.0f) t1 = 360.0f + t1;   // -> [0,360)
  if (t1 > 180.0f) t1 = 360.0f - t1; // espejo para tu montaje

  // HOMBRO: limitar a [0,100]
  if (t2 < 0.0f) t2 = 0.0f;
  if (t2 > 180.0f) t2 = 180.0f;

  // CODO: mapear [-90,+90] -> [0,180], y limitar
  t3 = t3 + 90.0f;
  if (t3 < 0.0f) t3 = 0.0f;
  if (t3 > 180.0f) t3 = 180.0f;

  // 8) Guardar/usar
  theta1 = t1;
  theta2 = t2;
  theta3 = t3;

  return true;
}

#include <math.h>

// Función para calcular la cinemática inversa de un 3R
// X, Y, Z: posición del efector final
// sol1 y sol2: structs donde se almacenan las dos soluciones (codo abajo y codo arriba)
bool ik_RRR(float X, float Y, float Z) {
    // Longitudes del brazo
    float a2 = 0.082;
    float a3 = 0.050;
    float d1 = 0.075;


    // Rotación de base
    float theta11 = atan2(Y, X);

    // Coordenadas en el plano del brazo
    float r = sqrt(X*X + Y*Y);
    float z = Z - d1;

    // Coseno de theta3
    float cos_t3 = (X*X+Y*Y+z*z - a2*a2 - a3*a3) / (2.0 * a2 * a3);
    Serial.println("5");
    Serial.println(cos_t3);
    if (fabs(cos_t3) > 1.0) {
        // Posición fuera del alcance
       // return false;
    }

    // Dos soluciones: codo arriba y codo abajo
    float sin_t3 = sqrt(1.0 - cos_t3*cos_t3);
    //float sin_t3_alt = -sin_t3;
    Serial.println("4");
    Serial.println(sin_t3);

    float theta3_1 = atan2(sin_t3,cos_t3);
    //float theta3_2 = atan2(sin_t3_alt, cos_t3);

    float theta2_1 = atan2(z,r) - atan2(a3 * sin_t3,a2 + a3 * cos_t3);
    //float theta2_2 = atan2(z, r) - atan2(a3 * sin_t3_alt, a2 + a3 * cos_t3);

    // Guardar resultados
    float deg1 = theta11 * 180.0 / PI;
    float deg2 = theta2_1 * 180.0 / PI;
    float deg3 = theta3_1 * 180.0 / PI;
    Serial.println("3");
    Serial.println(deg1);
    Serial.println(deg2);
    Serial.println(deg3);
     theta1 = constrain(deg1, 0, 180);
   theta2 = constrain(deg2, 0, 180);
    theta3 = constrain(deg3, 0, 180);


    return true; // cálculo exitoso
}
*/
bool ik_RRR(float X, float Y, float Z) {
    const float a2 = d2;    // 0.082
    const float a3 = d3;    // 0.050
    const float h  = d1;    // 0.075

    // Base
    float th1 = atan2f(Y, X);

    // Plano del brazo
    float r = sqrtf(X*X + Y*Y);
    float z = Z - h;

    // Ley del coseno
    float D = (r*r + z*z - a2*a2 - a3*a3) / (2.0f * a2 * a3);
    // Clamp numérico
    if (D > 1.0f) D = 1.0f;
    if (D < -1.0f) D = -1.0f;

    // Si está realmente fuera (p. ej. por ruido puede dar NaN si no clamped)
    if (isnan(D)) return false;

    // Dos ramas: elegimos una (p.ej. codo abajo). Si quieres la otra, cambia el signo.
    float s3  = sqrtf(fmaxf(0.0f, 1.0f - D*D));
    float th3_geom = atan2f(-s3, D);   // rama "codo abajo" (elige la que te convenga)

    // Hombro (usa th3_geom, NO la centrada)
    float num = a3 * sinf(th3_geom);
    float den = a2 + a3 * cosf(th3_geom);
    float th2_geom = atan2f(z, r) - atan2f(num, den);

    // Geométricos → grados
    const float RAD2DEG = 180.0f / PI;
    float g1 = th1       * RAD2DEG;    // base geom
    float g2 = th2_geom  * RAD2DEG;    // hombro geom
    float g3 = th3_geom  * RAD2DEG;    // codo geom

    // Mapea geométricos a "ángulo de SERVO" (coherente con la FK de arriba):
    // Base: limita a [0,180] porque tu servo sólo puede esto.
    if (g1 < 0.0f) g1 += 360.0f;
    if (g1 > 180.0f) g1 = 360.0f - g1;   // si tu montaje sólo cubre media vuelta

    float servo_base  = g1;
    float servo_brazo = g2;
    float servo_codo  = -g3;             //  <<--- INVERTIR codo para igualar la FK

    // Limita a los rangos del servo
    servo_base  = constrain(servo_base,  0.0f, 180.0f);
    servo_brazo = constrain(servo_brazo, 0.0f, 180.0f);
    servo_codo  = constrain(servo_codo,  0.0f, 180.0f);

    // Guarda resultados
    theta1 = servo_base;
    theta2 = servo_brazo;
    theta3 = servo_codo;

    return true;
}


/*bool ik_RRR(float x, float y, float z) {
  float theta11, theta21, theta31;

  // Cálculo del ángulo de la base
  theta11 = atan2(y, x);

  // Plano r-z
  float r = sqrt(x * x + y * y);
  float z_ = z - d1;

  // Ley del coseno para theta3
  float cos_t3 = (r * r + z_ * z_ - d2 * d2 - d3 * d3) / (2 * d2 * d3);
  cos_t3 = constrain(cos_t3, -1.0, 1.0); // Evita errores numéricos
  float sin_t3 = sqrt(1 - cos_t3 * cos_t3);
  theta31 = atan2(sin_t3, cos_t3);

  // Theta2
  theta21 = atan2(z_, r) - atan2(d3 * sin_t3, d2 + d3 * cos_t3);

  // Conversión a grados
  float deg1 = theta11 * 180.0 / PI;
  float deg2 = theta21 * 180.0 / PI;
  float deg3 = theta31 * 180.0 / PI;

  // Ajustar al rango del servo (0-180)
  theta1 = constrain(deg1, 0, 180);
  theta2 = constrain(deg2, 0, 180);
  theta3 = constrain(deg3, 0, 180);

  return true;

  
}*/

/*
void inicializar_matriz(){
  //Eje 1 [alpha, a, d, theta]
  Matriz_DH_CD[0][0] = -M_PI/2;
  Matriz_DH_CD[0][1] = 0.0;
  Matriz_DH_CD[0][2] = d1;
  Matriz_DH_CD[0][3] = 0.0;
  //Eje 2 [alpha, a, d, theta]
  Matriz_DH_CD[1][0] = 0.0;
  Matriz_DH_CD[1][1] = d2;
  Matriz_DH_CD[1][2] = 0.0;
  Matriz_DH_CD[1][3] = 0.0;
  //Eje 3 [alpha, a, d, theta]
  Matriz_DH_CD[2][0] = M_PI/2;
  Matriz_DH_CD[2][1] = d3;
  Matriz_DH_CD[2][2] = 0.0;
  Matriz_DH_CD[2][3] = 0.0;

}

*/
void inicializar_matriz(){
  // Eje 1 [alpha, a, d, theta]
  Matriz_DH_CD[0][0] =  M_PI/2; // <-- antes -M_PI/2
  Matriz_DH_CD[0][1] =  0.0;
  Matriz_DH_CD[0][2] =  d1;
  Matriz_DH_CD[0][3] =  0.0;

  // Eje 2
  Matriz_DH_CD[1][0] =  0.0;
  Matriz_DH_CD[1][1] =  d2;     // <-- usa a2 (longitud del 1er eslabón)
  Matriz_DH_CD[1][2] =  0.0;
  Matriz_DH_CD[1][3] =  0.0;

  // Eje 3
  Matriz_DH_CD[2][0] =  0.0;    // <-- antes +M_PI/2
  Matriz_DH_CD[2][1] =  d3;     // <-- usa a3 (longitud del 2º eslabón)
  Matriz_DH_CD[2][2] =  0.0;
  Matriz_DH_CD[2][3] =  0.0;
}



void dh_to_T(float DH[4][4],float alpha,float a,float d,float theta){
  float ca = cos(alpha);
  float sa = sin(alpha);
  float ct = cos(theta);
  float st = sin(theta);

  

DH[0][0] = ct;       DH[0][1] = -st * ca;    DH[0][2] = st * sa;     DH[0][3] = a * ct;
DH[1][0] = st;       DH[1][1] =  ct * ca;    DH[1][2] = -ct * sa;    DH[1][3] = a * st;
DH[2][0] = 0.0;      DH[2][1] = sa;          DH[2][2] = ca;          DH[2][3] = d;
DH[3][0] = 0.0;      DH[3][1] = 0.0;         DH[3][2] = 0.0;         DH[3][3] = 1.0;



}


// Función de multiplicación
void multiplicarMatrices(float A[N1][N1], float B[N1][N1],float C[N1][N1]) {
  for (int i = 0; i < N1; i++) {
    for (int j = 0; j < N1; j++) {
      C[i][j] = 0;
      for (int k = 0; k < N1; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}



void calcular_directa(){
  float alpha[N], a[N], d[N], theta[N];
  float A1[N1][N1],A2[N1][N1],A3[N1][N1];
  float T0[N1][N1],T1[N1][N1],T2[N1][N1];
    //Separamos los parametros de la matriz
    for (int i = 0; i < N; i++) {
        alpha[i] = Matriz_DH_CD[i][0];
        a[i]     = Matriz_DH_CD[i][1];
        d[i]     = Matriz_DH_CD[i][2];
        theta[i] = Matriz_DH_CD[i][3];
    }
    //Convertimos de angulos radianes y se lo agregamos a cada 
    //motor para calcular la posicon del angulo final
    theta[0] += baseAngle * M_PI / 180.0; 
    //theta[1] += -brazoAngle * M_PI / 180.0;
    theta[1] += brazoAngle * M_PI / 180.0;
    theta[2] += -codoAngle * M_PI / 180.0;

    //Calculamos la matriz de cada eje
    dh_to_T(A1,alpha[0], a[0], d[0], theta[0]);
    dh_to_T(A2,alpha[1], a[1], d[1], theta[1]);
    dh_to_T(A3,alpha[2], a[2], d[2], theta[2]);

   float identidad[N1][N1] = {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0}
};

    multiplicarMatrices(identidad,A1,T0);
    multiplicarMatrices(T0,A2,T1);
    multiplicarMatrices(T1,A3,res_CN);

   
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); // Frecuencia típica para servos

  // Inicializar servos en posición neutra
  moverServo(SERVO_BASE, baseAngle);
  moverServo(SERVO_BRAZO, brazoAngle);
  moverServo(SERVO_CODO, codoAngle);

  inicializar_matriz();
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
      Serial.print("Hola\n");
      // Parsear los 3 valores recibidos
      cmd.remove(0, 4); // eliminar "SET,"
      int coma1 = cmd.indexOf(',');
      int coma2 = cmd.lastIndexOf(',');


      if (coma1 > 0 && coma2 > coma1) {
        float b = cmd.substring(0, coma1).toFloat();
        float r = cmd.substring(coma1 + 1, coma2).toFloat();
        float c = cmd.substring(coma2 + 1).toFloat();
        Serial.println(b);
        Serial.println(r);
        Serial.println(c);
        

        //Activar funcion - Cinematica Inversa

        bool func = ik_RRR(b,r,c);
          Serial.print("Angulos para cada motor:");
          Serial.print(",");
          Serial.print(theta1);
          Serial.print(",");
          Serial.print(theta2);
          Serial.print(",");
          Serial.print(theta3);
          Serial.print(",");
          Serial.println(func);
        if (func == true){
          moverServo(SERVO_BASE, theta1);
          moverServo(SERVO_BRAZO, theta2);
          moverServo(SERVO_CODO, theta3);
        }
        baseAngle = theta1;
        brazoAngle = theta2;
        codoAngle = theta3;

        
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

  calcular_directa();
/*
  if (modo == 0){
    
    //Falta enviar el estado actual del eje de coordenadas en base a CN
    Serial.print(res_CN[0][3]); //X
    Serial.print(",");
    Serial.print(res_CN[1][3]); //Y
    Serial.print(",");
    Serial.println(res_CN[2][3]); //Z
  }else{

  }
  */

  // Enviar estado actual al navegador
  Serial.print(baseAngle);
  Serial.print(",");
  Serial.print(brazoAngle);
  Serial.print(",");
  Serial.print(codoAngle);
  Serial.print(",");
  Serial.print(res_CN[0][3]);
  Serial.print(",");
  Serial.print(res_CN[1][3]);
  Serial.print(",");
  Serial.print(res_CN[2][3]);
  Serial.print("\n");
  

  delay(500); // refresco cada medio segundo
}

// Función para mover un servo
void moverServo(int canal, int angulo) {
  int pulso = angleToPulse(angulo);
  pwm.setPWM(canal, 0, pulso);
}
