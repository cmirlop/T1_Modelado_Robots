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
float d1 = 0.03;
float d2 = 0.07;
float d3 = 0.10;

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
  Matriz_DH_CD[2][0] = -M_PI/2;
  Matriz_DH_CD[2][1] = d3;
  Matriz_DH_CD[2][2] = 0.0;
  Matriz_DH_CD[2][3] = 0.0;

}

bool ik_RRR(float x, float y, float z) {
  // 1. Rotación base (theta1)
  float theta12, theta22, theta32;
  theta12 = atan2(y, x);

  // 2. Calcular distancia en el plano XY
  float r = sqrt(x*x + y*y);  
  float s = z - d1;          

  // 3. Ley del coseno para theta3
  float D = (r*r + s*s - d2*d2 - d3*d3) / (2 * d2 * d3);
  if (fabs(D) > 1.0) {
    return false; // No existe solución
  }

  theta32 = atan2(sqrt(1 - D*D), D);  // codo arriba
  // theta3 = atan2(-sqrt(1 - D*D), D); // codo abajo (alternativa)

  // 4. Calcular theta2
  theta22 = atan2(s, r) - atan2(d3*sin(theta3), d2 + d3*cos(theta3));
   theta1 = theta12 * 180.0 / PI;
  theta2 = theta22 * 180.0 / PI;
  theta3 = theta32 * 180.0 / PI;
  Serial.println("aaaa");
  Serial.println(theta1);
  Serial.println(theta2);
  Serial.println(theta3);
  

  return true;
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
    theta[1] += brazoAngle * M_PI / 180.0;
    theta[2] += codoAngle * M_PI / 180.0;

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
        Serial.println(func);
        if (func == true){
          moverServo(SERVO_BASE, theta1);
          moverServo(SERVO_BRAZO, theta2);
          moverServo(SERVO_CODO, theta3);
        }

        
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
  Serial.print(baseAngle);
  Serial.print(",");
  Serial.print(brazoAngle);
  Serial.print(",");
  Serial.print(codoAngle);
  Serial.print("\n");
  

  delay(500); // refresco cada medio segundo
}

// Función para mover un servo
void moverServo(int canal, int angulo) {
  int pulso = angleToPulse(angulo);
  pwm.setPWM(canal, 0, pulso);
}
