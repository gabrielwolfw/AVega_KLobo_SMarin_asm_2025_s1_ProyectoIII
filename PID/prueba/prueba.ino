#include <NewPing.h>

#include <Servo.h>

// Pines
#define SERVO_PIN 9       // Pin del servo
#define TRIG_PIN 8        // Pin TRIG del HC-SR04
#define ECHO_PIN 7        // Pin ECHO del HC-SR04
#define POT_PIN A0        // Pin del potenciómetro
#define MAX_DISTANCE 50

// Límites del servo (invertidos porque el servo está al revés)
#define SERVO_ALTO 75     // Posición más alta (50 grados)
#define SERVO_BAJO 118    // Posición más baja (135 grados)

// Distancia mínima válida (cm)
#define DISTANCIA_MIN 1.0
//#define VALOR_SEGURO 50.0
#define MAX_CAMBIO 5

// Ganancias PID (ajusta estos valores según tu sistema)
double Kp = 2.5;          // Ganancia proporcional
double Ki = 0.12;         // Ganancia integral
double Kd = 0.7;          // Ganancia derivativa

// Variables de posición y setPoint
int pos = 95;             // Posición inicial del servo
int setPoint;        // Distancia objetivo inicial (cm)
float ultima_lectura_valida = setPoint;

// Variables PID
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError = 0, cumError = 0, rateError;
double outPut;

// Objeto para el servo
Servo myservo;

void setup() {
  Serial.begin(115200);
  
  // Configurar pines del sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Inicializar servo
  myservo.attach(SERVO_PIN);
  myservo.write(95);
  
  previousTime = millis();
  
  Serial.println("Sistema PID iniciado");
  Serial.println("Ajusta el setPoint con el potenciómetro");
}

void loop() {
  // Leer el setPoint del potenciómetro (mapear de 10 a 40 cm)
  int potValue = analogRead(POT_PIN);
  setPoint = map(potValue, 0, 1023, 10, 40);
  
  // Obtener la distancia del sensor ultrasónico
  float distancia = medirDistanciaControlada();
  
  // Calcular la salida del PID
  outPut = calcularPID(distancia);
  
  // Mapear la salida del PID a los ángulos del servo (invertido)
  pos = map(constrain(outPut, -58, 58), -58, 58, SERVO_ALTO, SERVO_BAJO);
  
  // Mover el servo
  myservo.write(pos);
  
  // Imprimir valores para graficar
  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(distancia);
  delay(10);
  
}

float medirDistancia() {
  // Limpiar el pin TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  
  // Enviar pulso de 10 microsegundos
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Leer el tiempo de eco con timeout
  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Si hay timeout, devolver la última lectura válida
  if (duracion == 0) {
    return ultima_lectura_valida;
  }
  
  // Calcular la distancia en cm
  float distancia = duracion * 0.0343 / 2;
  
  // Validar rango
  if (distancia < DISTANCIA_MIN || distancia > MAX_DISTANCE) {
    return ultima_lectura_valida;
  }
  
  return distancia;
}




float medirDistanciaControlada() {
  // Obtener nueva lectura
  float nueva_lectura = medirDistancia();
  
  // Controlar cambios bruscos
  if (abs(nueva_lectura - ultima_lectura_valida) > MAX_CAMBIO) {
    // Si el cambio es demasiado grande, acercarse gradualmente
    if (nueva_lectura > ultima_lectura_valida) {
      nueva_lectura = ultima_lectura_valida + MAX_CAMBIO;
    } else {
      nueva_lectura = ultima_lectura_valida - MAX_CAMBIO;
    }
  }
  
  // Si la nueva lectura es 50 (valor tope) y la anterior era mucho menor,
  // probablemente sea una lectura errónea, mantener el valor anterior
  if (nueva_lectura >= MAX_DISTANCE - 1 && ultima_lectura_valida < MAX_DISTANCE/2) {
    nueva_lectura = ultima_lectura_valida;
  }
  
  // Actualizar la última lectura válida
  ultima_lectura_valida = nueva_lectura;
  
  return nueva_lectura;
}


double calcularPID(float input) {
  currentTime = millis();
  elapsedTime = (currentTime - previousTime); // Convertir a segundos
  
  if (elapsedTime <= 0) return outPut;
  
  // Cálculo del error
  error = setPoint - input;
  
  // Componente integral con límite para evitar wind-up
  cumError += error * elapsedTime;
  cumError = constrain(cumError, -130, 130);
  
  // Componente derivativa
  rateError = (error - lastError) / elapsedTime;
  
  // Cálculo de la salida PID
  outPut = Kp * error + Ki * cumError + Kd * rateError;
  
  // Actualizar variables para la próxima iteración
  lastError = error;
  previousTime = currentTime;
  
  return outPut;
}

int medirDistancia1() {
  // Limpiar el pin TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Enviar pulso de 10 microsegundos
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Leer el tiempo de eco
  long duracion = pulseIn(ECHO_PIN, HIGH);
  
  // Calcular y devolver la distancia en cm
  return duracion * 0.034 / 2;
}