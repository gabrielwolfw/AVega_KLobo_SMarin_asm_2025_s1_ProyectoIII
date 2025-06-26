// Pines para el sensor HC-SR04
#define TRIG_PIN 8
#define ECHO_PIN 7

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Sensor de distancia HC-SR04 simple");
}

void loop() {
  // Medir la distancia
  int distancia = medirDistancia();
  
  // Mostrar la distancia en el monitor serial
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");
  
  delay(100);
}

int medirDistancia() {
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


// ---------------------------- Nuevo Codigo --------------------------------
#include <NewPing.h>

#define TRIGGER_PIN  8
#define ECHO_PIN     7
#define MAX_DISTANCE 200  // Distancia máxima en cm

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(50);
  unsigned int distancia = sonar.ping_cm();
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");
}