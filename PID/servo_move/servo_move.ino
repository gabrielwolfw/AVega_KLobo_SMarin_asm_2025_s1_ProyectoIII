#include <Servo.h>

#define SERVO_PIN 9

Servo myservo;

void setup() {
  myservo.attach(SERVO_PIN);

  // Mover a 40 grados
  myservo.write(90);
  delay(1000); // Esperar 1 segundo

  // Mover a 140 grados
  myservo.write(120);
  delay(1000); // Esperar 1 segundo
}

void loop() {
  // No hacer nada, el programa termina aquí
}