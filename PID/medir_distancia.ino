// Pines para el sensor HC-SR04
#define TRIG_PIN 8
#define ECHO_PIN 7

// Parámetros para filtrado
#define MUESTRAS 5             // Número de muestras para promediar
#define DISTANCIA_MIN 2.0      // Distancia mínima válida (cm)
#define DISTANCIA_MAX 400.0    // Distancia máxima válida (cm)
#define TIMEOUT 25000          // Timeout para pulseIn (microsegundos)

// Variables para filtrado
float lecturas[MUESTRAS];      // Array para almacenar lecturas
int indice_lectura = 0;        // Índice actual en el array
float distancia_anterior = 20.0; // Valor inicial seguro

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Inicializar el array de lecturas
  for (int i = 0; i < MUESTRAS; i++) {
    lecturas[i] = distancia_anterior;
  }
  
  Serial.println("Sensor de distancia HC-SR04 con filtrado");
  Serial.println("Mostrando distancia en centímetros");
  delay(1000);
}

void loop() {
  // Medir la distancia con filtrado
  float distancia = medirDistanciaFiltrada();
  
  // Mostrar la distancia en el monitor serial
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");
  
  delay(100); // Actualizar cada 100ms para mejor filtrado
}

float medirDistancia() {
  // Limpiar el pin TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Enviar pulso de 10 microsegundos
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Leer el tiempo de eco con timeout
  long duracion = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
  
  // Si hay timeout, devolver el valor anterior
  if (duracion == 0) {
    return distancia_anterior;
  }
  
  // Calcular la distancia en cm (velocidad del sonido = 343 m/s)
  float distancia = duracion * 0.0343 / 2;
  
  // Validar rango
  if (distancia < DISTANCIA_MIN || distancia > DISTANCIA_MAX) {
    return distancia_anterior;
  }
  
  // Guardar esta lectura como válida para la próxima vez
  distancia_anterior = distancia;
  
  return distancia;
}

float medirDistanciaFiltrada() {
  // Obtener una nueva lectura
  float nueva_lectura = medirDistancia();
  
  // Añadir la nueva lectura al array
  lecturas[indice_lectura] = nueva_lectura;
  
  // Avanzar el índice
  indice_lectura = (indice_lectura + 1) % MUESTRAS;
  
  // Calcular el promedio
  float suma = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    suma += lecturas[i];
  }
  
  // Devolver el promedio
  return suma / MUESTRAS;
}