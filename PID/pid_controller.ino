// Ganancias PID
double Kp = 0, Ki = 0, Kd = 0;

// Variable posicion deseada
int setPoint = 12;

// Variables de tiempo para control no bloqueante
unsigned long currentTime, previousTime;
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 10;

double elapsedTime;
double error, lastError = 0, cumError = 0, rateError;
double outPut = 0;

// Variable simulada de entrada
float input = 30.0;

void setup() {
  Serial.begin(115200);
  
  previousTime = millis();
  lastLoopTime = millis();
}

void loop() {
  currentTime = millis();
  
  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
    
    outPut = PID_Controller(input);

    Serial.print(setPoint);
    Serial.print(" ");
    Serial.println(input);
    
    lastLoopTime = currentTime;
  }
}

double PID_Controller(float input_value) { 
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  
  if (elapsedTime <= 0) return outPut;

  error = setPoint - input_value;
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;
  outPut = Kp * error + Ki * cumError + Kd * rateError;

  lastError = error;
  previousTime = currentTime;

  return outPut;
}