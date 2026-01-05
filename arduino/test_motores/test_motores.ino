// Pines para motor izquierdo (canal A del L298N)
const int ENA = 9;   // PWM velocidad izquierda
const int IN1 = 7;
const int IN2 = 8;

// Pines para motor derecho (canal B del L298N)
const int ENB = 10;  // PWM velocidad derecha
const int IN3 = 11;
const int IN4 = 12;

void setup() {
  // Configurar todos los pines como salida
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Parar motores al inicio
  stopMotors();
  delay(1000);
}

void loop() {
  // Avanzar 3 segundos
  moveForward(150);  // Velocidad media (0-255)
  delay(3000);

  // Parar 1 segundo
  stopMotors();
  delay(1000);

  // Retroceder 3 segundos
  moveBackward(150);
  delay(3000);

  // Parar 1 segundo
  stopMotors();
  delay(1000);

  // Girar a la izquierda (izq atrás, der adelante) 2 segundos
  turnLeft(150);
  delay(2000);

  // Parar 1 segundo
  stopMotors();
  delay(1000);

  // Girar a la derecha 2 segundos
  turnRight(150);
  delay(2000);

  // Parar 2 segundos antes de repetir
  stopMotors();
  delay(2000);
}

// Funciones auxiliares
void moveForward(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Izquierda atrás

  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Derecha adelante
}

void turnRight(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   // Izquierda adelante

  analogWrite(ENB, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Derecha atrás
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}