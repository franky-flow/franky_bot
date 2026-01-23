// Franky_Bot - Versión de Recuperación
#define ENA 5     
#define IN1 9
#define IN2 10
#define ENB 6     
#define IN3 11
#define IN4 12

#define LEFT_ENCODER_A  2   
#define LEFT_ENCODER_B  3
#define RIGHT_ENCODER_A 18  
#define RIGHT_ENCODER_B 19

volatile long left_ticks = 0;
volatile long right_ticks = 0;

void setup() {
  // Volvemos a 57600 que es lo que tienes en tu XACRO actual
  Serial.begin(57600); 
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightISR, RISING);

  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    // Comando 'e': Lectura de encoders (Rápida)
    if (c == 'e') {
      Serial.print(left_ticks);
      Serial.print(" ");
      Serial.println(right_ticks);
    }
    // Comando 'm': Movimiento
    else if (c == 'm') {
      int left_pwm = Serial.parseInt();
      int right_pwm = Serial.parseInt();
      
      setMotorLeft(left_pwm);
      setMotorRight(right_pwm);
    }
  }
}

void leftISR() { (digitalRead(LEFT_ENCODER_B) == LOW) ? left_ticks++ : left_ticks--; }
void rightISR() { (digitalRead(RIGHT_ENCODER_B) == LOW) ? right_ticks++ : right_ticks--; }

void setMotorLeft(int pwm) {
  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setMotorRight(int pwm) {
  if (pwm > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwm);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}