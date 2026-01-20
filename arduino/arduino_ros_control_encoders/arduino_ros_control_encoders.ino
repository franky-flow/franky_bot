// Franky_Bot Firmware ROS2 con Encoders 
// Pines L298N
#define ENA 5     // PWM velocidad izquierda
#define IN1 9     // Dirección izquierda
#define IN2 10  // Dirección izquierda
#define ENB 6     // PWM velocidad derecha
#define IN3 11    // Dirección derecha
#define IN4 12    // Dirección derecha

// Pines encoders Hall
#define LEFT_ENCODER_A  2   // Izquierda Input A
#define LEFT_ENCODER_B  3   // Izquierda Input B
#define RIGHT_ENCODER_A 18  // Derecha Input A
#define RIGHT_ENCODER_B 19  // Derecha Input B

volatile long left_ticks = 0;
volatile long right_ticks = 0;

void setup() {
  Serial.begin(57600);
  while (!Serial) { ; }  // Espera serial

  // decalaración pines
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightISR, RISING);

  stopMotors();
  Serial.println("OK");  // Respuesta inicial para ROS2
}

void loop() {
  // Protocolo: Esperar comandos de ROS2 y RESPONDER inmediatamente
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\r');  // ROS2 envía \r
    cmd.trim();
    
    // Comando "e" - Enviar encoders
    if (cmd == "e") {
      noInterrupts();
      long l = left_ticks;
      long r = right_ticks;
      interrupts();
      
      Serial.print(l);
      Serial.print(" ");
      Serial.println(r);  // println añade \n automáticamente
    }
    // Comando "m L R" - Mover motores
    else if (cmd.startsWith("m ")) {
      int spaceIdx = cmd.indexOf(' ', 2);
      if (spaceIdx != -1) {
        int left_pwm = cmd.substring(2, spaceIdx).toInt();
        int right_pwm = cmd.substring(spaceIdx + 1).toInt();
        setMotorLeft(left_pwm);
        setMotorRight(right_pwm);
      }
      Serial.println("OK");  // Confirmar recepción
    }
    // Comando "u" - PID values (aunque no se usan)
    else if (cmd.startsWith("u ")) {
      Serial.println("OK");
    }
    // Cualquier otro comando
    else {
      Serial.println("OK");
    }
  }
}

void leftISR() {
  if (digitalRead(LEFT_ENCODER_B) == LOW) left_ticks--;
  else left_ticks++;
}

void rightISR() {
  if (digitalRead(RIGHT_ENCODER_B) == LOW) right_ticks--;
  else right_ticks++;
}

void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(ENA, pwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (pwm < 0) {
    analogWrite(ENA, -pwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    analogWrite(ENA, 0);
  }
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(ENB, pwm);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (pwm < 0) {
    analogWrite(ENB, -pwm);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}