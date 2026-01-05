// Franky_Bot Firmware ROS2 - Compatible con diffdrive_arduino
// Pines L298N en Arduino Mega
#define ENA 5     // PWM velocidad izquierda
#define IN1 9     // Dirección izquierda 1
#define IN2 10    // Dirección izquierda 2
#define ENB 6     // PWM velocidad derecha
#define IN3 11    // Dirección derecha 1
#define IN4 12    // Dirección derecha 2

// Variables para encoders (desactivadas por ahora)
volatile long left_ticks = 0;
volatile long right_ticks = 0;
unsigned long last_encoder_time = 0;
const unsigned long encoder_interval = 100;  // ms (enviar cada 100ms)

void setup() {
  Serial.begin(57600);  // Baud rate requerido por diffdrive_arduino

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
  Serial.println("Franky_Bot ROS2 ready");
}

void loop() {
  // Procesar comandos entrantes
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("m ")) {              // Comando motor: "m L_PWM R_PWM"
      int spaceIndex = command.indexOf(' ', 2);
      if (spaceIndex != -1) {
        int left_pwm = command.substring(2, spaceIndex).toInt();
        int right_pwm = command.substring(spaceIndex + 1).toInt();

        setMotorLeft(left_pwm);
        setMotorRight(right_pwm);

        // Confirmación opcional (útil para debug)
        Serial.print("OK m ");
        Serial.print(left_pwm);
        Serial.print(" ");
        Serial.println(right_pwm);
      }
    }
  }

  // === FUTURO: envío periódico de encoders (descomenta cuando los conectes) ===
  // unsigned long now = millis();
  // if (now - last_encoder_time >= encoder_interval) {
  //   noInterrupts();
  //   long l = left_ticks;
  //   long r = right_ticks;
  //   interrupts();
  //   Serial.print("e ");
  //   Serial.print(l);
  //   Serial.print(" ");
  //   Serial.println(r);
  //   last_encoder_time = now;
  // }
}

void setMotorLeft(int pwm) {  // -255..255
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