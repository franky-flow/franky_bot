/*********************************************************************
 *  ROSArduinoMega - TEST FASE 3
 *  
 *  Comunicación Serial + Control de Motores + Lectura de Encoders
 *  
 *  Hardware: Arduino Mega 2560 Rev3 + L298N + Encoders
 *  
 *  Este archivo contiene la implementación de FASE 1 + FASE 2 + FASE 3
 *  para probar la comunicación serial, control de motores y encoders.
 *********************************************************************/

#include <Arduino.h>

/* ===== CONFIGURACIÓN GENERAL ===== */
#define BAUDRATE 57600  // Velocidad de comunicación serial
#define MAX_PWM  255    // Valor máximo PWM

/* ===== COMANDOS ===== */
// Comandos básicos (FASE 1)
#define GET_BAUDRATE   'b'  // Consultar velocidad baudios
#define PING           'p'  // Verificar conectividad
#define ANALOG_READ    'a'  // Leer pin analógico
#define DIGITAL_READ   'd'  // Leer pin digital
#define ANALOG_WRITE   'x'  // Escribir PWM (analogWrite)
#define DIGITAL_WRITE  'w'  // Escribir digital
#define PIN_MODE       'c'  // Configurar modo de pin

// Comandos de motores (FASE 2)
#define MOTOR_SPEEDS   'm'  // Velocidad motores (arg1=izq, arg2=der) en PWM
#define MOTOR_RAW_PWM  'o'  // Control directo PWM (arg1=izq, arg2=der)

// Comandos de encoders (FASE 3)
#define READ_ENCODERS  'e'  // Leer contadores de encoders
#define RESET_ENCODERS 'r'  // Resetear contadores a cero

/* ===== CONFIGURACIÓN PINES L298N (FASE 2) ===== */
// Motor Izquierdo
#define LEFT_MOTOR_FORWARD   8   // IN1 del L298N
#define LEFT_MOTOR_BACKWARD  9   // IN2 del L298N
#define LEFT_MOTOR_ENABLE    10  // ENA del L298N (PWM)

// Motor Derecho
#define RIGHT_MOTOR_FORWARD  11  // IN3 del L298N
#define RIGHT_MOTOR_BACKWARD 12  // IN4 del L298N
#define RIGHT_MOTOR_ENABLE   13  // ENB del L298N (PWM)

// Identificadores de motores
#define LEFT   0
#define RIGHT  1

/* ===== CONFIGURACIÓN PINES ENCODERS (FASE 3) ===== */
// Encoder Izquierdo - Usa interrupciones externas de Mega
#define LEFT_ENCODER_A   2   // Pin de interrupción (INT4)
#define LEFT_ENCODER_B   4   // Pin digital

// Encoder Derecho - Usa interrupciones externas de Mega
#define RIGHT_ENCODER_A  3   // Pin de interrupción (INT5)
#define RIGHT_ENCODER_B  5   // Pin digital

/* ===== VARIABLES GLOBALES ===== */
// Variables para parsear comandos seriales
int arg = 0;           // Índice del argumento actual (0=comando, 1=arg1, 2=arg2)
int index = 0;         // Índice dentro del string del argumento
char chr;              // Carácter leído
char cmd;              // Comando actual (letra única)
char argv1[16];        // Primer argumento como string
char argv2[16];        // Segundo argumento como string
long arg1;             // Primer argumento convertido a entero
long arg2;             // Segundo argumento convertido a entero

// Variables para encoders (FASE 3)
volatile long leftEncoderCount = 0;   // Contador del encoder izquierdo (volatile para ISR)
volatile long rightEncoderCount = 0;  // Contador del encoder derecho (volatile para ISR)

/* ===== FUNCIONES DE ENCODERS (FASE 3) - VERSIÓN SIMPLE Y ROBUSTA ===== */

/* ISR (Interrupt Service Routine) para encoder izquierdo
 * Se ejecuta en RISING (flanco de subida) del canal A
 * Lee el canal B inmediatamente para determinar dirección
 */
void leftEncoderISR() {
  // MÉTODO SIMPLE: Solo leer B cuando A hace RISING
  // B=HIGH -> adelante, B=LOW -> atrás
  int b_state = digitalRead(LEFT_ENCODER_B);
  
  if (b_state == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

/* ISR (Interrupt Service Routine) para encoder derecho
 * Se ejecuta en RISING (flanco de subida) del canal A
 * Lee el canal B inmediatamente para determinar dirección
 */
void rightEncoderISR() {
  // MÉTODO SIMPLE: Solo leer B cuando A hace RISING
  // B=HIGH -> adelante, B=LOW -> atrás
  int b_state = digitalRead(RIGHT_ENCODER_B);
  
  if (b_state == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

/* Inicializar encoders y configurar interrupciones */
void initEncoders() {
  // Configurar pines de encoders como INPUT_PULLUP explícitamente
  // Esto asegura que los pines tengan resistencias pull-up activas
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // Adjuntar interrupciones solo en flancos de SUBIDA (RISING)
  // RISING es más confiable para encoders que CHANGE
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
}

/* Leer valor del encoder especificado */
long readEncoder(int motor) {
  if (motor == LEFT) {
    return leftEncoderCount;
  } else {
    return rightEncoderCount;
  }
}

/* Resetear un encoder específico a cero */
void resetEncoder(int motor) {
  if (motor == LEFT) {
    leftEncoderCount = 0;
  } else {
    rightEncoderCount = 0;
  }
}

/* Resetear ambos encoders a cero */
void resetEncoders() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}

/* ===== FUNCIONES DE MOTORES (FASE 2) ===== */

/* Inicializar pines del driver de motores L298N */
void initMotorController() {
  // Configurar pines como salida
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  
  // Asegurarse de que los motores empiezan detenidos
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  
  // Habilitar los drivers (HIGH para habilitar)
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
}

/* Establecer velocidad de un motor individual
 * motor: LEFT o RIGHT
 * speed: -255 a +255 (negativo = reversa, positivo = adelante)
 */
void setMotorSpeed(int motor, int speed) {
  // Variable para manejar dirección
  unsigned char reverse = 0;
  
  // Determinar dirección según el signo
  if (speed < 0) {
    speed = -speed;  // Hacer positivo
    reverse = 1;     // Marcar como reversa
  }
  
  // Limitar velocidad al máximo
  if (speed > MAX_PWM)
    speed = MAX_PWM;
  
  // Aplicar velocidad y dirección según el motor
  if (motor == LEFT) {
    if (reverse == 0) {
      // Adelante: FORWARD=PWM, BACKWARD=0
      analogWrite(LEFT_MOTOR_FORWARD, speed);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    } else {
      // Reversa: FORWARD=0, BACKWARD=PWM
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      analogWrite(LEFT_MOTOR_BACKWARD, speed);
    }
  } 
  else if (motor == RIGHT) {
    if (reverse == 0) {
      // Adelante: FORWARD=PWM, BACKWARD=0
      analogWrite(RIGHT_MOTOR_FORWARD, speed);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    } else {
      // Reversa: FORWARD=0, BACKWARD=PWM
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, speed);
    }
  }
}

/* Establecer velocidad de ambos motores simultáneamente */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

/* ===== FUNCIONES AUXILIARES ===== */

/* Reiniciar variables de comando */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Ejecutar comando recibido */
int runCommand() {
  // Convertir argumentos string a enteros
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  // Procesar comando según la letra recibida
  switch(cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
      
    case PING:
      // Responder con el valor recibido para verificar comunicación
      Serial.println(arg1);
      break;
      
    case ANALOG_READ:
      // Leer pin analógico (A0-A15 en Mega)
      Serial.println(analogRead(arg1));
      break;
      
    case DIGITAL_READ:
      // Leer pin digital
      Serial.println(digitalRead(arg1));
      break;
      
    case ANALOG_WRITE:
      // Escribir PWM (0-255)
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
      
    case DIGITAL_WRITE:
      // Escribir digital (HIGH/LOW)
      if (arg2 == 0) 
        digitalWrite(arg1, LOW);
      else if (arg2 == 1) 
        digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
      
    case PIN_MODE:
      // Configurar modo del pin
      if (arg2 == 0) 
        pinMode(arg1, INPUT);
      else if (arg2 == 1) 
        pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    
    case MOTOR_SPEEDS:
      // Control de velocidad de motores (arg1=izquierdo, arg2=derecho)
      // Rango: -255 a +255 (negativo=reversa, positivo=adelante, 0=stop)
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;
    
    case MOTOR_RAW_PWM:
      // Control directo PWM (igual que MOTOR_SPEEDS en esta fase)
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;
    
    case READ_ENCODERS:
      // Leer valores de ambos encoders
      // Formato de respuesta: "valor_izq valor_der"
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    
    case RESET_ENCODERS:
      // Resetear contadores de encoders a cero
      resetEncoders();
      Serial.println("OK");
      break;
      
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* ===== SETUP - Se ejecuta una vez al inicio ===== */
void setup() {
  // Inicializar comunicación serial
  Serial.begin(BAUDRATE);
  
  // Inicializar encoders (FASE 3)
  initEncoders();
  
  // Inicializar controlador de motores (FASE 2)
  initMotorController();
  
  // Mensaje de inicio
  Serial.println("ROSArduinoMega TEST FASE 3 - Ready!");
}

/* ===== LOOP - Se ejecuta continuamente ===== */
void loop() {
  // Procesar datos disponibles en el puerto serial
  while (Serial.available() > 0) {
    
    // Leer siguiente carácter
    chr = Serial.read();

    // El carácter CR (13 = Enter) termina un comando
    if (chr == 13) {
      // Terminar strings con NULL
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      
      // Ejecutar el comando
      runCommand();
      
      // Resetear para el próximo comando
      resetCommand();
    }
    // El espacio delimita partes del comando
    else if (chr == ' ') {
      // Avanzar al siguiente argumento
      if (arg == 0) {
        arg = 1;
      }
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    // Caracteres normales
    else {
      if (arg == 0) {
        // El primer carácter es el comando
        cmd = chr;
      }
      else if (arg == 1) {
        // Añadir carácter al primer argumento
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        // Añadir carácter al segundo argumento
        argv2[index] = chr;
        index++;
      }
    }
  }
}