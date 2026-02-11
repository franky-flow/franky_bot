/*********************************************************************
 *  ROSArduinoMega - Versión Simplificada para Arduino Mega 2560
 *  
 *  Desarrollo incremental paso a paso para control de robot diferencial
 *  Basado en ROSArduinoBridge pero simplificado para aprendizaje
 *  
 *  Hardware: Arduino Mega 2560 Rev3 + L298N Motor Driver
 *  
 *  FASE ACTUAL: 2 - Control de Motores L298N
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
      
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* ===== SETUP - Se ejecuta una vez al inicio ===== */
void setup() {
  // Inicializar comunicación serial
  Serial.begin(BAUDRATE);
  
  // Inicializar controlador de motores (FASE 2)
  initMotorController();
  
  // Mensaje de inicio
  Serial.println("ROSArduinoMega FASE 2 - Ready!");
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