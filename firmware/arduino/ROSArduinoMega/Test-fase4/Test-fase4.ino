/*********************************************************************
 *  ROSArduinoMega - TEST FASE 4
 *  
 *  Control Completo: Serial + Motores + Encoders + PID
 *  
 *  Hardware: Arduino Mega 2560 Rev3 + L298N + Encoders
 *  
 *  Este archivo contiene la implementación completa de todas las fases
 *  para probar el sistema completo con control PID.
 *********************************************************************/

#include <Arduino.h>

/* ===== CONFIGURACIÓN GENERAL ===== */
#define BAUDRATE 115200  // Velocidad de comunicación serial (aumentada para reducir delays)
#define MAX_PWM  255     // Valor máximo PWM
#define MIN_PWM  45      // PWM mínimo para vencer fricción estática (zona muerta)

/* ===== CONFIGURACIÓN PID (FASE 4) ===== */
#define PID_RATE           30     // Hz - Frecuencia del bucle PID
#define PID_INTERVAL       (1000 / PID_RATE)  // ms - Intervalo entre cálculos PID
#define AUTO_STOP_INTERVAL 2000   // ms - Tiempo sin comandos para auto-stop

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
#define MOTOR_SPEEDS   'm'  // Velocidad motores (arg1=izq, arg2=der) en ticks/frame
#define MOTOR_RAW_PWM  'o'  // Control directo PWM sin PID (arg1=izq, arg2=der)

// Comandos de encoders (FASE 3)
#define READ_ENCODERS  'e'  // Leer contadores de encoders
#define RESET_ENCODERS 'r'  // Resetear contadores a cero

// Comandos de PID (FASE 4)
#define UPDATE_PID     'u'  // Actualizar parámetros PID (formato: Kp:Kd:Ki:Ko)

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

/* ===== ESTRUCTURA Y VARIABLES PID (FASE 4) ===== */

/* Estructura de datos PID para un motor
 * Contiene toda la información necesaria para el control PID de velocidad
 */
typedef struct {
  double TargetTicksPerFrame;  // Velocidad objetivo en ticks por frame
  long Encoder;                // Valor actual del encoder
  long PrevEnc;                // Valor anterior del encoder
  int PrevInput;               // Entrada anterior (para derivada mejorada)
  int ITerm;                   // Término integral acumulado
  long output;                 // Salida PWM calculada
} SetPointInfo;

// Estructuras PID para cada motor
SetPointInfo leftPID, rightPID;

// Parámetros PID (ajustables con comando 'u')
// Optimizados para movimientos suaves sin oscilaciones
int Kp = 12;  // Ganancia proporcional (muy reducida para movimientos suaves)
int Kd = 20;  // Ganancia derivativa (alta para máxima amortiguación)
int Ki = 0;   // Ganancia integral (desactivada - puede causar problemas en giros)
int Ko = 50;  // Factor de escalado

// Variables de control
unsigned char moving = 0;              // ¿Está el robot en movimiento?
unsigned long nextPID = PID_INTERVAL;  // Tiempo del próximo cálculo PID
long lastMotorCommand = AUTO_STOP_INTERVAL;  // Tiempo del último comando de motor

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

/* ===== FUNCIONES PID (FASE 4) ===== */

/* Resetear variables PID a estado inicial
 * Previene picos de arranque al iniciar movimiento
 * Ver: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 */
void resetPID() {
  // Resetear PID izquierdo
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(LEFT);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;
  
  // Resetear PID derecho
  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

/* Calcular PID para un motor
 * Implementa control PID mejorado evitando derivative kick
 * Ver: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
 */
void doPID(SetPointInfo * p) {
  long Perror;   // Error proporcional
  long output;   // Salida calculada
  int input;     // Entrada actual (ticks en este frame)
  
  // Calcular entrada actual (cuántos ticks se movió)
  input = p->Encoder - p->PrevEnc;
  
  // Calcular error (diferencia entre objetivo y actual)
  Perror = p->TargetTicksPerFrame - input;
  
  // Calcular salida PID
  // P: Error proporcional
  // D: Cambio en entrada (evita derivative kick)
  // I: Término integral acumulado
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  
  // Guardar encoder anterior para próximo cálculo
  p->PrevEnc = p->Encoder;
  
  // Añadir salida anterior (acumulativo)
  output += p->output;
  
  // Limitar salida y acumular integral
  if (output >= MAX_PWM) {
    output = MAX_PWM;
  } else if (output <= -MAX_PWM) {
    output = -MAX_PWM;
  } else {
    // Solo acumular integral si no estamos saturados
    p->ITerm += Ki * Perror;
  }
  
  // Guardar valores para próxima iteración
  p->output = output;
  p->PrevInput = input;
}

/* Actualizar control PID de ambos motores
 * Se llama periódicamente desde el loop principal
 */
void updatePID() {
  // Leer encoders actuales
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);
  
  // Si no estamos en movimiento, resetear PID
  if (!moving) {
    // Resetear solo una vez para evitar spikes
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) {
      resetPID();
    }
    return;
  }
  
  // Calcular PID para cada motor
  doPID(&rightPID);
  doPID(&leftPID);
  
  // Aplicar salidas PID a los motores
  setMotorSpeeds(leftPID.output, rightPID.output);
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
  
  // ZONA MUERTA: Aplicar PWM mínimo si speed > 0
  // Los motores DC necesitan un PWM mínimo para vencer fricción estática
  if (speed > 0 && speed < MIN_PWM) {
    speed = MIN_PWM;
  }
  
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
      // Control de velocidad con PID (FASE 4)
      // arg1 = ticks/frame izquierdo, arg2 = ticks/frame derecho
      // Resetear timer de auto-stop
      lastMotorCommand = millis();
      
      // Si ambos son cero, detener y resetear PID
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      } else {
        moving = 1;
      }
      
      // Establecer objetivos de velocidad para PID
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;
    
    case MOTOR_RAW_PWM:
      // Control directo PWM sin PID (FASE 2 compatible)
      // Útil para pruebas o control manual
      lastMotorCommand = millis();
      resetPID();
      moving = 0;  // Desactivar PID temporalmente
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
      // Resetear contadores de encoders y PID
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    
    case UPDATE_PID:
      // Actualizar parámetros PID
      // Formato: u Kp:Kd:Ki:Ko
      // Ejemplo: u 20:12:0:50
      {
        int i = 0;
        char *p = argv1;
        char *str;
        int pid_args[4];
        
        // Parsear string separado por ":"
        while ((str = strtok_r(p, ":", &p)) != NULL && i < 4) {
          pid_args[i] = atoi(str);
          i++;
        }
        
        // Actualizar parámetros
        Kp = pid_args[0];
        Kd = pid_args[1];
        Ki = pid_args[2];
        Ko = pid_args[3];
        Serial.println("OK");
      }
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
  
  // Inicializar PID (FASE 4)
  resetPID();
  
  // Mensaje de inicio
  Serial.println("ROSArduinoMega TEST FASE 4 - Ready!");
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
  
  // ===== BUCLE PID (FASE 4) =====
  // Ejecutar PID a intervalos regulares (30 Hz)
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // ===== AUTO-STOP POR SEGURIDAD (FASE 4) =====
  // Detener motores si no hay comandos recientes
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}