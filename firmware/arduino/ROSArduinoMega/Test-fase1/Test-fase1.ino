/*********************************************************************
 *  ROSArduinoMega - TEST FASE 1
 *  
 *  Comunicación Serial Básica SOLAMENTE
 *  
 *  Hardware: Arduino Mega 2560 Rev3
 *  
 *  Este archivo contiene SOLO la implementación de la FASE 1 para
 *  poder probar la comunicación serial de forma aislada.
 *********************************************************************/

#include <Arduino.h>

/* ===== CONFIGURACIÓN GENERAL ===== */
#define BAUDRATE 57600  // Velocidad de comunicación serial

/* ===== COMANDOS (FASE 1 SOLAMENTE) ===== */
#define GET_BAUDRATE   'b'  // Consultar velocidad baudios
#define PING           'p'  // Verificar conectividad
#define ANALOG_READ    'a'  // Leer pin analógico
#define DIGITAL_READ   'd'  // Leer pin digital
#define ANALOG_WRITE   'x'  // Escribir PWM (analogWrite)
#define DIGITAL_WRITE  'w'  // Escribir digital
#define PIN_MODE       'c'  // Configurar modo de pin

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
      
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* ===== SETUP - Se ejecuta una vez al inicio ===== */
void setup() {
  // Inicializar comunicación serial
  Serial.begin(BAUDRATE);
  
  // Mensaje de inicio
  Serial.println("ROSArduinoMega TEST FASE 1 - Ready!");
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