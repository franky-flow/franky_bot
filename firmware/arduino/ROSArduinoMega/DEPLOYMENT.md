# ROSArduinoMega - Documentación de Desarrollo Incremental

Documentación paso a paso del desarrollo de firmware para Arduino Mega 2560 Rev3 con control de motores L298N para integración con ROS.

---

## 📋 FASE 1: Comunicación Serial Básica

### ✅ Estado: COMPLETADA

### 🎯 Objetivo
Establecer comunicación bidireccional entre Arduino Mega 2560 y el PC mediante protocolo serial. El Arduino debe poder:
- Recibir comandos de un solo carácter con argumentos
- Parsear los comandos correctamente
- Ejecutar funciones básicas de Arduino
- Responder confirmaciones o datos solicitados

### 📝 Código Implementado

#### Estructura del Proyecto
```
ROSArduinoMega/
└── ROSArduinoMega.ino    (170 líneas aprox.)
```

#### Componentes Principales

**1. Parser de Comandos Serial**
- Variables globales para parsear: `cmd`, `argv1`, `argv2`, `arg1`, `arg2`
- Función `resetCommand()`: Limpia variables entre comandos
- Función `runCommand()`: Ejecuta el comando recibido

**2. Comandos Implementados (FASE 1)**
| Comando | Letra | Argumentos | Función |
|---------|-------|------------|---------|
| `GET_BAUDRATE` | `b` | - | Devuelve velocidad de comunicación |
| `PING` | `p` | valor | Devuelve el valor recibido (eco) |
| `ANALOG_READ` | `a` | pin | Lee pin analógico (0-1023) |
| `DIGITAL_READ` | `d` | pin | Lee pin digital (0/1) |
| `ANALOG_WRITE` | `x` | pin valor | Escribe PWM (0-255) |
| `DIGITAL_WRITE` | `w` | pin valor | Escribe digital (0=LOW, 1=HIGH) |
| `PIN_MODE` | `c` | pin modo | Configura pin (0=INPUT, 1=OUTPUT) |

**3. Protocolo de Comunicación**
```
Formato: <comando> <arg1> <arg2>\r
Ejemplos:
  b\r              → Responde: 57600
  p 123\r          → Responde: 123
  a 0\r            → Responde: valor de A0 (0-1023)
  x 13 128\r       → Escribe PWM 128 en pin 13, responde: OK
  c 13 1\r         → Configura pin 13 como OUTPUT, responde: OK
  w 13 1\r         → Escribe HIGH en pin 13, responde: OK
```

### 🔌 Hardware Necesario (FASE 1)
- **Arduino Mega 2560 Rev3**
- **Cable USB** (para programación y comunicación serial)
- *(Opcional)* LED + resistencia 220Ω en pin 13 para pruebas

### ⚙️ Configuración

#### Arduino IDE
1. Abrir Arduino IDE
2. Seleccionar placa: **Tools → Board → Arduino Mega or Mega 2560**
3. Seleccionar procesador: **Tools → Processor → ATmega2560**
4. Seleccionar puerto: **Tools → Port → /dev/ttyACM0** (o el que corresponda)
5. Baudrate: **57600** (configurado en código)

### 🧪 Pruebas y Validación

#### Prueba 1: Compilación
```bash
# En Arduino IDE:
# Sketch → Verify/Compile
# Debe compilar sin errores
```

#### Prueba 2: Carga del Sketch
```bash
# En Arduino IDE:
# Sketch → Upload
# Esperar mensaje: "Done uploading"
```

#### Prueba 3: Monitor Serial
1. Abrir Serial Monitor: **Tools → Serial Monitor**
2. Configurar:
   - Baud rate: **57600**
   - Line ending: **Carriage return** o **Both NL & CR**

3. Ejecutar comandos de prueba:

```
Comando: b
Resultado esperado: 57600

Comando: p 42
Resultado esperado: 42

Comando: c 13 1
Resultado esperado: OK

Comando: w 13 1
Resultado esperado: OK
(El LED integrado en pin 13 debe encenderse)

Comando: w 13 0
Resultado esperado: OK
(El LED debe apagarse)

Comando: a 0
Resultado esperado: valor entre 0-1023 (lectura de A0)

Comando: xyz 123
Resultado esperado: Invalid Command
```

#### Prueba 4: Desde Python (Alternativa)
```python
import serial
import time

# Conectar al Arduino
arduino = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(2)  # Esperar inicialización

# Leer mensaje de bienvenida
print(arduino.readline().decode().strip())  # "ROSArduinoMega FASE 1 - Ready!"

# Pruebas de comandos
def send_command(cmd):
    arduino.write((cmd + '\r').encode())
    time.sleep(0.1)
    response = arduino.readline().decode().strip()
    print(f"→ {cmd}\n← {response}\n")

send_command('b')           # Baudrate
send_command('p 999')       # Ping
send_command('c 13 1')      # Pin 13 como OUTPUT
send_command('w 13 1')      # LED ON
time.sleep(1)
send_command('w 13 0')      # LED OFF

arduino.close()
```

### 📊 Resultados Esperados

✅ **Éxito de la FASE 1 si:**
- El sketch compila sin errores
- Se carga correctamente en Arduino Mega 2560
- Aparece mensaje "ROSArduinoMega FASE 1 - Ready!" al abrir Serial Monitor
- Todos los comandos responden correctamente
- El LED del pin 13 responde a comandos `w 13 1` y `w 13 0`
- Los comandos inválidos responden "Invalid Command"

### 🔍 Conceptos Clave Aprendidos

**1. Parser de Comandos Serial**
- Lectura carácter por carácter con `Serial.read()`
- Uso de espacios como delimitadores
- Terminación de comando con CR (13)
- Conversión de strings a enteros con `atoi()`

**2. Comunicación Arduino**
- `Serial.begin(baudrate)`: Inicialización
- `Serial.available()`: Verificar datos disponibles
- `Serial.read()`: Leer un byte
- `Serial.println()`: Enviar respuesta

**3. Funciones Básicas Arduino**
- `pinMode()`: Configurar modo de pin
- `digitalWrite()`: Escribir digital
- `digitalRead()`: Leer digital  
- `analogWrite()`: PWM (0-255)
- `analogRead()`: Leer analógico (0-1023)

### 🔧 Solución de Problemas Comunes

| Problema | Causa Probable | Solución |
|----------|----------------|----------|
| No aparece mensaje inicial | Baudrate incorrecto | Verificar 57600 en Monitor Serial |
| Caracteres extraños | Baudrate desincronizado | Reconectar serial / reiniciar Arduino |
| No responde a comandos | Line ending incorrecto | Configurar "Carriage return" |
| "Invalid Command" siempre | Formato incorrecto | Verificar espacios y CR al final |
| Puerto no disponible | Arduino en uso | Cerrar otras conexiones seriales |

### 📌 Notas Importantes

- **Funciones Estándar**: Se utilizan funciones estándar de Arduino (`pinMode`, `digitalWrite`, etc.) para facilitar el aprendizaje
- **Compatible IDE**: El código es estándar y portable
- **Sin optimizaciones**: Código legible y educativo, optimizaciones en fases futuras
- **Errores Linter**: Es normal que VS Code muestre errores si no tiene configurado el path de Arduino. El código compila correctamente en Arduino IDE.

---

## 📋 FASE 2: Control de Motores L298N

### ✅ Estado: COMPLETADA

### 🎯 Objetivo
Implementar control básico de motores DC mediante driver L298N. El sistema debe poder:
- Configurar pines del Arduino para controlar el L298N
- Controlar dirección (adelante/atrás) de cada motor independientemente
- Controlar velocidad mediante PWM (0-255)
- Responder a comandos seriales para mover el robot

### 📝 Código Implementado

#### Nuevos Componentes Añadidos

**1. Configuración de Pines L298N**
```cpp
// Motor Izquierdo
#define LEFT_MOTOR_FORWARD   8   // IN1 del L298N
#define LEFT_MOTOR_BACKWARD  9   // IN2 del L298N  
#define LEFT_MOTOR_ENABLE    10  // ENA del L298N (PWM)

// Motor Derecho
#define RIGHT_MOTOR_FORWARD  11  // IN3 del L298N
#define RIGHT_MOTOR_BACKWARD 12  // IN4 del L298N
#define RIGHT_MOTOR_ENABLE   13  // ENB del L298N (PWM)
```

**2. Funciones de Control de Motores**
- `initMotorController()`: Inicializa pines y detiene motores
- `setMotorSpeed(motor, speed)`: Control individual (-255 a +255)
- `setMotorSpeeds(left, right)`: Control simultáneo de ambos motores

**3. Comandos Nuevos (FASE 2)**
| Comando | Letra | Argumentos | Función |
|---------|-------|------------|---------|
| `MOTOR_SPEEDS` | `m` | izq der | Control de velocidad (-255 a +255) |
| `MOTOR_RAW_PWM` | `o` | izq der | Control directo PWM (igual que 'm') |

**4. Protocolo de Comandos de Motores**
```
Formato: m <velocidad_izq> <velocidad_der>\r

Ejemplos:
  m 0 0\r          → Detener ambos motores
  m 100 100\r      → Adelante a velocidad media
  m 200 200\r      → Adelante a velocidad alta
  m -100 -100\r    → Reversa a velocidad media
  m 150 50\r       → Giro a la derecha
  m 50 150\r       → Giro a la izquierda
  m 100 -100\r     → Giro en el lugar (spin)
```

### 🔌 Conexiones Hardware L298N

#### Diagrama de Conexión

```
Arduino Mega 2560        L298N Motor Driver
==================      ==================
Pin 8  (Digital)  ----> IN1  (Motor Izq)
Pin 9  (Digital)  ----> IN2  (Motor Izq)
Pin 10 (PWM)      ----> ENA  (Enable Izq)

Pin 11 (PWM)      ----> IN3  (Motor Der)
Pin 12 (Digital)  ----> IN4  (Motor Der)
Pin 13 (PWM)      ----> ENB  (Enable Der)

GND               ----> GND
                        
                  L298N Motor Driver
                  ==================
                  OUT1 y OUT2 ----> Motor Izquierdo
                  OUT3 y OUT4 ----> Motor Derecho
                  
                  +12V ----> Fuente alimentación (7-12V)
                  GND  ----> GND común
                  +5V  ----> (Opcional) si hay jumper para alimentar Arduino
```

#### Detalles de Conexión

**Pines Arduino → L298N:**
- Los pines 8, 9, 11, 12 controlan dirección
- Los pines 10, 13 controlan velocidad (PWM)
- Todos los pines comparten GND común

**Alimentación:**
- **Motor Power (12V)**: Fuente externa 7-12V → terminal +12V del L298N
- **Arduino Power**: Via USB o jack de alimentación
- **GND común**: Conectar GND de Arduino con GND del L298N

### 🔧 Hardware Necesario (FASE 2)

✅ **Componentes:**
- Arduino Mega 2560 Rev3
- Driver L298N (módulo)
- 2 motores DC (6-12V)
- Fuente de alimentación externa (7-12V, mínimo 1A)
- Cables de conexión (Dupont macho-macho)
- Protoboard (opcional, para organizar conexiones)

### 🧪 Pruebas y Validación

#### Prueba 1: Compilación y Carga
```bash
# Arduino IDE:
# 1. Abrir ROSArduinoMega.ino
# 2. Verificar placa: Arduino Mega 2560
# 3. Sketch → Verify/Compile (debe compilar sin errores)
# 4. Sketch → Upload
```

#### Prueba 2: Verificación Serial
```
# Serial Monitor (57600 baud)
Al iniciar debe aparecer: "ROSArduinoMega FASE 2 - Ready!"
```

#### Prueba 3: Test de Motores (SIN MOVIMIENTO)
**⚠️ IMPORTANTE**: Antes de probar, levantar el robot para que las ruedas no toquen el suelo.

```
Comando: m 0 0
Resultado: OK (motores detenidos)

Comando: m 50 50
Resultado: OK (ambas ruedas giran adelante despacio)

Comando: m 0 0
Resultado: OK (motores detenidos)

Comando: m -50 -50
Resultado: OK (ambas ruedas giran atrás despacio)

Comando: m 0 0
Resultado: OK (motores detenidos)
```

#### Prueba 4: Test de Direcciones
```
# Giro a la derecha (rueda izquierda más rápida)
Comando: m 100 50
Resultado: OK

# Detener
Comando: m 0 0
Resultado: OK

# Giro a la izquierda (rueda derecha más rápida)
Comando: m 50 100
Resultado: OK

# Detener
Comando: m 0 0
Resultado: OK

# Giro en el lugar (ruedas en direcciones opuestas)
Comando: m 100 -100
Resultado: OK

# Detener
Comando: m 0 0
Resultado: OK
```

#### Prueba 5: Test con Python
```python
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(2)

print(arduino.readline().decode().strip())  # "ROSArduinoMega FASE 2 - Ready!"

def motor_command(left, right, duration=1):
    cmd = f"m {left} {right}\r"
    arduino.write(cmd.encode())
    print(f"→ {cmd.strip()}")
    response = arduino.readline().decode().strip()
    print(f"← {response}")
    time.sleep(duration)
    # Detener
    arduino.write(b"m 0 0\r")
    arduino.readline()
    time.sleep(0.5)

print("\n⚠️ LEVANTAR ROBOT - Prueba de motores...")
time.sleep(3)

motor_command(100, 100, 2)   # Adelante
motor_command(-100, -100, 2) # Reversa
motor_command(100, 50, 2)    # Derecha
motor_command(50, 100, 2)    # Izquierda

print("✅ Test completado")
arduino.close()
```

### 📊 Resultados Esperados

✅ **Éxito de la FASE 2 si:**
- El sketch compila sin errores
- Mensaje "ROSArduinoMega FASE 2 - Ready!" aparece
- Comandos `m` responden "OK"
- Motores giran en dirección correcta según comando
- Velocidad varía según valor PWM (0-255)
- Valores negativos causan reversa
- Comando `m 0 0` detiene motores inmediatamente

### 🔍 Conceptos Clave Aprendidos

**1. Driver de Motores L298N**
- **Puente H**: Permite invertir polaridad (cambiar dirección)
- **IN1, IN2**: Controlan dirección del motor izquierdo
- **IN3, IN4**: Controlan dirección del motor derecho
- **ENA, ENB**: Habilitan y controlan velocidad (PWM)

**2. Control de Dirección**
```
Motor ADELANTE:  FORWARD=PWM, BACKWARD=0
Motor REVERSA:   FORWARD=0,   BACKWARD=PWM
Motor STOP:      FORWARD=0,   BACKWARD=0
```

**3. PWM (Pulse Width Modulation)**
- Rango: 0-255 (0=detenido, 255=máxima velocidad)
- Frecuencia: ~490Hz en pines 10, 13 (Arduino Mega)
- Control análogo de velocidad con señal digital

**4. Robot Diferencial**
- **Adelante**: Ambos motores igual velocidad positiva
- **Reversa**: Ambos motores igual velocidad negativa
- **Giro**: Un motor más rápido que el otro
- **Spin**: Motores en direcciones opuestas

### 🔧 Solución de Problemas

| Problema | Causa Probable | Solución |
|----------|----------------|----------|
| Motores no giran | Falta alimentación externa | Verificar fuente 12V conectada al L298N |
| Un motor no gira | Cable suelto | Revisar conexiones OUT1-4 del L298N |
| Motores giran al revés | Cables invertidos | Invertir cables del motor en OUT1-2 o OUT3-4 |
| Velocidad muy baja | PWM insuficiente | Usar valores mayores (>100) |
| Arduino se resetea | Picos de corriente | Usar fuente externa separada para motores |
| Giro errático | GND no común | Conectar GND Arduino con GND L298N |

### ⚠️ Consideraciones de Seguridad

1. **Alimentación separada**: Nunca alimentar motores desde USB del Arduino
2. **GND común**: Siempre conectar GND de Arduino y L298N
3. **Pruebas elevadas**: Primera prueba con robot levantado
4. **Valores progresivos**: Empezar con PWM bajo (50-100)
5. **Stop de emergencia**: Siempre tener `m 0 0` preparado

### 📌 Notas Técnicas

**Diferencias con FASE 1:**
- Añadidas 3 funciones de control de motores
- 2 comandos nuevos (`m` y `o`)
- Inicialización de pines en `setup()`
- Uso de `analogWrite()` para PWM

**Compatibilidad ROSArduinoBridge:**
- Comandos `m` y `o` compatibles con interfaz ROS
- En FASE 4 añadiremos PID para control preciso
- Por ahora control directo PWM (open-loop)

---

## 🚀 Siguiente Fase: FASE 3 - Lectura de Encoders

En la próxima fase implementaremos:
- Configuración de interrupciones para encoders
- Lectura de pulsos de encoders
- Comandos para leer y resetear contadores
- Cálculo de velocidad y distancia

**Hardware adicional necesario:**
- 2 encoders rotativos (ópticos o magnéticos)
- Resistencias pull-up si no están integradas

---

## 🔮 OPTIMIZACIONES FUTURAS (Post-FASE 4)

### Manipulación Directa de Registros

Una vez completadas todas las fases y con el código funcionando, se pueden aplicar optimizaciones mediante acceso directo a registros del microcontrolador.

#### 🎯 Dónde Optimizar

**1. Lectura de Encoders (FASE 3)**
- **Actual**: `attachInterrupt()` con funciones Arduino
- **Optimizado**: Manipulación directa de registros PCINT
- **Ventaja**: Permite usar más pines simultáneamente, menor latencia
- **Ejemplo**:
```cpp
// En lugar de:
attachInterrupt(digitalPinToInterrupt(2), ISR_Left, CHANGE);

// Optimizado:
PCMSK2 |= (1 << PCINT18);  // Pin 2
PCICR |= (1 << PCIE2);
```

**2. Control PWM de Motores (FASE 2)**
- **Actual**: `analogWrite(pin, value)`
- **Optimizado**: Configuración directa de Timer/Counter
- **Ventaja**: Control de frecuencia PWM, mayor precisión
- **Ejemplo**:
```cpp
// En lugar de:
analogWrite(9, 128);

// Optimizado:
TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
TCCR2B = _BV(CS22);
OCR2A = 128;
```

**3. Lectura Digital Rápida**
- **Actual**: `digitalRead(pin)`
- **Optimizado**: Lectura directa de registros PORTx
- **Ventaja**: 10-20x más rápido
- **Ejemplo**:
```cpp
// En lugar de:
int valor = digitalRead(13);

// Optimizado:
int valor = (PINB & (1 << PB7)) ? HIGH : LOW;  // Pin 13 = PB7
```

#### 📚 Referencias para Optimización
- [Arduino Port Manipulation](https://www.arduino.cc/en/Reference/PortManipulation)
- [ATmega2560 Datasheet](https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf)
- [Brett Beauregard's PID Library](http://brettbeauregard.com/blog/category/pid/)

#### ⚠️ Consideraciones
- Aplicar optimizaciones **SOLO** después de tener el código base funcionando
- Documentar cada cambio de optimización
- Realizar pruebas comparativas antes/después
- Mantener legibilidad del código con comentarios explicativos

---

## 📝 Control de Versiones

| Fecha | Fase | Versión | Descripción |
|-------|------|---------|-------------|
| 2026-02-11 | FASE 1 | v1.0 | Comunicación serial básica implementada |
| 2026-02-11 | FASE 2 | v2.0 | Control de motores L298N con PWM implementado |

---

**Última actualización**: 2026-02-11  
**Autor**: Desarrollo incremental con Cline  
**Hardware**: Arduino Mega 2560 Rev3 + L298N
