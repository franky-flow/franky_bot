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
├── ROSArduinoMega.ino    (Código acumulativo de todas las fases)
├── Test-fase1.ino        (Solo FASE 1 para pruebas aisladas)
├── Test-fase2.ino        (FASE 1 + FASE 2 para pruebas)
├── Test-fase3.ino        (FASE 1 + FASE 2 + FASE 3 para pruebas)
├── Test-fase4.ino        (TODAS las fases - Sistema completo)
└── DEPLOYMENT.md         (Esta documentación)
```

**Nota sobre archivos de test:**
- `Test-fase1.ino`: Contiene SOLO comunicación serial (útil para probar sin hardware de motores)
- `Test-fase2.ino`: Contiene comunicación + motores (útil para probar hasta esta fase)
- `Test-fase3.ino`: Contiene comunicación + motores + encoders (útil para probar hasta esta fase)
- `Test-fase4.ino`: Sistema completo con control PID (producción)
- `ROSArduinoMega.ino`: Archivo principal con todas las fases acumuladas

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

## 📋 FASE 3: Lectura de Encoders

### ✅ Estado: COMPLETADA

### 🎯 Objetivo
Implementar lectura de encoders rotativos mediante interrupciones para obtener retroalimentación de posición. El sistema debe poder:
- Configurar interrupciones en pines específicos de Arduino Mega
- Detectar dirección de giro (adelante/atrás)
- Contar pulsos de encoder en tiempo real
- Responder a comandos para leer y resetear contadores

### 📝 Código Implementado

#### Nuevos Componentes Añadidos

**1. Configuración de Pines de Encoders**
```cpp
// Encoder Izquierdo
#define LEFT_ENCODER_A   2   // Pin de interrupción (INT4)
#define LEFT_ENCODER_B   4   // Pin digital

// Encoder Derecho
#define RIGHT_ENCODER_A  3   // Pin de interrupción (INT5)
#define RIGHT_ENCODER_B  5   // Pin digital
```

**2. Variables Globales de Encoders**
```cpp
volatile long leftEncoderCount = 0;   // volatile para acceso en ISR
volatile long rightEncoderCount = 0;
```

**3. Funciones de Encoders**
- `leftEncoderISR()`: Rutina de interrupción para encoder izquierdo
- `rightEncoderISR()`: Rutina de interrupción para encoder derecho
- `initEncoders()`: Inicializa pines y configura interrupciones
- `readEncoder(motor)`: Lee valor del contador
- `resetEncoder(motor)`: Resetea un contador específico
- `resetEncoders()`: Resetea ambos contadores

**4. Comandos Nuevos (FASE 3)**
| Comando | Letra | Argumentos | Función |
|---------|-------|------------|---------|
| `READ_ENCODERS` | `e` | - | Lee contadores de ambos encoders |
| `RESET_ENCODERS` | `r` | - | Resetea ambos contadores a 0 |

**5. Protocolo de Comandos de Encoders**
```
Comandos:
  e\r     → Lee encoders, responde: "valor_izq valor_der"
  r\r     → Resetea encoders, responde: "OK"

Ejemplos:
  e\r     → Responde: "0 0" (si están en reposo)
  e\r     → Responde: "145 -89" (después de movimiento)
  r\r     → Responde: "OK" (contadores reseteados)
  e\r     → Responde: "0 0" (después de reset)
```

### 🔌 Conexiones Hardware Encoders

#### Diagrama de Conexión

```
Arduino Mega 2560         Encoder Izquierdo         Encoder Derecho
==================        ==================        ==================
Pin 2 (INT4)     -------> Canal A (CLK)    
Pin 4 (Digital)  -------> Canal B (DT)
                                                    
Pin 3 (INT5)     ---------------------------------> Canal A (CLK)
Pin 5 (Digital)  ---------------------------------> Canal B (DT)

GND              -------> GND ----------------->    GND
+5V              -------> VCC (o +3.3V) ------->    VCC

Notas:
- Algunos encoders tienen resistencias pull-up integradas
- Si no las tienen, el código activa pull-ups internas
- Respetar polaridad VCC/GND según especificaciones del encoder
```

#### Pines de Interrupción Arduino Mega 2560

Arduino Mega tiene 6 pines de interrupción externa:
| Pin Físico | Interrupt | Uso en FASE 3 |
|------------|-----------|---------------|
| 2 | INT4 (0) | LEFT_ENCODER_A ✓ |
| 3 | INT5 (1) | RIGHT_ENCODER_A ✓ |
| 18 | INT3 (5) | Disponible |
| 19 | INT2 (4) | Disponible |
| 20 | INT1 (3) | Disponible |
| 21 | INT0 (2) | Disponible |

### 🔧 Hardware Necesario (FASE 3)

✅ **Componentes adicionales:**
- 2 Encoders rotativos (ópticos o magnéticos)
  - Resolución típica: 20-600 pulsos por revolución (PPR)
  - Salida: Tipo cuadratura (2 canales: A y B)
  - Voltaje: 3.3V o 5V compatible con Arduino
- Cables de conexión Dupont
- *(Opcional)* Resistencias pull-up 10kΩ si encoders no las tienen

### 🧪 Pruebas y Validación

#### Prueba 1: Compilación y Carga
```bash
# Arduino IDE:
# 1. Abrir Test-fase3.ino (o ROSArduinoMega.ino)
# 2. Verificar placa: Arduino Mega 2560
# 3. Sketch → Verify/Compile
# 4. Sketch → Upload
```

#### Prueba 2: Verificación Serial
```
# Serial Monitor (57600 baud)
Al iniciar debe aparecer: "ROSArduinoMega FASE 3 - Ready!"
```

#### Prueba 3: Test de Encoders Estáticos
```
Comando: e
Resultado esperado: 0 0 (o valores muy pequeños si hay ruido)

Comando: r
Resultado: OK

Comando: e
Resultado: 0 0
```

#### Prueba 4: Test de Dirección (Manual)
**⚠️ Levantar robot** - Girar ruedas manualmente

```
# Resetear contadores
Comando: r
Resultado: OK

# Leer estado inicial
Comando: e
Resultado: 0 0

# Girar rueda izquierda hacia ADELANTE (manualmente)
# Luego leer
Comando: e
Resultado esperado: Valor positivo en izquierda, ej: "50 0"

# Resetear
Comando: r

# Girar rueda izquierda hacia ATRÁS (manualmente)
# Luego leer
Comando: e
Resultado esperado: Valor negativo en izquierda, ej: "-45 0"

# Repetir para rueda derecha
```

#### Prueba 5: Test con Motores + Encoders
```python
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(2)

print(arduino.readline().decode().strip())  # "ROSArduinoMega FASE 3 - Ready!"

def send_cmd(cmd):
    arduino.write((cmd + '\r').encode())
    time.sleep(0.05)
    return arduino.readline().decode().strip()

# Reset encoders
print(f"Reset: {send_cmd('r')}")

# Leer inicial
print(f"Encoders iniciales: {send_cmd('e')}")

# ⚠️ LEVANTAR ROBOT
print("\n⚠️ Asegurar que robot está levantado...")
time.sleep(2)

# Mover adelante por 1 segundo
print("\nMoviendo adelante...")
print(send_cmd('m 100 100'))
time.sleep(1)
print(send_cmd('m 0 0'))

# Leer encoders después de movimiento
enc = send_cmd('e')
print(f"Encoders después adelante: {enc}")

# Resetear
send_cmd('r')

# Mover atrás por 1 segundo
print("\nMoviendo atrás...")
print(send_cmd('m -100 -100'))
time.sleep(1)
print(send_cmd('m 0 0'))

# Leer encoders (deberían ser negativos)
enc = send_cmd('e')
print(f"Encoders después atrás: {enc}")

arduino.close()
```

### 📊 Resultados Esperados

✅ **Éxito de la FASE 3 si:**
- El sketch compila sin errores
- Mensaje "ROSArduinoMega FASE 3 - Ready!" aparece
- Comando `e` responde con dos números separados por espacio
- Comando `r` responde "OK" y resetea contadores
- Girar rueda adelante aumenta el contador
- Girar rueda atrás disminuye el contador (negativo)
- Los valores cambian instantáneamente al girar (sin delay)
- Motores funcionan igual que FASE 2

### 🔍 Conceptos Clave Aprendidos

**1. Encoders de Cuadratura**
- **Canal A**: Señal principal de pulsos
- **Canal B**: Señal desfasada 90° para detectar dirección
- **Resolución**: PPR (Pulsos Por Revolución)
- **Conteo**: Se puede 1x, 2x o 4x según método

**2. Interrupciones en Arduino**
- **ISR (Interrupt Service Routine)**: Función que se ejecuta automáticamente
- **attachInterrupt()**: Asocia pin con función ISR
- **CHANGE**: Dispara en flanco de subida Y bajada (2x resolución)
- **volatile**: Indica que variable se modifica en ISR

**3. Detección de Dirección**
```
Adelante: Cuando A cambia, si B=HIGH → incrementar
Atrás:    Cuando A cambia, si B=LOW  → decrementar
```

**4. Cálculo de Distancia/Velocidad**
```
Pulsos totales = leftEncoderCount (o rightEncoderCount)
Revoluciones = Pulsos / PPR
Distancia = Revoluciones × Circunferencia_Rueda
Velocidad = Δ Pulsos / Δ Tiempo
```

### 🔧 Solución de Problemas

| Problema | Causa Probable | Solución |
|----------|----------------|----------|
| Contadores no cambian | Encoders no conectados | Verificar conexiones A, B, VCC, GND |
| Valores erráticos/saltos | Ruido eléctrico | Añadir capacitor 0.1µF entre VCC-GND |
| Solo cuenta en una dirección | Canal B mal conectado | Verificar pin B en pin correcto |
| Cuenta doble o errático | Rebote mecánico | Usar CHANGE en lugar de RISING |
| Direcciones invertidas | Canales A/B intercambiados | Invertir lógica o cables A-B |
| Arduino se cuelga | ISR muy lenta | Simplificar código en ISR (sin Serial) |

### ⚠️ Consideraciones Importantes

**Sobre las ISR (Interrupt Service Routines):**
1. **Rápidas**: ISR deben ejecutarse lo más rápido posible
2. **Sin Serial**: NO usar `Serial.print()` dentro de ISR
3. **volatile**: Variables compartidas deben ser `volatile`
4. **Atómicas**: Operaciones simples (++, --, =)

**Sobre Encoders:**
1. **Ruido**: Cables cortos y apantallados si hay interferencias
2. **Pull-ups**: Activadas en código si encoder no las tiene
3. **Velocidad**: Encoders deben soportar RPM máximas del motor
4. **Resolución**: Mayor PPR = mayor precisión pero más carga CPU

### 📌 Notas Técnicas

**Diferencias con FASE 2:**
- Añadidas 6 funciones de encoders (2 ISR + 4 utilitarias)
- 2 comandos nuevos (`e` y `r`)
- 2 variables globales `volatile`
- Uso de `attachInterrupt()` y `digitalPinToInterrupt()`
- Inicialización de encoders en `setup()`

**Compatibilidad ROSArduinoBridge:**
- Comandos `e` y `r` compatibles con ROS
- Formato de respuesta igual (dos valores separados por espacio)
- En FASE 4 usaremos estos valores para PID

**Conteo 1x vs 2x vs 4x:**
- **1x**: Solo flanco de subida de A
- **2x**: Ambos flancos de A (CHANGE) ✓ Usado aquí
- **4x**: Todos los flancos de A y B (máxima precisión)

---

## 📋 FASE 4: Control PID Completo

### ✅ Estado: COMPLETADA

### 🎯 Objetivo
Implementar control PID (Proporcional-Integral-Derivativo) para control preciso de velocidad de los motores con retroalimentación de encoders. El sistema debe poder:
- Mantener velocidad constante independiente de la carga
- Ajustar automáticamente PWM según feedback de encoders
- Permitir configuración de parámetros PID
- Implementar auto-stop por seguridad
- Integrar todas las fases en un sistema funcional completo

### 📝 Código Implementado

#### Nuevos Componentes Añadidos

**1. Estructura de Datos PID**
```cpp
typedef struct {
  double TargetTicksPerFrame;  // Velocidad objetivo en ticks por frame
  long Encoder;                // Valor actual del encoder
  long PrevEnc;                // Valor anterior del encoder
  int PrevInput;               // Entrada anterior (para derivada mejorada)
  int ITerm;                   // Término integral acumulado
  long output;                 // Salida PWM calculada
} SetPointInfo;

SetPointInfo leftPID, rightPID;  // Una estructura para cada motor
```

**2. Parámetros PID**
```cpp
int Kp = 20;  // Ganancia proporcional (respuesta a error actual)
int Kd = 12;  // Ganancia derivativa (predice error futuro)
int Ki = 0;   // Ganancia integral (corrige error acumulado)
int Ko = 50;  // Factor de escalado de salida
```

**3. Variables de Control**
```cpp
unsigned char moving = 0;              // Estado de movimiento
unsigned long nextPID = PID_INTERVAL;  // Temporizador PID
long lastMotorCommand = AUTO_STOP_INTERVAL;  // Timer auto-stop
```

**4. Funciones PID**
- `resetPID()`: Inicializa variables PID (previene spikes)
- `doPID(SetPointInfo*)`: Calcula salida PID para un motor
- `updatePID()`: Actualiza ambos motores con PID

**5. Constantes de Configuración**
```cpp
#define PID_RATE           30     // Hz - 30 veces por segundo
#define PID_INTERVAL       33     // ms - Intervalo entre cálculos
#define AUTO_STOP_INTERVAL 2000   // ms - Timeout de seguridad
#define MIN_PWM            30     // PWM mínimo (zona muerta)
```

**Nota sobre MIN_PWM (Zona Muerta):**
Los motores DC tienen una "zona muerta" donde el PWM es demasiado bajo para vencer la fricción estática. El código aplica automáticamente un PWM mínimo de 30 cuando se detecta movimiento (speed > 0). Este valor:
- Asegura que los motores arrancan inmediatamente
- Previene movimientos muy lentos que no vencen fricción
- Permite al PID trabajar con valores efectivos
- Es ajustable modificando `MIN_PWM` según tu motor

**6. Comandos Actualizados (FASE 4)**
| Comando | Letra | Argumentos | Función | Cambios FASE 4 |
|---------|-------|------------|---------|----------------|
| `MOTOR_SPEEDS` | `m` | izq der | Control de velocidad | Ahora usa PID (ticks/frame) |
| `MOTOR_RAW_PWM` | `o` | izq der | Control directo PWM | Sin cambios (bypass PID) |
| `RESET_ENCODERS` | `r` | - | Resetea encoders | Ahora también resetea PID |
| `UPDATE_PID` | `u` | Kp:Kd:Ki:Ko | Actualiza parámetros PID | **NUEVO** |

**7. Protocolo de Comandos PID**
```
Formato comando PID: u Kp:Kd:Ki:Ko\r
Ejemplos:
  u 20:12:0:50\r   → Parámetros por defecto
  u 25:15:1:50\r   → Más agresivo con integral
  u 15:10:0:50\r   → Más suave

Formato MOTOR_SPEEDS (ahora con PID):
  m 10 10\r        → 10 ticks/frame cada rueda (closed-loop)
  m 20 15\r        → Giro suave a la derecha con PID
  m 0 0\r          → Detener (resetea PID automáticamente)
```

### 🧠 Conceptos del Algoritmo PID

#### ¿Qué es PID?

PID es un algoritmo de control que ajusta continuamente la salida (PWM) basándose en:

**P (Proporcional)**
- Reacciona al error actual
- Error grande → corrección grande
- `Kp × Error`

**I (Integral)**
- Acumula errores pasados
- Corrige deriva continua
- `Ki × Σ Errores`

**D (Derivativa)**
- Predice error futuro
- Suaviza oscilaciones
- `Kd × (Error_actual - Error_anterior)`

#### Fórmula Implementada

```
output = (Kp × Perror - Kd × Δinput + ITerm) / Ko + output_anterior

Donde:
  Perror = TargetTicks - InputTicks
  Δinput = InputTicks - PrevInput  (evita derivative kick)
  ITerm = acumulado de (Ki × Perror)
```

#### Mejoras Implementadas

**1. Sin Derivative Kick**
- Usa cambio en entrada en lugar de cambio en error
- Evita picos al cambiar setpoint
- Ver: [Brett Beauregard - Derivative Kick](http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/)

**2. Anti-Windup Integral**
- Solo acumula integral cuando salida NO saturada
- Previene acumulación excesiva
- Ver: [Brett Beauregard - Tuning Changes](http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/)

**3. Inicialización Sin Spikes**
- Variables PID se inicializan a valores actuales
- Evita saltos al arrancar
- Ver: [Brett Beauregard - Initialization](http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/)

### 🔧 Ajuste de Parámetros PID

#### Valores por Defecto
```cpp
Kp = 20  // Proporcional
Kd = 12  // Derivativo
Ki = 0   // Integral (desactivado por defecto)
Ko = 50  // Escalado
```

#### Método de Ajuste (Ziegler-Nichols simplificado)

**1. Solo P (Ki=0, Kd=0)**
```
u 10:0:0:50     → Empezar conservador
u 20:0:0:50     → Aumentar si muy lento
u 30:0:0:50     → Seguir hasta oscilaciones
```

**2. Añadir D**
```
u 20:5:0:50     → Añadir derivativa
u 20:10:0:50    → Aumentar hasta suavizar
u 20:12:0:50    → Valor óptimo (defecto)
```

**3. Añadir I (si hay error residual)**
```
u 20:12:1:50    → Integral mínima
u 20:12:2:50    → Aumentar con cuidado
```

#### Síntomas y Soluciones

| Comportamiento | Problema | Solución |
|----------------|----------|----------|
| Oscila constantemente | Kp muy alto | Reducir Kp |
| Respuesta muy lenta | Kp muy bajo | Aumentar Kp |
| Oscilaciones rápidas | Kd muy bajo | Aumentar Kd |
| Sobrecompensación | Kd muy alto | Reducir Kd |
| Error permanente | Ki = 0 | Aumentar Ki (con cuidado) |
| Inestable con tiempo | Ki muy alto | Reducir Ki |

### 🧪 Pruebas y Validación

#### Prueba 1: Compilación y Carga
```bash
# Arduino IDE:
# 1. Abrir Test-fase4.ino (o ROSArduinoMega.ino)
# 2. Verificar placa: Arduino Mega 2560
# 3. Sketch → Verify/Compile
# 4. Sketch → Upload
```

#### Prueba 2: Verificación Serial
```
# Serial Monitor (57600 baud)
Al iniciar debe aparecer: "ROSArduinoMega FASE 4 - Ready!"
```

#### Prueba 3: Test de Control PID Básico
**⚠️ Levantar robot**

```
# Resetear todo
Comando: r
Resultado: OK

# Verificar parámetros PID actuales (implícitos: 20:12:0:50)
# No hay comando para leer, están en el código

# Comando con PID (en ticks/frame, no PWM directo)
Comando: m 10 10
Resultado: OK
# Robot mantiene velocidad constante
# Observar que encoders aumentan de forma regular

# Leer encoders mientras se mueve (repetir varias veces)
Comando: e
Resultado: valores aumentando constantemente

# Detener
Comando: m 0 0
Resultado: OK
```

#### Prueba 4: Comparación PID vs PWM Directo
```python
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(2)

print(arduino.readline().decode().strip())

def send_cmd(cmd):
    arduino.write((cmd + '\r').encode())
    time.sleep(0.05)
    return arduino.readline().decode().strip()

print("⚠️ LEVANTAR ROBOT\n")
time.sleep(2)

# Test 1: Control directo PWM (sin PID)
print("=== Test PWM Directo (comando 'o') ===")
send_cmd('r')  # Reset
send_cmd('o 100 100')  # PWM directo
time.sleep(2)
enc1 = send_cmd('e')
print(f"Encoders con PWM directo: {enc1}")
send_cmd('o 0 0')
time.sleep(1)

# Test 2: Control con PID (comando 'm')
print("\n=== Test PID (comando 'm') ===")
send_cmd('r')  # Reset
send_cmd('m 10 10')  # 10 ticks/frame objetivo
time.sleep(2)
enc2 = send_cmd('e')
print(f"Encoders con PID: {enc2}")
send_cmd('m 0 0')

print("\n✅ PID debe mantener velocidad más constante")
arduino.close()
```

#### Prueba 5: Test de Auto-Stop
```
# Enviar comando de movimiento
Comando: m 10 10
Resultado: OK

# NO enviar más comandos
# Esperar 3 segundos...
# Robot debe detenerse automáticamente después de 2 segundos
# (AUTO_STOP_INTERVAL = 2000 ms)
```

#### Prueba 6: Ajuste de Parámetros PID
```
# Probar diferentes parámetros
Comando: u 25:15:0:50
Resultado: OK

Comando: m 10 10
# Observar comportamiento (¿más rápido? ¿oscila?)

Comando: m 0 0

# Volver a valores por defecto
Comando: u 20:12:0:50
Resultado: OK
```

### 📊 Resultados Esperados

✅ **Éxito de la FASE 4 si:**
- El sketch compila sin errores (~500 líneas)
- Mensaje "ROSArduinoMega FASE 4 - Ready!" aparece
- Comando `m` con PID mantiene velocidad constante
- Comando `o` funciona igual que FASE 2 (sin PID)
- Comando `u` actualiza parámetros PID
- Auto-stop funciona después de 2 segundos sin comandos
- Robot se mueve suavemente sin oscilaciones
- Velocidad se mantiene ante cambios de carga

### 🔍 Conceptos Clave Aprendidos

**1. Control Closed-Loop vs Open-Loop**
- **Open-Loop (FASE 2)**: PWM directo, sin feedback
- **Closed-Loop (FASE 4)**: PID con feedback de encoders

**2. Estructura SetPointInfo**
- Encapsula todas las variables de un motor
- Permite pasar por referencia a funciones
- Mantiene historial para cálculos

**3. Temporización del Bucle PID**
- Ejecuta a 30 Hz (cada 33 ms)
- Frecuencia constante crítica para PID
- Usa `millis()` para timing no bloqueante

**4. Auto-Stop por Seguridad**
- Detiene motores si >2 segundos sin comandos
- Previene movimientos descontrolados
- Se resetea con cada comando válido

**5. TargetTicksPerFrame**
- Unidad de velocidad: pulsos de encoder por frame PID
- Frame = 1/30 segundo
- Ejemplo: 10 ticks/frame = 10 pulsos cada 33ms

### 🔧 Solución de Problemas

| Problema | Causa Probable | Solución |
|----------|----------------|----------|
| Robot oscila | Kp muy alto o Kd bajo | Reducir Kp o aumentar Kd |
| Respuesta lenta | Kp muy bajo | Aumentar Kp progresivamente |
| No mantiene velocidad | Error sin integral | Añadir Ki (empezar con 1) |
| Inestable con tiempo | Ki muy alto | Reducir Ki o desactivar (0) |
| Motores paran solos | Auto-stop activado | Enviar comandos cada <2 seg |
| No responde a cambios | PID saturado | Verificar límites PWM |
| Spikes al arrancar | PID no reseteado | Verificar resetPID() al parar |
| **Motores no arrancan con PWM bajo** | **Zona muerta** | **Ajustar MIN_PWM (30-50)** |
| **No gira o muy lento** | **MIN_PWM muy bajo** | **Aumentar MIN_PWM a 40-50** |

### ⚠️ Consideraciones Importantes

**Sobre Control PID:**
1. **Ajuste progresivo**: Empezar con valores bajos
2. **Un parámetro a la vez**: Ajustar Kp, luego Kd, luego Ki
3. **Observar comportamiento**: Oscilaciones, velocidad, estabilidad
4. **Documentar valores**: Anotar qué funciona para tu robot
5. **Condiciones específicas**: PID óptimo depende de: peso robot, tipo motor, voltaje, PPR encoders

**Sobre Temporización:**
1. **30 Hz es estándar**: Balance entre precisión y carga CPU
2. **No bloquear loop**: usar `millis()` no `delay()`
3. **Consistencia crítica**: Intervalos regulares = PID estable

**Diferencia comando 'm' vs 'o':**
- **m**: Closed-loop con PID (ticks/frame como unidad)
- **o**: Open-loop directo (PWM como unidad)
- Usar 'm' para navegación ROS
- Usar 'o' para diagnóstico/pruebas

### 📌 Notas Técnicas

**Diferencias con FASE 3:**
- Estructura `SetPointInfo` con 6 campos
- 3 funciones PID nuevas
- 1 comando nuevo (`u`)
- Modificación comando `m` (ahora usa PID)
- Modificación comando `r` (resetea PID también)
- Bucle PID a 30 Hz en `loop()`
- Auto-stop integrado en `loop()`
- ~500 líneas de código total

**Compatibilidad ROSArduinoBridge:**
- 100% compatible con protocolo original
- Comandos idénticos a versión oficial
- Listo para integración con ROS

**Entendiendo TargetTicksPerFrame:**
```
Si tu encoder tiene 300 PPR y quieres 1 rev/seg:
  300 pulsos/revolución ÷ 30 frames/segundo = 10 ticks/frame
  Comando: m 10 10
  
Para velocidad más lenta (0.5 rev/seg):
  150 pulsos/segundo ÷ 30 frames = 5 ticks/frame
  Comando: m 5 5
```

### 🧪 Pruebas Avanzadas

#### Test de Carga Variable
```
1. Levantar robot
2. Comando: m 10 10
3. Aplicar resistencia manual a una rueda
4. Observar: PID aumenta PWM automáticamente
5. Soltar: PID reduce PWM
→ Velocidad (ticks/frame) debe mantenerse constante
```

#### Test de Precisión
```python
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(2)

def send_cmd(cmd):
    arduino.write((cmd + '\r').encode())
    time.sleep(0.05)
    return arduino.readline().decode().strip()

print("Test de Precisión PID\n")
send_cmd('r')  # Reset

# Mover con velocidad objetivo
target_ticks = 10
send_cmd(f'm {target_ticks} {target_ticks}')

# Medir ticks por segundo durante 5 segundos
for i in range(5):
    time.sleep(1)
    enc = send_cmd('e')
    left, right = map(int, enc.split())
    print(f"Seg {i+1}: Left={left} Right={right}")

send_cmd('m 0 0')

# Calcular ticks por segundo
# Deberían ser aprox. target_ticks × 30 frames/seg = 300 ticks/seg
```

### 📊 Resultados Esperados

✅ **Éxito de la FASE 4 si:**
- Sistema completo compila sin errores
- Mensaje "ROSArduinoMega FASE 4 - Ready!" aparece
- Comando `m` mantiene velocidad constante (PID activo)
- Comando `o` funciona como PWM directo (sin PID)
- Comando `u` actualiza parámetros PID correctamente
- Auto-stop detiene robot después de 2 seg sin comandos
- Robot se mueve suavemente sin oscilaciones bruscas
- Velocidad se mantiene ante resistencias variables
- Todas las fases anteriores siguen funcionando

### 📚 Referencias Técnicas

- [Brett Beauregard's PID Series](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
- [PID Without a PhD](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library/)

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
| 2026-02-11 | FASE 3 | v3.0 | Lectura de encoders con interrupciones implementada |
| 2026-02-11 | FASE 4 | v4.0 | Control PID completo con closed-loop feedback |

---

## 🎉 PROYECTO COMPLETADO

El firmware ROSArduinoMega está **100% funcional** y listo para integración con ROS.

### Resumen de Funcionalidades

✅ **Comunicación Serial** (FASE 1): Protocolo de comandos robusto  
✅ **Control de Motores** (FASE 2): Driver L298N con PWM  
✅ **Lectura de Encoders** (FASE 3): Interrupciones con detección de dirección  
✅ **Control PID** (FASE 4): Closed-loop con feedback continuo  

### Comandos Disponibles

| Categoría | Comandos | Descripción |
|-----------|----------|-------------|
| **Básicos** | `b`, `p`, `a`, `d`, `x`, `w`, `c` | Utilidades Arduino |
| **Motores** | `m`, `o` | Control PID y directo |
| **Encoders** | `e`, `r` | Lectura y reset |
| **PID** | `u` | Ajuste de parámetros |

### Archivos Finales

- ✅ `ROSArduinoMega.ino` - Producción (todas las fases)
- ✅ `Test-fase1.ino` - Solo comunicación
- ✅ `Test-fase2.ino` - Comunicación + motores
- ✅ `Test-fase3.ino` - Comunicación + motores + encoders
- ✅ `Test-fase4.ino` - Sistema completo
- ✅ `DEPLOYMENT.md` - Documentación completa

### Próximos Pasos

1. **Calibrar PID** para tu robot específico
2. **Determinar PPR** de tus encoders
3. **Configurar ROS** para usar este firmware
4. **Opcional**: Aplicar optimizaciones de registros

---

**Última actualización**: 2026-02-11  
**Autor**: Desarrollo incremental con Cline  
**Hardware**: Arduino Mega 2560 Rev3 + L298N + Encoders  
**Estado**: ✅ PRODUCCIÓN
