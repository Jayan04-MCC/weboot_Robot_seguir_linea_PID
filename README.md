# Robot Seguidor de Línea - E-puck

Proyecto de simulación en Webots de un robot E-puck que sigue una línea negra usando control PID.

## Descripción General

Este proyecto implementa un algoritmo de seguimiento de línea usando el robot E-puck de Webots. El robot utiliza sensores de suelo (ground sensors) para detectar una pista negra sobre un fondo blanco y ajusta su trayectoria en tiempo real mediante un controlador PID.

## Características de la Pista

- **Forma**: Pista circular cerrada (anillo)
- **Ancho de la pista**: 6 cm (0.06 m)
- **Ancho del robot E-puck**: 7 cm (0.07 m)
- **Dificultad**: La pista es ligeramente más delgada que el robot, lo que hace el seguimiento más desafiante
- **Radio exterior**: 1.0 m
- **Radio interior**: 0.94 m
- **Color**: Negro oscuro (RGB: 0.05, 0.05, 0.05)

## Componentes del Sistema

### Hardware (Simulado)

#### Robot E-puck
- **Motores**: 2 motores diferenciales (ruedas izquierda y derecha)
- **Sensores**: 3 sensores de suelo infrarrojos (ground sensors)
  - `gs0`: Sensor izquierdo
  - `gs1`: Sensor central
  - `gs2`: Sensor derecho
  - Solo se utilizan gs0 y gs2 para el control

#### Sensores de Suelo
Los sensores de suelo del E-puck funcionan mediante reflexión de luz infrarroja:
- **Superficie blanca/clara**: ~1000 (alta reflexión)
- **Superficie negra**: ~300-400 (baja reflexión)
- **Rango de normalización**: [300, 1000]

## Algoritmo de Control

### Controlador PID

El robot utiliza un controlador PID (Proporcional-Integral-Derivativo) para mantener el seguimiento de la línea.

#### Parámetros PID

```cpp
Kp = 3.5    // Ganancia proporcional
Ki = 0.0005 // Ganancia integral
Kd = 1.2    // Ganancia derivativa
```

#### Componentes del PID

1. **Proporcional (P)**: Reacciona al error actual
   - Calcula la diferencia entre los sensores izquierdo y derecho
   - Error positivo → el sensor derecho ve más negro → girar a la izquierda
   - Error negativo → el sensor izquierdo ve más negro → girar a la derecha

2. **Integral (I)**: Corrige errores acumulados en el tiempo
   - Acumula el error histórico para eliminar desviaciones constantes
   - Limitada a [-50, 50] para evitar "windup" (saturación)

3. **Derivativa (D)**: Suaviza los cambios bruscos
   - Calcula la tasa de cambio del error
   - Previene oscilaciones y sobrepaso

### Cálculo del Error

```cpp
// Normalizar valores de sensores a [0, 1]
// 1 = negro (línea), 0 = blanco (fondo)
leftNorm = 1.0 - ((leftValue - 300) / (1000 - 300))
rightNorm = 1.0 - ((rightValue - 300) / (1000 - 300))

// Error = diferencia entre sensores
error = rightNorm - leftNorm
```

### Recuperación de Línea Perdida

Si ambos sensores detectan blanco (línea perdida):
```cpp
if (leftNorm < 0.1 && rightNorm < 0.1) {
    // Mantener última dirección conocida amplificada
    error = lastValidError * 1.2
}
```

### Control de Velocidad

```cpp
baseSpeed = 4.0         // Velocidad base de avance
maxCorrection = 3.5     // Corrección máxima permitida

// Calcular velocidades de las ruedas
leftSpeed = baseSpeed - pidOutput
rightSpeed = baseSpeed + pidOutput

// Limitar al rango [0, MAX_SPEED=6.28]
```

## Estructura del Código

### Archivo Principal: `line_follower_cpp.cpp`

```
1. Inicialización
   - Crear instancia del robot
   - Habilitar sensores de suelo (gs0, gs2)
   - Configurar motores en modo velocidad

2. Variables del PID
   - integral: Acumulador del error
   - previousError: Error del paso anterior
   - lastValidError: Última dirección válida detectada

3. Bucle Principal (TIME_STEP = 64ms)
   a) Leer valores de sensores
   b) Normalizar valores [0, 1]
   c) Calcular error
   d) Calcular componentes P, I, D
   e) Generar salida PID
   f) Ajustar velocidades de motores
   g) Imprimir información de debug
```

### Archivos del Proyecto

```
my_line_robot_projec/
├── controllers/
│   └── line_follower_cpp/
│       ├── line_follower_cpp.cpp    # Código del controlador
│       ├── Makefile                  # Archivo de compilación
│       └── build/
│           └── release/
│               └── line_follower_cpp.exe
├── worlds/
│   └── empty.wbt                     # Mundo de simulación
└── README.md                         # Este archivo
```

## Flujo de Datos

```
Sensores (gs0, gs2)
    ↓
Lectura de valores [300-1000]
    ↓
Normalización [0-1]
    ↓
Cálculo de error (rightNorm - leftNorm)
    ↓
Controlador PID
    ├── P = Kp × error
    ├── I = Ki × ∫error dt
    └── D = Kd × (derror/dt)
    ↓
Salida PID (limitada)
    ↓
Velocidades de motores
    ├── leftSpeed = baseSpeed - pidOutput
    └── rightSpeed = baseSpeed + pidOutput
    ↓
Actuadores (motores)
```

## Funcionamiento Detallado

### 1. Detección de Línea

Los sensores de suelo leen continuamente la superficie bajo el robot:
- Si el sensor izquierdo ve negro y el derecho ve blanco → el robot está desviado a la izquierda
- Si el sensor derecho ve negro y el izquierdo ve blanco → el robot está desviado a la derecha
- Si ambos ven negro → el robot está centrado en la línea
- Si ambos ven blanco → el robot perdió la línea

### 2. Ajuste de Dirección

El controlador PID convierte el error en un ajuste de velocidad:
- **Error positivo** (desviación a la derecha):
  - Reduce velocidad de rueda izquierda
  - Aumenta velocidad de rueda derecha
  - Resultado: giro a la izquierda

- **Error negativo** (desviación a la izquierda):
  - Aumenta velocidad de rueda izquierda
  - Reduce velocidad de rueda derecha
  - Resultado: giro a la derecha

### 3. Estabilización

La componente derivativa del PID suaviza los movimientos:
- Detecta cambios rápidos en el error
- Aplica corrección preventiva
- Evita oscilaciones y sobrepaso

## Compilación y Ejecución

### Compilar

1. Abrir Webots
2. Menú: **Build → Build** (o **Ctrl+F7**)
3. Verificar que no hay errores en la consola

### Ejecutar

1. Abrir el mundo: `worlds/empty.wbt`
2. Presionar el botón de **Play** en Webots
3. El robot comenzará a seguir la línea automáticamente

### Detener

- Presionar el botón de **Pause** o **Reset**

## Salida de Debug

El programa imprime información en tiempo real:

```
L:850 R:350 | Error:-0.714 | P:-2.499 I:-0.003 D:-1.234 | PID:-3.5 | Vel L:7.5 R:0.5
```

- **L**: Valor del sensor izquierdo (bruto)
- **R**: Valor del sensor derecho (bruto)
- **Error**: Error calculado [-1, 1]
- **P**: Componente proporcional
- **I**: Componente integral
- **D**: Componente derivativa
- **PID**: Salida total del controlador
- **Vel L/R**: Velocidades aplicadas a las ruedas

## Ajuste de Parámetros

### Para Mayor Velocidad
```cpp
baseSpeed = 5.0;  // Aumentar velocidad base
```

### Para Mayor Agresividad en Curvas
```cpp
Kp = 4.0;  // Aumentar ganancia proporcional
```

### Para Mayor Estabilidad
```cpp
Kd = 1.5;  // Aumentar ganancia derivativa
```

### Para Corregir Deriva Constante
```cpp
Ki = 0.001;  // Aumentar ganancia integral
```

## Solución de Problemas

### El robot no detecta la línea
- Verificar que se usan `E-puckGroundSensors` en el archivo .wbt
- Verificar que los sensores se llaman "gs0" y "gs2"
- Comprobar que la pista es de color oscuro (diffuseColor < 0.1)

### El robot oscila mucho
- Reducir Kp (ganancia proporcional)
- Aumentar Kd (ganancia derivativa)
- Reducir baseSpeed

### El robot pierde la línea en curvas
- Aumentar Kp para reacción más rápida
- Reducir baseSpeed para más control
- Ajustar el multiplicador de recuperación (1.2)

### El robot va muy lento
- Aumentar baseSpeed
- Aumentar maxCorrection
- Verificar que MAX_SPEED sea suficiente (6.28)

## Mejoras Futuras

1. Usar el tercer sensor (gs1) para mejor detección
2. Implementar velocidad adaptativa según la curvatura
3. Agregar detección de intersecciones
4. Implementar contador de vueltas
5. Optimizar parámetros PID mediante algoritmos genéticos

## Requisitos

- **Webots**: R2025a o superior
- **Compilador C++**: Incluido con Webots
- **Sistema Operativo**: Windows, Linux o macOS


## Autor

Jayan Michael Caceres Cuba
---

## Referencias

- [Documentación de Webots](https://cyberbotics.com/doc/guide/index)
- [E-puck Robot](https://www.gctronic.com/doc/index.php/E-Puck)
- [Control PID](https://en.wikipedia.org/wiki/PID_controller)
