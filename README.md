  # Real-Time Control & Dynamic Systems Sandbox ⚙️🤖

Este repositorio contiene un banco de pruebas experimental enfocado en la implementación y validación de algoritmos de control en tiempo real, modelado de sistemas dinámicos y adquisición de datos.

El objetivo principal de este proyecto es tender un puente entre la teoría de control automático (espacio de estados, lazos PID, estabilidad) y la implementación práctica bajo restricciones del mundo real (tiempos de muestreo, saturación de actuadores y ruido en lazos de retroalimentación).

---

## 🚀 Estructura del Repositorio

```text
real-time-control-sandbox/
│
├── src/                        # Código fuente principal (Python / adquisición en vivo)
│   ├── controller.py           # Lógica de control (LQR / PID / Compensadores)
│   └── acquisition.py          # Interfaz de adquisición de datos y telemetría
│
├── matlab-pid-lab/             # Simulador interactivo PID en tiempo real (MATLAB R2016b+)
│   └── PID_Control_Lab_2016b.m # Interfaz gráfica, dinámica RK4 y lógica Anti-Windup
│
├── simulations/                # Modelos y scripts de simulación (MATLAB / Simulink)
│   └── inverted_pendulum_model.m # Simulación dinámica en espacio de estados
│
├── docs/                       # Documentación, diagramas y animaciones
│   └── system_architecture.md
│
├── requirements.txt            # Dependencias del proyecto
└── LICENSE                     # Licencia MIT
```

---

## 🛠️ Tecnologías y Herramientas Utilizadas
- **Lenguajes:** Python, MATLAB / Simulink.
- **Librerías Python:** NumPy, SciPy, Matplotlib, Control Systems Library.
- **Control:** Espacio de estados (LQR / Observadores), lazos PID con Anti-Windup y derivada sobre la medición.

---

## 📊 Demostración y Pruebas

### 1. Simulador Interactivo PID (`simulations/PID_Control_Lab_2016b.m`)
Entorno gráfico en tiempo real desarrollado en **MATLAB (compatible con R2016b+)** para analizar de forma visual el comportamiento de un controlador PID sobre un sistema **masa-resorte-amortiguador** ($m\ddot{x} + b\dot{x} + kx = u$) con saturación de actuador:
- **Sintonización en vivo:** Ajuste dinámico de ganancias ($K_p$, $K_i$, $K_d$) y referencia (*Setpoint*) con integración numérica **RK4**.
- **Implementación práctica:** Derivada aplicada sobre la variable de proceso ($-K_d\dot{y}$) para evitar *derivative kick* y lógica **Anti-Windup (Clamping)** activable.
- **Análisis de perturbaciones:** Inyección de impulsos mecánicos externos y escenarios preconfigurados (*Solo P*, *Subamortiguado*, *Óptimo*).

Para ejecutarlo desde MATLAB:
```matlab
cd simulations
PID_Control_Lab_2016b
```

### 2. Métricas de Desempeño Evaluadas
El banco de pruebas evalúa la respuesta ante cambios de referencia escalón y perturbaciones externas midiendo:
1. **Tiempo de establecimiento ($t_s$)**
2. **Sobrepaso máximo ($M_p$)**
3. **Error en estado estacionario ($e_{ss}$)**

---

## 👤 Autor
**Agustín Ezequiel Feroci**  
*Ingeniería Electrónica | Control y Automatización*
