# Real-Time Control & Dynamic Systems Sandbox ⚙️🤖

Este repositorio contiene un banco de pruebas experimental enfocado en la implementación y validación de algoritmos de control en tiempo real, modelado de sistemas dinámicos y adquisición de datos.

El objetivo principal de este proyecto es tender un puente entre la teoría de control automático (espacio de estados, lazos PID, estabilidad) y la implementación práctica bajo restricciones del mundo real (tiempos de muestreo, saturación de actuadores y ruido en lazos de retroalimentación).

---

## 🚀 Estructura del Repositorio

```text
real-time-control-sandbox/
│
├── src/                  # Código fuente principal (Python / adquisición en vivo)
│   ├── controller.py     # Lógica de control (LQR / PID / Compensadores)
│   └── acquisition.py    # Interfaz de adquisición de datos y telemetría
│
├── simulations/          # Modelos y scripts de simulación (MATLAB / Simulink)
│   └── inverted_pendulum_model.m # Simulación dinámica del sistema
│
├── docs/                 # Documentación, diagramas y esquemáticos
│   └── system_architecture.md
│
├── requirements.txt      # Dependencias del proyecto
└── LICENSE               # Licencia MIT
```

---

## 🛠️ Tecnologías y Herramientas Utilizadas
* **Lenguajes:** Python, MATLAB / Simulink.
* **Librerías Python:** NumPy, SciPy, Matplotlib, Control Systems Library.
* **Control:** Espacio de estados, diseño de observadores, sintonización de lazos.

---

## 📊 Demostración y Pruebas
El sistema evalúa la respuesta ante perturbaciones escalón y ruido estocástico en el sensor, midiendo métricas clave de desempeño:
1. **Tiempo de establecimiento ($t_s$)**
2. **Sobrepaso máximo ($M_p$)**
3. **Error en estado estacionario ($e_{ss}$)**

---

## 👤 Autor
**Agustín Ezequiel Feroci**  
*Ingeniería Electrónica | Control y Automatización*
