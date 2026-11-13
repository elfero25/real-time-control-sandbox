"""
Módulo de Control y Simulación Gráfica - Péndulo Invertido
Autor: Agustín Ezequiel Feroci
Descripción: Simulación en lazo cerrado con realimentación de estados (u = -Kx)
             y visualización de variables de estado y esfuerzo de control.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt


class StateSpaceController:
    def __init__(self, A, B, C, K):
        self.A = np.array(A, dtype=float)
        self.B = np.array(B, dtype=float)
        self.C = np.array(C, dtype=float)
        self.K = np.array(K, dtype=float)

    def compute_control_action(self, x, u_max=15.0):
        """Calcula la acción de control u = -Kx con saturación física de actuador."""
        u = float(-np.dot(self.K, x))
        return np.clip(u, -u_max, u_max)


def build_inverted_pendulum_model():
    """Parámetros físicos nominales y matrices linealizadas del péndulo invertido."""
    M = 0.5    # Masa del carro (kg)
    m = 0.2    # Masa del péndulo (kg)
    b = 0.1    # Fricción del carro (N/m/s)
    I = 0.006  # Inercia del péndulo (kg*m^2)
    g = 9.81   # Gravedad (m/s^2)
    l = 0.3    # Distancia al centro de masa (m)

    p = I * (M + m) + M * m * (l ** 2)

    A = np.array([
        [0, 1, 0, 0],
        [0, -(I + m * l**2) * b / p, (m**2 * g * l**2) / p, 0],
        [0, 0, 0, 1],
        [0, -m * l * b / p, m * g * l * (M + m) / p, 0]
    ])

    B = np.array([
        [0],
        [(I + m * l**2) / p],
        [0],
        [m * l / p]
    ])

    C = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0]
    ])

    return A, B, C


def design_lqr(A, B, Q, R):
    """Calcula la ganancia óptima K resolviendo la ecuación algebraica de Riccati."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K.flatten()


def run_simulation():
    A, B, C = build_inverted_pendulum_model()

    # Matrices de ponderación LQR (penalizamos posición y ángulo del péndulo)
    Q = np.diag([10.0, 1.0, 100.0, 1.0])
    R = np.array([[0.1]])

    K = design_lqr(A, B, Q, R)
    controller = StateSpaceController(A, B, C, K)
    print(f"Ganancia LQR calculada K = {np.round(K, 3)}")

    # Estado inicial: carro en 0 m, péndulo inclinado 15 grados (0.2618 rad)
    x0 = [0.0, 0.0, np.deg2rad(15), 0.0]
    t_span = (0, 5.0)
    t_eval = np.linspace(t_span[0], t_span[1], 500)

    def closed_loop_dynamics(t, x):
        # Perturbación externa (impulso) a los 2.5 segundos
        disturbance = 4.0 if 2.5 <= t <= 2.6 else 0.0
        u = controller.compute_control_action(x, u_max=20.0)
        dx = A @ x + B.flatten() * (u + disturbance)
        return dx

    sol = solve_ivp(closed_loop_dynamics, t_span, x0, t_eval=t_eval)
    u_hist = [controller.compute_control_action(sol.y[:, i], u_max=20.0) for i in range(len(sol.t))]

    # Generación de gráficos
    plt.style.use("seaborn-v0_8-darkgrid")
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("Simulación de Control LQR - Péndulo Invertido en Lazo Cerrado", fontsize=13, fontweight="bold")

    # 1. Ángulo del péndulo
    axs[0].plot(sol.t, np.rad2deg(sol.y[2]), color="#e63946", linewidth=2, label="Ángulo $\\theta$ (deg)")
    axs[0].axvline(2.5, color="gray", linestyle="--", alpha=0.7, label="Perturbación (t = 2.5 s)")
    axs[0].set_ylabel("Ángulo [°]")
    axs[0].legend(loc="upper right")

    # 2. Posición del carro
    axs[1].plot(sol.t, sol.y[0], color="#1d3557", linewidth=2, label="Posición carro $x$ (m)")
    axs[1].axvline(2.5, color="gray", linestyle="--", alpha=0.7)
    axs[1].set_ylabel("Posición [m]")
    axs[1].legend(loc="upper right")

    # 3. Acción de control (Fuerza)
    axs[2].plot(sol.t, u_hist, color="#2a9d8f", linewidth=2, label="Fuerza de control $u$ (N)")
    axs[2].axhline(20.0, color="black", linestyle=":", alpha=0.6, label="Límite actuador (±20 N)")
    axs[2].axhline(-20.0, color="black", linestyle=":", alpha=0.6)
    axs[2].set_ylabel("Fuerza [N]")
    axs[2].set_xlabel("Tiempo [s]")
    axs[2].legend(loc="upper right")

    plt.tight_layout()
    plt.savefig("docs/lqr_response.png", dpi=200)
    print("Gráfico guardado en: docs/lqr_response.png")
    plt.show()


if __name__ == "__main__":
    run_simulation()