"""
Animación en Tiempo Real - Control LQR de Péndulo Invertido
Autor: Agustín Ezequiel Feroci
Descripción: Genera una animación física del carro y el péndulo estabilizados
             mediante realimentación de estados con rechazo de perturbaciones.
"""

import os
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle


def build_inverted_pendulum_model():
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

    return A, B, l


def design_lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K.flatten()


def run_animation():
    os.makedirs("docs", exist_ok=True)
    A, B, l = build_inverted_pendulum_model()
    L_visual = 2.0 * l  # Longitud visual de la barra (0.6 m)

    # Diseño de ganancia óptima LQR
    Q = np.diag([15.0, 1.0, 120.0, 1.0])
    R = np.array([[0.1]])
    K = design_lqr(A, B, Q, R)

    # Condición inicial: 18 grados de inclinación; perturbación a los 2.5 s
    x0 = [0.0, 0.0, np.deg2rad(18), 0.0]
    t_end = 5.0
    fps = 30
    t_eval = np.linspace(0, t_end, int(t_end * fps))

    def closed_loop_dynamics(t, x):
        disturbance = 6.0 if 2.5 <= t <= 2.65 else 0.0
        u = np.clip(float(-np.dot(K, x)), -20.0, 20.0)
        return A @ x + B.flatten() * (u + disturbance)

    sol = solve_ivp(closed_loop_dynamics, (0, t_end), x0, t_eval=t_eval)
    x_pos = sol.y[0]
    theta = sol.y[2]

    # Configuración de la figura
    plt.style.use("seaborn-v0_8-darkgrid")
    fig, (ax_anim, ax_plot) = plt.subplots(
        2, 1, figsize=(8, 6.5), gridspec_kw={"height_ratios": [1.3, 1]}
    )
    fig.suptitle(
        "Control LQR en Tiempo Real — Péndulo Invertido",
        fontsize=13,
        fontweight="bold"
    )

    # Panel superior: Escena física del carro y péndulo
    ax_anim.set_xlim(-1.2, 1.2)
    ax_anim.set_ylim(-0.2, 0.9)
    ax_anim.set_aspect("equal")
    ax_anim.set_ylabel("Altura [m]")
    ax_anim.axhline(0, color="#333333", linewidth=2)  # Riel

    cart_w, cart_h = 0.28, 0.14
    cart = Rectangle((-cart_w / 2, 0), cart_w, cart_h, fc="#1d3557", ec="black", zorder=3)
    ax_anim.add_patch(cart)

    rod, = ax_anim.plot([], [], color="#e63946", linewidth=3.5, zorder=4)
    bob, = ax_anim.plot([], [], "o", color="#e63946", markersize=11, zorder=5)
    status_text = ax_anim.text(
        0.02, 0.88, "", transform=ax_anim.transAxes,
        fontsize=10, fontweight="bold",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    # Panel inferior: Evolución temporal del ángulo
    ax_plot.set_xlim(0, t_end)
    ax_plot.set_ylim(-22, 22)
    ax_plot.set_xlabel("Tiempo [s]")
    ax_plot.set_ylabel("Ángulo θ [°]")
    ax_plot.axhline(0, color="gray", linestyle="--", linewidth=1)
    ax_plot.axvline(2.5, color="#f4a261", linestyle=":", linewidth=1.5, label="Perturbación externa")
    angle_line, = ax_plot.plot([], [], color="#e63946", linewidth=2, label="Ángulo θ (deg)")
    ax_plot.legend(loc="upper right")

    def init():
        rod.set_data([], [])
        bob.set_data([], [])
        angle_line.set_data([], [])
        status_text.set_text("")
        return cart, rod, bob, angle_line, status_text

    def update(frame):
        cx = x_pos[frame]
        ang = theta[frame]
        t_curr = t_eval[frame]

        # Posición del carro y extremo del péndulo
        cart.set_x(cx - cart_w / 2)
        pivot_x, pivot_y = cx, cart_h
        tip_x = pivot_x + L_visual * np.sin(ang)
        tip_y = pivot_y + L_visual * np.cos(ang)

        rod.set_data([pivot_x, tip_x], [pivot_y, tip_y])
        bob.set_data([tip_x], [tip_y])

        # Curva acumulada del ángulo
        angle_line.set_data(t_eval[:frame + 1], np.rad2deg(theta[:frame + 1]))

        state_label = "⚠️ PERTURBACIÓN" if 2.5 <= t_curr <= 2.75 else "✅ LQR ACTIVO"
        status_text.set_text(f"t = {t_curr:.2f} s | θ = {np.rad2deg(ang):+.1f}° | {state_label}")

        return cart, rod, bob, angle_line, status_text

    ani = FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=1000 / fps)

    plt.tight_layout()
    gif_path = "docs/pendulum_lqr.gif"
    print("Generando animación y guardando GIF (puede demorar unos segundos)...")
    ani.save(gif_path, writer=PillowWriter(fps=fps))
    print(f"GIF guardado exitosamente en: {gif_path}")
    plt.show()


if __name__ == "__main__":
    run_animation()