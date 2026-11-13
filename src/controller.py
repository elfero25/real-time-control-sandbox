"""
Módulo de Control - Banco de Pruebas Dinámicas
Autor: Agustín Ezequiel Feroci
Descripción: Implementación base de un lazo de control retroalimentado (Estado-Espacio / PID).
"""

import numpy as np
import matplotlib.pyplot as plt

class StateSpaceController:
    def __init__(self, A, B, C, K):
        """
        Inicializa el controlador por espacio de estados.
        x_dot = A*x + B*u
        y = C*x
        u = -K*x
        """
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.K = np.array(K)

    def compute_control_action(self, x):
        """Calcula la acción de control u = -Kx"""
        u = -np.dot(self.K, x)
        return u

if __name__ == "__main__":
    print("Inicializando banco de pruebas de control...")
    A = [[0, 1], [0, -2]]
    B = [[0], [1]]
    C = [[1, 0]]
    K = [10, 5]
    
    controller = StateSpaceController(A, B, C, K)
    print("Controlador inicializado correctamente con ganancia K:", K)
