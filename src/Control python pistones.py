import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Constantes y parámetros del sistema
L = 1.0  # Longitud del péndulo (m)
M = 1.0  # Masa en la parte superior del péndulo (kg)
m = 5.0  # Masa de la base (kg)
g = 9.81  # Aceleración de la gravedad (m/s^2)
mu = 0.1  # Coeficiente de fricción en la base
Kp = 10.0  # Ganancia proporcional del controlador PID
Ki = 5.0  # Ganancia integral del controlador PID
Kd = 2.0  # Ganancia derivativa del controlador PID

# Modelo del sistema
def pendulo_invertido(t, y):
    x, x_dot, alpha, alpha_dot, theta, theta_dot, int_error = y
    
    # Controlador PID
    error = -alpha
    int_error += error * dt
    der_error = alpha_dot
    F = Kp * error + Ki * int_error + Kd * der_error
    
    # Limitar la fuerza del actuador
    F = np.clip(F, -50, 50)
    
    # Ecuaciones del modelo (simplificadas y linealizadas)
    x_ddot = F / m
    alpha_ddot = (M * L * theta_dot**2 - F) / (m + M)
    theta_ddot = -(M * g * np.sin(theta) + F * np.cos(theta)) / (M * L)

    return [x_dot, x_ddot, alpha_dot, alpha_ddot, theta_dot, theta_ddot, int_error]

# Condiciones iniciales
x0, x_dot0, alpha0, alpha_dot0, theta0, theta_dot0, int_error0 = 0, 0, 0, 0, np.pi / 6, 0, 0
y0 = [x0, x_dot0, alpha0, alpha_dot0, theta0, theta_dot0, int_error0]

# Parámetros de simulación
t_span = (0, 10)
dt = 0.01
t_eval = np.arange(t_span[0], t_span[1], dt)

# Resolver las ecuaciones diferenciales del sistema
sol = solve_ivp(pendulo_invertido, t_span, y0, t_eval=t_eval)

# Graficar la posición de la base (alpha) y el ángulo del péndulo (theta) en el tiempo
plt.figure(figsize=(12, 6))
plt.plot(sol.t, sol.y[2], label="Posición de la base (alpha)")
plt.plot(sol.t, sol.y[4], label="Ángulo del péndulo (theta)")
plt.xlabel("Tiempo (s)")
plt.ylabel("Ángulo (rad)")
plt.legend()
plt.grid()
plt.show()
