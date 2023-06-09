# Importar librerías necesarias
import matplotlib.pyplot as plt
import numpy as np

# Parámetros iniciales
x0 = 0  # Posición inicial
v0 = 0  # Velocidad inicial
m = 1   # Masa del objeto
inclinacion_maxima = np.radians(20)  # Inclinación máxima permitida en radianes
velocidad_maxima = 10 * 1000 / 3600  # Velocidad máxima en m/s

# Tiempo
t = np.linspace(0, 10, 100)  # Rango de tiempo de 0 a 10 segundos con 100 puntos de datos

# Variable de inclinación para experimentar
inclinacion = np.radians(10)  # Inclinación del palo en radianes

# Calcular posición y velocidad
x = []
v = []
for tiempo in t:
    # Verificar si se excede la inclinación máxima permitida
    if abs(inclinacion) > inclinacion_maxima:
        print("El objeto se ha caído.")
        break

    # Calcular aceleración horizontal adicional proporcional a la inclinación
    aceleracion_horizontal = 0.5 * inclinacion

    # Calcular la aceleración total considerando la aceleración horizontal adicional y la gravedad
    aceleracion_total = -9.8 + aceleracion_horizontal

    # Calcular la velocidad y verificar si se excede la velocidad máxima permitida
    velocidad = v0 + aceleracion_total * tiempo
    if abs(velocidad) > velocidad_maxima:
        print("El objeto ha alcanzado la velocidad máxima y se ha detenido.")
        velocidad = np.sign(velocidad) * velocidad_maxima

    # Calcular la posición
    posicion = x0 + v0 * tiempo + (1/2) * aceleracion_total * tiempo**2

    x.append(posicion)
    v.append(velocidad)

# Gráfico de posición en función del tiempo
plt.figure(1)
plt.plot(t, x)
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición (m)')
plt.title('Posición del objeto en función del tiempo')

# Gráfico de velocidad en función del tiempo
plt.figure(2)
plt.plot(t, v)
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad (m/s)')
plt.title('Velocidad del objeto en función del tiempo')

# Mostrar los gráficos
plt.show()
