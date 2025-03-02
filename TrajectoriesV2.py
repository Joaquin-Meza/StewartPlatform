import numpy as np
import matplotlib.pyplot as plt

# Parámetros
T = 1  # Duración de un ciclo (s)
num_cycles = 5  # Número de ciclos
T_apoyo = 0.6 * T  # Duración de la fase de apoyo
T_oscilacion = 0.4 * T  # Duración de la fase de oscilación
z_max = 50  # Altura máxima durante la oscilación (mm)
theta_max = 15  # Máxima rotación (grados)
v_apoyo = 100  # Velocidad en la fase de apoyo (mm/s)
v_oscilacion = 50  # Velocidad en la fase de oscilación (mm/s)

# Tiempo para un solo ciclo
t_apoyo = np.linspace(0, T_apoyo, int(100 * T_apoyo))
t_oscilacion = np.linspace(0, T_oscilacion, int(100 * T_oscilacion))

# Trayectorias de apoyo
X_apoyo = v_apoyo * t_apoyo  # Movimiento lineal hacia adelante
Z_apoyo = np.zeros_like(t_apoyo)  # Sin elevación durante apoyo
Theta_apoyo = theta_max * np.sin(2 * np.pi * t_apoyo / T_apoyo)  # Rotación suave

# Trayectorias de oscilación
X_oscilacion = X_apoyo[-1] + v_oscilacion * t_oscilacion  # Movimiento hacia adelante
Z_oscilacion = z_max * np.sin(np.pi * t_oscilacion / T_oscilacion)  # Elevación sinusoidal
Theta_oscilacion = theta_max * np.sin(np.pi * t_oscilacion / T_oscilacion)  # Rotación

# Combinar fases para un solo ciclo
X_single = np.concatenate([X_apoyo, X_oscilacion])
Z_single = np.concatenate([Z_apoyo, Z_oscilacion])
Theta_single = np.concatenate([Theta_apoyo, Theta_oscilacion])
t_single = np.concatenate([t_apoyo, T_apoyo + t_oscilacion])

# Repetir ciclos
t_total = np.tile(t_single, num_cycles) + np.repeat(np.arange(num_cycles) * T, len(t_single))
X_total = np.tile(X_single, num_cycles)
Z_total = np.tile(Z_single, num_cycles)
Theta_total = np.tile(Theta_single, num_cycles)

# Visualización de la trayectoria de un ciclo
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t_total, X_total, label='X (mm)')
plt.title('Trayectorias en el Ciclo de Marcha')
plt.ylabel('Posición X (mm)')
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(t_total, Z_total, label='Z (mm)', color='orange')
plt.ylabel('Posición Z (mm)')
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(t_total, Theta_total, label='Theta (grados)', color='green')
plt.ylabel('Rotación Theta (°)')
plt.xlabel('Tiempo (s)')
plt.grid()

plt.tight_layout()
plt.show()


# Desfase temporal de medio ciclo
shift = len(t_single) // 2  # Desfase de medio ciclo

# Trayectorias de la segunda plataforma (desfasadas)
X_Platform1 = X_total
Z_Platform1 = Z_total
Theta_Platform1 = Theta_total

X_Platform2 = np.roll(X_total, shift)
Z_Platform2 = np.roll(Z_total, shift)
Theta_Platform2 = np.roll(Theta_total, shift)

# Visualización de ambas plataformas
plt.figure(figsize=(10, 6))
plt.plot(t_total, X_Platform1, label='Plataforma 1 - X (mm)', color='blue')
plt.plot(t_total, X_Platform2, label='Plataforma 2 - X (mm)', color='red', linestyle='--')
plt.title('Posición X de Ambas Plataformas')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición X (mm)')
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(t_total, Z_Platform1, label='Plataforma 1 - Z (mm)', color='blue')
plt.plot(t_total, Z_Platform2, label='Plataforma 2 - Z (mm)', color='red', linestyle='--')
plt.title('Posición Z de Ambas Plataformas')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición Z (mm)')
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(t_total, Theta_Platform1, label='Plataforma 1 - Theta (°)', color='blue')
plt.plot(t_total, Theta_Platform2, label='Plataforma 2 - Theta (°)', color='red', linestyle='--')
plt.title('Rotación Theta de Ambas Plataformas')
plt.xlabel('Tiempo (s)')
plt.ylabel('Rotación Theta (°)')
plt.legend()
plt.grid()
plt.show()


import pandas as pd

# Crear DataFrame con las trayectorias
data = pd.DataFrame({
    'Tiempo': t_total,
    'X_Plataforma1': X_Platform1,
    'Z_Plataforma1': Z_Platform1,
    'Theta_Plataforma1': Theta_Platform1,
    'X_Plataforma2': X_Platform2,
    'Z_Plataforma2': Z_Platform2,
    'Theta_Plataforma2': Theta_Platform2,
})

# Guardar como CSV
#data.to_csv('trayectorias_plataformas.csv', index=False)

# Guardar como archivo MAT
#from scipy.io import savemat
"""
savemat('trayectorias_plataformas.mat', {
    'Tiempo': t_total,
    'X_Plataforma1': X_Platform1,
    'Z_Plataforma1': Z_Platform1,
    'Theta_Plataforma1': Theta_Platform1,
    'X_Plataforma2': X_Platform2,
    'Z_Plataforma2': Z_Platform2,
    'Theta_Plataforma2': Theta_Platform2,
})
"""