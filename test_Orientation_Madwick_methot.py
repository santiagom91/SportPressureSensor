import time
import board
import busio
import adafruit_bno055
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MadgwickAHRS:
    def __init__(self, sample_period=0.01, beta=0.1):
        self.sample_period = sample_period
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion of sensor frame relative to auxiliary frame
    
    def update(self, gyro, accel):
        q = self.q
        
        gyro = np.deg2rad(gyro)  # Convert gyro measurements from degrees to radians per second
        
        # Normalize accelerometer measurements
        accel /= np.linalg.norm(accel)
        
        # Compute quaternion derivative
        q_dot = 0.5 * self.quaternion_multiply(q, np.hstack([0, gyro]))
        
        # Compute estimated gravity direction
        h = self.quaternion_multiply(self.quaternion_multiply(q, np.array([0, *accel])), self.quaternion_conjugate(q))
        gravity = np.array([0, 0, 1])
        
        # Compute gradient
        F = np.array([
            [2 * (q[1]*q[3] - q[0]*q[2]) - accel[0]],
            [2 * (q[0]*q[1] + q[2]*q[3]) - accel[1]],
            [2 * (0.5 - q[1]**2 - q[2]**2) - accel[2]]
        ])
        
        J = np.array([
            [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
            [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
            [0, -4 * q[1], -4 * q[2], 0]
        ])
        
        # Compute gradient descent step
        step = np.dot(J.T, F)
        
        # Normalize step
        step /= np.linalg.norm(step)
        
        # Update quaternion
        q_dot -= self.beta * step.T
        
        # Integrate quaternion
        q += q_dot * self.sample_period
        self.q = q / np.linalg.norm(q)
        
    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def quaternion_conjugate(self, q):
        w, x, y, z = q
        return np.array([w, -x, -y, -z])

# Inizializza l'interfaccia I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Inizializza il sensore BNO055
sensor = adafruit_bno055.BNO055()

# Inizializza il filtro di Madgwick
madgwick = MadgwickAHRS()

# Parametri per la visualizzazione del grafico 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Inizializzazione dei dati per il tracciamento
x_data, y_data, z_data = [], [], []

# Funzione per l'aggiornamento del grafico
def update_plot(i):
    global x_data, y_data, z_data
    
    # Ottieni i dati di accelerazione, angoli di orientamento e velocità angolare dal sensore BNO055
    accel_x, accel_y, accel_z = sensor.acceleration
    gyro_x, gyro_y, gyro_z = sensor.gyro
    roll, pitch, yaw = sensor.euler
    
    # Esegui il filtro di Madgwick per ottenere un'accurata stima dell'orientamento
    madgwick.update([gyro_x, gyro_y, gyro_z], [accel_x, accel_y, accel_z])
    roll_madgwick, pitch_madgwick, yaw_madgwick = madgwick.q[1:]
    
    # Aggiorna i dati di tracciamento
    x_data.append(np.cos(roll_madgwick) * np.cos(pitch_madgwick))
    y_data.append(np.sin(roll_madgwick) * np.cos(pitch_madgwick))
    z_data.append(np.sin(pitch_madgwick))
    
    # Aggiorna il grafico
    ax.clear()
    ax.plot(x_data, y_data, z_data, color='b')
    ax.scatter(x_data[-1], y_data[-1], z_data[-1], color='r')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

# Esegui l'aggiornamento del grafico in modo continuo
ani = plt.FuncAnimation(fig, update_plot, interval=50)
plt.show()


