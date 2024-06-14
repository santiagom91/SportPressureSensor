import time
import board
import busio
import adafruit_bno055
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Inizializza il bus I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Crea una figura per il grafico 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Lista per memorizzare i dati degli spostamenti
displacements = []

# Funzione per aggiornare il grafico 3D
def update_plot():
    ax.clear()
    # Estrai i componenti x, y e z dei dati di spostamento
    x_values = [d[0] for d in displacements]
    y_values = [d[1] for d in displacements]
    z_values = [d[2] for d in displacements]
    # Aggiungi la linea tracciata nel grafico 3D
    ax.plot(x_values, y_values, z_values)
    # Etichette degli assi
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Titolo del grafico
    plt.title('Linea degli spostamenti')
    # Mostra il grafico
    plt.draw()
    plt.pause(0.01)

# Funzione per la calibrazione del sensore
'''def calibrate_sensor():
    print("Avvia la calibrazione del sensore...")
    while not sensor.calibrated:
        time.sleep(0.05)
    print("Calibrazione completata!")
    '''

# Avvia la calibrazione del sensore
#calibrate_sensor()

# Ciclo di acquisizione e aggiornamento del grafico
while True:
    # Leggi i dati di accelerazione dal sensore
    accel_x, accel_y, accel_z = sensor.acceleration
    # Aggiungi il vettore spostamento alla lista
    displacements.append((accel_x, accel_y, accel_z))
    # Limita la lunghezza della lista degli spostamenti a 100 campioni
    if len(displacements) > 100:
        displacements.pop(0)
    # Aggiorna il grafico 3D
    update_plot()
    # Attendi un po' tra una lettura e l'altra
    time.sleep(0.01)
