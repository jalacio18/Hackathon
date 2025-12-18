from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ESTADOS DEL ROBOT
AVANZAR_RECTO     = 1
GIRAR_IZQUIERDA   = 2
GIRAR_DERECHA     = 3
GIRAR_180_GRADOS  = 4

# PARÁMETROS DE MOVIMIENTO
VELOCIDAD_AVANCE = 2.0
VELOCIDAD_GIRO   = 2.2

# UMBRALES DE DISTANCIA (m)
DISTANCIA_PARED    = 0.30
DISTANCIA_PELIGRO  = 0.15
DISTANCIA_OBJETIVO = 0.45

# TIEMPOS DE GIRO (s)
TIEMPO_GIRO_90_GRADOS = 1.0
TIEMPO_SOBREGIRO      = 0.4
 def main():

    estado_actual = AVANZAR_RECTO

    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    # MOTORES
    motor_derecho   = sim.getObject("/PioneerP3DX/rightMotor")
    motor_izquierdo = sim.getObject("/PioneerP3DX/leftMotor")

    # SENSORES ULTRASÓNICOS
    sensor_frontal   = sim.getObject("/PioneerP3DX/ultrasonicSensor[3]")
    sensor_derecho   = sim.getObject("/PioneerP3DX/ultrasonicSensor[2]")
    sensor_izquierdo = sim.getObject("/PioneerP3DX/ultrasonicSensor[5]")

    tiempo_inicio_giro = sim.getSimulationTime()

