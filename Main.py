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

while sim.getSimulationTime() < 180:

        lectura_frontal   = sim.readProximitySensor(sensor_frontal)
        lectura_derecha   = sim.readProximitySensor(sensor_derecho)
        lectura_izquierda = sim.readProximitySensor(sensor_izquierdo)

        distancia_frontal   = lectura_frontal[1]   if lectura_frontal[0]   else 5.0
        distancia_derecha   = lectura_derecha[1]   if lectura_derecha[0]   else 5.0
        distancia_izquierda = lectura_izquierda[1] if lectura_izquierda[0] else 5.0

        tiempo_actual = sim.getSimulationTime()

        if estado_actual == AVANZAR_RECTO:

            error_lateral = DISTANCIA_OBJETIVO - distancia_izquierda
            correccion = max(min(error_lateral, 0.6), -0.6)

            sim.setJointTargetVelocity(
                motor_izquierdo, VELOCIDAD_AVANCE - correccion
            )
            sim.setJointTargetVelocity(
                motor_derecho, VELOCIDAD_AVANCE + correccion
            )

            if distancia_frontal < DISTANCIA_PARED:

                sim.setJointTargetVelocity(motor_izquierdo, 0)
                sim.setJointTargetVelocity(motor_derecho, 0)
                sim.step()

                if distancia_izquierda < DISTANCIA_PARED and distancia_derecha < DISTANCIA_PARED:
                    estado_actual = GIRAR_180_GRADOS
                elif distancia_derecha < DISTANCIA_PARED:
                    estado_actual = GIRAR_IZQUIERDA
                elif distancia_izquierda < DISTANCIA_PARED:
                    estado_actual = GIRAR_DERECHA
                else:
                    estado_actual = GIRAR_IZQUIERDA

                tiempo_inicio_giro = tiempo_actual

        elif estado_actual == GIRAR_IZQUIERDA:

            sim.setJointTargetVelocity(motor_derecho,  VELOCIDAD_GIRO)
            sim.setJointTargetVelocity(motor_izquierdo, -VELOCIDAD_GIRO)

            if tiempo_actual - tiempo_inicio_giro > TIEMPO_GIRO_90_GRADOS + TIEMPO_SOBREGIRO:
                estado_actual = AVANZAR_RECTO

        elif estado_actual == GIRAR_DERECHA:

            sim.setJointTargetVelocity(motor_derecho, -VELOCIDAD_GIRO)
            sim.setJointTargetVelocity(motor_izquierdo,  VELOCIDAD_GIRO)

            if tiempo_actual - tiempo_inicio_giro > TIEMPO_GIRO_90_GRADOS + TIEMPO_SOBREGIRO:
                estado_actual = AVANZAR_RECTO

        elif estado_actual == GIRAR_180_GRADOS:

            sim.setJointTargetVelocity(motor_derecho, -1.6)
            sim.setJointTargetVelocity(motor_izquierdo,  1.6)

            if tiempo_actual - tiempo_inicio_giro > 3.0:
             estado_actual = AVANZAR_RECTO

        sim.step()


