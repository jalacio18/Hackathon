from coppeliasim_zmqremoteapi_client import RemoteAPIClient

AVANZAR_RECTO   = 1
GIRAR_IZQUIERDA = 2
GIRAR_DERECHA   = 3
GIRAR_180       = 4

VELOCIDAD_AVANCE = 2.0
VELOCIDAD_GIRO   = 2.2

DISTANCIA_PARED   = 0.30
DISTANCIA_PELIGRO = 0.15
DIST_OBJETIVO     = 0.45

TIEMPO_GIRO_90   = 1.0
TIEMPO_SOBREGIRO = 0.4


def main():

    estado = AVANZAR_RECTO

    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    motor_d = sim.getObject("/PioneerP3DX/rightMotor")
    motor_i = sim.getObject("/PioneerP3DX/leftMotor")

    s_f = sim.getObject("/PioneerP3DX/ultrasonicSensor[3]")
    s_d = sim.getObject("/PioneerP3DX/ultrasonicSensor[2]")
    s_i = sim.getObject("/PioneerP3DX/ultrasonicSensor[5]")

    t0 = sim.getSimulationTime()

    while sim.getSimulationTime() < 180:

        f = sim.readProximitySensor(s_f)
        d = sim.readProximitySensor(s_d)
        i = sim.readProximitySensor(s_i)

        dist_f = f[1] if f[0] else 5.0
        dist_d = d[1] if d[0] else 5.0
        dist_i = i[1] if i[0] else 5.0

        t = sim.getSimulationTime()

        # AVANZAR RECTO
       
        if estado == AVANZAR_RECTO:

            error = DIST_OBJETIVO - dist_i
            corr = max(min(error, 0.6), -0.6)

            sim.setJointTargetVelocity(motor_i, VELOCIDAD_AVANCE - corr)
            sim.setJointTargetVelocity(motor_d, VELOCIDAD_AVANCE + corr)

            if dist_f < DISTANCIA_PARED:

                # 🔑 PARAR ANTES DE DECIDIR
                sim.setJointTargetVelocity(motor_i, 0)
                sim.setJointTargetVelocity(motor_d, 0)
                sim.step()

                if dist_i < DISTANCIA_PARED and dist_d < DISTANCIA_PARED:
                    estado = GIRAR_180

                elif dist_d < DISTANCIA_PARED:
                    estado = GIRAR_IZQUIERDA

                elif dist_i < DISTANCIA_PARED:
                    estado = GIRAR_DERECHA

                else:
                    estado = GIRAR_IZQUIERDA

                t0 = t

        # GIRAR IZQUIERDA

        elif estado == GIRAR_IZQUIERDA:

            sim.setJointTargetVelocity(motor_d,  VELOCIDAD_GIRO)
            sim.setJointTargetVelocity(motor_i, -VELOCIDAD_GIRO)

            if t - t0 > TIEMPO_GIRO_90 + TIEMPO_SOBREGIRO:
                estado = AVANZAR_RECTO

        # GIRAR DERECHA
        elif estado == GIRAR_DERECHA:

            sim.setJointTargetVelocity(motor_d, -VELOCIDAD_GIRO)
            sim.setJointTargetVelocity(motor_i,  VELOCIDAD_GIRO)

            if t - t0 > TIEMPO_GIRO_90 + TIEMPO_SOBREGIRO + 0.3:
                estado = AVANZAR_RECTO

        # GIRAR 180
        
        elif estado == GIRAR_180:

            sim.setJointTargetVelocity(motor_d, -1.6)
            sim.setJointTargetVelocity(motor_i,  1.6)

            if t - t0 > 3.0:
                estado = AVANZAR_RECTO

        sim.step()

    sim.stopSimulation()


if __name__ == "__main__":
    main()