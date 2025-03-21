import hub
import motor
import motor_pair
import runloop
from time import sleep, ticks_ms

motor_pair.pair(motor_pair.PAIR_1, hub.port.A, hub.port.C)

async def resetarMotor():
    motor.run_to_absolute_position(hub.port.A, 0, 1000)
    motor.run_to_absolute_position(hub.port.C, 0, 1000)

# andar
async def moverTank(rotacaoA: float, rotacaoC: float, velocidadeA:int, velocidadeC: int, acel: int, desacel: int):
    # Reseta os motores
    motor.reset_relative_position(hub.port.A, 0)
    motor.reset_relative_position(hub.port.C, 0)

    # Inicia os motores
    motor.run_for_degrees(hub.port.A, int((rotacaoA * -360)), velocidadeA, stop=motor.HOLD, acceleration=acel, deceleration=desacel)
    await motor.run_for_degrees(hub.port.C, int((rotacaoC * 360)), velocidadeC, stop=motor.HOLD, acceleration=acel, deceleration=desacel)

async def andarReto(rotacao: float, velocidade: int, acel: int, desacel: int):
    # Reseta os motores
    motor.reset_relative_position(hub.port.A, 0)
    motor.reset_relative_position(hub.port.C, 0)

    # Inicia os motores
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, -int((rotacao * -360)), velocidade, velocidade, stop=motor.HOLD, acceleration=acel, deceleration=desacel)

# Valores: kp: 0.3
# Valores: ki = 0.003
# Valores: kd = 0
class controlePID:
    def __init__(self, kp, ki, kd):
        # Variaveis usada no PID
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.PID_error = 0
        self.previous_error = 0
        self.PID_value = 0
        self.PID_p = 0
        self.PID_i = 0
        self.PID_d = 0
        self.elapsedTime = 0
        self.Time = 0
        self.timePrev = 0

    def calcula_saida_pid(self, ideal:int, real:int):
        # Define as varaiveis como globais
        global PID_p
        global PID_i
        global PID_d
        global Time

        # Next we calculate the error between the setpoint and the real value
        self.PID_error = ideal - real;
        # Calculate the P value
        self.PID_p = 0.01*self.kp * self.PID_error;
        # Calculate the I value in a range on +-6
        self.PID_i = 0.01 * self.PID_i + (self.ki * self.PID_error);

        # For derivative we need real time to calculate speed change rate
        self.timePrev = self.Time;                            #the previous time is stored before the actual time read
        self.Time = ticks_ms();                            #actual time read
        self.elapsedTime = (self.Time - self.timePrev) / 1000;
        # Now we can calculate the D calue
        self.PID_d = 0.01*self.kd*((self.PID_error - self.previous_error)/self.elapsedTime);
        # Final total PID value is the sum of P + I + D
        self.PID_value = self.PID_p + self.PID_i + self.PID_d;

        # We define PWM range between 0 and 255
        if(self.PID_value < -1):
            self.PID_value = -1;

        if(self.PID_value > 1):
            self.PID_value = 1;

        if self.PID_value >-0.01 and self.PID_value < 0.01:
            self.PID_value = 0

        return self.PID_value;

    async def curvar(self, graus: int):
        # Reseta os motores
        motor.reset_relative_position(hub.port.A, 0)
        motor.reset_relative_position(hub.port.C, 0)

        #reseta o yaw
        hub.motion_sensor.reset_yaw(0)

        while True:
            tiltangles = hub.motion_sensor.tilt_angles()
            yaw = tiltangles[0] / 10

            if yaw>=(graus-2) and yaw <=(graus+2):
                break

            pid_calculado = self.calcula_saida_pid(graus, int(yaw))

            # Inicia os motores
            motor.run_for_degrees(hub.port.A, int(pid_calculado * 360), abs(int(pid_calculado*500)), stop=motor.HOLD, acceleration=1000, deceleration=1000)
            await motor.run_for_degrees(hub.port.C, int(pid_calculado * 360), abs(int(pid_calculado*500)), stop=motor.HOLD, acceleration=1000, deceleration=1000)

    async def andar_curvando(self):
        # Reseta os motores
        motor.reset_relative_position(hub.port.A, 0)
        motor.reset_relative_position(hub.port.C, 0)
        
        pid_calculado=0

        #reseta o yaw
        hub.motion_sensor.reset_yaw(0)
        for i in range(0, 5):
            motor.run_for_degrees(hub.port.C, -72, int(1000 / 5) , stop=motor.HOLD, acceleration=1000, deceleration=1000)
            print(hub.motion_sensor.tilt_angles()[0])
            while True:
                tiltangles = hub.motion_sensor.tilt_angles()
                yaw = tiltangles[0] / 10
                if yaw>=(18-2) and yaw <=(30+2) and i == 0: break
                elif yaw>=(36-2) and yaw <=(60+2) and i == 1: break
                elif yaw>=(54-2) and yaw <=(90+2) and i == 2: break
                elif yaw>=(72-2) and yaw <=(90+2) and i == 3: break
                elif yaw>=(90-2) and yaw <=(90+2) and i == 4: break
                        
                if i==0: pid_calculado = self.calcula_saida_pid(18, int(yaw))
                elif i==1: pid_calculado = self.calcula_saida_pid(36, int(yaw))
                elif i==2: pid_calculado = self.calcula_saida_pid(54, int(yaw))
                elif i==3: pid_calculado = self.calcula_saida_pid(72, int(yaw))
                elif i==4: pid_calculado = self.calcula_saida_pid(90, int(yaw))
                # Inicia os motores
                await motor.run_for_degrees(hub.port.A, int(pid_calculado * 360), abs(int(pid_calculado*1000)), stop=motor.HOLD, acceleration=1000, deceleration=1000)
                sleep(0.0001)

async def main():
    Escreva o código aqui

runloop.run(main())
