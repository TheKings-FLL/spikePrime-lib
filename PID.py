import hub
import motor
import motor_pair
import runloop
from time import sleep, ticks_ms

# Rodas
motor_pair.pair(motor_pair.PAIR_1, hub.port.A, hub.port.C)
# Engrenagens
motor_pair.pair(motor_pair.PAIR_1, hub.port.C, hub.port.D)

async def resetarMotor():
    """
    Função para resetar os motores (leva ele para a posição 0)
    """
    motor.run_to_absolute_position(hub.port.A, 0, 1000, direction=motor.SHORTEST_PATH)
    motor.run_to_absolute_position(hub.port.C, 0, 1000, direction=motor.SHORTEST_PATH)
    motor.run_to_absolute_position(hub.port.E, 0, 1000, direction=motor.SHORTEST_PATH)
    motor.run_to_absolute_position(hub.port.D, 0, 1000, direction=motor.SHORTEST_PATH)
    

# andar
async def moverTank(porta1: int, porta2: int, rotacao1: float, rotacao2: float, velocidade1:int, velocidade2: int, acel: int, desacel: int):
    """
    Função para mover motores no estilo Tank

    Args:
        porta1:int & porta2: int -> porta do hub, ex gub.port.A
        rotacao1 & rotacao2: float -> rotações que o motor deve executar, ex: 0.5 = meia rotação
        velocidade1 & velocidade2: int -> velocidade do motor
        - Motor Pequeno: -110 a 1100
        - Motor Grande: -1050 a 1050
        acel: int -> velocidade de aceleração do motor
        desacel: int -> velocidade de desecaleração do motor

    """

    # Reseta a posição relativa dos motores
    motor.reset_relative_position(porta1, 0)
    motor.reset_relative_position(porta2, 0)

    rotacao1 = int(rotacao1 * 360)
    rotacao2 = int(rotacao2 * 360)

    # Inicia os motores
    motor.run_for_degrees(porta1, rotacao1, velocidade1, stop=motor.HOLD, acceleration=acel, deceleration=desacel)
    await motor.run_for_degrees(porta2, rotacao2, velocidade2, stop=motor.HOLD, acceleration=acel, deceleration=desacel)

async def andarReto(rotacao: float, velocidade: int, acel: int, desacel: int):
    """
    Função para andar reto

    Args:
        rotacao:float -> rotações que o motor deve executar, ex: 0.5 = meia rotação
        velocidadeA & velocidadeC: int -> velocidade do motor
        - Motor Pequeno: -110 a 1100
        - Motor Grande: -1050 a 1050
        acel: int -> velocidade de aceleração do motor
        desacel: int -> velocidade de desecaleração do motor
    """
    # Reseta os motores
    motor.reset_relative_position(hub.port.A, 0)
    motor.reset_relative_position(hub.port.C, 0)

    rotacao = int(rotacao * 360)

    # Inicia os motores
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, -rotacao, velocidade, velocidade, stop=motor.HOLD, acceleration=acel, deceleration=desacel)

# Valores: kp: 0.3
# Valores: ki = 0.003
# Valores: kd = 0
class controlePID:
    """
    Classe utilizada para implementar o controle PID na hora de curvar

    ------- Atributos -------
    kp: float -> o valor do P
    ki: float -> o valor do I
    kd: float -> o valor do D

    ------- Métodos -------
    calcula_saida_pid(setpoint: int, real:int) -> retorna a saída do PID calculado
    curvar(graus: int) -> a quantidade de graus desejada para curvar
    curvar_andando() -> DESENVOLVIMENTO, TEM QUE MELHORAR // curvar enquanto anda

    """
    def __init__(self, kp: float, ki: float, kd: float):
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

    def calcula_saida_pid(self, setpoint:int, real:int):
        """
        Função usada para calcular a saída do PID

        Args:
            setpoint: int -> o valor alvo
            real: float -> o valor atual de leitura do sensor
        Returns:
            PID_value: float -> saída do PID
        """
        # Define as varaiveis como globais
        global PID_p
        global PID_i
        global PID_d
        global Time

        self.PID_error = setpoint - real;
        # Calcula o vlaor de P
        self.PID_p = 0.01*self.kp * self.PID_error;
        # Calcula o valor de I
        self.PID_i = 0.01 * self.PID_i + (self.ki * self.PID_error);

        # Para o D, precisamos do tempo de execução real para calcular a taxa de mudança
        self.timePrev = self.Time;                         # O valor anterior é salvo antes de ler o valor atual
        self.Time = ticks_ms();                            # Leitura do valor atual
        self.elapsedTime = (self.Time - self.timePrev) / 1000;
        # Aqui, calculamos o D
        self.PID_d = 0.01*self.kd*((self.PID_error - self.previous_error)/self.elapsedTime);
        # O valor da saída do PID é a soma de todas as constantes
        self.PID_value = self.PID_p + self.PID_i + self.PID_d;

        # Define o limite de saída
        if(self.PID_value < -1):
            self.PID_value = -1;

        elif(self.PID_value > 1):
            self.PID_value = 1;

        if self.PID_value > -0.01 and self.PID_value < 0.01:
            self.PID_value = 0

        return self.PID_value;

    async def curvar(self, graus: int):
        """
        Função utilizada para curvar com o controle PID

        Args:
            graus: int -> a quantidade de graus que deseja curvar
        """
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
        """
        FUNCIONA TUDO TRAVADO, TEM QUE MELHORAR

        FUnção usada para andar curvando
        """
        # Reseta os motores
        motor.reset_relative_position(hub.port.A, 0)
        motor.reset_relative_position(hub.port.C, 0)

        pid_calculado = 0

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
    # Escreva o código aqui
    pass

runloop.run(main())
