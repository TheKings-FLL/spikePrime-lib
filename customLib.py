import motor
import motor_pair
import runloop
from hub import port, motion_sensor

def comandosInicias():
    """
    Função utilizada para definir comanos iniciais para o funcionamente do robô, como os pares de motores, por exemplo
    """
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    motor_pair.pair(motor_pair.PAIR_2, port.C, port.D)

def reiniciarMotor(porta):
    """
    Função utilizada para levar o motor até a sua posição inicial (0)

    Parâmetros:
    - motor: a porta que o motor está localizado (A, B, C, D, E, F)
    """
    try:
        motor.run_to_absolute_position(port=port.str(porta).strip().upper(), acceleration=100, deceleration=100, velocity=1000)
    except TypeError:
        print("Porta digitada inválida")

class pidMotor(): 
    """
    Classe para implementar o controle PID em motores.

    Parâmetros:
    - kp: Ganho proporcional
    - ki: Ganho integral
    - kd: Ganho derivativo
    - setpoint: Valor alvo do motor
    - lastError: Último erro
    - integral: Soma de todos os erros para o termo integral
    - derivativa: Variação do erro para o termo derivativo
    """
    def __init__(self, kp, kd, ki, setpoint):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.setpoint = setpoint
        self.lastError = 0
        self.integral = 0
        self.derivativa = 0 
    
    async def updateError(self, valorAtual):
        """
        Calcula o sinal de controle baseado no erro atual.

        Parâmetros:
        - valorAtual: A angulação atual do motor

        Retorna:
        - controlSignal: A correção que deve ser feita no motor
        """
        erro = self.setpoint - valorAtual
        self.integral += erro # calcula o erro total
        self.derivativa = erro  - self.lastError
        controlSignal = (self.kp * erro) + (self.ki * self.integral) + (self.kd * self.derivativa)
        self.lastError = erro

        return controlSignal

    async def andar(self, aceleracao=100, velocidadeMax=1000):
        """
        Função utiliada para fazer o robô andar reto com o uso do PID

        Parâmetros:
        - aceeleracao = a aceleração do motor em graus/s² (segundos quadrados)
        - velocidadeMax: Velocidade máxima que o PID pode atingir, velocidade máxima: 1000º/s² (velocidade positiva -> andar para frente, velocidade negativa -> andar para trás)
        """
        motor.reset_relative_position(port=port.A, position=0)
        motor.reset_relative_position(port=port.E, position=0)
        ajusteMotorA = self.updateError(valorAtual=motor.relative_position(port.A))
        ajusteMotorE = self.updateError(valorAtual=motor.relative_position(port.E))

        while motor.relative_position(port.A) != (self.setpoint * 360) or motor.relative_position(port.E) != (self.setpoint * 360):
            # Atualiza o ajuste do motor
            ajusteMotorA = self.updateError(valorAtual=motor.relative_position(port.A))
            ajusteMotorE = self.updateError(valorAtual=motor.relative_position(port.E))

            # Limita a velocidade máxima do PID para o motor A
            if ajusteMotorA > abs(velocidadeMax):
                ajusteMotorA = velocidadeMax
            elif ajusteMotorA < -abs(velocidadeMax):
                ajusteMotorA = -abs(velocidadeMax)
            
            # Limita a velocidade máxima do PID para o motor E
            if ajusteMotorE > abs(velocidadeMax):
                ajusteMotorE = velocidadeMax
            elif ajusteMotorE < -abs(velocidadeMax):
                ajusteMotorE = -abs(velocidadeMax)
            
            motor_pair.move_tank(pair=motor_pair.PAIR_1, left_velocity=ajusteMotorE, right_velocity=ajusteMotorA, acceleration=aceleracao)
        motor_pair.stop(pair=motor_pair.PAIR_1)
    
    async def moverEngrenagem(self, aceleracao=100, velocidadeMax = 1000):
        """
        Função utiliada para controlar os motores de engrenagem com o uso do PID

        Parâmetros:
        - aceeleracao: A aceleração do motor em graus/s² (segundos quadrados)
        - velocidadeMax: Velocidade máxima que o PID pode atingir, velocidade máxima: 1000º/s (velocidade positiva -> andar para frente, velocidade negativa -> andar para trás)
        """
        motor.reset_relative_position(port=port.C, position=0)
        motor.reset_relative_position(port=port.D, position=0)
        ajusteMotorC = self.updateError(motor.relative_position(port.C))
        ajusteMotorD = self.updateError(motor.relative_position(port.D))

        while motor.relative_position(port.C) != (self.setpoint * 360) or motor.relative_position(port.D) != (self.setpoint * 360):
            ajusteMotorC = self.updateError(motor.relative_position(port.C))
            ajusteMotorD = self.updateError(motor.relative_position(port.D))

            # Limita a velocidade máxima do PID para o motor C
            if ajusteMotorC > abs(velocidadeMax):
                ajusteMotorC = velocidadeMax
            elif ajusteMotorC < -abs(velocidadeMax):
                ajusteMotorC = -abs(velocidadeMax)
            
            # Limita a velocidade máxima do PID para o motor D
            if ajusteMotorD > abs(velocidadeMax):
                ajusteMotorD = velocidadeMax
            elif ajusteMotorD < -abs(velocidadeMax):
                ajusteMotorD = -abs(velocidadeMax)
            
            motor_pair.move_tank(pair=motor_pair.PAIR_2, left_velocity=ajusteMotorC, right_velocity=ajusteMotorD, acceleration=aceleracao)
        motor_pair.stop(pair=motor_pair.PAIR_2)

class pidGiroscopio():
    """
    Classe para implementar o controle PID no giroscópio.

    Parâmetros:
    - kp: Ganho proporcional
    - ki: Ganho integral
    - kd: Ganho derivativo
    - setpoint: Valor alvo do motor
    - lastError: Último erro
    - integral: Soma de todos os erros para o termo integral
    - derivativa: Variação do erro para o termo derivativo
    """
    def __init__(self, kp, kd, ki, setpoint):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.setpoint = setpoint
        self.lastError = 0
        self.integral = 0
        self.derivativa = 0
    
    async def updateError(self, valorAtual):
        """
        Calcula o sinal de controle baseado no erro atual.

        Parâmetros:
        - valorAtual: A angulação atual do giroscópio

        Retorna:
        - controlSignal: A correção que deve ser feita no motor
        """
        erro = self.setpoint - valorAtual
        self.integral += erro
        self.derivativa = erro - self.lastError
        controlSignal = (self.kp * erro) + (self.ki * self.integral) + (self.kd * self.derivativa)
        self.lastError = erro

        return controlSignal

    async def curvar(self, aceleracao, velocidadeCurva):
        """
        Função utilizada para implementar o uso do PID para curvas usando giroscópio

        Parâmetros:
        - aceleracao = A aceleração do motor em graus/s² (segundo quadrado)
        - velocidadeCurva: A velocidade deseja na curva, de 0 a 1000 graus por segundo
        """

        # Reinicia o sensor giroscópio
        motion_sensor.reset_yaw()

        # Atualiza os valores do PID
        anguloAtual = motion_sensor.tilt_angles()
        anguloAtualFiltrado = anguloAtual[1]
        velocidade = self.updateError(anguloAtualFiltrado)

        while self.setpoint != anguloAtualFiltrado:
            anguloAtual = motion_sensor.tilt_angles()
            anguloAtualFiltrado = anguloAtual[1]
            velocidade = self.updateError(anguloAtualFiltrado)
            motor_pair.move_tank(pair=motor_pair.PAIR_1 ,acceleration=aceleracao, left_velocity=velocidade * velocidadeCurva, right_velocity=-velocidade * velocidadeCurva)

comandosInicias()
controlePidMotor = pidMotor(kp=0.1, kd=0.2, ki=0.3, setpoint=0.4)
controlePidGiroscopio = pidGiroscopio(kp=1, kd=1, ki=1, setpoint=90)

await controlePidMotor.andar(aceleracao=100)
controlePidMotor = pidMotor(kp=0.1, kd=0.2, ki=0.3, setpoint=2)
await controlePidMotor.moverEngrenagem(aceleracao=50, velocidadeMax=1000)
await controlePidGiroscopio
