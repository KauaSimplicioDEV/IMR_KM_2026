# Se for executar no VSCode, executar:
# 1. Criar e ativar o ambiente virtual
# python ou python3 -m venv venv_robotica
# source venv_robotica/bin/activate     # No Linux
# venv_robotica\Scripts\activate        # No Windows
#
# 2. Instalar as dependências leves
# pip install pygame numpy

#Se for rodar no Colab, executar i código diretamente


import pygame
import math
import numpy as np

# Constantes de Configuração
LARGURA_TELA = 800
ALTURA_TELA = 600
FPS = 60
COR_FUNDO = (30, 30, 30)
COR_ROBO = (0, 180, 255)
COR_DIRECAO = (255, 50, 50)
COR_TRAJETORIA = (100, 200, 100)
COR_ALVO = (255, 220, 0)
COR_ALVO_ATINGIDO = (100, 255, 100)

# --- Parâmetros do controlador proporcional (go-to-goal) ---
K_V = 1.2              # ganho proporcional de velocidade linear (para v = Kv * rho)
K_P = 3.0              # ganho proporcional angular (Kp da formula omega = Kp * erro_theta)
V_MAX = 150.0          # limite de velocidade linear (px/s)
OMEGA_MAX = 4.0        # limite de velocidade angular (rad/s)
DISTANCIA_TOLERANCIA = 5.0  # epsilon: distancia para considerar alvo atingido (px)


class DiffDriveRobot:
    def __init__(self, x, y, theta=0.0, wheelbase=30.0, radius=15.0):
        # Estado do robô: [x, y, theta]
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)  # em radianos
        
        # Parâmetros físicos (em pixels)
        self.L = float(wheelbase)  # Distância entre rodas
        self.radius = float(radius)
        
        # Entradas de controle
        self.v = 0.0      # Velocidade linear (pixels/s)
        self.omega = 0.0  # Velocidade angular (rad/s)
        
        # Histórico de posições para plotar rastro
        self.history = []

    def set_wheel_velocities(self, v_left, v_right):
        """Converte velocidade das rodas em velocidade linear e angular."""
        self.v = (v_right + v_left) / 2.0
        self.omega = (v_right - v_left) / self.L

    def set_direct_velocity(self, v, omega):
        """Comando direto de velocidade linear e angular (padrão cmd_vel)."""
        self.v = v
        self.omega = omega

    def update(self, dt):
        """Integração numérica da cinemática diferencial."""
        # Atualização angular
        self.theta += self.omega * dt
        # Normaliza o ângulo entre [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Atualização de posição cartesiana
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        
        # Guarda histórico para desenhar o rastro
        if len(self.history) == 0 or np.hypot(self.x - self.history[-1][0], self.y - self.history[-1][1]) > 5:
            self.history.append((self.x, self.y))
            if len(self.history) > 500:
                self.history.pop(0)

    def draw(self, surface):
        # 1. Desenha o rastro
        if len(self.history) > 1:
            pygame.draw.lines(surface, COR_TRAJETORIA, False, self.history, 2)
            
        # 2. Desenha o corpo do robô
        pos_int = (int(self.x), int(self.y))
        pygame.draw.circle(surface, COR_ROBO, pos_int, int(self.radius))
        
        # 3. Desenha a linha indicadora da direção (orientação theta)
        linha_frente_x = self.x + (self.radius + 10) * math.cos(self.theta)
        linha_frente_y = self.y + (self.radius + 10) * math.sin(self.theta)
        pygame.draw.line(surface, COR_DIRECAO, pos_int, (int(linha_frente_x), int(linha_frente_y)), 3)


def normaliza_angulo(angulo):
    """Normaliza um ângulo para o intervalo [-pi, pi]."""
    return (angulo + math.pi) % (2 * math.pi) - math.pi


def controlador_proporcional(robot, alvo):
    """
    Controlador P (go-to-goal) seguindo exatamente as formulas:

        theta_desejado = atan2(y_alvo - y, x_alvo - x)
        erro_theta      = theta_desejado - theta
        omega           = Kp * erro_theta

    A distancia rho = |alvo - pose| e usada tanto para o criterio de parada
    (rho < epsilon) quanto, de forma proporcional, para a velocidade linear.
    """
    dx = alvo[0] - robot.x
    dy = alvo[1] - robot.y
    rho = math.hypot(dx, dy)

    theta_desejado = math.atan2(dy, dx)
    erro_theta = normaliza_angulo(theta_desejado - robot.theta)

    v = K_V * rho
    omega = K_P * erro_theta

    # Saturação das velocidades
    v = max(-V_MAX, min(V_MAX, v))
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, omega))

    return v, omega, rho


def main():
    pygame.init()
    screen = pygame.display.set_mode((LARGURA_TELA, ALTURA_TELA))
    pygame.display.set_caption("Aula 01 - Exercicio 3: Controlador Proporcional (Go-to-Goal)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)

    robot = DiffDriveRobot(x=LARGURA_TELA // 2, y=ALTURA_TELA // 2, theta=0.0)

    alvo = None          # (x, y) ou None se nao houver alvo definido
    alvo_atingido = False

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0  # Delta time em segundos

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # clique com botao esquerdo define novo alvo
                    alvo = event.pos
                    alvo_atingido = False

        # Controle proporcional em direcao ao alvo
        rho = None
        if alvo is not None and not alvo_atingido:
            v, omega, rho = controlador_proporcional(robot, alvo)

            if rho < DISTANCIA_TOLERANCIA:
                # Alvo atingido: para o robo automaticamente
                robot.set_direct_velocity(0.0, 0.0)
                alvo_atingido = True
            else:
                robot.set_direct_velocity(v, omega)
        else:
            robot.set_direct_velocity(0.0, 0.0)

        robot.update(dt)

        # Renderização
        screen.fill(COR_FUNDO)

        if alvo is not None:
            cor_alvo = COR_ALVO_ATINGIDO if alvo_atingido else COR_ALVO
            pygame.draw.circle(screen, cor_alvo, (int(alvo[0]), int(alvo[1])), 8, width=2)
            pygame.draw.circle(screen, cor_alvo, (int(alvo[0]), int(alvo[1])), 2)
            # Raio de tolerancia (epsilon) ao redor do alvo
            pygame.draw.circle(screen, cor_alvo, (int(alvo[0]), int(alvo[1])), int(DISTANCIA_TOLERANCIA), width=1)

        robot.draw(screen)

        # Painel de Telemetria
        if alvo is None:
            linha_alvo = "Alvo: (nenhum, clique na tela)"
        else:
            linha_alvo = f"Alvo: ({alvo[0]}, {alvo[1]}) | rho = {rho:.1f} px"

        info_txt = [
            f"Pose X: {robot.x:.1f} px | Y: {robot.y:.1f} px | Theta: {math.degrees(robot.theta):.1f} deg",
            f"Comandos: v = {robot.v:.1f} px/s | omega = {robot.omega:.2f} rad/s",
            linha_alvo,
            "Status: ALVO ATINGIDO (parado)" if alvo_atingido else "Status: navegando ate o alvo...",
            "Controles: clique com o botao esquerdo do mouse para definir um novo alvo",
        ]
        for i, txt in enumerate(info_txt):
            rendered = font.render(txt, True, (220, 220, 220))
            screen.blit(rendered, (15, 15 + i * 20))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
