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
COR_INICIO = (255, 220, 0)

# --- Parâmetros da máquina de estados (malha aberta) ---
V_RETA = 80.0                  # velocidade linear durante o trecho reto (px/s)
T_RETA = 2.0                   # duração do trecho reto (s)
OMEGA_GIRO = math.pi / 2.0     # velocidade angular durante o giro (rad/s) -> 90 graus em 1s
T_GIRO = 1.0                   # duração do giro (s)
NUM_LADOS = 4                  # repetições (lados do quadrado)

ESTADO_RETA = 0
ESTADO_GIRO = 1
ESTADO_PARADO = 2


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


class QuadradoMalhaAberta:
    """Máquina de estados por tempo: RETA (2s) -> GIRO (1s), repetida NUM_LADOS vezes."""

    def __init__(self, num_lados=NUM_LADOS):
        self.num_lados = num_lados
        self.estado = ESTADO_RETA
        self.timer = 0.0
        self.lados_completos = 0

    def update(self, robot, dt):
        self.timer += dt

        if self.estado == ESTADO_RETA:
            robot.set_direct_velocity(V_RETA, 0.0)
            if self.timer >= T_RETA:
                self.timer = 0.0
                self.estado = ESTADO_GIRO

        elif self.estado == ESTADO_GIRO:
            robot.set_direct_velocity(0.0, OMEGA_GIRO)
            if self.timer >= T_GIRO:
                self.timer = 0.0
                self.lados_completos += 1
                if self.lados_completos >= self.num_lados:
                    self.estado = ESTADO_PARADO
                else:
                    self.estado = ESTADO_RETA

        elif self.estado == ESTADO_PARADO:
            robot.set_direct_velocity(0.0, 0.0)

    def nome_estado(self):
        return {
            ESTADO_RETA: "RETA",
            ESTADO_GIRO: "GIRO",
            ESTADO_PARADO: "CICLO COMPLETO",
        }[self.estado]


def main():
    pygame.init()
    screen = pygame.display.set_mode((LARGURA_TELA, ALTURA_TELA))
    pygame.display.set_caption("Aula 01 - Exercicio 2: Quadrado em Malha Aberta")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)

    x0, y0, theta0 = LARGURA_TELA // 2, ALTURA_TELA // 2, 0.0
    robot = DiffDriveRobot(x=x0, y=y0, theta=theta0)
    maquina = QuadradoMalhaAberta()

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0  # Delta time em segundos

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # A máquina de estados decide v e omega; nenhuma tecla controla o robô
        maquina.update(robot, dt)
        robot.update(dt)

        # Renderização
        screen.fill(COR_FUNDO)

        # Marca a posição/orientação inicial para comparação visual
        pygame.draw.circle(screen, COR_INICIO, (int(x0), int(y0)), 4)
        linha_ini_x = x0 + 25 * math.cos(theta0)
        linha_ini_y = y0 + 25 * math.sin(theta0)
        pygame.draw.line(screen, COR_INICIO, (int(x0), int(y0)), (int(linha_ini_x), int(linha_ini_y)), 1)

        robot.draw(screen)

        # Erro acumulado em relação ao ponto/orientação inicial
        erro_pos = math.hypot(robot.x - x0, robot.y - y0)
        erro_theta = math.degrees((robot.theta - theta0 + math.pi) % (2 * math.pi) - math.pi)

        # Painel de Telemetria
        info_txt = [
            f"Estado: {maquina.nome_estado()} | Lado {min(maquina.lados_completos + 1, NUM_LADOS)}/{NUM_LADOS} | t_estado = {maquina.timer:.2f}s",
            f"Pose X: {robot.x:.1f} px | Y: {robot.y:.1f} px | Theta: {math.degrees(robot.theta):.1f} deg",
            f"Comandos: v = {robot.v:.1f} px/s | omega = {robot.omega:.2f} rad/s",
            f"Erro vs inicio -> posicao: {erro_pos:.2f} px | orientacao: {erro_theta:.2f} deg",
            "Ponto amarelo = pose inicial do robo (referencia para o fechamento do quadrado)",
        ]
        for i, txt in enumerate(info_txt):
            rendered = font.render(txt, True, (220, 220, 220))
            screen.blit(rendered, (15, 15 + i * 20))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
