"""Parte 2 — mapeamento do labirinto.

1. Explora o labirinto inteiro com DFS, usando só os sensores do robô
   (/culling_games/robot_sensors) e o /move_command. Não usa o /get_map.
2. Monta o mapa a partir do que os sensores viram e salva em mapa_parte2.csv.
3. Calcula a rota mais curta (BFS) no mapa construído e a executa até o alvo.
4. Verificação: compara a rota com a da Parte 1, calculada no mapa real
   (/get_map). Se as duas tiverem o mesmo tamanho, o mapa criado é suficiente
   para reproduzir a rota ótima.
"""
import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd

from .grafo import montar_grid, encontrar, bfs, caminho_para_direcoes, desenhar, DIRECOES
from .explorador import explorar, mapa_para_grid, SENSORES


class Mapeador(Node):
    def __init__(self):
        super().__init__('mapeador_parte2')
        self.intervalo = self.declare_parameter('intervalo', 0.05).value
        self.verificar = self.declare_parameter('verificar', True).value

        self.ultima_leitura = None
        self.contador = 0
        self.create_subscription(RobotSensors, '/culling_games/robot_sensors',
                                 self.ao_receber_sensores, qos_profile_sensor_data)

        self.cli_mover = self.create_client(MoveCmd, '/move_command')
        self.cli_mapa = self.create_client(GetMap, '/get_map')
        while not self.cli_mover.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando /move_command... (o jogo está rodando?)')

    # ---------- comunicação com o simulador ----------
    def ao_receber_sensores(self, msg):
        self.ultima_leitura = {s: getattr(msg, s) for s in SENSORES}
        self.contador += 1

    def ler_sensores(self):
        """Espera duas mensagens novas, para garantir que a leitura já é da
        posição atual (e não de antes do último movimento)."""
        alvo = self.contador + 2
        while self.contador < alvo:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ultima_leitura

    def chamar(self, cliente, req):
        futuro = cliente.call_async(req)
        rclpy.spin_until_future_complete(self, futuro)
        return futuro.result()

    def mover_cmd(self, direcao):
        req = MoveCmd.Request()
        req.direction = direcao
        return self.chamar(self.cli_mover, req)

    def mover(self, direcao):
        resp = self.mover_cmd(direcao)
        time.sleep(self.intervalo)
        return resp.success

    def descobrir_posicao(self):
        """Descobre a posição inicial sem sair do lugar: manda o robô andar
        contra uma parede; o movimento falha, mas a resposta traz robot_pos."""
        leitura = self.ler_sensores()
        for direcao in DIRECOES:
            if leitura[direcao] == 'b':
                resp = self.mover_cmd(direcao)
                return tuple(resp.robot_pos)
        # sem paredes em volta (improvável): anda e volta
        direcao = next(iter(DIRECOES))
        resp = self.mover_cmd(direcao)
        oposto = {'up': 'down', 'down': 'up', 'left': 'right', 'right': 'left'}[direcao]
        resp = self.mover_cmd(oposto)
        return tuple(resp.robot_pos)

    # ---------- fluxo principal ----------
    def executar(self):
        inicio = self.descobrir_posicao()
        self.get_logger().info(f'Robô começa em {inicio}. Explorando com DFS...')

        def progresso(pos, passos):
            if passos % 50 == 0:
                self.get_logger().info(f'{passos} movimentos de exploração...')

        mapa, alvo, passos = explorar(inicio, self.ler_sensores, self.mover, progresso)
        self.get_logger().info(
            f'Exploração concluída: {passos} movimentos, {len(mapa)} células mapeadas.')

        if alvo is None:
            self.get_logger().error('O alvo não foi encontrado durante a exploração.')
            return

        grid = mapa_para_grid(mapa, inicio, alvo)
        self.salvar_csv(grid, 'mapa_parte2.csv')

        caminho = bfs(grid, inicio, alvo)
        comandos = caminho_para_direcoes(caminho)
        print('\nMapa construído pelo robô, com a rota mais curta:')
        print(desenhar(grid, caminho))
        self.get_logger().info(f'Rota no mapa construído (BFS): {len(comandos)} passos')

        if self.verificar:
            self.comparar_com_mapa_real(grid, inicio, alvo, len(comandos))

        self.get_logger().info('Executando a rota até o alvo...')
        for direcao in comandos:
            if not self.mover(direcao):
                self.get_logger().error(f'Falha ao mover {direcao}.')
                return
        self.get_logger().info(f'Alvo alcançado em {len(comandos)} passos!')

    def salvar_csv(self, grid, nome):
        with open(nome, 'w', newline='') as f:
            csv.writer(f).writerows(grid)
        self.get_logger().info(f'Mapa salvo em {nome}')

    def comparar_com_mapa_real(self, grid, inicio, alvo, tamanho_rota):
        """Usa o /get_map só para conferir o resultado (não para navegar)."""
        if not self.cli_mapa.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_map indisponível; pulando verificação.')
            return
        resp = self.chamar(self.cli_mapa, GetMap.Request())
        real = montar_grid(resp.occupancy_grid_flattened, resp.occupancy_grid_shape)
        rota_real = bfs(real, encontrar(real, 'r'), encontrar(real, 't'))
        tamanho_real = len(rota_real) - 1

        # células livres do mapa real que o robô deixou de mapear
        livres_real = {(i, j) for i, l in enumerate(real) for j, c in enumerate(l) if c != 'b'}
        livres_mapa = {(i, j) for i, l in enumerate(grid) for j, c in enumerate(l) if c != 'b'}
        faltando = livres_real - livres_mapa

        self.get_logger().info(
            f'Verificação | rota Parte 1 (mapa real): {tamanho_real} passos | '
            f'rota Parte 2 (mapa construído): {tamanho_rota} passos | '
            f'células livres não mapeadas: {len(faltando)}')
        if tamanho_real == tamanho_rota:
            self.get_logger().info('OK: o mapa construído reproduz a rota ótima da Parte 1.')
        else:
            self.get_logger().warn('As rotas têm tamanhos diferentes!')


def main(args=None):
    rclpy.init(args=args)
    no = Mapeador()
    try:
        no.executar()
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
