"""Parte 1 — navegação com mapa.

1. Pede o mapa completo ao simulador (/get_map).
2. Encontra o robô ('r') e o alvo ('t').
3. Calcula a rota mais curta com BFS.
4. Envia os comandos um a um para /move_command.
"""
import time

import rclpy
from rclpy.node import Node
from cg_interfaces.srv import GetMap, MoveCmd

from .grafo import montar_grid, encontrar, bfs, caminho_para_direcoes, desenhar


class NavegadorComMapa(Node):
    def __init__(self):
        super().__init__('navegador_parte1')
        # Intervalo entre movimentos, em segundos (ajustável via --ros-args -p intervalo:=0.5)
        self.intervalo = self.declare_parameter('intervalo', 0.2).value
        self.cli_mapa = self.create_client(GetMap, '/get_map')
        self.cli_mover = self.create_client(MoveCmd, '/move_command')

        for cli, nome in ((self.cli_mapa, '/get_map'), (self.cli_mover, '/move_command')):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Aguardando o serviço {nome}... (o jogo está rodando?)')

    def chamar(self, cliente, requisicao):
        futuro = cliente.call_async(requisicao)
        rclpy.spin_until_future_complete(self, futuro)
        return futuro.result()

    def obter_mapa(self):
        resp = self.chamar(self.cli_mapa, GetMap.Request())
        return montar_grid(resp.occupancy_grid_flattened, resp.occupancy_grid_shape)

    def mover(self, direcao):
        req = MoveCmd.Request()
        req.direction = direcao
        return self.chamar(self.cli_mover, req)

    def executar(self):
        grid = self.obter_mapa()
        inicio = encontrar(grid, 'r')
        alvo = encontrar(grid, 't')
        self.get_logger().info(
            f'Mapa {len(grid)}x{len(grid[0])} | robô em {inicio} | alvo em {alvo}')

        caminho = bfs(grid, inicio, alvo)
        if caminho is None:
            self.get_logger().error('Não existe caminho até o alvo.')
            return

        comandos = caminho_para_direcoes(caminho)
        self.get_logger().info(f'Rota mais curta (BFS): {len(comandos)} passos')
        print(desenhar(grid, caminho))
        print('Comandos:', ' '.join(comandos))

        for n, direcao in enumerate(comandos, start=1):
            resp = self.mover(direcao)
            if not resp.success:
                self.get_logger().error(f'Falha no passo {n} ({direcao}) em {tuple(resp.robot_pos)}')
                return
            time.sleep(self.intervalo)
        self.get_logger().info(
            f'Alvo alcançado em {len(comandos)} passos! Posição final: {tuple(resp.robot_pos)}')


def main(args=None):
    rclpy.init(args=args)
    no = NavegadorComMapa()
    try:
        no.executar()
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
