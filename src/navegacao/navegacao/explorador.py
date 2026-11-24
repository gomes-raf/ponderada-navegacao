"""Exploração do labirinto com DFS (sem dependência de ROS).

A lógica recebe duas funções, para poder ser testada sem o simulador:
    ler_sensores() -> dict com as 8 células vizinhas ('up', 'down', ..., 'down_right')
    mover(direcao)  -> bool (True se o robô andou)

O robô começa em `inicio` e:
  1. anota no mapa tudo o que os sensores veem;
  2. vai para um vizinho livre ainda não visitado (DFS);
  3. quando não há mais vizinhos novos, volta pelo mesmo caminho (backtracking).
A exploração termina quando o robô volta ao início sem nada novo para visitar,
ou seja, todo o labirinto alcançável foi mapeado.

O alvo ('t') é apenas anotado: o robô não pisa nele durante a exploração,
porque o simulador encerra o jogo quando o alvo é alcançado.
"""
from .grafo import DIRECOES, BLOQUEADO

# sensor -> (delta_linha, delta_coluna)
SENSORES = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
    'up_left': (-1, -1),
    'up_right': (-1, 1),
    'down_left': (1, -1),
    'down_right': (1, 1),
}

OPOSTO = {'up': 'down', 'down': 'up', 'left': 'right', 'right': 'left'}


def explorar(inicio, ler_sensores, mover, ao_mover=None):
    """Explora todo o labirinto. Retorna (mapa, alvo, passos).

    mapa: dict {(linha, coluna): 'b' | 'f' | 't'}
    alvo: posição do alvo (ou None se não foi visto)
    passos: número de movimentos feitos na exploração
    """
    mapa = {inicio: 'f'}
    alvo = None
    visitados = {inicio}
    pilha = []           # direções usadas para chegar até a célula atual
    pos = inicio
    passos = 0

    def anotar(pos):
        nonlocal alvo
        leitura = ler_sensores()
        for sensor, (di, dj) in SENSORES.items():
            cel = (pos[0] + di, pos[1] + dj)
            valor = leitura[sensor]
            if valor == 'r':          # nunca deveria acontecer; trata como livre
                valor = 'f'
            mapa[cel] = valor
            if valor == 't':
                alvo = cel

    anotar(pos)
    while True:
        # procura um vizinho livre, não visitado e que não seja o alvo
        proxima = None
        for direcao, (di, dj) in DIRECOES.items():
            viz = (pos[0] + di, pos[1] + dj)
            if mapa.get(viz) == 'f' and viz not in visitados:
                proxima = direcao
                break

        if proxima is not None:
            di, dj = DIRECOES[proxima]
            if not mover(proxima):
                # sensor e simulador discordaram: marca como parede e segue
                mapa[(pos[0] + di, pos[1] + dj)] = BLOQUEADO
                continue
            pos = (pos[0] + di, pos[1] + dj)
            visitados.add(pos)
            pilha.append(proxima)
            passos += 1
            anotar(pos)
        elif pilha:
            volta = OPOSTO[pilha.pop()]
            di, dj = DIRECOES[volta]
            mover(volta)
            pos = (pos[0] + di, pos[1] + dj)
            passos += 1
        else:
            break   # voltou ao início e não há mais nada para explorar

        if ao_mover:
            ao_mover(pos, passos)

    return mapa, alvo, passos


def mapa_para_grid(mapa, inicio, alvo):
    """Converte o dict do mapa em matriz. Células nunca vistas viram 'b'."""
    linhas = max(i for i, _ in mapa) + 1
    colunas = max(j for _, j in mapa) + 1
    grid = [[BLOQUEADO] * colunas for _ in range(linhas)]
    for (i, j), valor in mapa.items():
        if i >= 0 and j >= 0:
            grid[i][j] = valor
    grid[inicio[0]][inicio[1]] = 'r'
    if alvo:
        grid[alvo[0]][alvo[1]] = 't'
    return grid
