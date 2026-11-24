"""Funções de grafo para o labirinto (sem dependência de ROS).

O labirinto é uma matriz de strings:
    'b' = bloqueado, 'f' = livre, 'r' = robô, 't' = alvo
Cada célula não bloqueada é um nó do grafo; há uma aresta entre duas
células livres vizinhas (cima, baixo, esquerda, direita).
"""
from collections import deque

# direção -> (delta_linha, delta_coluna)
DIRECOES = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
}

BLOQUEADO = 'b'


def montar_grid(flat, shape):
    """Converte a lista achatada do /get_map em matriz (linhas x colunas)."""
    linhas, colunas = shape
    return [list(flat[i * colunas:(i + 1) * colunas]) for i in range(linhas)]


def encontrar(grid, valor):
    """Retorna (linha, coluna) da primeira célula com o valor dado."""
    for i, linha in enumerate(grid):
        for j, celula in enumerate(linha):
            if celula == valor:
                return (i, j)
    return None


def vizinhos(grid, pos):
    """Gera (direcao, vizinho) para cada vizinho livre de pos."""
    linhas, colunas = len(grid), len(grid[0])
    i, j = pos
    for direcao, (di, dj) in DIRECOES.items():
        ni, nj = i + di, j + dj
        if 0 <= ni < linhas and 0 <= nj < colunas and grid[ni][nj] != BLOQUEADO:
            yield direcao, (ni, nj)


def bfs(grid, inicio, alvo):
    """Busca em largura: retorna a lista de posições do caminho mais curto
    de inicio até alvo (inclusive), ou None se não houver caminho.

    Como todas as arestas têm custo 1, a BFS garante o menor número de passos.
    """
    fila = deque([inicio])
    pai = {inicio: None}

    while fila:
        atual = fila.popleft()
        if atual == alvo:
            break
        for _, viz in vizinhos(grid, atual):
            if viz not in pai:
                pai[viz] = atual
                fila.append(viz)

    if alvo not in pai:
        return None

    caminho = []
    no = alvo
    while no is not None:
        caminho.append(no)
        no = pai[no]
    caminho.reverse()
    return caminho


def caminho_para_direcoes(caminho):
    """Converte uma lista de posições em comandos ('up', 'down', ...)."""
    inverso = {delta: d for d, delta in DIRECOES.items()}
    comandos = []
    for (i1, j1), (i2, j2) in zip(caminho, caminho[1:]):
        comandos.append(inverso[(i2 - i1, j2 - j1)])
    return comandos


def desenhar(grid, caminho=()):
    """Retorna o labirinto em texto, marcando o caminho com '.'."""
    marcados = set(caminho)
    simbolos = {'b': '█', 'f': ' ', 'r': 'R', 't': 'T'}
    linhas = []
    for i, linha in enumerate(grid):
        s = ''
        for j, c in enumerate(linha):
            if (i, j) in marcados and c not in ('r', 't'):
                s += '·'
            else:
                s += simbolos.get(c, '?')
        linhas.append(s)
    return '\n'.join(linhas)
