# Desafio de navegação autônoma — Culling Games

Solução da ponderada de navegação autônoma, feita com base em [culling_games](https://github.com/rmnicola/culling_games). A solução fica no pacote `src/navegacao`.


## Como rodar

Requisitos: ROS 2 e Pygame (`sudo apt install python3-pygame`).

```bash
colcon build
source install/setup.bash
```

**Terminal 1, simulador:**
```bash
ros2 run cg maze -- --map 1.csv
```

**Terminal 2, solução** (rode `source install/setup.bash` antes):
```bash
ros2 run navegacao parte1   # Parte 1: navegação com mapa
ros2 run navegacao parte2   # Parte 2: mapeamento + navegação
```

O jogo para de aceitar movimentos quando o robô chega ao alvo. Por isso, reinicie o simulador entre uma parte e outra. A velocidade se ajusta com `--ros-args -p intervalo:=0.1` (segundos entre passos).

## Parte 1: navegação com mapa

O nó pede o labirinto pelo `/get_map` e o transforma em um grafo: cada célula livre é um nó, ligado às células livres vizinhas (cima, baixo, esquerda e direita). Uma **BFS** encontra o caminho mais curto do robô até o alvo, e os comandos são enviados um a um pelo `/move_command`.

**Por que BFS e não Dijkstra?** Todo passo tem o mesmo custo. Nesse caso a BFS já garante a rota mais curta, em O(V + E). O Dijkstra chegaria à mesma rota, mas com o custo extra da fila de prioridade, O((V + E) log V). Ele só seria necessário se os passos tivessem custos diferentes.

## Parte 2: mapeamento do labirinto

O nó **não usa o `/get_map`** para navegar. Ele explora o labirinto com uma **DFS com backtracking**: a cada célula, lê os 8 sensores (`/culling_games/robot_sensors`), anota o que vê e segue para um vizinho livre ainda não visitado. Quando não há mais vizinhos novos, volta pelo mesmo caminho. Assim ele percorre todo o labirinto e retorna ao início. O alvo é anotado, mas o robô não pisa nele durante a exploração, porque o jogo termina quando ele chega lá.

Com o mapa pronto (salvo em `mapa_parte2.csv`), a mesma BFS da Parte 1 calcula a rota, e o robô vai até o alvo.

**Comprovação:** ao final, o nó consulta o `/get_map` apenas para comparar os resultados. No `1.csv`, as duas rotas têm **102 passos** e nenhuma célula livre fica sem mapear, ou seja, o mapa construído reproduz a rota ótima da Parte 1.

## Estrutura

```
src/navegacao/navegacao/
├── grafo.py       # grid, BFS e conversão de caminho em comandos
├── explorador.py  # exploração por DFS a partir dos sensores
├── parte1.py      # nó ROS da Parte 1
└── parte2.py      # nó ROS da Parte 2
```
