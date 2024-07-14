from copy import deepcopy
import random


class Tablero:
    def __init__(self):
        self.state = [[".", ".", "."], [".", ".", "."], [".", ".", "."]]
        self.asignacion_casillas = {i + 1: (i // 3, i % 3) for i in range(9)}
        self.simbolos = {"X", "O"}
        self.na_values = "."
        self.winner = None

    def __str__(self):
        return self.get_state_representation()

    def get_state_representation(self, state=None):
        if not state:
            state = self.state
        result = ""
        for n in range(1, 10):
            i, j = self.asignacion_casillas[n]
            result += " | " + (
                str(n) if state[i][j] not in self.simbolos else state[i][j]
            )
            if n % 3 == 0:
                result += " |\n"
        return result

    def get_result(self, n: int, symbol: str, state=None) -> list:
        if state is None:
            state = self.state
        state = deepcopy(state)

        i, j = self.asignacion_casillas[n]
        state[i][j] = symbol
        return state

    def set_square(self, n: int, symbol: str) -> None:
        self.state = self.get_result(n, symbol)

    def check_columns(self, state):
        j = 0
        is_end = False
        while not is_end and j < len(state[0]):
            is_end = (
                state[0][j] != self.na_values
                and state[0][j] == state[1][j] == state[2][j]
            )
            if is_end:
                self.winner = state[0][j]
            j += 1
        return is_end

    def check_lines(self, state) -> bool:
        i = 0
        is_end = False
        while not is_end and i < len(state):
            if state[i][0] != self.na_values:
                is_end = state[i][0] == state[i][1] == state[i][2]
                if is_end:
                    self.winner = state[i][0]
            i += 1
        return is_end

    def check_diagonals(self, state) -> bool:
        if state[1][1] == self.na_values:
            return False

        diag_1 = state[0][0] == state[1][1] == state[2][2]
        diag_2 = state[0][2] == state[1][1] == state[2][0]

        is_end = diag_1 or diag_2
        if is_end:
            self.winner = state[1][1]
        return is_end

    def check_no_more_moves(self, state) -> bool:
        return len(self.get_possible_moves(state)) == 0

    def is_end_game(self, state=None) -> tuple[bool, str]:
        if state is None:
            state = self.state

        return (
            self.check_columns(state)
            or self.check_diagonals(state)
            or self.check_lines(state)
            or self.check_no_more_moves(state),
            self.winner,
        )

    def get_possible_moves(self, state=None):
        if state == None:
            state = self.state

        possible_moves = []
        for n in range(1, 10):
            i, j = self.asignacion_casillas[n]
            if state[i][j] == self.na_values:
                possible_moves.append(n)

        return possible_moves

    def make_move(self, player):
        if type(player) == Bot:
            player.play_next_best(self.state)
        else:
            move = player.get_player_move(self)
            self.set_square(move, "O")

    def play(self, players):
        random.shuffle(players)

        turn = 0
        end = False
        while not end:
            print(self)
            print(f"It's {players[turn % 2]}'s turn")
            self.make_move(players[turn % 2])
            turn += 1
            end, winner = self.is_end_game()

        print(self)
        print("WINNER:", winner)


class Node:
    def __init__(self, state: list, type, father):
        self.state = deepcopy(state)
        self.type = type
        self.father = father
        self.value = None
        self.alpha = None if not self.father else self.father.alpha
        self.beta = None if not self.father else self.father.beta
        self.best_node = None
        self.best_nodes = None


class Bot:
    def __init__(self, tablero: Tablero):
        self.tablero = tablero
        self.nodes = dict()
        self.types = ["max", "min"]
        self.symbols = {"max": "X", "min": "O"}
        self.moves = []
        self.root = None

    def __str__(self):
        return "Bot"

    def isGoal(self, state):
        return self.tablero.is_end_game(state)

    def get_possible_moves(self, state):
        return self.tablero.get_possible_moves(state)

    def get_result(self, action, symbol, state):
        state_copy = deepcopy(state)
        return self.tablero.get_result(action, symbol, state_copy)

    def generate_tree(self, node=None):
        global global_nodes

        if not node:
            node = Node(self.tablero.state, "max", None)
            self.nodes[str(node.state)] = node
            self.root = node

            global_nodes += 1

        is_goal, winner = self.isGoal(node.state)

        if is_goal:
            node.value = 0 if not winner else 1 if self.symbols["max"] == winner else -1
            self.tablero.winner = None
            return node.value, [node]

        new_type = "max" if node.type == "min" else "min"

        actions = self.get_possible_moves(node.state)
        values = []
        new_nodes = {}

        for action in actions:
            result = self.get_result(
                action, self.symbols[node.type], deepcopy(node.state)
            )
            if str(result) not in self.nodes:
                new_node = Node(result, new_type, node)
                global_nodes += 1
                value, best_nodes = self.generate_tree(new_node)
                new_node.best_nodes = best_nodes
                self.nodes[str(new_node.state)] = new_node
            else:
                new_node = self.nodes[str(result)]
                value = new_node.value
                best_nodes = new_node.best_nodes

            if value in values:
                length = len(new_nodes[value])
                if len(best_nodes) + 1 < length:
                    new_nodes[value] = [node, *best_nodes]
            else:
                values.append(value)
                new_nodes[value] = [node, *best_nodes]

            if node.type == "max":
                if node.alpha is None or new_node.beta > node.alpha:
                    node.alpha = new_node.beta
            else:
                if node.beta is None or new_node.alpha < node.beta:
                    node.beta = new_node.alpha

            if node.alpha and node.beta and node.alpha >= node.beta:
                break

        if node.type == "max":
            # print(node.alpha)
            best = max(values)  # node.alpha  # max(values)
        else:
            best = min(values)  # node.beta  # min(values)

        node.value = best
        node.best_node = new_nodes[best][1]

        return best, new_nodes[best]

    def get_next_best(self, state) -> Node:
        node = self.nodes[str(state)]
        return node.best_node

    def play_next_best(self, state):
        if not self.root:
            self.generate_tree()
        node = self.get_next_best(state)
        self.tablero.state = deepcopy(node.state)


class Player:
    def __init__(self, name) -> None:
        self.name = name

    def __repr__(self) -> str:
        return self.name

    def get_player_move(self, tablero: Tablero) -> int:
        move = ""
        possible_moves = tablero.get_possible_moves()
        while not move.isnumeric() or int(move) not in possible_moves:
            move = input("Player move: ")

        move = int(move)
        return move


global_nodes = 0

if __name__ == "__main__":
    t = Tablero()
    bot = Bot(t)
    player = Player("Lyδia")
    t.play([bot, player])
    bot.generate_tree()

    print("Nodos buscados:", global_nodes)
