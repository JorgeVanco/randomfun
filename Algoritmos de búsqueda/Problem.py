from copy import deepcopy
from colored import Fore, Style


class Problem:
    def __init__(self, representation):
        self.representation = representation.replace(" ", "").split("\n")

        end_representation = []
        for line in self.representation:
            if line:
                end_representation.append([char for char in line])
        self.representation = end_representation

        self.start, self.end = self.find_start_end()

    def find_start_end(self):
        for i in range(len(self.representation)):
            for j in range(len(self.representation[i])):
                if self.representation[i][j] == "E":
                    end = (i, j)
                elif self.representation[i][j] == "S":
                    start = (i, j)
        return Nodo(start, 0), Nodo(end, None)

    def get_next_nodes(self, nodo):
        i, j = nodo.position
        nodes = []
        for i_, j_ in zip([i - 1, i, i, i + 1], [j, j + 1, j - 1, j]):
            is_inside_bounds = (
                i_ >= 0
                and i_ < len(self.representation)
                and j_ >= 0
                and j_ < len(self.representation[i])
            )
            if is_inside_bounds and self.representation[i_][j_] != "#":
                nodes.append(Nodo((i_, j_), nodo.path_cost + 1))

        return nodes

    def get_start(self):
        return self.start

    def get_end(self):
        return self.end

    def represent_solution(self, solution, explored):
        rep = deepcopy(self.representation)

        for node in explored:
            i, j = node.position
            if rep[i][j] not in ("E", "S"):
                rep[i][j] = f"{Fore.blue}%{Style.reset}"

        for node in solution[1:-1]:
            i, j = node.position
            rep[i][j] = f"{Fore.green}${Style.reset}"

        for i in range(len(rep)):
            for j in range(len(rep[i])):
                if rep[i][j] == "#":
                    print(f"{Fore.red}#{Style.reset}", end=" ")
                else:
                    print(rep[i][j], end=" ")
            print()


class Nodo:
    def __init__(self, position, cost):
        self.position = position
        self.previous = None
        self.cost = 0
        self.path_cost = cost

    def __repr__(self):
        return f"({self.position}, {self.path_cost}, {self.cost})"
