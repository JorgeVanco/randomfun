from copy import deepcopy
from colored import Fore, Style


class Nodo:
    def __init__(self, value) -> None:
        self.value = value
        self.previous = None
        self.cost = 0


class Calle:
    def __init__(self) -> None:
        pass


class Grafo:
    def __init__(self, dirigido) -> None:
        self.vertex = None
        self.adyacencia = dict()
        self.dirigido = dirigido

    def anadir_edge(self, from_node, to_node, coste=1):
        dict_nodo = self.adyacencia.get(from_node, dict())
        dict_nodo[to_node] = coste
        self.adyacencia[from_node] = dict_nodo
        if not self.dirigido:
            dict_nodo = self.adyacencia.get(to_node, dict())
            dict_nodo[from_node] = coste
            self.adyacencia[to_node] = dict_nodo

    def print_graph(self):
        for node in self.adyacencia:
            print(node, self.adyacencia[node])


class Problem(Grafo):
    def __init__(self):
        self.start = None
        self.end = None

    def find_start_end(self):
        pass
        # return Nodo(start, 0), Nodo(end, None)

    def get_next_nodes(self, nodo):
        return list(self.adyacencia[nodo].keys())

    def get_start(self):
        return self.start

    def get_end(self):
        return self.end

    def represent_solution(self, solution):
        pass


# class Nodo:
#     def __init__(self, position, cost):
#         self.position = position
#         self.previous = None
#         self.cost = 0
#         self.path_cost = cost

#     def __repr__(self):
#         return f"{self.position}, {self.path_cost}, {self.cost}"
