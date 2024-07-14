from Problem import Problem
import time


class SearchAgent:
    def __init__(self, problem: Problem, type_search: str = "BFS"):
        self.problem = problem
        self.type_search = type_search
        self.explored = set()

        if self.type_search == "BFS":
            self.frontier = Queue()
        elif self.type_search == "DFS":
            self.frontier = Stack()
        elif self.type_search == "A*":
            self.frontier = AStarQueue()
        elif self.type_search == "Greedy":
            self.frontier = AStarQueue()

    def search(self):
        if self.is_frontier_empty():
            start = self.problem.get_start()
            self.add_to_frontier(start)

        node = self.get_next_node()
        if node:
            if self.is_goal(node):
                return node

            self.add_to_explored(node)
            self.expand_frontier(node)
            return self.search()

        return None

    def get_next_node(self):
        if self.is_frontier_empty():
            return None
        return self.frontier.get()

    def expand_frontier(self, node):
        for child in self.problem.get_next_nodes(node):
            if self.type_search in ("BFS", "DFS"):
                if not self.check_explored(child) and not self.frontier.exists(child):
                    child.previous = node
                    self.add_to_frontier(child)
            else:
                if self.type_search == "A*":
                    child.cost = self.get_a_star_cost(child)
                else:
                    child.cost = self.heuristic(child)

                if not self.frontier.exists(child) and not self.check_explored(child):
                    child.previous = node
                    self.add_to_frontier(child)

    def add_to_frontier(self, node):
        self.frontier.add(node)

    def is_goal(self, node):
        return node.position == self.problem.end.position

    def is_frontier_empty(self):
        return self.frontier.is_empty()

    def add_to_explored(self, node):
        self.explored.add(node)

    def check_explored(self, node):
        for object in self.explored:
            if object.position == node.position:
                if self.type_search in ("A*", "Greedy"):
                    if object.cost <= node.cost:
                        return True
                else:
                    return True
        return False

    def get_a_star_cost(self, node):
        # node.path_cost = self.get_path_cost(node)
        return self.get_path_cost(node) + self.heuristic(node)

    def heuristic(self, node):
        end = self.problem.get_end()
        return abs(end.position[0] - node.position[0]) + abs(
            end.position[1] - node.position[1]
        )

    def get_path_cost(self, node):
        return node.path_cost


class Stack:
    def __init__(self):
        # LIFO
        self.stack = []

    def add(self, node):
        self.stack.append(node)

    def get(self):
        return self.stack.pop()

    def is_empty(self):
        return len(self.stack) == 0

    def exists(self, object):
        # print(object, self.stack)
        for o in self.stack:
            if object.position == o.position:
                return True
        return False


class Queue:
    def __init__(self):
        # FIFO
        self.queue = []

    def add(self, node):
        self.queue.append(node)

    def get(self):
        return self.queue.pop(0)

    def is_empty(self):
        return len(self.queue) == 0

    def exists(self, object):
        for o in self.queue:
            if object.position == o.position:
                return True
        return False


class AStarQueue:
    def __init__(self):
        self.queue = []

    def add(self, node):
        i = 0
        position_founded = False
        while i < len(self.queue) and not position_founded:
            if node.cost < self.queue[i].cost:
                position_founded = True
            else:
                i += 1

        self.queue.insert(i, node)

    def get(self):
        return self.queue.pop(0)

    def is_empty(self):
        return len(self.queue) == 0

    def exists(self, object):
        for o in self.queue:
            if object.position == o.position and object.cost >= o.cost:
                return True
        return False

    def __repr__(self) -> str:
        return str(self.queue)


if __name__ == "__main__":
    problem_representation = """
    . . . . . . #
    . # # # # . #
    . . . . # . #
    . # # E # . #
    . # . . # . #
    . # # # # . #
    S . . . . . #
    """

    problem_representation = """
    . . . . . . . . E
    . # # # # # # # .
    . # . . . . . # .
    . # . # # # . # . 
    . # . # . . . # .
    . # . # . # # . . 
    . S . # . . . . #
    """
    problem_representation = """
    E # # # # # # # # # # # # # # # # # # # #
    · · · · · · # · # · # · · · · · # · · · #
    # # # · # # # · # · # # # · # # # · # · #
    # · # · · · · · # · · · · · · · # · # · #
    # · # · # # # · # · # # # # # # # · # · #
    # · · · · · # · # · · · · · · · # · # · #
    # · # # # # # · # · # # # · # # # · # # #
    # · · · · · # · · · # · · · # · · · # · #
    # · # # # · # # # · # # # · # # # · # · #
    # · · · # · # · · · # · · · # · · · # · #
    # · # # # # # # # · # # # # # # # · # · #
    # · # · # · · · · · # · # · · · # · · · #
    # · # · # · # # # # # · # · # # # · # · #
    # · # · # · · · # · · · · · # · # · # · #
    # · # · # · # · # # # # # · # · # · # # #
    # · # · · · # · · · # · · · # · · · # · #
    # · # # # · # · # · # # # · # · # · # · #
    # · # · # · # · # · · · · · · · # · · · #
    # · # · # # # # # # # # # # # · # # # · #
    # · · · · · · · · · · · # · · · # · · · #
    # # # # # # # # # # # # # # # # # # # S #

    """

    # print(problem.representation)
    for type_ in ["DFS", "BFS", "Greedy", "A*"]:
        print(type_)
        problem = Problem(problem_representation)
        search_agent = SearchAgent(problem, type_)
        end = search_agent.search()
        solution = []
        while end:
            # print(end)
            solution.append(end)
            end = end.previous

        problem.represent_solution(solution, search_agent.explored)

        print()
