from Problem import Problem, Nodo

# Maze generator: https://www.dcode.fr/maze-generator


class SearchAgent:
    """
    This class defines a search agent that can perform different search algorithms
    (BFS, DFS, A*, Greedy) to find the solution to a given problem.

    Attributes:
    - problem: An instance of the Problem class defining the search space.
    - type_search: A string specifying the type of search algorithm to use ("BFS", "DFS", "A*", or "Greedy").
    - explored: A set that keeps track of the explored nodes to avoid revisiting them.
    - frontier: A data structure (queue, stack, or priority queue) used to store nodes that are yet to be explored.
    """

    def __init__(self, problem: Problem, type_search: str = "BFS") -> None:
        """
        Initializes the search agent with a given problem and search algorithm.
        Depending on the type of search, the appropriate data structure for the frontier is chosen.
        """
        self.problem = problem
        self.type_search = type_search
        self.explored = set()

        # Initialize frontier based on the search type
        if self.type_search == "BFS":
            # Breadth-First Search uses a FIFO queue
            self.frontier = Queue()
        elif self.type_search == "DFS":
            # Depth-First Search uses a LIFO stack
            self.frontier = Stack()
        elif self.type_search == "A*":
            # A* search uses a priority queue with cost function
            self.frontier = AStarQueue()
        elif self.type_search == "Greedy":
            # Greedy search also uses a priority queue with heuristic only
            self.frontier = AStarQueue()

    def search(self) -> Nodo | None:
        """
        Executes the chosen search algorithm to find a solution.
        Returns the goal node if found, or None if no solution exists.
        """
        # If the frontier is empty, start by adding the initial node to the frontier
        if self.is_frontier_empty():
            start = self.problem.get_start()
            self.add_to_frontier(start)

        # Get the next node from the frontier
        new_node = self.get_next_node()
        while new_node:
            node = new_node

            # If the current node is the goal, return the goal node
            if self.is_goal(node):
                return node

            # Mark the node as explored and expand the frontier with its neighbors
            self.add_to_explored(node)
            self.expand_frontier(node)

            # Get the next node from the frontier
            new_node = self.get_next_node()

        return None  # No solution found

    def get_next_node(self) -> None | Nodo:
        """
        Retrieves the next node from the frontier based on the search type.
        Returns None if the frontier is empty.
        """
        if self.is_frontier_empty():
            return None
        return self.frontier.get()

    def expand_frontier(self, node) -> None:
        """
        Expands the frontier by adding all valid neighboring nodes that haven't been explored or added to the frontier.
        For informed search algorithms (A* and Greedy), the node cost is calculated.
        """
        for child in self.problem.get_next_nodes(node):

            if self.type_search == "A*":
                child.cost = self.get_a_star_cost(child)
            elif self.type_search == "Greedy":
                child.cost = self.heuristic(child)

            if not self.frontier.exists(child) and not self.check_explored(child):
                child.previous = node
                self.add_to_frontier(child)

    def add_to_frontier(self, node) -> None:
        """
        Adds a node to the frontier (queue, stack, or priority queue).
        """
        self.frontier.add(node)

    def is_goal(self, node) -> bool:
        """
        Checks if the given node is the goal node by comparing its position with the problem's goal position.
        """
        return node.position == self.problem.end.position

    def is_frontier_empty(self) -> bool:
        """
        Checks if the frontier is empty.
        """
        return self.frontier.is_empty()

    def add_to_explored(self, node) -> None:
        """
        Adds a node to the explored set to ensure it is not revisited.
        """
        self.explored.add(node)

    def check_explored(self, node) -> bool:
        """
        Checks if a node has already been explored. For A* and Greedy, it ensures the new node has a lower or equal cost.
        """
        for object in self.explored:
            if object.position == node.position:
                if self.type_search in ("A*", "Greedy"):
                    if object.cost <= node.cost:
                        return True
                else:
                    return True
        return False

    def get_a_star_cost(self, node) -> float | int:
        """
        Calculates the cost of a node for A* search by combining path cost and heuristic cost.
        """
        return self.get_path_cost(node) + self.heuristic(node)

    def heuristic(self, node) -> float:
        """
        Heuristic function to estimate the distance to the goal.
        Uses Manhattan distance as the heuristic.
        """
        end = self.problem.get_end()
        return abs(end.position[0] - node.position[0]) + abs(
            end.position[1] - node.position[1]
        )

    def get_path_cost(self, node) -> float | int:
        """
        Returns the path cost accumulated in reaching the given node.
        """
        return node.path_cost


class Stack:
    """
    Stack class for DFS. Implements LIFO (Last In First Out) behavior.
    """

    def __init__(self) -> None:
        self.stack = []

    def add(self, node) -> None:
        self.stack.append(node)

    def get(self) -> Nodo:
        return self.stack.pop()

    def is_empty(self) -> bool:
        return len(self.stack) == 0

    def exists(self, object) -> bool:
        for o in self.stack:
            if object.position == o.position:
                return True
        return False
        # return any(object.position == o.position for o in self.stack)


class Queue:
    """
    Queue class for BFS. Implements FIFO (First In First Out) behavior.
    """

    def __init__(self) -> None:
        self.queue = []

    def add(self, node: Nodo) -> None:
        self.queue.append(node)

    def get(self) -> Nodo:
        return self.queue.pop(0)

    def is_empty(self) -> bool:
        return len(self.queue) == 0

    def exists(self, object: Nodo) -> bool:
        for o in self.queue:
            if object.position == o.position:
                return True
        return False
        # return any(object.position == o.position for o in self.queue)


class AStarQueue:
    """
    Priority Queue class for A* and Greedy search. Nodes are ordered by cost (A*) or heuristic (Greedy).
    """

    def __init__(self) -> None:
        self.queue = []

    def add(self, node) -> None:
        """
        Inserts a node in the queue, maintaining the order based on the node's cost.
        """
        i = 0
        position_founded = False
        while i < len(self.queue) and not position_founded:
            if node.cost < self.queue[i].cost:
                position_founded = True
            else:
                i += 1

        self.queue.insert(i, node)

    def get(self) -> Nodo:
        return self.queue.pop(0)

    def is_empty(self) -> bool:
        return len(self.queue) == 0

    def exists(self, object: Nodo) -> bool:
        for o in self.queue:
            if object.position == o.position and object.cost >= o.cost:
                return True
        return False
        # return any(
        #     object.position == o.position and object.cost >= o.cost for o in self.queue
        # )

    def __repr__(self) -> str:
        return str(self.queue)


if __name__ == "__main__":
    # Problem representations are different mazes; the agent will attempt to solve them.

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

    problem_representation = """
    S..############################
    ............#........#........#
    ####..#######..#..#######..#..#
    #..............#...........#..#
    #..##########..#..#######..####
    #........#..#..#.....#........#
    #..#######..#..#..####..#..####
    #..#...........#.....#..#.....#
    #######..#######..####..####..#
    #..#.....#........#........#..#
    #..#######..#######..#..####..#
    #........#........#..#..#.....#
    #..####..####..#############..#
    #..#..............#.....#.....#
    ####..##########..####..####..#
    #...........#.....#.....#.....#
    #..#######..####..#..#..####..#
    #........#..#........#..#.....#
    #..#######..####..#######..#..#
    #.....#.....#...........#..#..
    ############################.E
    """

    problem_representation = """
    E..########################################################################################
    ...#..#.....#..#.....#..#...........#..#.............................#.....#.....#........#
    #..#..#..#..#..#..#..#..####..####..#..#..####..#######..#..####..#######..####..#######..#
    #........#..#.....#..#.....#.....#..#........#.....#.....#.....#.....#.................#..#
    ####..#..####..#..####..#######..#############..#..#..#############..#..##########..####..#
    #.....#..#..#..#.....#.....#.....#.....#..#.....#..#..#..#.....#........#........#........#
    #######..#..####..####..#..#..####..####..####..#..####..#..##########..#..#############..#
    #..#...........#.....#..#.................#..#..#.....#........#..............#........#..#
    #..####..#..####..#..#..####..#######..#..#..#######..#..##########..#############..####..#
    #.....#..#.....#..#.....#.....#........#...........#.....#........#.....#.....#.....#.....#
    ####..#..#############..####..#..#..####..####..####..#..#..#..#..####..#..#..#..#..####..#
    #.....#..#........#........#..#..#..#.....#.....#.....#..#..#..#..#.....#..#.....#........#
    ####..#..#..#######..##########..#..####..################..####..####..#############..####
    #..#.....#..#.....#...........#..#.....#..#.....#...........#...........#..#..#..#........#
    #..#..####..#..####..##########..#############..####..#..####..#######..#..#..#..####..#..#
    #..#.....#..#.....#.....#..#..#.....#........#.....#..#..#..#..#.....#........#.....#..#..#
    #..#..####..#..####..####..#..##########..#######..#..#..#..#..#..#..#######..#..#..#######
    #..#..#...........#.....#..#..#..#..............#..#..#.....#..#..#..#........#..#..#..#..#
    #..#..####..####..#..#..#..#..#..#..#..####..#..#..#..#######..#..################..#..#..#
    #..............#..#..#..#..#..#..#..#.....#..#........#.....#..#..............#...........#
    #..#############..#..#..#..#..#..#..##########..####..#..#######..###################..####
    #..#........#..#.....#..#........#.....#.....#.....#.....#........#.....#...........#.....#
    ##########..#..#..####..#..####..#######..#..#..####..##########..#..####..#..####..#..#..#
    #.....#...........#.....#.....#.....#.....#.....#...........#..............#..#........#..#
    #..#..#..#..####..####..####..####..#..################..###################..#############
    #..#.....#..#..#.....#.....#..#...........#..#.....#..#.....#..#..........................#
    #..####..####..####..####..##########..#..#..#..####..#..####..#############..####..####..#
    #.....#.....#..#.....#.....#..#.....#..#..#........#.....#........#..............#.....#..#
    #..#..#######..#..#######..#..####..#..##########..#..##########..#..#..##########..####..#
    #..#.....#.....#.....#.....#....................#...........#........#...........#..#.....#
    ####..#######..#..####..##########..#..##########..#..#..#######..#..#..#######..#######..#
    #..#..#........#.....#........#.....#..#...........#..#...........#..#..#.....#..#.....#..#
    #..####..#..#######..#######..#..###################..#######..#######..#..####..#..#..#..#
    #..#.....#...........#..#..#..#...........#..#........#..............#..#.....#.....#..#..#
    #..####..#..#..####..#..#..#..#..#..#..####..#..#..#..#######..##########..#######..####..#
    #..#..#..#..#.....#.....#........#..#.....#.....#..#..#.....#..#..#..#...........#.....#..#
    #..#..#..####..#..#..#############..####..####..##########..####..#..#######..#..#######..#
    #.....#..#.....#..#..#.....#........#..#..#..#........#...........#.....#.....#.....#..#..#
    #..####..####..####..#..##########..#..#..#..#..#..#############..#..####..#######..#..#..#
    #........#........#.....#...........#........#..#...........#.................#...........
    ########################################################################################.S
    """

    # Loop through each search type and attempt to solve the problem
    for type_ in ["DFS", "BFS", "Greedy", "A*"]:
        print(type_)
        problem = Problem(problem_representation)
        search_agent = SearchAgent(problem, type_)
        try:
            end = search_agent.search()
        except RecursionError as e:
            end = None
            print("Couldn't solve it because it is too big")

        # Store and print the solution path
        solution = []
        while end:
            solution.append(end)
            end = end.previous

        # Represent the solution visually
        problem.represent_solution(solution, search_agent.explored)
        print()
