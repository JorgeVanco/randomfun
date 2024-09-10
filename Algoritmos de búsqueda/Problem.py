from copy import deepcopy
from typing import Any, Generator
from colored import Fore, Style


class Nodo:
    """
    Represents a node in the search space. Each node has:
    - position: A tuple (i, j) indicating the position in the grid.
    - previous: A reference to the previous node in the path.
    - cost: The total cost to reach this node (used in A* and Greedy algorithms).
    - path_cost: The path cost to reach this node from the start (incremented at each step).
    """

    def __init__(self, position, cost):
        self.position = position  # Tuple representing the (i, j) position of the node
        # Pointer to the previous node (used to trace back the solution)
        self.previous = None
        self.cost = 0  # Heuristic or total cost (used in informed search)
        self.path_cost = cost  # Cumulative cost to reach this node

    def __repr__(self):
        """
        Returns a string representation of the node, showing its position and costs.
        """
        return f"({self.position}, {self.path_cost}, {self.cost})"


class Problem:
    """
    Represents a problem to be solved by the search agent. The problem is defined by:
    - representation: A string-based grid representation where 'S' is the start, 'E' is the end,
      '.' are walkable spaces, and '#' are obstacles.

    This class provides methods to:
    - Parse the grid and find the start and end nodes.
    - Get valid neighboring nodes from a given position.
    - Visualize the solution path and explored nodes.
    """

    def __init__(self, representation) -> None:
        """
        Initializes the problem by converting the input string representation into a 2D grid.
        Identifies the start ('S') and end ('E') positions.
        """
        # Clean and split input into lines
        self.representation = representation.replace(" ", "").split("\n")

        end_representation = []
        for line in self.representation:
            if line:
                # Create a 2D grid representation
                end_representation.append([char for char in line])
        self.representation = end_representation

        # Find and store start and end nodes
        self.start, self.end = self.find_start_end()

    def find_start_end(self) -> tuple[Nodo, Nodo]:
        """
        Identifies and returns the start ('S') and end ('E') positions in the grid.
        Returns them as Nodo objects.
        """
        for i in range(len(self.representation)):
            for j in range(len(self.representation[i])):
                if self.representation[i][j] == "E":  # End position
                    end = (i, j)
                elif self.representation[i][j] == "S":  # Start position
                    start = (i, j)
        return Nodo(start, 0), Nodo(end, None)

    def get_next_nodes(self, nodo: Nodo) -> Generator[Nodo, Any, None]:
        """
        Returns a list of valid neighboring nodes from the given node (nodo).
        The neighbors are determined by checking the adjacent grid positions that are not blocked by walls ('#').
        Each valid neighbor is returned as a Nodo object with an updated path cost.
        """
        i, j = nodo.position
        nodes = []

        # Check four possible directions (up, right, left, down)
        for i_, j_ in zip([i - 1, i, i, i + 1], [j, j + 1, j - 1, j]):
            # Ensure the position is within grid bounds and not a wall ('#')
            is_inside_bounds = (
                i_ >= 0
                and i_ < len(self.representation)
                and j_ >= 0
                and j_ < len(self.representation[i])
            )
            if is_inside_bounds and self.representation[i_][j_] != "#":
                # Add the valid neighboring node with incremented path cost
                # nodes.append(Nodo((i_, j_), nodo.path_cost + 1))
                yield Nodo((i_, j_), nodo.path_cost + 1)
        # return nodes

    def get_start(self) -> Nodo:
        """
        Returns the start node of the problem.
        """
        return self.start

    def get_end(self) -> Nodo:
        """
        Returns the end node of the problem.
        """
        return self.end

    def represent_solution(self, solution, explored) -> None:
        """
        Visualizes the solution path and explored nodes on the grid.
        - Explored nodes are marked with blue '%'.
        - The solution path (excluding the start and end) is marked with green '$'.
        - Obstacles ('#') are displayed in red.
        """
        # Make a deep copy of the grid to modify for visualization
        rep = deepcopy(self.representation)

        # Mark explored nodes in blue (excluding start and end)
        for node in explored:
            i, j = node.position
            if rep[i][j] not in ("E", "S"):
                rep[i][j] = f"{Fore.blue}%{Style.reset}"

        # Mark the solution path in green (excluding start and end)
        for node in solution[1:-1]:
            i, j = node.position
            rep[i][j] = f"{Fore.green}${Style.reset}"

        # Print the updated grid with colors
        for i in range(len(rep)):
            for j in range(len(rep[i])):
                if rep[i][j] == "#":
                    print(f"{Fore.red}#{Style.reset}", end=" ")  # Red for walls
                else:
                    print(rep[i][j], end=" ")  # Default for other elements
            print()
