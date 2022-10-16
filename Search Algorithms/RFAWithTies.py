import json
from config import *
from matplotlib import pyplot
import heapq as hp
import itertools
from AnimatePath import AnimatePath


class Node():
    '''
        Here we create a node class which store the following
            1. Position of the node in the grid
            2. Parent of the node
            3. f(n),g(n) and h(n) values for the node
    '''

    def __init__(self, position=None, parent=None) -> None:
        self.position = position
        self.parent = parent
        self.f = 0
        self.g = 0
        self.h = 0


class RFAWithTies():

    def __init__(self, tie_break=LARGE_G_VALUE) -> None:
        self.open_list = []  # binary heap which sorts nodes according to least f,g, or h values
        self.closed_list = []  # list with expanded nodes
        self.open_list_dict = dict()  # connects node objects to the binary heap

        # -----------------------------------
        # Valid moves: up, down, left, right
        # -----------------------------------
        self.valid_moves = [(0, 1), (0, -1), (-1, 0), (1, 0)]

        # --------------------------------------------------------------
        # Used to break ties in favour of either large or small g value
        # --------------------------------------------------------------
        self.tie_breaker_pref = tie_break
        self.grid_worlds = None
        with open('Gridworlds/gridworlds.json', 'r') as f:
            # Reading from json file
            self.grid_worlds = json.load(f)

        self.grid_world = None

        self.start = None
        self.target = None
        self.actual_grid = None

        self.explored_grid = None

        self.counter_expanded_nodes = 0

        # use counter to resolve ties
        self.counter = itertools.count()  # MISSION_HEAP

        self.observed_paths = []

    def print_path(self, current) -> list:
        '''
            This function returns a list of positions that
            the agent travels to get from start to the goal state.
        '''
        path = []

        while (current.parent):
            path.append(current.position)
            current = current.parent

        return path[::-1]

    def get_manhattan_dist(self, start, goal) -> int:
        '''
            This function returns Norm 1 distance between start and the goal state
        '''
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def open_list_pop(self) -> tuple:  # MISSION_HEAP
        '''
            TD - This function used to be get_priority(node)
            This function returns the node with the least f value
            from the Open list
            TBD: Implementation with priority heap/queue
            This function also implements the effect of ties
            i.e. It breaks the ties in favour of
        '''

        f, t, c, chosen_position = hp.heappop(self.open_list)
        return f, t, c, chosen_position

    def get_valid_moves(self, current) -> list:
        '''
            Here we check the following
                1. Does the node lie within grid boundaries?
                2. Is there an obstacle?
        '''
        current_legal_moves = []

        for move in self.valid_moves:

            # ---------------------------------------------------------
            # Adding the move to current position updates the position
            # ---------------------------------------------------------
            new_position = tuple(map(sum, zip(move, current)))

            if (new_position[0] >= 0 and new_position[1] >= 0):
                if (new_position[0] < len(self.explored_grid) and new_position[1] < len(self.explored_grid)):
                    if not self.explored_grid[new_position[0]][new_position[1]] == 1:
                        current_legal_moves.append(new_position)

        return current_legal_moves

    def check_and_update_node_in_open_list(self, child_state) -> None:
        '''
            This function checks whether a child node is in the open list
            and whether it has a better f value
            It also adds the node to the open list if it's not already there
        '''
        existing_node = self.open_list_dict.get(child_state.position)  # MISSION_HEAP

        if existing_node:

            if child_state.f < existing_node.f:
                # ---------------------------------
                # update the node in the open list
                # ---------------------------------
                existing_node.g = child_state.g
                existing_node.h = child_state.h
                existing_node.f = child_state.f
                existing_node.parent = child_state.parent

        else:
            self.open_list_push(child_state)

    def check_node_in_closed_list(self, child_state) -> bool:
        '''
            This function checks whether a child node is in the closed list
        '''
        for n in self.closed_list:
            if (n.position == child_state.position):
                return True

        return False



    def open_list_push(self, node) -> Node:
        '''
            pushes new node onto the open_list heap and dictionary
        '''
        if self.tie_breaker_pref == LARGE_G_VALUE:
            hp.heappush(self.open_list, (node.f, node.h, next(self.counter), node.position))
        else:
            hp.heappush(self.open_list, (node.f, node.g, next(self.counter), node.position))
        self.open_list_dict[node.position] = node

    def a_star(self, start_node, goal) -> Node:
        '''
            Implementation of the A* algorithm
        '''
        # --------------------------
        # Push node in the open list
        # --------------------------
        self.open_list_push(start_node)

        current = start_node

        while (self.open_list != []):

            # ---------------------------------------------
            # Calculate the h and f values of Current node
            # ---------------------------------------------
            f_value, tie_break, c, cell = self.open_list_pop()
            current = self.open_list_dict[cell]
            current.h = self.get_manhattan_dist(current.position, goal)
            current.f = current.g + current.h

            # ------------------------------------
            # Popping the node from the open list
            # ------------------------------------
            del self.open_list_dict[cell]

            if current.position == goal:
                break
            # ---------------------------------------------------
            # Append the current node to the closed list
            # ---------------------------------------------------
            self.closed_list.append(current)

            moves = self.get_valid_moves(current.position)

            for move in moves:
                child_state = Node(move)
                # -------------------------------------------
                # Set the child's parent as the current node
                # -------------------------------------------
                child_state.parent = current

                # -----------------------------------------------
                # Update the h, g and f values of the child node
                # -----------------------------------------------
                child_state.h = self.get_manhattan_dist(child_state.position, goal)
                child_state.g = current.g + self.get_manhattan_dist(
                    child_state.position, current.position)
                child_state.f = child_state.g + child_state.h

                # ------------------------------------------------
                # If the node is already in closed list then skip
                # ------------------------------------------------
                if self.check_node_in_closed_list(child_state):
                    continue

                # ----------------------------------------------
                # If the node is already in open list then update the node
                # depending on the g value Else push the node in the open list
                # ----------------------------------------------
                self.check_and_update_node_in_open_list(child_state)

        # ----------------------------------------------------------------------
        # If open list is empty in the end then return current to indicate no path
        # ----------------------------------------------------------------------

        self.counter_expanded_nodes += len(self.closed_list)
        return current

    def move_in_real_grid(self, path) -> list:
        '''
            uses get_valid_moves() to update agent's memory
        '''
        travelled_path = []
        for cell in path:
            if self.actual_grid[cell[0]][cell[1]] != 1:
                travelled_path.append(cell)
                obs = self.observe_nearby_cells(current_state=cell)
                self.observed_paths.append(obs)
            else:
                break
        return travelled_path

    def observe_nearby_cells(self, current_state) -> list:
        '''
            uses get_valid_moves() to update agent's memory
        '''
        explored = []
        field_of_view = self.get_valid_moves(current=current_state)
        for cell in field_of_view:
            if self.actual_grid[cell[0]][cell[1]] == 1 and self.explored_grid[cell[0]][cell[1]] == 0:
                self.explored_grid[cell[0]][cell[1]] = 1
                explored.append(cell)
        return explored

    def animate_path(self, path):
        '''
            function to visualize the final path taken by the agent in the grid
        '''
        ani = AnimatePath(grid=self.actual_grid, path=path, start=self.start, target=self.target, observed = self.observed_paths)
        ani.show_path()

    def visualize(self, path):
        for point in path:
            self.actual_grid[point[0]][point[1]] = 4
        self.actual_grid[self.start[0]][self.start[1]] = 3
        self.actual_grid[self.target[0]][self.target[1]] = 5

        pyplot.imshow(self.actual_grid, cmap="RdBu")
        pyplot.show()


    def run(self, grid_index=0) -> None:
        '''
            This function runs the A* algorithm on the generated grid
        '''
        self.generate_grid(grid_index)
        final_path = [self.start]
        start_node = Node(position=self.start)

        end = False
        path_exist = True

        

        while path_exist and not end:
            # ---------------------------------------------------
            # Check the surroundings and update the explored grid
            # ---------------------------------------------------
            observed_node = self.observe_nearby_cells(current_state=self.start)
            self.observed_paths.append(observed_node)

            # ----------------------------------------------
            # Empty open and closed list
            # ----------------------------------------------
            self.open_list = []
            self.closed_list = []
            self.open_list_dict.clear()

            planned_dest = self.a_star(start_node, self.target)
            if planned_dest.position == self.target:

                # ---------------------------------------------------------------
                # Make agent move along the planned path till it hits an obstacle
                # ---------------------------------------------------------------
                travelled_path = self.move_in_real_grid(path=self.print_path(planned_dest))

                if (travelled_path and travelled_path[-1] == self.target):
                    end = True
                else:
                    # --------------------------------------------------------
                    # Set a new start state as the last node in travelled path
                    # --------------------------------------------------------
                    for n in self.closed_list:
                        if (n.position == travelled_path[-1]):
                            start_node = n

                final_path.extend(travelled_path)
            else:
                path_exist = False

        if not path_exist:
            print("Cannot reach the target, nodes expanded : " + str(self.counter_expanded_nodes))

        else:
            print("Number of nodes expanded : " + str(self.counter_expanded_nodes))
            #print(len(final_path))
            self.animate_path(path=final_path)    #uncomment to animate
            self.visualize(path=final_path)       #uncomment to visualize

    def generate_grid(self, grid_index) -> None:
        '''
            This function generates N*N grid with the following properties
                1. Obstacles generated with 30% probability to construct a maze like structure
                2. Target position generated in the lower right corner of the maze
                3. Agent position generated in the upper right corner of the maze
                3. Obstacles denoted with 1
        '''

        self.grid_world = self.grid_worlds.get(str(grid_index))

        self.start = tuple(self.grid_world.get("Start"))
        self.target = tuple(self.grid_world.get("Target"))
        self.actual_grid = self.grid_world.get("Grid")

        # Dummy Test Case
        # self.start = (0,0)
        # self.target = (4,4)
        # self.actual_grid = [[0,0,0,0,0],
        #                 [0,0,0,0,0],
        #                 [0,0,0,0,0],
        #                 [0,0,0,0,0],
        #                 [0,0,0,0,"X"]]

        self.explored_grid = [[0] * len(self.actual_grid) for _ in range(len(self.actual_grid))]


if __name__ == "__main__":
    obj_rfa = RFAWithTies(tie_break=LARGE_G_VALUE)
    # for i in range(0, 30):
    obj_rfa.run(grid_index=2)
    obj_rfa.counter_expanded_nodes = 0
