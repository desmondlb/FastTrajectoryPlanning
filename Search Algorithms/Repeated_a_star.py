from matplotlib import pyplot

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


class FastTrajectoryReplanning():

    def __init__(self) -> None:
        self.open_list = []
        self.closed_list = []

        # valid moves up, down, left, right
        self.valid_moves = [(0, 1), (0, -1), (-1, 0), (1, 0)]

        self.counter = 0

        self.grid = None
        self.real_grid = None

    def print_path(self, current) -> list:
        '''
            This function prints a list of positions that
            the agent travels to get from start to the goal state.
        '''
        path = []

        while (current.parent):
            path.append(current.position)
            current = current.parent

        #print(path[::-1])
        return path[::-1]

    def get_manhattan_dist(self, start, goal) -> int:
        '''
            This function returns Norm 1 distance between start and the goal state
        '''
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def get_priority_node(self) -> Node:
        '''
            This function returns the node with the least f value
            from the Open list
        '''
        priority_node = self.open_list[0]

        for n in self.open_list:
            if (n.f <= priority_node.f):
                priority_node = n

        return priority_node

    def get_valid_moves(self, current) -> list:
        '''
            Here we check the following
                1. Has the node been already visited - not implemented here yet
                2. Does the node lie within grid boundaries
                3. Is there an obstacle? (TBD)
                4. ***TD - Updated the function parameter to take position tuple instead of node object
        '''
        current_legal_moves = []

        for move in self.valid_moves:
            new_position = tuple(map(sum, zip(move, current)))

            if (new_position[0] >= 0 and new_position[1] >= 0):
                if (new_position[0] < len(self.grid) and new_position[1] < len(self.grid)):
                    if self.grid[new_position[0]][new_position[1]] == 1:
                        pass
                    else:
                        current_legal_moves.append(new_position)

        return current_legal_moves

    def perform_move(self, move, current_position) -> tuple:
        '''
            This function performs the selected move on the current_position
            Returns the updated position
        '''
        return tuple(map(sum, zip(move, current_position)))

    def check_node_in_open_list(self, child_state) -> bool:
        '''
            This function checks whether a child node is in the open list
            and whether it has a better f value
        '''
        for n in self.open_list:
            if ((n.position == child_state.position) and n.f < child_state.f):
                return True

        return False

    def check_node_in_closed_list(self, child_state) -> bool:
        '''
            This function checks whether a child node is in the closed list
            and whether it has a better f value..... TD - Redundant Step
        '''
        for n in self.closed_list:
            if ((n.position == child_state.position) and n.f < child_state.f):
                return True

        return False

    def a_star(self, grid, start, goal) -> None:
        '''
            Implementation of the simple A* algorithm
        '''
        start_node = Node(position=start)
        start_node.g = 0
        self.open_list.append(start_node)

        while (self.open_list != []):

            current = self.get_priority_node()
            current.h = self.get_manhattan_dist(start, goal)    # TD - shouldn't this be self.get_manhattan_dist(current.position, goal)?
            current.f = current.g + current.h

            # Popping the node from the open list
            self.open_list.remove(current)

            if current.position == goal:
                return current

            # f_n = self.counter + h_n

            #################### DEBUGGING ################################

            if current.position == (2, 2):
                print(current.position)

            if current.position == (3, 4):
                print(current.position)

            #######################################################

            # TD - current should be added to closed_list here
            self.closed_list.append(current)

            moves = self.get_valid_moves(current.position)

            for move in moves:
                child_state = Node(move)
                child_state.parent = current

                # else:
                child_state.h = self.get_manhattan_dist(child_state.position, goal)
                child_state.g = current.g + self.get_manhattan_dist(
                    child_state.position, current.position)
                child_state.f = child_state.g + child_state.h

                if self.check_node_in_closed_list(child_state):     #checks if the node has been visited
                    continue

                if self.check_node_in_open_list(child_state):
                    continue

                else:
                    self.open_list.append(child_state)

            #self.closed_list.append(current)      //TD - shifted to before the loop
        if self.open_list == []:
            return

    def run(self, start=None, goal=None) -> None:
        '''
            This function runs the A* algorithm on the generated grid
        '''
        #self.generate_grid() __ generate grid in main function
        self.open_list = []
        self.closed_list = []
        curr = self.a_star(self.grid, start, goal)
        # self.print_path(curr)
        '''
        if curr:
            self.print_path(curr)
        else:
            print("Cannot reach the target")
        '''
        return self.print_path(curr)

    def generate_grid(self) -> None:
        '''
            This function generates N*N grid with the following properties
                1. Obstacles generated with 30% probability to construct a maze like structure
                2. Target position denoted with "X"
                3. Obstacles denoted with 1
        '''
        # Dummy grid

        '''
        self.grid = [[0, 0, 0],
                     [0, 0, 0],
                     [0, 0, 0]]

        self.real_grid = [[0, 1, 'X'],
                          [0, 1, 0],
                          [0, 0, 0]]
        '''

        #agent's memory
        self.grid = [[0, 0, "X", 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0]]

        #real environment grid
        self.real_grid = [[0, 1, "X", 0, 0, 0, 0],
                          [0, 1, 1, 1, 0, 1, 0],
                          [0, 0, 0, 1, 1, 0, 0],
                          [0, 1, 1, 1, 0, 1, 0],
                          [0, 1, 1, 0, 0, 1, 0],
                          [0, 0, 1, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0]]



    #function to move agent through the grid; path recorded in travelled_path
    def move_in_real_grid(self, current_state, path) -> list:
        travelled_path = []
        for cell in path:
            if obj.real_grid[cell[0]][cell[1]] != 1:
                current_state = cell
                travelled_path.append(current_state)
            else:
                break
        return travelled_path


    #uses get_valid_moves() to update agent's memory
    def observe_nearby_cells(self, current_state) -> None:
        field_of_view = self.get_valid_moves(current=current_state)
        for cell in field_of_view:
            if self.real_grid[cell[0]][cell[1]] == 1 and self.grid[cell[0]][cell[1]] == 0:
                self.grid[cell[0]][cell[1]] = 1


    #function to visualize the final path taken by the agent in the grid (needs tweaking)
    def temporary_visualize(self, path):
        for point in path:
            self.real_grid[point[0]][point[1]] = 0.5
        pyplot.imshow(self.real_grid)
        pyplot.show()




if __name__ == "__main__":
    obj = FastTrajectoryReplanning()
    obj.generate_grid()
    start_state = (0, 0)
    goal_state = (0, 2)
    final_path = []
    final_path.append(start_state)
    end = False

    '''
    This loop executes the following logic
        1. agent observes nearby cells and accordingly updates FastTrajectoryPlanning.grid
        2. derives a planned path using a_star and grid
        3. travels in the real_grid based on planned_path
        4. appends the travelled path to the final_path
        6. prints and visualizes the final path taken
    
    '''
    while not end:
        obj.observe_nearby_cells(current_state=start_state)
        #print(obj.grid)
        #print(start_state)
        #print(goal_state)
        planned_path = obj.run(start=start_state, goal=goal_state)
        #print(planned_path)

        travelled_path = obj.move_in_real_grid(current_state=start_state, path=planned_path)
        #print(travelled_path)

        if travelled_path[-1] == goal_state:
            end = True
        else:
            start_state = travelled_path[-1]

        final_path.extend(travelled_path)

    print(final_path)
    obj.temporary_visualize(path=final_path)




    #path = obj.run(start=(0, 0), goal=(0, 2))
