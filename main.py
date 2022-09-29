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

    
    def print_path(self, current):
        path = []

        while(current.parent):
            path.append(current.position)
            current = current.parent
        
        print(path[::-1])
        
    def get_manhattan_dist(self, start, goal) -> int:
        '''
            This function returns Norm 1 distance between start and the goal state
        '''
        return abs(start[0]-goal[0]) + abs(start[1]-goal[1])

    def get_priority_node(self) -> Node:
        '''
            This function returns the node with the least f value
            from the Open list
        '''
        priority_node = self.open_list[0]

        for n in self.open_list:
            if(n.f <= priority_node.f):
                priority_node = n

        return priority_node

    def get_valid_moves(self, current) -> list:
        '''
            Here we check the following
                1. Has the node been already visited
                2. Does the node lie within grid boundaries
                3. Is there an obstacle? (TBD)
        '''
        current_legal_moves = []

        for move in self.valid_moves:
            new_position = tuple(map(sum, zip(move, current.position)))
            if(new_position[0] >= 0 and new_position[1] >= 0):
                if(new_position[0] < len(self.grid) and new_position[1] < len(self.grid)):
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
            if((n.position == child_state.position) and n.f < child_state.f):
                return True             
            
        return False

    def check_node_in_closed_list(self, child_state) -> bool:
        '''
            This function checks whether a child node is in the closed list
            and whether it has a better f value
        '''
        for n in self.closed_list:
            if((n.position == child_state.position) and n.f < child_state.f):
                return True             
            
        return False

    def a_star(self, grid, start, goal) -> None:
        '''
            Implementation of the simple A* algorithm
        '''
        start_node = Node(position=start)
        start_node.g = 0
        self.open_list.append(start_node)

        while(self.open_list != []):
            
            current = self.get_priority_node()
            current.h = self.get_manhattan_dist(start, goal)
            current.f = current.g + current.h

            # Popping the node from the open list
            self.open_list.remove(current)

            if current.position == goal:
                return current

            # f_n = self.counter + h_n

            moves = self.get_valid_moves(current)

            for move in moves:
                child_state = Node(move)
                child_state.parent = current
                
                # else:
                child_state.h = self.get_manhattan_dist(child_state.position, goal)
                child_state.g = current.g + self.get_manhattan_dist(
                    child_state.position, current.position)
                child_state.f = child_state.g + child_state.h

                if self.check_node_in_closed_list(child_state):
                    continue

                if self.check_node_in_open_list(child_state):
                    continue
                
                else:
                    self.open_list.append(child_state)

            self.closed_list.append(current)

            


    def run(self, start=None, goal=None) -> None:
        '''
            This function runs the A* algorithm on the generated grid
        '''
        self.generate_grid()
        curr = self.a_star(self.grid, start, goal)
        self.print_path(curr)

    def generate_grid(self) -> None:
        '''
            This function generates N*N grid with the following properties
                1. Obstacles generated with 30% probability to construct a maze like structure
                2. Target position denoted with "X"
                3. Obstacles denoted with 1
        '''

        # Dummy grid
        self.grid = [[0,0,0,0,0],
                    [0,0,0,0,0],
                    [0,0,0,0,0],
                    [0,0,0,"X",0],
                    [0,0,0,0,0]]



if __name__ == "__main__":
    obj = FastTrajectoryReplanning()

    obj.run(start = (0, 0), goal = (3,3))