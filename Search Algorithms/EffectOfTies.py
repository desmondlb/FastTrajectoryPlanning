import json
from config import *
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

    def __init__(self, tie_break = LARGE_G_VALUE) -> None:
        self.open_list = []
        self.closed_list = []
        # self.actual_grid, self.explored_grid = self.generate_grid()

        #-----------------------------------
        # Valid moves: up, down, left, right
        #-----------------------------------
        self.valid_moves = [(0, 1), (0, -1), (-1, 0), (1, 0)]

        #--------------------------------------------------------------
        # Used to break ties in favour of either large or small g value
        #--------------------------------------------------------------
        self.tie_breaker_pref = tie_break
        self.grid_worlds = None
        with open('Gridworlds/grid.json', 'r') as f:
            # Reading from json file
            self.grid_worlds = json.load(f)
            
        self.grid_world = None

        self.start = None
        self.target = None
        self.actual_grid = None

        self.explored_grid = None

        self.counter_expanded_nodes = 0


    
    def print_path(self, current) -> None:
        '''
            This function prints a list of positions that
            the agent travels to get from start to the goal state.
        '''
        path = []

        while(current.parent):
            path.append(current.position)
            current = current.parent
        
        return path[::-1]
        
    def get_manhattan_dist(self, start, goal) -> int:
        '''
            This function returns Norm 1 distance between start and the goal state
        '''
        return abs(start[0]-goal[0]) + abs(start[1]-goal[1])

    def get_priority_node(self) -> Node:
        '''
            This function returns the node with the least f value
            from the Open list
            TBD: Implementation with priority heap/queue

            This function also implements the effect of ties
            i.e. It breaks the ties in favour of 
        '''
        llst_nodes_smallest_f = []
        priority_node = self.open_list[0]

        for n in self.open_list:
            if(n.f < priority_node.f):
                priority_node = n

        #---------------------------------------------
        # Push all the nodes with the smallest f value
        #---------------------------------------------
        for n in self.open_list:
            if(n.f == priority_node.f):
                llst_nodes_smallest_f.append(n)

        if len(llst_nodes_smallest_f)==1:
            return llst_nodes_smallest_f[0]

        #-------------------------------------------
        # Break ties in favour of the set preference
        #-------------------------------------------
        chosen_node = llst_nodes_smallest_f[0]

        for n in llst_nodes_smallest_f:
            if self.tie_breaker_pref==LARGE_G_VALUE:
                if(n.g > chosen_node.g):
                    chosen_node = n
            else:
                if(n.g < chosen_node.g):
                    chosen_node = n

        return chosen_node

    def get_valid_moves(self, current) -> list:
        '''
            Here we check the following
                1. Does the node lie within grid boundaries
                2. Is there an obstacle?
        '''
        current_legal_moves = []

        for move in self.valid_moves:

            #---------------------------------------------------------
            # Adding the move to current position updates the position
            #---------------------------------------------------------
            new_position = tuple(map(sum, zip(move, current)))

            if(new_position[0] >= 0 and new_position[1] >= 0):
                if(new_position[0] < len(self.explored_grid) and new_position[1] < len(self.explored_grid)):
                    if not self.explored_grid[new_position[0]][new_position[1]]==1:
                        current_legal_moves.append(new_position)

        return current_legal_moves


    def check_node_in_open_list(self, child_state) -> bool:
        '''
            This function checks whether a child node is in the open list
            and whether it has a better f value
        '''
        for n in self.open_list:
            if(n.position == child_state.position):
                return True             
            
        return False

    def check_node_in_closed_list(self, child_state) -> bool:
        '''
            This function checks whether a child node is in the closed list
            and whether it has a better f value
        '''
        for n in self.closed_list:
            if(n.position == child_state.position):
                return True             
            
        return False

    def a_star(self, start, goal) -> None:
        '''
            Implementation of the simple A* algorithm
        '''
        start_node = Node(position=start)
        start_node.g = 0
        self.open_list.append(start_node)
        current = start_node

        while(self.open_list != []):
            
            #---------------------------------------------
            # Calculate the h and f values of Current node
            #---------------------------------------------
            current = self.get_priority_node()
            current.h = self.get_manhattan_dist(start, goal)
            current.f = current.g + current.h
            
            #------------------------------------
            # Popping the node from the open list
            #------------------------------------
            self.open_list.remove(current)
            #---------------------------------------------------
            # Append the current node to the closed list
            #---------------------------------------------------
            self.closed_list.append(current)

            

            if current.position == goal:
                break

            moves = self.get_valid_moves(current.position)

            for move in moves:
                child_state = Node(move)
                #-------------------------------------------
                # Set the child's parent as the current node
                #-------------------------------------------
                child_state.parent = current
                
                #-----------------------------------------------
                # Update the h, g and f values of the child node
                #-----------------------------------------------
                child_state.h = self.get_manhattan_dist(child_state.position, goal)
                child_state.g = current.g + self.get_manhattan_dist(
                    child_state.position, current.position)
                child_state.f = child_state.g + child_state.h

                #------------------------------------------------
                # If the node is already in closed list then skip
                #------------------------------------------------
                if self.check_node_in_closed_list(child_state):
                    continue
                
                #----------------------------------------------
                # If the node is already in open list then skip
                #----------------------------------------------
                # Update the priority? (TBD)
                if self.check_node_in_open_list(child_state):
                    continue
                
                #-----------------------------------------
                # Else add the child node to the open list
                #-----------------------------------------
                else:
                    self.open_list.append(child_state)

            

        #----------------------------------------------------------------------
        # If open list is empty in the end then return current to indicate no path
        #----------------------------------------------------------------------
        # if not self.open_list: return current

        self.counter_expanded_nodes += len(self.closed_list) - 1
        return current
            

    #function to move agent through the grid; path recorded in travelled_path
    def move_in_real_grid(self, current_state, path) -> list:
        travelled_path = []
        for cell in path:
            if self.actual_grid[cell[0]][cell[1]] != 1:
                current_state = cell
                travelled_path.append(current_state)
            else:
                break
        return travelled_path
    
    #uses get_valid_moves() to update agent's memory
    def observe_nearby_cells(self, current_state) -> None:
        field_of_view = self.get_valid_moves(current=current_state)
        for cell in field_of_view:
            if self.actual_grid[cell[0]][cell[1]] == 1 and self.explored_grid[cell[0]][cell[1]] == 0:
                self.explored_grid[cell[0]][cell[1]] = 1


    #function to visualize the final path taken by the agent in the grid (needs tweaking)
    def temporary_visualize(self, path):
        for point in path:
            self.actual_grid[point[0]][point[1]] = 0.5
        pyplot.imshow(self.actual_grid)
        pyplot.show()

    def run(self, grid_index = 0) -> None:
        '''
            This function runs the A* algorithm on the generated grid
        '''
        self.generate_grid(grid_index)
        final_path = []
        final_path.append(self.start)
        end = False
        path_exist = True
        # start = self.start
        # goal = self.target

        while path_exist and not end:
            
            # self.counter_expanded_nodes = 0
            
            # print(self.closed_list)
            # print(self.counter_expanded_nodes)
            # Check the surroundings and update the explored grid
            self.observe_nearby_cells(current_state=self.start)

            # Empty open and closed list
            self.open_list = []
            self.closed_list = []

            planned_dest = self.a_star(self.start, self.target)
            
            if planned_dest.position == self.target:

                # trace planned path back to the the node after start and make that move
                travelled_path = self.move_in_real_grid(
                    current_state=self.start, path=self.print_path(planned_dest))

                if(travelled_path and travelled_path[-1] == self.target):
                    end = True
                else:
                    # Set a new start state as the last node in travelled path
                    self.start = travelled_path[-1]
                
                final_path.extend(travelled_path)
            else:
                path_exist = False
    

        if not path_exist: print("Cannot reach the target")

        else:
            print("Number of nodes expanded : " + str(self.counter_expanded_nodes))
            # print("Nodes expanded : " + str([n.position for n in self.closed_list]))
            # print(final_path)
            self.temporary_visualize(path=final_path)

    def generate_grid(self, grid_index) -> None:
        '''
            This function generates N*N grid with the following properties
                1. Obstacles generated with 30% probability to construct a maze like structure
                2. Target position denoted with "X"
                3. Obstacles denoted with 1
        '''
        
        self.grid_world = self.grid_worlds.get(str(grid_index))

        self.start = tuple(self.grid_world.get("Start"))
        self.target = tuple(self.grid_world.get("Target"))
        self.actual_grid = self.grid_world.get("Grid")

        self.explored_grid = [[0]*len(self.actual_grid) for _ in range(len(self.actual_grid))]

        #-----------
        # Dummy grid
        #-----------
        # actual_grid = [[0,0,0,0,0],
        #                 [0,0,0,0,0],
        #                 [0,0,0,0,0],
        #                 [0,0,0,0,0],
        #                 [0,0,0,0,"X"]]

        # actual_grid = [[0,0,1,0,0],
        #                 [0,0,1,0,0],
        #                 [0,0,0,1,0],
        #                 [0,0,1,"X",0],
        #                 [0,0,0,0,0]]

        # actual_grid = [[0,0,0,0,0],
        #                 [0,0,1,0,0],
        #                 [0,0,1,0,0],
        #                 [0,0,1,0,0],
        #                 [0,0,0,1,"X"]]

        # explored_grid = [[0]*len(actual_grid) for _ in range(len(actual_grid))]

        # return actual_grid, explored_grid



if __name__ == "__main__":
    # obj1 = FastTrajectoryReplanning(tie_break=SMALL_G_VALUE)
    # obj1.run(start = (0, 0), goal = (4,4))
    obj2 = FastTrajectoryReplanning(tie_break=LARGE_G_VALUE)

    for i in range(10):
        obj2.run(grid_index=i)
        obj2.counter_expanded_nodes = 0