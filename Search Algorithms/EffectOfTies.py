from config import *
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

        #-----------------------------------
        # Valid moves: up, down, left, right
        #-----------------------------------
        self.valid_moves = [(0, 1), (0, -1), (-1, 0), (1, 0)]

        self.actual_grid = None
        self.explored_grid = None

        #--------------------------------------------------------------
        # Used to break ties in favour of either large or small g value
        #--------------------------------------------------------------
        self.tie_breaker_pref = LARGE_G_VALUE

    
    def print_path(self, current) -> None:
        '''
            This function prints a list of positions that
            the agent travels to get from start to the goal state.
        '''
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
            TBD: Implementation with priority heap/queue

            This function also implements the effect of ties
            i.e. It breaks the ties in favour of 
        '''
        llst_nodes_smallest_f = []
        priority_node = self.open_list[0]

        for n in self.open_list:
            if(n.f <= priority_node.f):
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
                if(n.g >= chosen_node.g):
                    chosen_node = n
            else:
                if(n.g <= chosen_node.g):
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
            new_position = tuple(map(sum, zip(move, current.position)))

            if(new_position[0] >= 0 and new_position[1] >= 0):
                if(new_position[0] < len(self.explored_grid) and new_position[1] < len(self.explored_grid)):
                    if not self.explored_grid[new_position[0]][new_position[1]]==1:
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

    def a_star(self, grid, start, goal) -> None:
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

            moves = self.get_valid_moves(current)

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


        return current
            


    def run(self, start=None, goal=None, tie_break=None) -> None:
        '''
            This function runs the A* algorithm on the generated grid
        '''
        self.tie_breaker_pref = tie_break
        self.generate_grid()
        planned_dest = self.a_star(self.grid, start, goal)

        if planned_dest.position == goal:
            self.print_path(planned_dest)
            # trace planned path back to the the node after start and make that move
            # Update current position if it's not blocked
            # If blocked then start = current state
            # Check the surroundings and update the explored grid
            # Empty open and closed list
            # Call A* again with the new start state


        # elif planned_path.position != start:
            # Maybe this elif condition is not needed
            # Encountered a dead end??
            # Take a new step
            # Check the surroundings and update the explored grid
            # Call A* again with the new start state
            # start = curr.position
        else: print("Cannot reach the target")

        print("Number of nodes expanded : " + str(len(self.closed_list)))
        print("Nodes expanded : " + str([n.position for n in self.closed_list]))

    def generate_grid(self) -> None:
        '''
            This function generates N*N grid with the following properties
                1. Obstacles generated with 30% probability to construct a maze like structure
                2. Target position denoted with "X"
                3. Obstacles denoted with 1
        '''
        #-----------
        # Dummy grid
        #-----------
        self.grid = [[0,0,1,0,0],
                    [0,0,1,0,0],
                    [0,0,1,0,0],
                    [0,0,1,"X",0],
                    [0,0,0,0,0]]



if __name__ == "__main__":
    obj1 = FastTrajectoryReplanning()
    obj1.run(start = (0, 0), goal = (3,3), tie_break=SMALL_G_VALUE)
    obj2 = FastTrajectoryReplanning()
    obj2.run(start = (0, 0), goal = (3,3), tie_break=LARGE_G_VALUE)