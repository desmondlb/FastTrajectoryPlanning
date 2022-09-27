
class Node():
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

    def get_manhattan_dist(self, start, goal):
        # Returns Norm 1 distance between start and the goal state
        return abs(start[0]-goal[0]) + abs(start[1]-goal[1])

    def get_priority_node(self):
        priority_node = self.open_list[0]

        for n in self.open_list:
            if(n.f <= priority_node.f):
                priority_node = n

        return priority_node

    def get_valid_moves(self, current):
        '''
            Here we check the following
                1. Has the node been already visited
                2. Does the node lie within grid boundaries
        '''
        current_legal_moves = []

    def perform_move(self, move):
        for m in self.valid_moves:
            pass

    def check_node_in_open_list(self, child_state):
        for n in self.open_list:
            if((n.position == child_state.position) and n.f < child_state.f):
                return True             
            
        return False

    def check_node_in_closed_list(self, child_state):
        for n in self.closed_list:
            if((n.position == child_state.position) and n.f < child_state.f):
                return True             
            
        return False

    def a_star(self, grid, start, goal):
        
        self.open_list.append(Node(position=start))

        while(self.open_list != []):
            
            current = self.get_priority_node()
            current.h = self.get_manhattan_dist(start, goal)
            current.g = current.h
            current.f = current.g + current.h

            # Popping the node from the open list
            self.open_list.remove(current)

            # f_n = self.counter + h_n

            moves = self.get_valid_moves(current)

            for move in moves:
                child_state = Node(self.perform_move(move))
                child_state.parent = current
                if child_state == goal:
                    break
                else:
                    child_state.h = self.get_manhattan_dist(child_state, goal)
                    child_state.g = current.g + self.get_manhattan_dist(child_state, current)
                    child_state.f = child_state.g + child_state.h

                if not self.check_node_in_open_list(child_state):
                    continue
                
                if not self.check_node_in_closed_list(child_state):
                    continue

                else:
                    self.open_list.append(child_state)

            self.closed_list.append(current)


    def run(self):
        grid = [[0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,'X',0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0]]

        start = (0, 0)
        goal = (7,8)

        self.a_star(grid, start, goal)

if __name__ == "__main__":
    obj = FastTrajectoryReplanning()

    obj.run()