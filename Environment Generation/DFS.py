from random import randint
import json
import pickle

class Cell():
    def __init__(self) -> None:
        self.grid = []
        self.r = 10
        self.c= 10
        self.start = (0,0)
        self.grid_val = []
        self.target = (randint(0,self.r),randint(0,self.c))
        #print("Target",self.target)
        # random target

    def getTarget(self):
        return self.getTarget

    def initialise_grid(self):
        for i in range(0,self.r):
            self.grid_val.append([])
            self.grid.append([])
            for j in range(0,self.c):
                self.grid_val[i].append(False) #grid to find which cell hasn't been visited
                self.grid[i].append(0)
                # Initialise all values to False at the beginning.Once traveled will be set to True

    def valid_move(self,curr_pos) -> bool:
        if (curr_pos[0]>= 0 and curr_pos[1] >=0 and curr_pos[0]<self.r+self.start[0] and curr_pos[1]<self.c+self.start[1]):
            return True
        else:
            return False

    def unvisited(self,grid_val,r,c) -> tuple:
        for i in range(0,r):
            for j in range(0,c):
                if(grid_val[i][j]==False and (i,j)!=self.target):
                    return ((i,j))
                
        return (-1,-1)

    def DFS(self,curr_pos):
        #print(curr_pos)
        self.grid_val[curr_pos[0]][curr_pos[1]]=True
        #Marking visited position as true in the grid
 
        for x in [-1,1]:
            cell_x = curr_pos[0]+x
            cell_y = x+curr_pos[1]  #To find 4 neighbours
            #print((curr_pos[0],cell_y))
            #print(self.valid_move((curr_pos[0],cell_y)))
            #print(self.unvisited(self.grid_val,self.r,self.c)!=(-1,-1))
            if(self.valid_move((cell_x,curr_pos[1])) and self.grid_val[cell_x][curr_pos[1]]==False and (cell_x,curr_pos[1])!= self.target): #Condition for cell move is valid and not visited
                number = randint(1,100)
                if(number<30):
                    self.grid[cell_x][curr_pos[1]] = 1
                else:
                     self.grid[cell_x][curr_pos[1]] = 0
                self.DFS((cell_x,curr_pos[1])) #Call from neighbour cell to DFS
            elif(self.valid_move((curr_pos[0],cell_y)) and self.grid_val[curr_pos[0]][cell_y]==False and (curr_pos[0],cell_y)!=self.target): #Condition for cell move is valid and not visited
                number = randint(1,100)
                if(number<30):
                    self.grid[curr_pos[0]][cell_y] = 1
                else:
                     self.grid[curr_pos[0]][cell_y] = 0
                self.DFS((curr_pos[0],cell_y))
        if self.unvisited(self.grid_val,self.r,self.c)!=(-1,-1) and self.unvisited(self.grid_val,self.r,self.c)!=self.target:
            self.DFS(self.unvisited(self.grid_val,self.r,self.c))
        elif self.unvisited(self.grid_val,self.r,self.c)==(-1,-1):
            return self.grid
        else:
            print("Unexpected problem")
        return self.grid

if __name__ == "__main__":   
    obj = Cell()
    start = (0,0)
    obj.initialise_grid()
    grid = obj.DFS(start)
    print(grid)
    dictionary = {
    "start": start,
    "target": obj.getTarget(),
    "grid":grid
    }
    #pickled_object= pickle.dumps(dictionary)
    with open("grid.json",'wb') as outfile:
       pickle.dump(dictionary, outfile)
    

