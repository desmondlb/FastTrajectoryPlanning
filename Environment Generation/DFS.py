from collections import deque
from random import randint
import json
import pickle
import random
import sys

class Cell():
    def __init__(self) -> None:
        self.grid = []
        self.r = 101
        self.c= 101
        self.grid_val = []
        self.start = (randint(0,int(0.2*self.r)),randint(0,int(0.2*self.c)))
        self.target = (randint(int(0.8*self.r),self.r-1),randint(int(0.8*self.c),self.c -1))
        #print("Target",self.target)
        # random target

    def getTarget(self):
        return self.target

    def getStart(self):
        return self.start

    def initialise_grid(self):
        for i in range(0,self.r):
            self.grid_val.append([])
            self.grid.append([])  
            for j in range(0,self.c):
                self.grid_val[i].append(False) #grid to find which cell hasn't been visited
                self.grid[i].append(0)
                # Initialise all values to False at the beginning.Once traveled will be set to True

    def valid_move(self,curr_pos) -> bool:
        if (curr_pos[0]>= 0 and curr_pos[1] >=0 and curr_pos[0]<self.r and curr_pos[1]<self.c):
            return True
        else:
            return False

    def unvisited(self,grid_val,r,c) -> tuple:
        for i in range(0,r):
            for j in range(0,c):
                if(grid_val[i][j]==False and (i,j)!=self.target):
                    return ((i,j))
                
        return (-1,-1)

    def iterativeDFS(self,curr_pos):
        stack = deque()
        stack.append(curr_pos)
        #print(curr_pos)
        
        while stack:
            curr_pos = stack.pop()
            if (self.grid_val[curr_pos[0]][curr_pos[1]]==False):
                self.grid_val[curr_pos[0]][curr_pos[1]]=True
                #Marking visited position as true in the grid
        
                for x in [-1,1]:
                    cell_x = curr_pos[0]+x
                    cell_y = x+curr_pos[1]  #To find 4 neighbours
                    #print((curr_pos[0],cell_y))
                    #print(self.valid_move((curr_pos[0],cell_y)))
                    #print(self.unvisited(self.grid_val,self.r,self.c)!=(-1,-1))
                    if(self.valid_move((cell_x,curr_pos[1])) and self.grid[cell_x][curr_pos[1]] !=1 and self.grid_val[cell_x][curr_pos[1]]==False and (cell_x,curr_pos[1])!= self.target): #Condition for cell move is valid and not visited
                        self.grid[cell_x][curr_pos[1]] = 1 if random.random() < 0.2 else 0
                        stack.append((cell_x,curr_pos[1]))
                        #self.DFS((cell_x,curr_pos[1])) #Call from neighbour cell to DFS
                    elif(self.valid_move((curr_pos[0],cell_y)) and self.grid[curr_pos[0]][cell_y] !=1 and self.grid_val[curr_pos[0]][cell_y]==False and (curr_pos[0],cell_y)!=self.target): #Condition for cell move is valid and not visited
                        self.grid[curr_pos[0]][cell_y] = 1 if random.random() < 0.2 else 0
                        stack.append((curr_pos[0],cell_y))
                        #self.DFS((curr_pos[0],cell_y))
                    else:
                        continue
        if self.unvisited(self.grid_val,self.r,self.c)!=(-1,-1) and self.unvisited(self.grid_val,self.r,self.c)!=self.target:
            #self.iterativeDFS(self.unvisited(self.grid_val,self.r,self.c))
            stack.append(self.unvisited(self.grid_val,self.r,self.c))
        elif self.unvisited(self.grid_val,self.r,self.c)==(-1,-1):
            return self.grid
        else:
            print("Unexpected problem")
        return self.grid
        

if __name__ == "__main__":   
    #sys.setrecursionlimit(10250)
    dictionary = {}
    for i in range(0,50):
        obj = Cell()
        obj.initialise_grid()
        start = obj.getStart()
        grid = obj.iterativeDFS(start)
        dictionary[i]=  {
        "Start": start,
        "Target": obj.getTarget(),
        "Grid":grid
        }
    #pickled_object= pickle.dumps(dictionary)
    with open("Gridworlds/gridworlds.json",'w+') as f:
       json.dump(dictionary, f)
    

