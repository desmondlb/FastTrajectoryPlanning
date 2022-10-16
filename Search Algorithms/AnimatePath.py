import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class AnimatePath:

    def __init__(self, grid=None, path=None, start=None,
                 target=None, observed=None) -> None:
        self.grid = grid
        self.path = path
        self.start = start
        self.target = target
        self.explored = observed


    def animate(self, i):
        im = plt.imshow(self.grid, cmap="RdBu")
        point = self.path[i]
        self.grid[point[0]][point[1]] = 4
        for ob in self.explored[i]:
            self.grid[ob[0]][ob[1]] = 7 
        im.set_array(self.grid)
        return [im]

    def show_path(self):
        fig = plt.figure()
        self.grid[self.start[0]][self.start[1]] = 3
        self.grid[self.target[0]][self.target[1]] = 5
        anim = FuncAnimation(fig, self.animate, interval=10, frames=len(self.path), blit=True)
        plt.show()

