from RFAWithTies import RFAWithTies
from AdaptiveAStar import FTRAdaptive
from RepeatedBackwardAStar import FTRBackward
from config import *
import time
from matplotlib import pyplot as plt


'''
    This class is designed to compare the following A* Algorithms
    1. Repeated Forward A*(RFA) Vs Adaptive A*
    2. Repeated Forward A*(RFA) Vs Repeated Backward A*
    3. Different Tie-breakers
'''
class FastTrajectoryReplanning:

    def __init__(self, number_of_grids=1) -> None:
        self.number_of_grids = number_of_grids

    def rfa_vs_adaptive(self):
        obj_rfa = RFAWithTies(tie_break=LARGE_G_VALUE)
        obj_adaptive_a_star = FTRAdaptive(tie_break=LARGE_G_VALUE)

        times_rfa = []
        times_adap = []

        nodes_expanded_rfa = []
        nodes_expanded_adap = []

        for i in range(0, self.number_of_grids):
            start = time.time()
            obj_rfa.run(grid_index=i)
            end = time.time()
            times_rfa.append(end - start)
            nodes_expanded_rfa.append(obj_rfa.counter_expanded_nodes)
            obj_rfa.counter_expanded_nodes = 0

            start = time.time()
            obj_adaptive_a_star.run(grid_index=i)
            end = time.time()
            times_adap.append(end - start)
            nodes_expanded_adap.append(obj_adaptive_a_star.counter_expanded_nodes)
            obj_adaptive_a_star.counter_expanded_nodes = 0

        plt.plot([i for i in range(self.number_of_grids)], times_rfa, label="Repeated Forward A*")
        plt.plot([i for i in range(self.number_of_grids)], times_adap, label="Adaptive A*")
        plt.legend()
        plt.show()
        plt.plot([i for i in range(self.number_of_grids)], nodes_expanded_rfa, label="Repeated Forward A*")
        plt.plot([i for i in range(self.number_of_grids)], nodes_expanded_adap, label="Adaptive A*")
        plt.legend()
        plt.show()

    def rfa_tie_breaking(self):

        obj_rfa_large_g = RFAWithTies(tie_break=LARGE_G_VALUE)
        obj_rfa_small_g = RFAWithTies(tie_break=SMALL_G_VALUE)

        times_rfa_large_g = []
        times_rfa_small_g = []

        nodes_expanded_rfa_small_g = []
        nodes_expanded_rfa_large_g = []

        for i in range(0, self.number_of_grids):
            start = time.time()
            obj_rfa_large_g.run(grid_index=i)
            end = time.time()
            times_rfa_large_g.append(end - start)
            nodes_expanded_rfa_large_g.append(obj_rfa_large_g.counter_expanded_nodes)
            obj_rfa_large_g.counter_expanded_nodes = 0

            start = time.time()
            obj_rfa_small_g.run(grid_index=i)
            end = time.time()
            times_rfa_small_g.append(end - start)
            nodes_expanded_rfa_small_g.append(obj_rfa_small_g.counter_expanded_nodes)
            obj_rfa_small_g.counter_expanded_nodes = 0

        plt.plot([i for i in range(self.number_of_grids)], times_rfa_large_g, label="Time: RFA* Large g ties")
        plt.plot([i for i in range(self.number_of_grids)], times_rfa_small_g, label="Time: RFA* Small g ties")
        plt.legend()
        plt.show()
        plt.plot([i for i in range(self.number_of_grids)], nodes_expanded_rfa_large_g,
                 label="Expanded Nodes: RFA* Large g ties")
        plt.plot([i for i in range(self.number_of_grids)], nodes_expanded_rfa_small_g,
                 label="Expanded Nodes: RFA* Small g ties")
        plt.legend()
        plt.show()

    def rfa_vs_backward(self):
        obj_rfa = RFAWithTies(tie_break=LARGE_G_VALUE)
        obj_rba = FTRBackward(tie_break=LARGE_G_VALUE)

        times_rfa = []
        times_rba = []

        nodes_expanded_rfa = []
        nodes_expanded_rba = []

        for i in range(0, self.number_of_grids):
            start = time.time()
            obj_rfa.run(grid_index=i)
            end = time.time()
            times_rfa.append(end - start)
            nodes_expanded_rfa.append(obj_rfa.counter_expanded_nodes)
            obj_rfa.counter_expanded_nodes = 0

            start = time.time()
            obj_rba.run(grid_index=i)
            end = time.time()
            times_rba.append(end - start)
            nodes_expanded_rba.append(obj_rba.counter_expanded_nodes)
            obj_rba.counter_expanded_nodes = 0

        plt.plot([i for i in range(self.number_of_grids)], times_rfa, label="Repeated Forward A*")
        plt.plot([i for i in range(self.number_of_grids)], times_rba, label="Repeated Backward A*")
        plt.legend()
        plt.show()
        plt.plot([i for i in range(self.number_of_grids)], nodes_expanded_rfa, label="Repeated Forward A*")
        plt.plot([i for i in range(self.number_of_grids)], nodes_expanded_rba, label="Repeated Backward A*")
        plt.legend()
        plt.show()


if __name__ == "__main__":
    obj = FastTrajectoryReplanning(number_of_grids=5)
    # ------------------------------------------------
    # Comparing the performance of RFA Vs Adaptive A*
    # ------------------------------------------------
    obj.rfa_vs_adaptive()

    # ----------------------------------------
    # Comparing the performance of RFA Vs RBA
    # ----------------------------------------
    obj.rfa_vs_backward()

    #----------------------------------------------------
    # Comparing the performance of different tie-breakers
    #----------------------------------------------------
    #obj.rfa_tie_breaking()

