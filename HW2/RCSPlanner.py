import heapdict
import numpy as np


class RCSPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = []

        self.fine_directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (-1, -1), (-1, 1), (1, 1), (1, -1)]
        self.coarse_directions = [(0, -2), (2, 0), (0, 2), (-2, 0), (-2, -2), (-2, 2), (2, 2), (2, -2)]

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []

        # TODO: Task 4
        COARSE = 0
        FINE = 1
        open_heapdict = heapdict.heapdict()
        closed_heapdict = heapdict.heapdict()
        open_heapdict[str(self.planning_env.start)] = node(self.planning_env.start, None, None,0) # (rank, node(state,resolution, parent))
        while open_heapdict:
            current_node_str ,curr_node = open_heapdict.popitem()

            if self.planning_env.state_validity_checker(curr_node.state):
                if current_node_str not in closed_heapdict:
                   
                    if np.array_equal(curr_node.state, self.planning_env.goal):
                        return  self.reconstruct_path(curr_node.parent)
                    
                    self.expanded_nodes.append(curr_node.state)# TODO:check if its the right location to add the expanded node

                    for direction in self.coarse_directions:
                        new_state = curr_node.state + direction
                        new_rank = curr_node.rank + 1
                        new_node = node(new_state, COARSE, curr_node,new_rank)
                        open_heapdict[str(new_state)] = new_node
                    closed_heapdict[current_node_str] = curr_node
                    
            if not np.array_equal(curr_node.state,self.planning_env.start) and curr_node.resolution == COARSE:
                for direction in self.fine_directions:
                    parant = curr_node.parent
                    new_state = parant.state + direction
                    open_heapdict[str(new_state)] =  node(new_state, FINE, parant,new_rank)
        
        return np.array(plan)                


    def reconstruct_path(self, node):
        '''
        Reconstruct the path from the goal to the start using parent pointers.
        # YOU DON'T HAVE TO USE THIS FUNCTION!!!
        '''
        path = []
        while node:
            path.append(node.state)  # Append the state
            node = node.parent  # Move to the parent
        path.reverse()
        print(path)
        return np.array(path)
    
    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        DO NOT MODIFY THIS FUNCTION!!!
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes

class node:
    def __init__(self, state,resolution, parent,rank):
        self.state = state
        self.parent = parent
        self.resolution = resolution
        self.rank = rank
    
    def __lt__(self, other):
        return self.rank < other.rank
