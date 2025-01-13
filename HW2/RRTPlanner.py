import numpy as np
from RRTTree import RRTTree
import time


class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.step_size = 5
        self.start = self.planning_env.start
        self.goal = self.planning_env.goal

    def sample_random_config(self, goal_prob, goal):
        if np.random.rand() < goal_prob:
            return goal
        else:
            while True:
                x = np.random.randint(self.planning_env.xlimit[0], self.planning_env.xlimit[1])
                y = np.random.randint(self.planning_env.ylimit[0], self.planning_env.ylimit[1])
                new_config = np.array([x, y])
                if self.planning_env.state_validity_checker(new_config):
                    return new_config

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()
        self.tree.add_vertex(self.start)
        plan = []
        while not self.tree.is_goal_exists(self.goal):
            new_config = self.sample_random_config(self.goal_prob, self.goal)
            _, neighbor = self.tree.get_nearest_state(new_config)
            self.extend(neighbor, new_config)
        curr_id = self.tree.get_idx_for_state(self.goal)
        while curr_id != self.tree.get_idx_for_state(self.start):
            plan.append(self.tree.vertices[curr_id].state)
            curr_id = self.tree.edges[curr_id]
        plan.append(self.tree.vertices[curr_id].state)
        plan.reverse()
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        run_time = time.time() - start_time
        print('Total time: {:.2f}'.format(run_time))

        return np.array(plan), self.compute_cost(plan), run_time

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        cost = 0
        for i in range(len(plan) - 1):
            cost += np.linalg.norm(plan[i] - plan[i + 1])
        return cost

    def extend(self, near_config, rand_config):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        if self.tree.get_idx_for_state(rand_config) is None:
            if self.ext_mode == "E1":
                if self.planning_env.edge_validity_checker(near_config, rand_config):
                    id2 = self.tree.add_vertex(rand_config)
                    id1 = self.tree.get_idx_for_state(near_config)
                    self.tree.add_edge(id1, id2, np.linalg.norm(near_config - rand_config))
            elif self.ext_mode == "E2":
                dis_to_rand = np.linalg.norm(near_config - rand_config)
                if dis_to_rand < self.step_size:
                    new_config = rand_config
                else:
                    direction = rand_config - near_config
                    new_config = near_config + direction / dis_to_rand * self.step_size
                if self.planning_env.edge_validity_checker(near_config, new_config):
                    id2 = self.tree.add_vertex(new_config)
                    id1 = self.tree.get_idx_for_state(near_config)
                    self.tree.add_edge(id1, id2, np.linalg.norm(near_config - new_config))
