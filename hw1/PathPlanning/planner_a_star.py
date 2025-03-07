import cv2
import numpy as np
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def is_valid(self, x, y):
        return True if (self.map[y, x]) else False

    def create_neighbour(self, curr_pos):
        neighbour = []
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1), (-1, 1), (1, 1), (-1, -1), (1, -1)]
        for dir in directions:
            x, y = (curr_pos[0] + dir[0] * self.inter, curr_pos[1] + dir[1] * self.inter)
            if self.is_valid(x, y):
                neighbour.append((x, y))
        return neighbour

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        while(self.queue):
            # TODO: A Star Algorithm
            min_cost, min_idx = float("inf"), -1
            
            # Find the node with the minimum cost (f) value, f = g + h
            for idx, node in enumerate(self.queue):
                if self.g[node] + self.h[node] < min_cost:
                    min_cost = self.g[node] + self.h[node]
                    min_idx = idx
            curr_node = self.queue.pop(min_idx)

            # Check if reach the goal
            if utils.distance(curr_node, goal) < inter:
                self.goal_node = curr_node
                break
            
            # Explore
            neighbour = self.create_neighbour(curr_node)
            for node in neighbour:
                if node not in self.parent:
                    self.queue.append(node)
                    self.g[node] = self.g[curr_node] + inter
                    self.h[node] = utils.distance(node, goal)
                    self.parent[node] = curr_node
                elif self.g[node] > self.g[curr_node] + inter:
                    self.g[node] = self.g[curr_node] + inter
                    self.parent[node] = curr_node
        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
