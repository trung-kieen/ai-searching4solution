
# Author: @trung-kieen Nguyen Khac Trung Kien


from collections.abc import Iterable
from os import sendfile
from typing import Any, override
from warnings import simplefilter
from heapq import heappush, heappop


class Node:
    def __init__(self, parent, action, path_cost, state):
        self.parent  = parent
        self.state = state
        self.action = action
        self.path_cost = path_cost
    def __repr__(self) -> str:
        return self.state

    def __lt__(self , other):
        return self.path_cost < other
    def __gt__(self , other):
        return self.path_cost > other
class Fronttier:
    def __init__(self) -> None:
        self.frontier: list[Node] = []

    def add(self, node) -> None:
        raise Exception("Require implement this")

    def contain_state(self, state):
        return len([node for node in self.frontier if node.state == state ]) == 1

    def empty(self):
        return len(self.frontier) == 0

    def remove(self) -> Node:
        """
        return node
        """
        raise Exception("Require implement this")
    def __repr__(self) -> str:
        return str(self.frontier)

class QueueFrontier(Fronttier):
    def __init__(self) -> None:
        super().__init__()

    @override
    def add(self, node) -> None:
        self.frontier.append(node)
    @override
    def remove(self) -> Node:
        if self.empty(): raise Exception("No solution found")
        return self.frontier.pop(0)


class HeapFrontier(Fronttier):
    def __init__(self) -> None:
        super().__init__()

    @override
    def add(self, node) -> None:
        heappush(self.frontier, node)
    @override
    def remove(self) -> Node:
        if self.empty(): raise Exception("No solution found")
        return heappop(self.frontier)





class BFSGraph:
    def __init__(self, start_city, goal, graph_adj_list) -> None:
        self.start = start_city
        self.goal = goal
        self.graph = graph_adj_list
        self.explored = set()
        self.frontier: Fronttier = QueueFrontier()
        self.seq_actions = []
        self.seq_states = []



    def neighbors(self, state) -> Iterable:
        return self.graph[state]



    def expand_tree(self, chosen_node: Node):
        for adj_node in self.neighbors(chosen_node.state):
            state, cost = adj_node
            if not self.frontier.contain_state(state) and not state in self.explored:
                total_cost = chosen_node.path_cost + cost
                # Perform action B on node A will move to state B
                child = Node(parent= chosen_node, action = state, path_cost=total_cost , state= state)
                self.frontier.add(child)

    def solution(self, chosen_node):
        # Response result
        while chosen_node.parent is not None:
            self.seq_actions.append(chosen_node.action)
            self.seq_states.append(chosen_node.state)
            chosen_node  = chosen_node.parent


        self.seq_actions.append(chosen_node.action)
        self.seq_states.append(chosen_node.state)

        self.seq_states.reverse()
        self.seq_actions.reverse()
        return self.seq_states
    def solve(self):
        start_node = Node(parent = None, action = None, path_cost = 0 , state = self.start)
        self.frontier.add(start_node)
        while True:
            if self.frontier.empty():
                raise Exception("No solution found")
            chosen_node = self.frontier.remove()
            self.explored.add(start_node)
            if chosen_node.state == self.goal:
                return self.solution(chosen_node)
            self.expand_tree(chosen_node)



class UCSGraph(BFSGraph):
    def __init__(self, start_city, goal, graph_adj_list) -> None:
        super().__init__(start_city, goal, graph_adj_list)
        self.explored = dict()
        self.frontier = HeapFrontier()


    @override
    def expand_tree(self, chosen_node: Node):
        for adj_node in self.neighbors(chosen_node.state):
            state, cost = adj_node
            total_cost = chosen_node.path_cost + cost
            if state not in self.explored or total_cost < self.explored[state][0] :
                total_cost = chosen_node.path_cost + cost
                self.explore(state, total_cost, chosen_node)
                child = Node(parent= chosen_node, action = state, path_cost=total_cost , state= state)
                self.frontier.add(child)
    def explore(self,state,  cost = 0 , parent = None):
        self.explored[state] = (cost, parent)
    @override
    def solution(self, chosen_node):
        return super().solution(chosen_node)
    @override
    def solve(self):
        start_node = Node(parent = None, action = None, path_cost = 0 , state = self.start)
        self.frontier.add(start_node)
        self.explore(self.start)
        while True:
            if self.frontier.empty():
                raise Exception("No solution found")
            chosen_node = self.frontier.remove()
            if chosen_node.state == self.goal:
                return self.solution(chosen_node)
            self.expand_tree(chosen_node)


graph = {
    'Arad': [('Zerind', 75), ('Sibiu', 140), ('Timisoara', 118)],
    'Zerind': [('Arad', 75), ('Oradea', 71)],
    'Oradea': [('Zerind', 71), ('Sibiu', 151)],
    'Sibiu': [('Arad', 140), ('Oradea', 151), ('Fagaras', 99), ('Rimnicu Vilcea', 80)],
    'Timisoara': [('Arad', 118), ('Lugoj', 111)],
    'Lugoj': [('Timisoara', 111), ('Mehadia', 70)],
    'Mehadia': [('Lugoj', 70), ('Drobeta', 75)],
    'Drobeta': [('Mehadia', 75), ('Craiova', 120)],
    'Craiova': [('Drobeta', 120), ('Rimnicu Vilcea', 146), ('Pitesti', 138)],
    'Rimnicu Vilcea': [('Sibiu', 80), ('Craiova', 146), ('Pitesti', 97)],
    'Fagaras': [('Sibiu', 99), ('Bucharest', 211)],
    'Pitesti': [('Rimnicu Vilcea', 97), ('Craiova', 138), ('Bucharest', 101)],
    'Bucharest': [('Fagaras', 211), ('Pitesti', 101), ('Giurgiu', 90), ('Urziceni', 85)],
    'Giurgiu': [('Bucharest', 90)],
    'Urziceni': [('Bucharest', 85), ('Vaslui', 142), ('Hirsova', 98)],
    'Hirsova': [('Urziceni', 98), ('Eforie', 86)],
    'Eforie': [('Hirsova', 86)],
    'Vaslui': [('Urziceni', 142), ('Iasi', 92)],
    'Iasi': [('Vaslui', 92), ('Neamt', 87)],
    'Neamt': [('Iasi', 87)]
}
start = 'Arad'
goal = 'Bucharest'
bfs = BFSGraph(start_city=start, goal=goal, graph_adj_list=graph)
ucs = UCSGraph(start_city=start, goal=goal, graph_adj_list=graph)
print( "BFS:",  bfs.solve())
print( "UCS:", ucs.solve())
