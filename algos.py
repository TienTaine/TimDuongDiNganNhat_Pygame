import pygame
from const import *
from maze import *
import heapq
import math
from queue import PriorityQueue


def DFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    stack = [start_node]  

    nodes = [None] * g.get_length()  
    nodes[start_node.id] = start_node  

    father = [-1] * g.get_length()  

    visited = set()  
    backtracked = set()  
   
    while stack:
        current_node = stack.pop()  

        if current_node in backtracked:
            if current_node  not in visited:
                current_node.set_color(BLUE, sc) 
                pygame.time.delay(5)
                pygame.display.update()
            continue
        
        current_node.set_color(YELLOW, sc)
        pygame.time.delay(10)
        pygame.display.update()
        
        visited.add(current_node)
        
        if current_node == goal_node:
            path = []
            start_node.set_color(start_color, sc)
            goal_node.set_color(PURPLE, sc)

            node = goal_node
            while node != start_node:
                path.append(node)
                node = nodes[father[node.id]]  

        
            for i in range(len(path) - 1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i + 1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()

            pygame.draw.line(sc, WHITE, path[len(path) - 1].rect.center, start_node.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()

            return True

        neighbors = g.get_neighbors(current_node)
        unvisited_neighbors = [neighbor for neighbor in neighbors if nodes[neighbor.id] is None]
        for neighbor in neighbors:
            if neighbor not in visited and neighbor not in backtracked:
                stack.append(neighbor)  
                nodes[neighbor.id] = neighbor  
                father[neighbor.id] = current_node.id
                neighbor.set_color(RED, sc)
                pygame.time.delay(5)
                pygame.display.update()

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

        if not unvisited_neighbors:
            backtracked.add(current_node)
            current_node.set_color(BLUE, sc)
            pygame.time.delay(5)
            pygame.display.update()
            
        current_node.set_color(current_node.color, sc)

    start_node.set_color(start_color, sc)
    raise NotImplementedError('not implemented')

def BFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = [start_node.id]  
    closed_set = []
    nodes = [None] * g.get_length()  

    nodes[start_node.id] = start_node  

    father = [-1] * g.get_length()  

    while open_set:
        current_id = open_set.pop(0)  
        current_node = nodes[current_id]  

        current_node.set_color(YELLOW, sc)
        pygame.time.delay(5)
        pygame.display.update()

        if current_node == goal_node:
            start_node.set_color(start_color, sc)
            goal_node.set_color(PURPLE, sc)

            path = []
            node = goal_node
            while node != start_node:
                path.append(node)
                node = nodes[father[node.id]]  

            for i in range(len(path) - 1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i + 1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()
                
            pygame.draw.line(sc, WHITE,path[len(path) -1 ].rect.center, start_node.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()

            return True

        closed_set.append(current_node.id)

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor.id not in closed_set and neighbor.id not in open_set:
                open_set.append(neighbor.id)  
                nodes[neighbor.id] = neighbor  
                father[neighbor.id] = current_node.id
                neighbor.set_color(RED, sc)
                pygame.time.delay(5)
                pygame.display.update()

        current_node.set_color(BLUE, sc)  
        pygame.time.delay(5)
        pygame.display.update()

        current_node.set_color(current_node.color, sc)

    start_node.set_color(start_color, sc)
    raise NotImplementedError('not implemented')

def Dijkstra(g: SearchSpace, sc: pygame.Surface):
    print('Implement Dijkstra algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = PriorityQueue()
    closed_set = set()
    father = {}
    cost = {}

    open_set.put((0, start_node))
    start_node.set_color(start_color, sc)

    cost[start_node] = 0

    while not open_set.empty():
        current_cost, current_node = open_set.get()

        current_node.set_color(YELLOW, sc)
        pygame.time.delay(5)
        pygame.display.update()

        if current_node == goal_node:
            start_node.set_color(start_color, sc)
            goal_node.set_color(PURPLE, sc)

            path = []
            node = goal_node
            while node != start_node:
                path.append(node)
                node = father[node]

            for i in range(len(path) - 1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i + 1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()

            pygame.draw.line(sc, WHITE, path[len(path) -1 ].rect.center, start_node.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()
            return True

        closed_set.add(current_node)

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor not in closed_set:
                new_cost = cost[current_node] + 1
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    open_set.put((new_cost, neighbor))
                    father[neighbor] = current_node
                    neighbor.set_color(RED, sc)
                    pygame.time.delay(5)
                    pygame.display.update()

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

        current_node.set_color(current_node.color, sc)

    start_node.set_color(start_color, sc)
    raise NotImplementedError('not implemented')

def Astar(g: SearchSpace, sc: pygame.Surface):
    print('Implement AStar algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = PriorityQueue()
    closed_set = set()
    open_set.put((0, start_node))  
    start_node.set_color(start_color, sc)
    father = {}
    g_cost = {node: math.inf for node in g.grid_cells}
    g_cost[start_node] = 0
    f_cost = {node: math.inf for node in g.grid_cells}
    f_cost[start_node] = 0

    while not open_set.empty():
        current_cost, current_node = open_set.get() 
    
        current_node.set_color(YELLOW, sc)
        pygame.time.delay(5)
        pygame.display.update()

        if current_node == goal_node:
            start_node.set_color(start_color, sc)
            goal_node.set_color(PURPLE, sc)

            path = []
            node = goal_node
            while node != start_node:
                path.append(node)
                node = father[node]

            for i in range(len(path) - 1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i + 1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()

            pygame.draw.line(sc, WHITE, path[len(path) - 1].rect.center, start_node.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()
            return True

        closed_set.add(current_node)

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor not in closed_set:
                dx = abs(current_node.id % COLS - neighbor.id % COLS)  
                dy = abs(current_node.id // COLS - neighbor.id // COLS)  
                if dx == 1 and dy == 1:  
                    new_g_cost = g_cost[current_node] + math.sqrt(2)
                else:  
                    new_g_cost = g_cost[current_node] + 1
                if neighbor not in g_cost or new_g_cost < g_cost[neighbor]:
                    g_cost[neighbor] = new_g_cost
                    f_cost[neighbor] = g_cost[neighbor] + heuristic(neighbor, goal_node)
                    open_set.put((f_cost[neighbor], neighbor))
                    father[neighbor] = current_node
                    neighbor.set_color(RED, sc)
                    pygame.time.delay(5)
                    pygame.display.update()

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

        current_node.set_color(current_node.color, sc)

    start_node.set_color(start_color, sc)
    raise NotImplementedError('not implemented')

def heuristic(node, goal_node):
    node_x = node.id % COLS
    node_y = node.id // COLS
    goal_x = goal_node.id % COLS
    goal_y = goal_node.id // COLS
    return math.sqrt((node_x - goal_x)**2 + (node_y - goal_y)**2)