import pygame
from const import *
from maze import *
import heapq
import math

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
class iNode:
    def __init__(self, node, cost):
        self.node = node
        self.cost = cost
    
    def __lt__(self, other):
        return self.cost < other.cost 
def dijkstra(g: SearchSpace, sc: pygame.Surface):
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = []
    closed_set = set()
    father = {}
    cost = {}

    heapq.heappush(open_set, iNode(start_node, 0))
    start_node.set_color(start_color, sc)

    cost[start_node] = 0

    while open_set:
        current = heapq.heappop(open_set)
        current_node = current.node
        current_cost = current.cost
        
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
                
            pygame.draw.line(sc, WHITE,path[len(path) -1 ].rect.center, start_node.rect.center, 3)
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
                    heapq.heappush(open_set, iNode(neighbor, new_cost))
                    father[neighbor] = current_node
                    neighbor.set_color(RED, sc)
                    pygame.time.delay(5)
                    pygame.display.update()

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

        current_node.set_color(current_node.color, sc)

    start_node.set_color(start_color, sc)
    return False


def heuristic(node: Node, goal: Node) -> float:
    dx = abs(node.rect.centerx - goal.rect.centerx)
    dy = abs(node.rect.centery - goal.rect.centery)
    return math.sqrt(dx**2 + dy**2)

def astar(g: SearchSpace, sc: pygame.Surface):
    print('Implement A* algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = [(0, start_node)]  
    heapq.heapify(open_set)
    closed_set = set()  
    g_score = {node: math.inf for node in g.grid_cells}  
    g_score[start_node] = 0  
    came_from = {}  

    while open_set:
        current_node = heapq.heappop(open_set)[1]  

        if current_node == goal_node:
            path = []
            start_node.set_color(start_color, sc)
            goal_node.set_color(PURPLE, sc)

            node = goal_node
            while node != start_node:
                path.append(node)
                node = came_from[node]

            for i in range(len(path) - 1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i + 1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()

            pygame.draw.line(sc, WHITE, path[len(path) - 1].rect.center, start_node.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()

            return True

        closed_set.add(current_node)

        current_node.set_color(YELLOW, sc)
        pygame.time.delay(10)
        pygame.display.update()

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current_node] + 1  

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score

                if neighbor not in [node for _, node in open_set]:
                    heapq.heappush(open_set, (g_score[neighbor] + heuristic(neighbor, goal_node), neighbor))
                else:
                    index = [node for _, node in open_set].index(neighbor)
                    open_set[index] = (g_score[neighbor] + heuristic(neighbor, goal_node), neighbor)
                    heapq.heapify(open_set)

                neighbor.set_color(RED, sc)
                pygame.time.delay(5)
                pygame.display.update()

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

    start_node.set_color(start_color, sc)
    raise ValueError('No path found')