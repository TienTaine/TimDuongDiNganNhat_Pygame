import pygame
from const import *
from maze import *
from collections import deque
import heapq
 
# Chạy ok  
def DFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = []
    closed_set = set()
    father = {}

    open_set.append(start_node)
    start_node.set_color(start_color, sc)

    while open_set:
        current_node = open_set.pop()
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

            for i in range(len(path)-1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i+1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()
                
            pygame.draw.line(sc, WHITE,path[len(path) -1 ].rect.center, start_node.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()

            return True

        closed_set.add(current_node)

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor not in closed_set and neighbor not in open_set:
                open_set.append(neighbor)
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

def dfs(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = [start_node.id]  # Initialize open_set with the start node's ID
    closed_set = []
    nodes = [None] * g.get_length()  # Initialize nodes as a list with None values

    nodes[start_node.id] = start_node  # Store the start node in the nodes list

    father = [-1] * g.get_length()  # Initialize father with -1 for all nodes

    while open_set:
        current_id = open_set.pop()
        current_node = nodes[current_id]  # Access the node directly from the nodes list

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
                node = nodes[father[node.id]]  # Access the node directly from the nodes list

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
                nodes[neighbor.id] = neighbor  # Store the neighbor node in the nodes list
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

# Chạy ok
def BFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = deque()
    closed_set = set()
    father = {}
    cost = {}

    open_set.append(start_node)
    start_node.set_color(start_color, sc)

    cost[start_node] = 0

    while open_set:
        current_node = open_set.popleft()
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
            if neighbor not in closed_set and neighbor not in open_set:
                open_set.append(neighbor)
                father[neighbor] = current_node
                cost[neighbor] = cost[current_node] + 1
                neighbor.set_color(RED, sc)
                pygame.time.delay(5)
                pygame.display.update()

        current_node.set_color(BLUE, sc)  # Change color to yellow after being removed from open_set
        pygame.time.delay(5)
        pygame.display.update()

        # Reset the color of the current_node back to its original color
        current_node.set_color(current_node.color, sc)

    start_node.set_color(start_color, sc)
    raise NotImplementedError('not implemented')

def bfs(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')
    start_node = g.start
    goal_node = g.goal

    start_color = start_node.color

    open_set = [start_node.id]  # Initialize open_set with the start node's ID
    closed_set = []
    nodes = [None] * g.get_length()  # Initialize nodes as a list with None values

    nodes[start_node.id] = start_node  # Store the start node in the nodes list

    father = [-1] * g.get_length()  # Initialize father with -1 for all nodes

    while open_set:
        current_id = open_set.pop(0)  # Dequeue the first element from open_set
        current_node = nodes[current_id]  # Access the node directly from the nodes list

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
                node = nodes[father[node.id]]  # Access the node directly from the nodes list

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
                open_set.append(neighbor.id)  # Enqueue the neighbor node's ID to the end of open_set
                nodes[neighbor.id] = neighbor  # Store the neighbor node in the nodes list
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

 
 # Chưa chạy đc   

def Dijkstra(g: SearchSpace, sc: pygame.Surface):
    start_node = g.start
    goal_node = g.goal

    open_set = [(0, start_node.id)]  # Initialize open_set with a tuple of (cost, node_id)
    closed_set = set()
    nodes = [None] * g.get_length()  # Initialize nodes as a list with None values

    nodes[start_node.id] = start_node  # Store the start node in the nodes list

    father = [-1] * g.get_length()  # Initialize father as a list with -1 values
    g_score = [float('inf')] * g.get_length()  # Initialize g_score as a list with infinity values
    g_score[start_node.id] = 0

    while open_set:
        current_cost, current_node_id = heapq.heappop(open_set)
        current_node = nodes[current_node_id]

        if current_node == goal_node:
            path = reconstruct_path(g, father, start_node, goal_node)
            return path

        closed_set.add(current_node_id)

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor.id in closed_set:
                continue

            tentative_g_score = g_score[current_node.id] + g[current_node.id][neighbor.id]

            if tentative_g_score < g_score[neighbor.id]:
                father[neighbor.id] = current_node.id
                g_score[neighbor.id] = tentative_g_score

                if neighbor.id not in [node[1] for node in open_set]:
                    heapq.heappush(open_set, (g_score[neighbor.id], neighbor.id))
                    nodes[neighbor.id] = neighbor

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

    return None

def reconstruct_path(g: SearchSpace, father, start_node, goal_node):
    path = []
    current_node = goal_node

    while current_node != start_node:
        path.insert(0, current_node.id)
        current_node = g.nodes[father[current_node.id]]

    path.insert(0, start_node.id)
    return path

# Chạy ok
class NodeWrapper:
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

    heapq.heappush(open_set, NodeWrapper(start_node, 0))
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
                    heapq.heappush(open_set, NodeWrapper(neighbor, new_cost))
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

def get_node_by_id(nodes, node_id):
    for node in nodes:
        if node.id == node_id:
            return node
    return None

def heuristic(node: Node, goal: Node) -> int:
    # Hàm heuristic được sử dụng để tính toán chi phí heuristic từ một node đến mục tiêu.
    # Trong trường hợp này, chúng ta sẽ sử dụng khoảng cách Manhattan giữa các node.
    return abs(node.rect.x - goal.rect.x) + abs(node.rect.y - goal.rect.y)

def reconstruct_path(node: Node, nodes: list, start_node: Node) -> list:
    path = []
    while node != start_node:
        path.append(node)
        node = nodes[node.parent]
    return path[::-1]

def AStar(g: SearchSpace, sc: pygame.Surface):
    print('Implement A* algorithm')

    open_set = [g.start.id]
    closed_set = []
    nodes = [None] * g.get_length()

    nodes[g.start.id] = g.start

    g.start.g_cost = 0
    g.start.h_cost = heuristic(g.start, g.goal)
    g.start.f_cost = g.start.g_cost + g.start.h_cost

    while open_set:
        current_id = open_set.pop(0)
        current_node = nodes[current_id]

        current_node.set_color(YELLOW, sc)
        pygame.time.delay(5)
        pygame.display.update()

        if current_node == g.goal:
            g.start.set_color(g.start.color, sc)
            g.goal.set_color(PURPLE, sc)

            path = reconstruct_path(g.goal, nodes, g.start)

            for i in range(len(path) - 1):
                pygame.draw.line(sc, WHITE, path[i].rect.center, path[i + 1].rect.center, 3)
                pygame.time.delay(10)
                pygame.display.update()

            pygame.draw.line(sc, WHITE, path[len(path) - 1].rect.center, g.start.rect.center, 3)
            pygame.time.delay(10)
            pygame.display.update()

            return True

        closed_set.append(current_node.id)

        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor.id in closed_set:
                continue

            new_g_cost = current_node.g_cost + 1

            if neighbor.id not in open_set:
                open_set.append(neighbor.id)
                nodes[neighbor.id] = neighbor

            if new_g_cost < neighbor.g_cost:
                neighbor.g_cost = new_g_cost
                neighbor.h_cost = heuristic(neighbor, g.goal)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                neighbor.parent = current_node.id

                neighbor.set_color(RED, sc)
                pygame.time.delay(5)
                pygame.display.update()

        current_node.set_color(BLUE, sc)
        pygame.time.delay(5)
        pygame.display.update()

        current_node.set_color(current_node.color, sc)

        open_set.sort(key=lambda id: nodes[id].f_cost)

    g.start.set_color(g.start.color, sc)
    return False