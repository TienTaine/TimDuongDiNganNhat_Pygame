import pygame
from const import *
import random

# you can change the random seed but when you submit your work, it should be run on my random seed!
random.seed(2345)

class Node:
    def __init__(self, x, y, a, id, is_brick=False) -> None:
        self.rect = pygame.Rect(x, y, a, a)
        self.is_brick = is_brick
        self.color = BLACK if self.is_brick else WHITE
        self.id = id
        self.g_cost = 0  # Cost từ điểm bắt đầu đến node hiện tại
        self.h_cost = 0  # Heuristic cost từ node hiện tại đến điểm đích
        self.f_cost = 0  # Tổng g_cost và h_cost

        self.parent = None  # Node cha của node hiện tại
    def draw(self, sc:pygame.Surface) -> None:
        pygame.draw.rect(sc, self.color, self.rect)

    def _set_color(self, color):
        self.color = color

    def set_color(self, color, sc:pygame.Surface):
        self.color = color
        self.draw(sc)

        # change the speed here
        pygame.time.delay(5)
        pygame.display.update()

    def calculate_f_cost(self):
        self.f_cost = self.g_cost + self.h_cost
class SearchSpace:
    def __init__(self) -> None:
        self.grid_cells:list[Node] = []
        for i in range(ROWS):
            for j in range(COLS):
                # define the brick's appearing
                is_brick = True if random.randint(1,3) == 1 else False
                self.grid_cells.append(Node(j*(A+A1)+BOUND, i*(A+A1)+BOUND, A, i*COLS+j, is_brick))

        self.start:Node = self.grid_cells[0]
        self.start.is_brick = False
        self.start._set_color(ORANGE)
        self.goal:Node = self.grid_cells[-1]
        self.goal.is_brick = False
        self.goal._set_color(PURPLE)

    def draw(self, sc:pygame.Surface):
        for node in self.grid_cells:
            node.draw(sc)
        pygame.display.flip()

    def get_length(self):
        return len(self.grid_cells)

    def is_goal(self, node:Node):
        return node.id == self.goal.id

    def get_neighbors(self, node: Node) -> list[Node]:
        x, y = node.id%COLS, node.id//COLS

        # define the directions of agent
        up    = (y-1)*COLS + x if y-1 >= 0 else None
        down  = (y+1)*COLS + x if y+1 < ROWS else None
        left  = y*COLS + (x-1) if x-1 >= 0 else None
        right = y*COLS + (x+1) if x+1 < COLS else None

        left_up = (y-1)*COLS + (x-1) if y-1 >= 0 and x-1 >= 0 else None
        left_down = (y+1)*COLS + (x-1) if y+1 < ROWS and x-1 >= 0 else None
        right_up = (y-1)*COLS + (x+1) if y-1 >= 0 and x+1 < COLS else None
        right_down = (y+1)*COLS + (x+1) if y+1 < ROWS and x+1 < COLS else None

        directions = [up, down, left, right, left_up, left_down, right_up, right_down]
        # directions = [up, down, left, right]
        neighbors = []
        for dir_ in directions:
            if dir_ is not None and not self.grid_cells[dir_].is_brick:
                neighbors.append(self.grid_cells[dir_])

        return neighbors