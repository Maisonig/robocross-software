import time

import cv2
import numpy as np


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def get_euclidian(dx, dy):
    return np.sqrt(dx ** 2 + dy ** 2)


class Grid:

    def __init__(self, arr: np.ndarray, steering, path_discrete):

        self.forward_neighbours = [
            (path_discrete, steering, 1.4),
            (path_discrete, 0., 1),
            (path_discrete, -steering, 1.4),
            # (path_discrete, steering / 2, 1.2),
            # (path_discrete, -steering / 2, 1.2),
        ]

        self.backward_neighbours = [
            (-path_discrete, steering, 1.4),
            (-path_discrete, 0., 1),
            (-path_discrete, -steering, 1.4),
            (-path_discrete, steering / 2, 1.2),
            (-path_discrete, -steering / 2, 1.2),
        ]

        self.both_neighbours = [
            (path_discrete, steering, 1.4),
            (path_discrete, 0., 1),
            (path_discrete, -steering, 1.4),
            (-path_discrete, steering, 2.8),
            (-path_discrete, 0., 2),
            (-path_discrete, -steering, 2.8),
        ]

        self.neighbours = self.forward_neighbours

        self.init_grid(arr)
        self.grid = arr
        self.heuristic = np.zeros_like(arr, dtype=np.float32)
        self.pass_weight = np.zeros_like(arr, dtype=np.float32)
        self.weight = np.zeros_like(arr, dtype=np.float32)
        self.dir = np.zeros_like(arr, dtype=np.float32)

        self.visited = np.zeros_like(arr, dtype=np.bool_)
        self.closed = np.zeros_like(arr, dtype=np.bool_)

        self.parent = np.zeros_like(arr, dtype=tuple)

    def init_grid(self, arr):
        self.grid = arr
        self.heuristic = np.zeros_like(arr, dtype=np.float32)
        self.pass_weight = np.zeros_like(arr, dtype=np.float32)
        self.weight = np.zeros_like(arr, dtype=np.float32)
        self.dir = np.zeros_like(arr, dtype=np.float32)

        self.visited = np.zeros_like(arr, dtype=np.bool_)
        self.closed = np.zeros_like(arr, dtype=np.bool_)

        self.parent = np.zeros_like(arr, dtype=tuple)

    def get_neighbours(self, node):
        neighbours = []
        for r, th, p_w in self.neighbours:
            th = self.dir[node[1], node[0]] + th
            x, y = polar_to_decart(r, th)
            x, y = int(x), int(y)
            neighbour = [node[0] + x, node[1] + y]
            try:
                self.pass_weight[neighbour[1], neighbour[0]] = p_w
                self.dir[neighbour[1], neighbour[0]] = th
                neighbours.append(neighbour)
            except IndexError:
                pass
        return neighbours


def reconstruct_path(grid, nodes):
    p = []
    p_ = []
    nodes = list(reversed(nodes))
    start = nodes[0]
    while start != nodes[-1]:
        x, y = grid.parent[start[1], start[0]]
        th = grid.dir[start[1], start[0]]
        p.append((x, y, th))
        p_.append((x, y))
        start = (grid.parent[start[1], start[0]])
    p = list(reversed(p))
    th = grid.dir[nodes[0][1], nodes[0][0]]
    p.append((nodes[0][0], nodes[0][1], th))
    return p


class AstarFinder:
    NON_WALKABLE_WEIGHT = 255

    def __init__(self, robot_radius: float, timeout: float = 1., goal_radius: float = 1.):
        self.goalRad = goal_radius
        self.robotRad = robot_radius
        self.timeout = timeout

        self.circleRad = 1 + int(np.sqrt(self.robotRad ** 2 + self.robotRad ** 2))
        self.circleThick = int(self.circleRad - self.robotRad)

    def check_collision(self, grid, node, radius):
        try:
            col = np.copy(grid.grid[int(node[1] - radius):int(node[1] + radius),
                          int(node[0] - radius):int(node[0] + radius)])
            cv2.circle(col, [int(radius), int(radius)], self.circleRad, [127], self.circleThick * 2)
        except cv2.error:
            return False
        try:
            if np.max(col) == 255:
                return False
            else:
                return True
        except Exception as e:
            print(e)
            return False

    def get_path(self, grid, start_pos, goal_pos):

        x0 = start_pos[0]
        y0 = start_pos[1]
        a0 = start_pos[2]
        x1 = goal_pos[0]
        y1 = goal_pos[1]

        grid.heuristic[y0, x0] = get_euclidian(abs(x0 - x1), abs(y0 - y1))
        grid.weight[y0, x0] = grid.heuristic[y0, x0] + grid.grid[y0, x0]
        grid.dir[y0, x0] = a0

        open_list = [(x0, y0)]
        open_list_weights = [grid.weight[y0, x0]]
        closed_list = []

        start_time = time.time()

        while True:
            # Поиск индекса минимального веса, обозначение ячейки как текущей и удаление ячейки и веса из открытого
            # списка
            if len(open_list) == 0:
                if len(closed_list) <= 1:
                    return "Robot in collision"
                else:
                    return reconstruct_path(grid, closed_list)
            min_index = open_list_weights.index(min(open_list_weights))
            current = open_list[min_index]
            open_list.pop(min_index)
            open_list_weights.pop(min_index)
            grid.visited[current[1], current[0]] = True
            #
            closed_list.append(current)
            grid.closed[current[1], current[0]] = True

            # Если достигли финишной ноды, прерываем цикл и возвращаем путь
            if x1 - self.goalRad < current[0] < x1 + self.goalRad:
                if y1 - self.goalRad < current[1] < y1 + self.goalRad:
                    return reconstruct_path(grid, closed_list)

            # Если за время таймаута путь не обнаружен, прерываем цикл и ничего не возвращаем
            if time.time() > start_time + self.timeout:
                # return "Timeout"
                pass

            neighbours = grid.get_neighbours(current)
            for neighbour in neighbours:
                x = neighbour[0]
                y = neighbour[1]
                grid_weight = grid.grid[y, x]
                if grid_weight == self.NON_WALKABLE_WEIGHT or grid.closed[y, x] or not self.check_collision(grid, neighbour, self.robotRad):
                    pass
                else:
                    heuristic = get_euclidian(abs(x - x1), abs(y - y1))
                    total = heuristic + grid.pass_weight[y, x] + grid_weight
                    if total < grid.weight[current[1], current[0]] or not grid.visited[y, x]:
                        grid.heuristic[y, x] = heuristic
                        grid.weight[y, x] = total
                        grid.parent[y, x] = (current[0], current[1])
                        if not grid.visited[y, x]:
                            grid.visited[y, x] = True
                            open_list.append(neighbour)
                            open_list_weights.append(total)
