import cv2
import time
import math
import numpy as np


from util.reeds_shepp import Steering, Gear, get_optimal_word
from util.utils import polar_to_decart


def get_euclidian(dx, dy):
    return np.sqrt(dx ** 2 + dy ** 2)


def word_to_path(start_pose, word, curvature=1.0, step=0.1):
    """
    Преобразует оптимальное слово Reeds-Shepp в траекторию (x,y)

    :param start_pose: Начальная позиция (x, y, yaw) в метрах и радианах
    :param word: Список Letter из get_optimal_word()
    :param curvature: Максимальная кривизна (1/радиус)
    :param step: Шаг дискретизации траектории (в метрах)
    :return: Список точек пути [(x0,y0), (x1,y1), ...]
    """
    x, y, yaw = start_pose
    path = [(x, y)]
    radius = 1.0 / curvature

    for letter in word:
        length = abs(letter.param)
        direction = np.sign(letter.param)
        steps = int(length / step)

        # Прямолинейное движение
        if letter.steering == Steering.STRAIGHT:
            dx = step * direction * math.cos(yaw)
            dy = step * direction * math.sin(yaw)

            if letter.gear == Gear.BACKWARD:
                dx *= -1
                dy *= -1

            for _ in range(steps):
                x += dx
                y += dy
                path.append((x, y))

        # Дуговое движение
        else:
            d_theta = (step / radius) * direction
            if letter.steering == Steering.RIGHT:
                d_theta *= -1

            if letter.gear == Gear.BACKWARD:
                d_theta *= -1

            # Центр вращения
            if letter.steering == Steering.LEFT:
                cx = x - radius * math.sin(yaw)
                cy = y + radius * math.cos(yaw)
            else:
                cx = x + radius * math.sin(yaw)
                cy = y - radius * math.cos(yaw)

            for _ in range(steps):
                yaw += d_theta
                if letter.steering == Steering.LEFT:
                    x = cx + radius * math.sin(yaw)
                    y = cy - radius * math.cos(yaw)
                else:
                    x = cx - radius * math.sin(yaw)
                    y = cy + radius * math.cos(yaw)

                path.append((x, y))

        yaw = yaw % (2 * math.pi)  # Нормализация угла

    return path


class HybridAstarGrid:

    def __init__(self, arr: np.ndarray, steering, path_discrete):

        self.forward_neighbours = [
            (path_discrete, 0., 1),
            (path_discrete, steering, 1.4),
            (path_discrete, -steering, 1.4),
        ]

        self.backward_neighbours = [
            (-path_discrete, 0., 1),
            (-path_discrete, steering, 1.4),
            (-path_discrete, -steering, 1.4),
        ]

        self.both_neighbours = [
            (path_discrete, steering, 1.4),
            (path_discrete, 0., 1),
            (path_discrete, -steering, 1.4),
            (-path_discrete, steering, 2.8),
            (-path_discrete, 0., 2.),
            (-path_discrete, -steering, 2.8),
        ]

        self.pathDiscrete = path_discrete

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
        self.grid = np.array(arr, dtype=np.float32)
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
            x, y = round(x), round(y)
            neighbour = [node[0] + x, node[1] + y]
            try:
                self.pass_weight[neighbour[1], neighbour[0]] = p_w
                self.dir[neighbour[1], neighbour[0]] = th
                neighbours.append(neighbour)
            except IndexError:
                pass
        return neighbours


class HybridAstarFinder:

    def __init__(self, robot_radius: float, timeout: float = 1., goal_radius: float = 1., non_walkable_weight = 255):
        self.goalRad = goal_radius
        self.robotRad = robot_radius
        self.timeout = timeout
        self.non_walkable_weight = non_walkable_weight

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

    def reconstruct_path(self, grid, path_raw, goal_orientation):
        start = path_raw[-1]
        p = [(start[0], start[1], goal_orientation)]
        while start != path_raw[0]:
            x, y = grid.parent[start[1], start[0]]
            th = grid.dir[y, x]
            p.append((x, y, th))
            start = (grid.parent[start[1], start[0]])
        p = list(reversed(p))
        return p

    def get_path(self, grid, start_pos, goal_pos):

        x0 = start_pos[0]
        y0 = start_pos[1]
        th0 = start_pos[2]
        x1 = goal_pos[0]
        y1 = goal_pos[1]
        th1 = goal_pos[2]

        grid.heuristic[y0, x0] = get_euclidian(abs(x0 - x1), abs(y0 - y1))
        grid.weight[y0, x0] = grid.heuristic[y0, x0] + grid.grid[y0, x0]
        grid.dir[y0, x0] = th0

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
            koef = 50
            sp = (x0 / koef, y0 / koef, th0)
            gp = (x1 / koef, y1 / koef, th1)
            word_path = get_optimal_word(sp, gp)
            path = word_to_path(sp, word_path, 1., 0.01)
            pp = []
            for a, b in path:
                pp.append((a * koef, b * koef))
            return pp
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
                    if th1 != th0:
                        return self.reconstruct_path(grid, closed_list, th1)
                    else:
                        return self.reconstruct_path(grid, [(x0, y0)], th1)

            # Если за время таймаута путь не обнаружен, прерываем цикл и ничего не возвращаем
            if time.time() > start_time + self.timeout:
                return "Timeout"

            neighbours = grid.get_neighbours(current)
            for neighbour in neighbours:
                x = neighbour[0]
                y = neighbour[1]
                grid_weight = grid.grid[y, x]
                if grid_weight == self.non_walkable_weight or grid.closed[y, x] or not self.check_collision(grid, neighbour, self.robotRad):
                    pass
                else:
                    heuristic = get_euclidian(abs(x - x1), abs(y - y1))
                    total = heuristic + grid.pass_weight[y, x] + grid_weight * 10
                    if total < grid.weight[current[1], current[0]] or not grid.visited[y, x]:
                        grid.heuristic[y, x] = heuristic
                        grid.weight[y, x] = total
                        grid.parent[y, x] = (current[0], current[1])
                        if not grid.visited[y, x]:
                            grid.visited[y, x] = True
                            open_list.append((neighbour[0], neighbour[1]))
                            open_list_weights.append(total)
