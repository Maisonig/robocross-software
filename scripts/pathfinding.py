import cv2
import time
import numpy as np

from utils import polar_to_decart


def get_euclidian(dx, dy):
    return np.sqrt(dx ** 2 + dy ** 2)


class Grid:

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
            (-path_discrete, steering, 14.),
            (-path_discrete, 0., 10.),
            (-path_discrete, -steering, 14.),
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
    # th = grid.dir[nodes[0][1], nodes[0][0]]
    # p.append((nodes[0][0], nodes[0][1], th))
    return p


class AstarFinder:
    NON_WALKABLE_WEIGHT = 255

    def __init__(self, robot_radius: float, timeout: float = 1., goal_radius: float = 1.):
        self.goalRad = goal_radius
        self.robotRad = robot_radius
        self.timeout = timeout

        self.circleRad = 1 + int(np.sqrt(self.robotRad ** 2 + self.robotRad ** 2))
        self.circleThick = int(self.circleRad - self.robotRad)

        self.uturnState = 0
        self.uturnDist = 0
        self.uturnGoal = None

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
                # print('timeout')
                # return reconstruct_path(grid, closed_list)
                return "Timeout"
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
                    total = heuristic + grid.pass_weight[y, x] + grid_weight * 10
                    if total < grid.weight[current[1], current[0]] or not grid.visited[y, x]:
                        grid.heuristic[y, x] = heuristic
                        grid.weight[y, x] = total
                        grid.parent[y, x] = (current[0], current[1])
                        if not grid.visited[y, x]:
                            grid.visited[y, x] = True
                            open_list.append(neighbour)
                            open_list_weights.append(total)

    def get_uturn(self, grid, start_pos):
        path = []
        if self.uturnState == 0:
            self.uturnGoal = start_pos
            self.uturnDist = 0.
            self.uturnState = 1
        if self.uturnState == 1:
            cur = (start_pos[0], start_pos[1])
            grid.dir[cur[1], cur[0]] = start_pos[2]
            self.uturnDist += get_euclidian(cur[0] - self.uturnGoal[0], cur[1] - self.uturnGoal[1])
            while self.check_collision(grid, cur, self.robotRad) and self.uturnDist < 56:
                neighbour = grid.get_neighbours(cur)[1]
                neighbour_dir = grid.dir[neighbour[1], neighbour[0]]
                path.append([neighbour[0], neighbour[1], neighbour_dir])
                cur = neighbour
            self.uturnGoal = start_pos
            print(f"{self.uturnDist}")
            try:
                path.pop(-1)
                if len(path) <= 1:
                    self.uturnState = 2
                    self.uturnGoal = start_pos
                    return 'End of first u-turn step'
                return path
            except IndexError:
                self.uturnState = 2
                self.uturnGoal = start_pos

                return 'End of first u-turn step'
        if self.uturnState == 2:
            grid.neighbours = grid.backward_neighbours
            cur = (start_pos[0], start_pos[1])
            grid.dir[cur[1], cur[0]] = start_pos[2]
            goal = get_euclidian(cur[0] - self.uturnGoal[0], cur[1] - self.uturnGoal[1])
            while goal < 56:
                neighbour = grid.get_neighbours(cur)[1]
                neighbour_dir = grid.dir[neighbour[1], neighbour[0]]
                path.append([neighbour[0], neighbour[1], neighbour_dir])
                cur = neighbour
                goal = get_euclidian(cur[0] - self.uturnGoal[0], cur[1] - self.uturnGoal[1])
            try:
                path.pop(-1)
                if len(path) <= 1:
                    self.uturnState = 3
                    grid.neighbours = grid.forward_neighbours
                    return 'End of second u-turn step'
                return path
            except IndexError:
                self.uturnState = 3
                grid.neighbours = grid.forward_neighbours
                return 'End of second u-turn step'


# a = np.zeros((100, 100), dtype=np.uint8)
# cv2.circle(a, [50, 50], 40, [70], -1)
# gr = Grid(np.copy(a), 0.24, 5)
# finder = AstarFinder(5, 1, 20)
#
# path = finder.get_path(gr, [15, 15, 0], [85, 85])
# if type(path) is list:
#     for p in path:
#         cv2.circle(a, [p[0], p[1]], 2, [255], -1)
# else:
#     print(path)
# cv2.imshow('Image', a)
# cv2.waitKey(0)
