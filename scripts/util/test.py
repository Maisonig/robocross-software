import math
import numpy as np
import scripts.util.reeds_shepp as rs
from scripts.util.reeds_shepp import Steering, Gear


def word_to_path(start_pose, word, curvature=2.0, step=0.2):
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


word = rs.get_optimal_word((0., 0., 0.), (-2, 0, 1.57))

# Преобразуем в координаты
path = word_to_path(
    start_pose=(0.0, 0.0, 0.0),
    word=word,
    curvature=1,  # Максимальная кривизна
    step=0.01      # Шаг дискретизации
)

# Визуализация
import matplotlib.pyplot as plt
plt.plot(*zip(*path), 'b-')
plt.axis('equal')
plt.show()