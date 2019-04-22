import numpy as np

L_1 = 100.9
L_2 = 222.1
L_3 = 136.2


def getThetas(position):
    x = position[0]
    y = position[1]
    z = position[2]
    r = np.sqrt(x ** 2 + y ** 2)
    s = z - L_1
    c_squared = s ** 2 + r ** 2
    D = (c_squared - L_2 ** 2 - L_3 ** 2) / (2 * L_2 * L_3)
    t_1 = np.arctan2(y, x)
    t_3 = np.arctan2(np.sqrt(1 - D ** 2) / D)
    t_2 = -np.arctan2(s, r) - np.arctan2(
        (np.cos(np.pi / 2 - t_3) / (L_2 + L_3 * np.sin(np.pi / 2 - t_3)))
    )
    return [t_1, t_2, t_3]

