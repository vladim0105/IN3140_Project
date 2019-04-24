import numpy as np

L_1 = 100.9 / 10
L_2 = 222.1 / 10
L_3 = 136.2 / 10


def distToSphereCenter(center_coords, coords):
    x = (coords[0] - center_coords[0]) ** 2
    y = (coords[1] - center_coords[1]) ** 2
    z = (coords[2] - center_coords[2]) ** 2
    return np.sqrt(x + y + z)


def isValid(cart_coord):
    x = cart_coord[0]
    y = cart_coord[1]
    z = cart_coord[2]
    dist = distToSphereCenter([0, 0, L_1], [x, y, z])
    inner = round(L_2 - L_3, 4)
    outer = round(L_2 + L_3, 4)
    if dist < inner or dist > outer:
        return False
    return True


def getThetas(position):
    if not (isValid(position)):
        print("Coordinates not valid!")
        return None
    x = position[0]
    y = position[1]
    z = position[2]
    r = np.sqrt(x ** 2 + y ** 2)
    s = z - L_1
    c_squared = s ** 2 + r ** 2
    D = (c_squared - L_2 ** 2 - L_3 ** 2) / (2 * L_2 * L_3)
    t_1 = np.arctan2(y, x) + np.pi / 2
    t_3 = np.arctan2(np.sqrt(1 - D ** 2), D)
    t_2 = -np.arctan2(s, r) + np.arctan2(
        np.cos(np.pi / 2 - t_3), (L_2 + L_3 * np.sin(np.pi / 2 - t_3))
    )

    return [t_1, t_2, t_3]

