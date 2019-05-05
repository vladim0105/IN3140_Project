import numpy as np

L_1 = 11.0
L_2 = 22.3
L_3 = 17.0
L_4 = 8.0
t_4 = np.radians(180 - 45)


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

    t_1 = np.arctan2(y, x) + np.pi / 2

    r = np.sqrt(x ** 2 + y ** 2)
    s = z - L_1

    c_3 = np.sqrt(L_3 ** 2 + L_4 ** 2 - 2 * L_3 * L_4 * np.cos(t_4))
    cw_1 = (L_3 ** 2 - L_4 ** 2 + c_3 ** 2) / (2 * L_3 * c_3)
    cw_2 = (L_2 ** 2 + c_3 ** 2 - (r ** 2 + s ** 2)) / (2 * L_2 * c_3)
    w_1 = np.arctan2(np.sqrt(1 - cw_1 ** 2), cw_1)
    t_3 = np.pi - w_1 - np.arctan2(np.sqrt(1 - cw_2 ** 2), cw_2)

    ca_2 = (L_2 ** 2 - (c_3) ** 2 + (s ** 2 + r ** 2)) / (
        2 * L_2 * np.sqrt(s ** 2 + r ** 2)
    )
    t_2 = np.pi / 2 - np.arctan2(s, r) - np.arctan2(np.sqrt(1 - ca_2 ** 2), ca_2)
    return [t_1, t_2, t_3]

