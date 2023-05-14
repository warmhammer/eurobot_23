import math
import tf.transformations as tft


def get_cube_to_marker_mtxs(cube_markers):
    dolly_markers = cube_markers["dolly_cube_markers"]
    enemy_markers = cube_markers["enemy_cube_markers"]
    dolly_mtxs = {}
    enemy_mtxs = {}
    for marker in dolly_markers:
        id = marker["id"]
        x = marker["x"]
        y = marker["y"]
        mtx = __get_marker_to_cube_mtx(x, y)
        dolly_mtxs[id] = tft.inverse_matrix(mtx)
    for marker in enemy_markers:
        id = marker["id"]
        x = marker["x"]
        y = marker["y"]
        mtx = __get_marker_to_cube_mtx(x, y)
        enemy_mtxs[id] = tft.inverse_matrix(mtx)
    return {"dolly": dolly_mtxs, "enemy": enemy_mtxs}


def __get_marker_to_cube_mtx(x, y):
    if x == 0:
        if y < 0:
            mtx = tft.euler_matrix(math.pi / 2, 0, 0)
        else:
            mtx = tft.euler_matrix(-math.pi / 2, math.pi, 0)
        mtx[1, 3] = y
        return mtx
    
    elif y == 0:
        if x < 0:
            mtx = tft.euler_matrix(math.pi / 2, 0, -math.pi / 2)
        else:
            mtx = tft.euler_matrix(math.pi / 2, 0, math.pi / 2)
        mtx[0, 3] = x
        return mtx
