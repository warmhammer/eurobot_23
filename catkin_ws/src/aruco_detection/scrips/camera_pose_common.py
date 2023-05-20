import numpy as np
import cv2
from cv2 import aruco


def get_static_markers_corners(markers_dict):
    markers_corners = {}

    for marker in markers_dict:
        id = marker["id"]
        size = marker["size"]
        x = marker["coordinates"]["x"]
        y = marker["coordinates"]["y"]

        corners = np.array([[x - size/2, y + size/2, 0],
                            [x + size/2, y + size/2, 0],
                            [x + size/2, y - size/2, 0],
                            [x - size/2, y - size/2, 0]], dtype=np.float32)

        markers_corners[id] = corners

    return markers_corners


def get_table_obj_points(markers_corners):
    min_x = np.min([np.min(matrix[:, 0])
                   for matrix in markers_corners.values()])
    max_x = np.max([np.max(matrix[:, 0])
                   for matrix in markers_corners.values()])
    min_y = np.min([np.min(matrix[:, 1])
                   for matrix in markers_corners.values()])
    max_y = np.max([np.max(matrix[:, 1])
                   for matrix in markers_corners.values()])
    center_x = (max_x + min_x) / 2
    center_y = (max_y + min_y) / 2

    table_pose = center_x, center_y, 0, 0, 0, 0

    obj_points = np.array([[min_x, max_y, 0],
                           [max_x, max_y, 0],
                           [max_x, min_y, 0],
                           [min_x, min_y, 0]], np.float32)

    corners_array = np.array(list(markers_corners.values()))
    keys_list = list(markers_corners.keys())

    top_left_id = keys_list[np.where(
        (corners_array == obj_points[0]).all(axis=2))[0][0]]
    top_right_id = keys_list[np.where(
        (corners_array == obj_points[1]).all(axis=2))[0][0]]
    bottom_right_id = keys_list[np.where(
        (corners_array == obj_points[2]).all(axis=2))[0][0]]
    bottom_left_id = keys_list[np.where(
        (corners_array == obj_points[3]).all(axis=2))[0][0]]

    ordered_ids = top_left_id, top_right_id, bottom_right_id, bottom_left_id

    obj_points[:, 0] -= center_x
    obj_points[:, 1] -= center_y

    return ordered_ids, table_pose, obj_points


def get_corners_by_condition(corners, mean_arr, x_op, y_op):
    result = np.logical_and(
        x_op(corners[:, 0], mean_arr[0]), y_op(corners[:, 1], mean_arr[1]))
    return corners[result][0]


def get_img_points(ordered_ids, corners_dict):
    top_left_id, top_right_id, bottom_right_id, bottom_left_id = ordered_ids

    for id, corners in corners_dict.items():
        mean_arr = np.mean(corners, axis=0)
        if id == top_left_id:
            top_left_corners = get_corners_by_condition(
                corners, mean_arr, np.less, np.less)
        elif id == top_right_id:
            top_right_corners = get_corners_by_condition(
                corners, mean_arr, np.greater, np.less)
        elif id == bottom_right_id:
            bottom_right_corners = get_corners_by_condition(
                corners, mean_arr, np.greater, np.greater)
        elif id == bottom_left_id:
            bottom_left_corners = get_corners_by_condition(
                corners, mean_arr, np.less, np.greater)

    return np.array([[bottom_right_corners, bottom_left_corners, top_left_corners, top_right_corners]])


def get_4x4_rotation_matrix(rvec, tvec):
    rmat, _ = cv2.Rodrigues(rvec)
    rmat_4x4 = np.eye(4)
    rmat_4x4[:3, :3] = rmat
    rmat_4x4[2, :2] *= -1
    rmat_4x4[:2, 2] *= -1
    tvec = tvec.reshape(1, 3)[0]
    tvec[:2] *= -1
    rmat_4x4[:3, 3] = tvec

    return rmat_4x4


def get_quadrilateral_area(points):
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    x4, y4 = points[3]
    return abs((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2 + abs((x4-x3)*(y1-y3)-(x1-x3)*(y4-y3))/2

def get_marker_obj_points(markers_dict):
    for marker in markers_dict:
        size = marker["size"]
        break
    return np.array([[-size/2, size/2, 0],
                     [size/2, size/2, 0],
                     [size/2, -size/2, 0],
                     [-size/2, -size/2, 0]], dtype=np.float32)


def get_table_rotation_matrix(frame, marker_obj_points, ordered_ids, table_obj_points, camera_mtx, dist_coeffs):
    aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters,
                                          cameraMatrix=camera_mtx, distCoeff=dist_coeffs)
    ids = np.ravel(ids).tolist()
    ids_set = set(ids) & set(ordered_ids)
    if len(ids_set) < 1:
        raise ValueError()

    elif len(ids_set) == 4:
        corners_dict = {}
        for id in ordered_ids:
            corners_dict[id] = corners[ids.index(id)][0]
        img_points = get_img_points(ordered_ids, corners_dict)
        _, rvec, tvec = cv2.solvePnP(table_obj_points, img_points, camera_mtx, dist_coeffs)
        rmat_4x4 = get_4x4_rotation_matrix(rvec, tvec)
        return rmat_4x4
    else:
        max_area = 0
        max_area_marker_id = None
        for id in ids_set:
            area = get_quadrilateral_area(corners[ids.index(id)][0])
            if area > max_area:
                max_area = area
                max_area_marker_id = id
        img_points = corners[ids.index(max_area_marker_id)]
        img_points[0] = np.roll(img_points[0], -2, axis=0)
        _, rvec, tvec = cv2.solvePnP(marker_obj_points, img_points, camera_mtx, dist_coeffs)
        rmat_4x4 = get_4x4_rotation_matrix(rvec, tvec)
        return rmat_4x4


def get_table_pose_and_rotation_matrix(frame, static_markers_dict, camera_matrix, dist_coeffs):
    try:
        markers_corners = get_static_markers_corners(static_markers_dict)
        ordered_ids, table_pose, table_obj_points = get_table_obj_points(markers_corners)
        marker_obj_points = get_marker_obj_points(static_markers_dict)
        rmtx = get_table_rotation_matrix(frame, marker_obj_points, ordered_ids, table_obj_points, 
                                        camera_matrix, dist_coeffs)
    except:
        return False, None, None
    
    return True, table_pose, rmtx
