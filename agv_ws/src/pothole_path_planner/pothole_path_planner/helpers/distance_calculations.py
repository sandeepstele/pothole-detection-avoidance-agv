import numpy as np

def get_horiz_dist_to_line(start_point: np.ndarray, end_point: np.ndarray, pothole_coords: np.ndarray):

    difference = end_point-start_point

    if difference[1] != 0:
        slope = (difference[1])/(difference[0])
        intercept = start_point[1] - slope * start_point[0]
        intersect_y = slope * pothole_coords[0] + intercept
    else:
        intersect_y = start_point[1]

    return abs(intersect_y-pothole_coords[1])