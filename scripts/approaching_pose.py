#! /usr/bin/env python3
'''
    File name: approaching_pose.py
    Author: Francisco Melo
    Mail: francisco.raposo.melo@tecnico.ulisboa.pt
    Date created: X/XX/XXXX
    Date last modified: X/XX/XXXX
    Python Version: 3.7
'''

import math

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from ellipse import plot_ellipse

import numpy as np

# Radius increment in cm
R_STEP = 1


def get_angle(pos1, pos2):
    "Angle between two points"
    return math.atan2(pos1[1] - pos2[1], pos1[0] - pos2[0])


def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def approachingfiltering_ellipses(personal_space, approaching_filter, idx):
    """Filters the approaching area."""
    # Approaching Area filtering - remove points tha are inside the personal space of a person
    if idx == 1:
        approaching_filter = [(x, y) for x, y in zip(
            approaching_filter[0], approaching_filter[1]) if not personal_space.contains_point([x, y])]
    else:
        cx = [j[0] for j in approaching_filter]
        cy = [k[1] for k in approaching_filter]
        approaching_filter = [(x, y) for x, y in zip(
            cx, cy) if not personal_space.contains_point([x, y])]
    return approaching_filter


def approaching_area_filtering(approaching_area, contour_points):
    """ Filters the approaching area by checking the points inside personal or group space."""

    polygon = Polygon(contour_points)

    approaching_filter = []
    approaching_zones = []
    aux_list = []

    cnt = 0
    for x, y in zip(approaching_area[0], approaching_area[1]):
        if not polygon.contains(Point([x, y])):
            cnt += 1
        else:
            break

    if cnt != 0:
        px = approaching_area[0][0:cnt]
        py = approaching_area[1][0:cnt]

        x_area = []
        y_area = []

        c1 = approaching_area[0][cnt:]
        c2 = approaching_area[1][cnt:]

        x_area = np.concatenate([c1, px])
        y_area = np.concatenate([c2, py])
    else:
        x_area = approaching_area[0]
        y_area = approaching_area[1]

    for x, y in zip(x_area, y_area):
        if not polygon.contains(Point([x, y])):
            approaching_filter.append((x, y))
            aux_list.append((x, y))

        elif aux_list:
            approaching_zones.append(aux_list)
            aux_list = []

    if aux_list:
        approaching_zones.append(aux_list)

    return approaching_filter, approaching_zones


def approaching_heuristic(group_radius, pspace_radius, group_pos, approaching_filter, contour_points, approaching_zones):
    """ """

    approaching_radius = group_radius
    approaching_radius += R_STEP
    if not approaching_filter:
        while not approaching_filter and approaching_radius <= pspace_radius:

            approaching_area = None
            approaching_filter = None
            approaching_zones = None

            approaching_area = plot_ellipse(
                semimaj=approaching_radius, semimin=approaching_radius, x_cent=group_pos[0], y_cent=group_pos[1], data_out=True)
            approaching_filter, approaching_zones = approaching_area_filtering(
                approaching_area, contour_points)

            approaching_radius += R_STEP

    return approaching_filter, approaching_zones


def zones_center(approaching_zones, group_pos, group_radius):
    """ """
    # https://stackoverflow.com/questions/26951544/algorithm-find-the-midpoint-of-an-arc
    center_x = []
    center_y = []
    orientation = []

    for zone in approaching_zones:
        # Sort points clockwise
        zone.sort(key=lambda c: math.atan2(c[0], c[1]))

        idx = int(len(zone) / 2)
        center_x.append(zone[idx][0])
        center_y.append(zone[idx][1])
        orientation.append(get_angle(group_pos, (zone[idx][0], zone[idx][1])))

    return center_x, center_y, orientation


def approaching_pose(robot_pose, approaching_area, group_center):
    """Chooses the nearest center point to the robot from the multiple approaching area."""
    min_dis = 0
    for i, item in enumerate(approaching_area):
        if i == 0:
            min_dis = euclidean_distance(
                robot_pose[0], robot_pose[1], approaching_area[i][0], approaching_area[i][1])
            min_idx = 0
        else:
            dis = euclidean_distance(
                robot_pose[0], robot_pose[1], approaching_area[i][0], approaching_area[i][1])

            if dis < min_dis:
                min_dis = dis
                min_idx = i
    goal_x = approaching_area[min_idx][0]
    goal_y = approaching_area[min_idx][1]
    orientation = math.atan2(
        group_center[1] - goal_y, group_center[0] - goal_x)

    goal_pose = [goal_x, goal_y, orientation]

    return goal_pose
