#! /usr/bin/env python3
'''
    File name: obstacles.py
    Author: Francisco Melo
    Mail: francisco.raposo.melo@tecnico.ulisboa.pt
    Date created: X/XX/XXXX
    Date last modified: X/XX/XXXX
    Python Version: 3.7
'''
import numpy as np
import math

from bresenham import bresenham

# Relation between personal frontal space and back space
BACK_FACTOR = 1.3

# Robot diameter in cm 
ROBOT_DIM = 60

# CONSTANTS
# Human Body Dimensions top view in cm 
HUMAN_Y = 45
HUMAN_X = 20


def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def find_collision(x0, y0, x1, y1, costmap, width):

    bresenham_points = list(bresenham(x0, y0, x1, y1))

    for point in bresenham_points:
        index = point[0] * width + point[1]
        if costmap[index] > 0 :
            print("Intersection")
            return point[0], point[1]
    return None


def adapt_parameters(groups, pparams, gparams, resolution, costmap, origin, width, robot_dim):
    """ """

    ox = origin[0]
    oy = origin [1]
    group_params = []
    groups_params = []

    for j, group in enumerate(groups):
    # Personal Space Adaptation

        sx = pparams[j][0]
        sy = pparams[j][1]
        sx_back = sx / BACK_FACTOR
        for person in group:

            px = person[0]   # in cm
            py = person[1]  # in cm

            x0 = int((px - (resolution/2) - ox) / resolution) # in index 
            y0 = int((py - (resolution/2) - oy) / resolution)  # in index

            angles = [person[2], person[2] + math.pi / 2,
                    person[2] + math.pi, person[2] + (3 * math.pi) / 2]

            for idx, angle in enumerate(angles):

            # d is the search distance to the wall =  gaussian parameter  + robot diameter + safety margin
                if idx == 0:
                    d = sx + robot_dim + 20 
                elif idx == 1 or idx == 3:
                    d = sy + robot_dim + 20
                elif idx == 2:
                    d = sx_back + robot_dim + 20



                px1 = px + (d * math.cos(angle))  # in cm
                py1 = py + (d * math.sin(angle))  # in cm

                x1 = int((px1 - (resolution/2) - ox) / resolution) # in index 
                y1 = int((py1 - (resolution/2) - oy) / resolution)  # in index

                g = find_collision(x0, y0, x1, y1, costmap, width)

                if g is not None:
                    dx = (g[0] * resolution) + (resolution/2) + ox # in cm 
                    dy = (g[0] * resolution) + (resolution/2) + oy # in cm
    

                    # dis is the distance from a person to a wall in a specific orientation
                    dis = euclidean_distance(px, py, dx, dy)  # dis in cm

                    if idx == 0:
                        if dis - sx < robot_dim:  # Check if robot is able to naviagte
                            if dis <= sx:  # Personal space is overlaping obstacle
                                sx = dis
                                if sx < HUMAN_X/2:
                                    sx = HUMAN_X/2
                            elif dis - robot_dim >= HUMAN_X / 2:
                                sx = dis - robot_dim
                                print("NEW sx " + str(sx))
                            else:
                                print("Impossible to adapt parameter sx")

                    elif idx == 1 or idx == 3:
                        if dis - sy < robot_dim:  # Check if robot is able to naviagte
                            if dis <= sy:  # Personal space is overlaping obstacle
                                sy = dis
                                if sy < HUMAN_Y/2:
                                    sy = HUMAN_Y/2
                            elif dis - robot_dim >= HUMAN_Y / 2:
                                sy = dis - robot_dim
                                print("NEW sy " + str(sy))

                            else:
                                print("Impossible to adapt parameter sy")
                    elif idx == 2:

                        if dis - sx_back < robot_dim:  # Check if robot is able to naviagte
                            if dis <= sx_back:  # Personal space is overlaping obstacle
                                sx_back = dis
                                if sx_back < HUMAN_X/2:
                                    sx_back = HUMAN_X/2
                            elif dis - robot_dim >= HUMAN_X / 2:
                                sx_back = dis - robot_dim
                                print("NEW sx_back " + str(sx_back))
                            else:
                                print("Impossible to adapt parameter sx_back")
            parameters = {"sx": sx, "sy": sy, "sx_back":sx_back}
            group_params.append(parameters)
        groups_params.append(group_params)

    return groups_params

