# -*- coding: utf-8 -*-
#! /usr/bin/env python
'''
    File name: space_modeling.py
    Author: Francisco Melo
    Mail: francisco.raposo.melo@tecnico.ulisboa.pt
    Date created: X/XX/XXXX
    Date last modified: X/XX/XXXX
    Python Version: 3.7
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import math
from ellipse import plot_ellipse
import sys
from shapely.geometry.point import Point
from shapely import affinity
from approaching_pose import approachingfiltering_ellipses
from gaussian_modeling import estimate_gaussians


SHOW_PLOT = True

# CONSTANTS
# Human Body Dimensions top view in m
HUMAN_Y = 45
HUMAN_X = 20

# Personal Space Maximum 45 - 120 cm - Initial Parameters
# PSPACEX = 80
# PSPACEY = 60
PSPACEX = 120
PSPACEY = 90

STRIDE = 65

# Porpotinal factor between pspace size in x and y axis
PFACTOR = PSPACEX / PSPACEY

INCREMENT = 1


# Relation between personal frontal space and back space
BACK_FACTOR = 1.3


def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def rotate(px, py, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """

    qx = math.cos(angle) * px - math.sin(angle) * py
    qy = math.sin(angle) * px + math.cos(angle) * py
    return qx, qy


def draw_arrow(x, y, angle, ax):  # angle in radians
    """Draws an arrow given a pose."""
    r = 10  # or whatever fits you
    ax.arrow(x, y, r * math.cos(angle), r * math.sin(angle),
             head_length=1, head_width=1, shape='full', color='blue')


def draw_person_top(x, y, angle, ax):
    """Draws a persons from a top view."""
    top_y = HUMAN_Y / 2
    top_x = HUMAN_X / 2
    plot_ellipse(semimaj=top_x, semimin=top_y,
                 phi=angle, x_cent=x, y_cent=y, ax=ax)


def draw_personalspace(x, y, angle, ax, sx, sy, plot_kwargs, idx):
    """Draws personal space of an inidivdual and it."""
    draw_arrow(x, y, angle, ax)  # orientation arrow angle in radians
    ax.plot(x, y, 'bo', markersize=8)
    draw_person_top(x, y, angle, ax)
    ax.text(x + 3, y + 3, "$P_" + str(idx) + "$", fontsize=12)

    plot_ellipse(semimaj=sx, semimin=sy, phi=angle, x_cent=x, y_cent=y, ax=ax,
                 plot_kwargs=plot_kwargs)

    # Multiply sx and sy by 2 to get the diameter
    return patches.Ellipse((x, y), sx * 2, sy * 2, math.degrees(angle))


def group_radius(persons, group_pose):
    """Computes the radius of a group."""
    group_radius = 0  # average of the distance of the group members to the center
    pspace_radius = 0  # Based on the closest person to the group center
    ospace_radius = 0  # Based on the farthest persons to the group center

    sum_radius = 0
    for person in persons:
        # average of the distance between the group members and the center of the group, o-space radius
        distance = euclidean_distance(person[0],
                                      person[1], group_pose[0], group_pose[1])
        sum_radius += distance

        if ospace_radius == 0:
            ospace_radius = distance
        else:
            ospace_aux = distance
            if ospace_aux < ospace_radius:
                ospace_radius = ospace_aux

        if pspace_radius == 0:
            pspace_radius = distance
        else:
            pspace_aux = distance
            if pspace_aux > pspace_radius:
                pspace_radius = pspace_aux

    pspace_radius += HUMAN_X / 2
    ospace_radius -= HUMAN_X / 2
    group_radius = sum_radius / len(persons)
    return group_radius, pspace_radius, ospace_radius


def pspace_intersection(person1, person2, sigmax, sigmay):
    """Returns the intersection between two personal spaces."""

    ellipse1 = create_shapely_ellipse(
        (person1[0], person1[1]), (sigmax, sigmay), person1[2])

    # second ellipse in red
    ellipse2 = create_shapely_ellipse(
        (person2[0], person2[1]), (sigmax, sigmay), person2[2])

    return ellipse1.intersection(ellipse2)


def create_shapely_ellipse(center, lengths, angle=0):
    """
    create a shapely ellipse. adapted from
    https://gis.stackexchange.com/a/243462
    """
    angle = math.degrees(angle)
    circ = Point(center).buffer(1)
    ell = affinity.scale(circ, int(lengths[0]), int(lengths[1]))
    ellr = affinity.rotate(ell, angle)
    return ellr


def minimum_personalspace(sx, sy):
    """Checks if the parameters are at least the human body dimensions."""
    if sy < HUMAN_Y / 2:  # the personal space should be at least the size of the individual
        sy = HUMAN_Y / 2

    if sx < HUMAN_X / 2:  # the personal space should be at least the size of the individual
        sx = HUMAN_X / 2

    return sx, sy


def parameters_computation(person1, person2, sigmax=PSPACEX, sigmay=PSPACEY):
    """Estimates the parameters of the personal space to avoid personal space intersections."""
    ellipse1 = create_shapely_ellipse(
        (person1[0], person1[1]), (sigmax, sigmay), person1[2])

    # second ellipse in red
    ellipse2 = create_shapely_ellipse(
        (person2[0], person2[1]), (sigmax, sigmay), person2[2])

    intersect = ellipse1.intersection(ellipse2)

    # Maneira 2
    diff_angles = abs(person1[2] - person2[2])

    # If the members of the group have the same orientation
    if diff_angles == round(0):

        # Calculation of the angle between the persons

        # Rotation of the person to compute the angle between them
        (px1, py1) = rotate(person1[0], person1[1], person1[2])  # ?
        (px2, py2) = rotate(person2[0], person2[1], person2[2])  # ?

        hip = euclidean_distance(px1, py1, px2, py2)
        co = abs(px2 - px1)
        angle = math.sin(co / hip)  # nao esta bem

        # dis = euclidean_distance(person1[0], person1[1],person2[0], person2[1])
        afactor = ((angle / (2 * math.pi)) + INCREMENT) ** 2

        # o que varias entre duas pessoas com mesma orientacao é sua distancia
        # heuristica nesse caso deve ser a distancia entre estes apenas?
        # o que acontece se tiver translacao em x e y
    ##############################################
    else:
        # Generates a weight between 1 and 2 based on the difference of the angles
        # INCREMENT = 1

        # Squared
        # afactor = ((diff_angles**2) / (2 * math.pi)) + INCREMENT

        # Linear
        afactor = (diff_angles / (2 * math.pi)) + INCREMENT

        # Exponential
        # afactor = (math.exp(diff_angles) / (2 * math.pi)) + INCREMENT

        # Logarithmic
        # afactor = (math.log2(diff_angles) / (2 * math.pi)) + INCREMENT

    area = ellipse1.area - (afactor * 1 * intersect.area)

    if area >= 0:
        area1 = area
    else:
        afactor = 1
        area1 = ellipse1.area - (afactor * 1 * intersect.area)
   # Ellipse area = pi * a * b

    # a = area/(pi * b)
    # sx = area1 / (math.pi * sy)

    # area  = pi * a * b
    # b = a /PFACTOR
    # area = pi * a * a/PFACTOR

    if sigmay <= HUMAN_Y / 2:  # If one of the parameters is already a minimum value, we fix the paramter and determine the other
        sy = HUMAN_Y / 2
        sx = area1 / (math.pi * sy)

    elif sigmax <= HUMAN_X / 2:  # If one of the parameters is already a minimum value, we fix the paramter and determine the other
        sx = HUMAN_X / 2
        sy = area1 / (math.pi * sx)
    else:

        sx = math.sqrt((area1 * PFACTOR) / math.pi)
        sy = sx / PFACTOR

    return sx, sy


def iterative_intersections(person1, person2, sigmax=PSPACEX, sigmay=PSPACEY):
    """Computes the parameters to avoid personal space overlapping between two persons."""

    intersect = pspace_intersection(person1, person2, sigmax, sigmay)

    sx = sigmax
    sy = sigmay

    while intersect.area != 0:
        (sx, sy) = parameters_computation(person1, person2, sx, sy)

        # Check if two persons area overlap due to wrong human dimensions
        intersect_human_dimensions = pspace_intersection(
            person1, person2, HUMAN_X / 2, HUMAN_Y / 2)
        if intersect_human_dimensions.area != 0:
            sx = HUMAN_X / 2
            sy = HUMAN_Y / 2
            break

        else:
            intersect = pspace_intersection(person1, person2, sx, sy)

    return sx, sy


def calc_o_space(persons):
    """Calculates the o-space center of the group given group members pose"""
    c_x = 0
    c_y = 0

    # Group size
    g_size = len(persons)

    for person in persons:
        c_x += person[0] + np.cos(person[2]) * STRIDE
        c_y += person[1] + np.sin(person[2]) * STRIDE

    center = [c_x / g_size, c_y / g_size]

    return center


def plot_group(group_pose, group_radius, pspace_radius, ospace_radius, ax, persons, sx, sy):
    """Plots o-space, p-space, group center and approaching area."""
    # O Space Modeling
    ax.plot(group_pose[0], group_pose[1], 'rx', markersize=8)
    plot_kwargs = {'color': 'r', 'linestyle': '-', 'linewidth': 1}

    plot_ellipse(semimaj=ospace_radius, semimin=ospace_radius, x_cent=group_pose[0],
                 y_cent=group_pose[1], ax=ax, plot_kwargs=plot_kwargs)

    # P Space Modeling
    plot_ellipse(semimaj=pspace_radius, semimin=pspace_radius, x_cent=group_pose[0],
                 y_cent=group_pose[1], ax=ax, plot_kwargs=plot_kwargs)

    # approaching circle area
    plot_kwargs = {'color': 'c', 'linestyle': ':', 'linewidth': 2}
    plot_ellipse(semimaj=group_radius, semimin=group_radius, x_cent=group_pose[0],
                 y_cent=group_pose[1], ax=ax, plot_kwargs=plot_kwargs)
    approaching_area = plot_ellipse(semimaj=group_radius, semimin=group_radius, x_cent=group_pose[0],
                                    y_cent=group_pose[1], data_out=True)

    # Possible approaching area computation and personal space ploting
    plot_kwargs = {'color': 'g', 'linestyle': '-', 'linewidth': 0.8}

    approaching_filter = approaching_area

    for idx, person in enumerate(persons, start=1):
        shapely_diff_sy = 0.5  # Error between python modules used
        shapely_diff_sx = 1
        personal_space = draw_personalspace(
            person[0], person[1], person[2], ax, sx -
            shapely_diff_sx, sy - shapely_diff_sy, plot_kwargs,
            idx)  # plot using ellipse.py functions

        # Approaching Area filtering - remove points that are inside the personal space of a person
        approaching_filter = approachingfiltering_ellipses(
            personal_space, approaching_filter, idx)

    # possible approaching positions
    approaching_x = [j[0] for j in approaching_filter]
    approaching_y = [k[1] for k in approaching_filter]
    ax.plot(approaching_x, approaching_y, 'c.', markersize=5)


def estimate_ellipse_parameters(group_radius, pspace_radius, ospace_radius, group_pose, persons, group_nb):
    """ Estimates ellipse parameters representation of personal space"""
    # Groups of 2 elements:
    if group_nb == 2:

        # Side-by-side arragement
        if round(persons[0][2]) == round(persons[1][2]):  # side-by-side

            sy = euclidean_distance(persons[0][0], persons[0][1], persons[1][0],
                                    persons[1][1]) / 2
            sx = sy * PFACTOR

        # vis-a-vis arrangement
        elif abs(round(persons[0][2] + math.pi, 2)) == abs(round(persons[1][2], 2)):

            sx = euclidean_distance(persons[0][0], persons[0][1], persons[1][0],
                                    persons[1][1]) / 2
            sy = sx / PFACTOR

        # other arrangements
        else:

            (sx, sy) = iterative_intersections(
                persons[0], persons[1], sigmax=PSPACEX, sigmay=PSPACEY)

    # Groups with > 2 elements:
    else:  # The typical arragement  of a group of more than 2 persons is tipically circular

        sx = PSPACEX
        sy = PSPACEY

        # Checks for intersections between all members of the group
        for i in range(len(persons) - 1):
            w = i
            for j in range(w + 1, len(persons)):
                (sx, sy) = iterative_intersections(
                    persons[i], persons[j], sigmax=sx, sigmay=sy)

    # Check if the parameters are possible
    # If the persons are too far away from each other the personal space should be limited
    if sy > PSPACEY or sx > PSPACEX:
        sx = PSPACEX
        sy = PSPACEY
    else:
        # Check if the parameters are less then human dimensions
        (sx, sy) = minimum_personalspace(sx, sy)
    return sx, sy


class SpaceModeling:
    """Models the personal space, group space and estimates the possibles approaching areas."""

    def __init__(self, groups):

        n = len(groups)  # Number of groups in the file

        # Lists intialization
        self.persons = [[] for i in range(n)]
        self.group_data = {'group_pose': [], 'group_radius': [],
                           'ospace_radius': [], 'pspace_radius': [], 'group_nb': []}
        # Stores the personal space parameteres for each group
        self.pspace_param = [[] for i in range(n)]

        for num,group in enumerate(groups):


            self.group_data['group_nb'].append(
                len(group))  # Numbers of members in a group

            for person in group:

                self.persons[num].append(person)

            self.persons[num] = tuple(self.persons[num])


            self.group_data['group_pose'].append(
                    tuple(calc_o_space(self.persons[num])))

            # computes group radius given the center of the o-space
            group_radius1, pspace_radius, ospace_radius = group_radius(
                self.persons[num], self.group_data['group_pose'][num])
            self.group_data['group_radius'].append(group_radius1)
            self.group_data['ospace_radius'].append(ospace_radius)
            self.group_data['pspace_radius'].append(pspace_radius)

    def solve(self):
        """ Estimates the personal space and group space."""

        # Iterate over groups
        for k in range(len(self.group_data['group_nb'])):
            print("Modeling Group " + str(k + 1) + " ...")

            group_radius = self.group_data['group_radius'][k]
            pspace_radius = self.group_data['pspace_radius'][k]
            ospace_radius = self.group_data['ospace_radius'][k]
            group_pose = self.group_data['group_pose'][k]
            persons = self.persons[k]
            group_nb = self.group_data['group_nb'][k]

            sx, sy = estimate_ellipse_parameters(
                group_radius, pspace_radius, ospace_radius, group_pose, persons, group_nb)
            # Stores the parameters of the personal of the individuals of the group
            self.pspace_param[k] = (sx, sy)

            group_params = [self.group_data['ospace_radius']
                            [k], self.group_data['ospace_radius'][k]]

            approaching_poses, persons_costmap, map_limits = estimate_gaussians(persons, self.group_data, k, self.pspace_param[k], self.pspace_param[k][0]/BACK_FACTOR, group_params)

        return self.pspace_param, group_params, approaching_poses
