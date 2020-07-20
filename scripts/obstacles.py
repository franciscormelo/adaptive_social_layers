#! /usr/bin/env python3
'''
    File name: obstacles.py
    Author: Francisco Melo
    Mail: francisco.raposo.melo@tecnico.ulisboa.pt
    Date created: X/XX/XXXX
    Date last modified: X/XX/XXXX
    Python Version: 3.7
'''
import matplotlib.pyplot as plt
import numpy as np
import math

import sys
from algorithm import SpaceModeling


from gaussian_modeling import estimate_gaussians, plot_person, plot_group
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


def find_collision(x0, y0, x1, y1, costmap):

    bresenham_points = list(bresenham(x0, y0, x1, y1))

    for point in bresenham_points:
        if costmap[point[1]][point[0]] == 2:
            print("Intersection")
            return point[0], point[1], costmap

        costmap[point[1]][point[0]] = 3

    return None


class Obstacles(SpaceModeling):
    """ """

    def __init__(self, fh):
        super().__init__(fh)

    def fill_object(self, points, costmap):
        """ """
        for x in range(points[0][0], points[1][0]):
            for y in range(points[0][1], points[2][1]):
                costmap[y, x] = 2

        return costmap

    def plt_obstacles(self, N, persons_costmap, map_limits):
        """ """

        # Gets the coordinates of a windows around the group
        xmin = 0
        xmax = map_limits[1] - map_limits[0]
        ymin = 0
        ymax = map_limits[3] - map_limits[2]

        x_shift = - map_limits[0]
        y_shift = - map_limits[2]

        X = np.linspace(xmin, xmax, N)
        Y = np.linspace(ymin, ymax, N)

        X, Y = np.meshgrid(X, Y)

        obs_costmap = np.zeros([N, N])

        costmap = persons_costmap
        cond = costmap < obs_costmap
        costmap[cond] = obs_costmap[cond]

        fig, ax = plt.subplots(1, 1, tight_layout=True)

        im = plt.imshow(costmap, cmap="jet", extent=[
                        xmin, xmax, ymin, ymax], origin="lower")
        plt.colorbar()

        points = []

        def onclick(event):
            nonlocal points, obs_costmap

            i_x = int((event.xdata * N) / xmax)
            i_y = int((event.ydata * N) / ymax)
            points.append([i_x, i_y])

            if len(points) == 3:
                obs_costmap = self.fill_object(points, obs_costmap)
                points = []
                cond = costmap < obs_costmap
                costmap[cond] = obs_costmap[cond]
                im.set_data(costmap)
                im.autoscale()
                fig.canvas.draw()

        fig.canvas.mpl_connect('button_press_event', onclick)

        plt.show()

        fig2, ax2 = plt.subplots(1, 1, tight_layout=True)
        cs = ax2.contour(X, Y, costmap, cmap="jet", linewidths=0.8, levels=10)
        fig2.colorbar(cs)

        plt.show()

        l_sxy, lsx_back, group_params = self.adapt_parameters(
            x_shift, y_shift, costmap, xmax, ymax, N)

        persons_costmap = np.zeros([N, N])

        approaching_poses, persons_costmap, map_limits = estimate_gaussians(
            self.persons[0], self.group_data, 0, l_sxy, lsx_back, group_params, diff_params=True)

        costmap = np.zeros([N, N])
        costmap = persons_costmap
        cond = costmap < obs_costmap
        costmap[cond] = obs_costmap[cond]

        return costmap, obs_costmap, persons_costmap

    def adapt_parameters(self, x_shift, y_shift, costmap, xmax, ymax, N):
        """ """

        l_sxy = []
        lsx_back = []
        group_params = [self.group_data['ospace_radius']
                        [0], self.group_data['ospace_radius'][0]]

        # Group Space Adaptation
        idx = 0
        group_radius = self.group_data['group_radius'][idx]
        pspace_radius = self.group_data['pspace_radius'][idx]
        ospace_radius = self.group_data['ospace_radius'][idx]
        group_pos = self.group_data['group_pose'][idx]
        group_angles = [0, math.pi/2, math.pi, (3*math.pi)/2]
        
        
        gx = group_pos[0] + x_shift  # in cm
        gy = group_pos[1] + y_shift  # in cm

        xg0 = int((gx * N) / xmax)  # in index
        yg0 = int((gy * N) / ymax)  # in index
        
        for idx, angle in enumerate(group_angles):
            d = group_params[0] + ROBOT_DIM + 20
            

            xg1 = gx + (d * math.cos(angle))  # in cm
            yg1 = gy + (d * math.sin(angle))  # in cm

            xi = int((xg1 * N) / xmax)  # in index
            yi = int((yg1 * N) / ymax)  # in index

            g = find_collision(xg0, yg0, xi, yi, costmap)

            if g is not None:
                dx = (g[0] * xmax) / N  # in cm
                dy = (g[1] * ymax) / N  # in cm
                costmap = g[2]

                # dis is the distance from a person to a wall in a specific orientation
                dis = euclidean_distance(gx, gy, dx, dy)  # dis in cm
                
                if idx == 0 or idx ==2:
                    if dis - group_params[0] < ROBOT_DIM:  # Check if robot is able to naviagte
                        group_params[0] = dis - ROBOT_DIM
                        print("NEW group x " + str(group_params[0]))

                elif idx == 1 or idx == 3:
                    if dis - group_params[1] < ROBOT_DIM:  # Check if robot is able to naviagte
                        group_params[1] = dis - ROBOT_DIM
                        print("NEW group y " + str(group_params[1]))
    
  
        # Personal Space Adaptation
        for person in self.persons[0]:
            param = self.pspace_param[0]
            sx = param[0]
            sy = param[1]
            sx_back = sx / BACK_FACTOR

            px = person[0] + x_shift  # in cm
            py = person[1] + y_shift  # in cm

            x0 = int((px * N) / xmax)  # in index
            y0 = int((py * N) / ymax)  # in index
            angles = [person[2], person[2] + math.pi / 2,
                      person[2] + math.pi, person[2] + (3 * math.pi) / 2]

            for idx, angle in enumerate(angles):

              # d is the search distance to the wall =  gaussian parameter  + robot diameter + safety margin
                if idx == 0:
                    d = sx + ROBOT_DIM + 20
                elif idx == 1 or idx == 3:
                    d = sy + ROBOT_DIM + 20
                elif idx == 2:
                    d = sx_back + ROBOT_DIM + 20

                px1 = px + (d * math.cos(angle))  # in cm
                py1 = py + (d * math.sin(angle))  # in cm

                x1 = int((px1 * N) / xmax)  # in index
                y1 = int((py1 * N) / ymax)  # in index

                g = find_collision(x0, y0, x1, y1, costmap)

                if g is not None:
                    dx = (g[0] * xmax) / N  # in cm
                    dy = (g[1] * ymax) / N  # in cm
                    costmap = g[2]

                    # dis is the distance from a person to a wall in a specific orientation
                    dis = euclidean_distance(px, py, dx, dy)  # dis in cm

                    if idx == 0:
                        if dis - sx < ROBOT_DIM:  # Check if robot is able to naviagte
                            if dis <= sx:  # Personal space is overlaping obstacle
                                sx = dis
                                if sx < HUMAN_X/2:
                                    sx = HUMAN_X/2
                            elif dis - ROBOT_DIM >= HUMAN_X / 2:
                                sx = dis - ROBOT_DIM
                                print("NEW sx " + str(sx))
                            else:
                                print("Impossible to adapt parameter sx")

                    elif idx == 1 or idx == 3:
                        if dis - sy < ROBOT_DIM:  # Check if robot is able to naviagte
                            if dis <= sy:  # Personal space is overlaping obstacle
                                sy = dis
                                if sy < HUMAN_Y/2:
                                    sy = HUMAN_Y/2
                            elif dis - ROBOT_DIM >= HUMAN_Y / 2:
                                sy = dis - ROBOT_DIM
                                print("NEW sy " + str(sy))

                            else:
                                print("Impossible to adapt parameter sy")
                    elif idx == 2:

                        if dis - sx_back < ROBOT_DIM:  # Check if robot is able to naviagte
                            if dis <= sx_back:  # Personal space is overlaping obstacle
                                sx_back = dis
                                if sx_back < HUMAN_X/2:
                                    sx_back = HUMAN_X/2
                            elif dis - ROBOT_DIM >= HUMAN_X / 2:
                                sx_back = dis - ROBOT_DIM
                                print("NEW sx_back " + str(sx_back))
                            else:
                                print("Impossible to adapt parameter sx_back")

            l_sxy.append([sx, sy])
            lsx_back.append(sx_back)

        return l_sxy, lsx_back, group_params


def main():
    if len(sys.argv) > 1:
        file = "data/" + sys.argv[1]

        with open(file) as fh:
            app = Obstacles(fh)
            approaching_poses, persons_costmap, map_limits = app.solve()

            N = len(persons_costmap)
            costmap, obs_costmap, persons_costmap = app.plt_obstacles(
                N, persons_costmap, map_limits)

            fh.close()

            X = np.linspace(map_limits[0], map_limits[1], N)
            Y = np.linspace(map_limits[2], map_limits[3], N)

            X, Y = np.meshgrid(X, Y)
            # fig, ax = plt.subplots(1, 1, tight_layout=True)

            # cs = ax.contour(X, Y, costmap, cmap="jet",
            #                 linewidths=0.8, levels=10)
            # fig.colorbar(cs)
            # plot_kwargs = {'color': 'g', 'linestyle': '-', 'linewidth': 0.8}
            # for person in app.persons[0]:
            #     plot_person(person[0], person[1], person[2], ax, plot_kwargs)

            # idx = 0
            # group_radius = app.group_data['group_radius'][idx]
            # pspace_radius = app.group_data['pspace_radius'][idx]
            # ospace_radius = app.group_data['ospace_radius'][idx]
            # group_pos = app.group_data['group_pose'][idx]
            # plot_group(group_pos, group_radius,
            #            pspace_radius, ospace_radius, ax)
            # ax.set_aspect(aspect=1)
            # plt.show()

            # fig2, ax2 = plt.subplots(1, 1, tight_layout=True)
            # im = plt.imshow(costmap, cmap="jet",
            #                 extent=map_limits, origin="lower")
            # plt.colorbar()
            # ax2.set_aspect(aspect=1)
            # plt.show()
            idx = 0
            group_radius = app.group_data['group_radius'][idx]
            pspace_radius = app.group_data['pspace_radius'][idx]
            ospace_radius = app.group_data['ospace_radius'][idx]
            group_pos = app.group_data['group_pose'][idx]
            group_params = [app.group_data['ospace_radius']
                            [0], app.group_data['ospace_radius'][0]]
            back = 80/1.3
            approaching_poses_fixed, persons_costmap_fixed, map_limits_fixed = estimate_gaussians(
                app.persons[0], app.group_data, 0, [80, 60], back, group_params, diff_params=False)
            
            plot_kwargs = {'color': 'g', 'linestyle': '-', 'linewidth': 0.8}
            fig = plt.figure()
            ax1 = fig.add_subplot(1, 2, 1)
            cs1 = ax1.contour(X, Y, costmap, cmap="jet",
                              linewidths=0.8, levels=10)
            
            for person in app.persons[0]:
                plot_person(person[0], person[1], person[2], ax1, plot_kwargs)
            plot_group(group_pos, group_radius,pspace_radius, ospace_radius, ax1)

            ax2 = fig.add_subplot(1, 2, 2)
            costmap_fixed = np.zeros([N, N])
            costmap_fixed = persons_costmap_fixed
            cond = costmap_fixed < obs_costmap
            costmap_fixed[cond] = obs_costmap[cond]
            cs2 = ax2.contour(X, Y, costmap_fixed, cmap="jet",
                              linewidths=0.8, levels=10)
            for person in app.persons[0]:
                plot_person(person[0], person[1], person[2], ax2, plot_kwargs)
            plot_group(group_pos, group_radius,pspace_radius, ospace_radius, ax2)
            
            # a_circle = plt.Circle((550, 0), ROBOT_DIM/2)
            # b_circle = plt.Circle((550, 0), ROBOT_DIM/2)
            # ax1.add_artist(a_circle)
            # ax2.add_artist(b_circle)

            ax1.set_aspect(aspect=1)
            ax2.set_aspect(aspect=1)
            plt.show()
    else:
        print("Usage: %s <filename>" % (sys.argv[0]))


if __name__ == "__main__":
    main()
