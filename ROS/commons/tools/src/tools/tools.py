#! /usr/bin/env python

from __future__ import division
import rospy
from roboland_msgs.msg import ServiceRobotCollider
from geometry_msgs.msg import Pose2D
from rospy.numpy_msg import numpy_msg
import numpy as np
from scipy import ndimage
from math import *
import matplotlib.pyplot as plt
from copy import deepcopy

EPS = 1E-5

class SplineException(Exception):
    def __init__(self, message):
        super(SplineException, self).__init__(message)

class BSpline:
    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

        def __str__(self):
            return 'x: {} y: {}'.format(self.x, self.y)

        def __cmp__(self):
            if np.fabs(self.x - other.x) < EPS and \
               np.fabs(self.y - other.y) < EPS:
                return 0
            elif self.x > other.x:
                return 1
            elif self.x == other.x:
                if self.y > other.y:
                    return 1
            return -1

    class Spline:
        def __init__(self, points, s_i_1):
            if len(points) == 3:
                self.points = points
                self.start_x = self.points[0].x
                self.end_x = self.points[-2].x
                self.calculate_control_points(s_i_1)
            elif len(points) == 2:
                self.start_x = points[0].x
                self.end_x = points[1].x

                self.points = []
                self.points.append(points[0])
                self.points.append(points[0])
                self.points.append(points[1])

                self.control_points = []
                self.control_points.append(BSpline.Point(0, s_i_1.y))
                self.control_points.append(BSpline.Point(1/3,
                                                         (2 * points[0].y + points[1].y) / 3))
                self.control_points.append(BSpline.Point(2/3,
                                                         (points[0].y + 2 * points[1].y) / 3))
                self.control_points.append(BSpline.Point(1, points[1].y))

        def __str__(self):
            return 'Points: {} \n Control points: {}'.format(', '.join([str(p) for p in self.points]),
                                                             ', '.join([str(p) for p in self.control_points]))

        def calculate_control_points(self, s_i_1):
            self.control_points = []
            self.control_points.append(BSpline.Point(0,
                                                     s_i_1.y))
            self.control_points.append(BSpline.Point(1 / 3,
                                                     (2 * self.points[0].y + self.points[1].y) / 3))
            self.control_points.append(BSpline.Point(2 / 3,
                                                     (self.points[0].y + 2 * self.points[1].y) / 3))
            self.control_points.append(BSpline.Point(1,
                                                     (self.points[0].y + 4 * self.points[1].y \
                                                      + self.points[2].y) / 6))

        def get_last_control_point(self):
            return self.control_points[-1]

        def get_control_points(self):
            return [BSpline.Point((p.x * (self.end_x - self.start_x) + self.start_x), p.y) \
                    for p in self.control_points]

        def is_in_range(self, x):
            if x < self.start_x:
                return -1
            elif x > self.end_x:
                return 1

            return 0

        def get_value(self, x):
            if self.is_in_range(x) != 0:
                raise SplineException('X {} is not in '
                                      'range {} and {}'.format(x,
                                                               self.start_x,
                                                               self.end_x))

            ps = [p.y for p in self.control_points]

            x = (x - self.start_x) / (self.end_x - self.start_x)

            while len(ps) > 1:
                temp_ps = []
                for i in range(len(ps) - 1):
                    val = (1 - x) * ps[i] + x * ps[i + 1]
                    temp_ps.append(val)
                ps = deepcopy(temp_ps)

            return ps[0]

        def get_der_value(self, x):
            if self.is_in_range(x) != 0:
                raise SplineException('X {} is not in '
                                      'range {} and {}'.format(x,
                                                               self.start_x,
                                                               self.end_x))

            x = (x - self.start_x) / (self.end_x - self.start_x)
            ps = [p.y for p in self.control_points]

            return 3 * (1 - x) ** 2 * (ps[1] - ps[0]) + \
                6 * (1 - x) * x * (ps[2] - ps[1]) + \
                3 * x**2 * (ps[3]- ps[2])

        def get_der_2_value(self, x):
            if self.is_in_range(x) != 0:
                raise SplineException('X {} is not in '
                                      'range {} and {}'.format(x,
                                                               self.start_x,
                                                               self.end_x))

            x = (x - self.start_x) / (self.end_x - self.start_x)
            ps = [p.y for p in self.control_points]

            return 6 * (1 - x) * (ps[2] - 2 * ps[1] + ps[0]) + \
                6 * x * (ps[3] - 2 * ps[2] + ps[1])

    def __init__(self, initial_points):
        self.splines = []

        self.splines.append(BSpline.Spline(initial_points[:2], initial_points[0]))

        for i in range(2, len(initial_points)):
            self.add_point(initial_points[i])

    def add_point(self, point):
        last_spline = self.splines[-1]
        del self.splines[-1]

        last_points = last_spline.points[1:]
        last_points.append(point)
        self.splines.append(BSpline.Spline(last_points,
                                           self.splines[-1].get_last_control_point() if len(self.splines) else last_points[0]))
        self.splines.append(BSpline.Spline([last_points[1], last_points[2]],
                                           self.splines[-1].get_last_control_point()))

    def get_value(self, x):
        for spline in self.splines:
            if spline.is_in_range(x) == 0:
                return spline.get_value(x)

        return 0

    def get_der_value(self, x):
        for spline in self.splines:
            if spline.is_in_range(x) == 0:
                return spline.get_der_value(x)

        return 0

    def get_der_2_value(self, x):
        for spline in self.splines:
            if spline.is_in_range(x) == 0:
                return spline.get_der_2_value(x)

        return 0

    def erase(self, x):
        self.splines = [spline for spline in self.splines if spline.is_in_range(x) != 1]

    def eraseUpper(self, x):
        self.splines = [spline for spline in self.splines if spline.is_in_range(x) != -1]

        print len(self.splines)

        for spline in self.splines:
            if spline.is_in_range(x) == 1:
                continue

            last_spline = self.splines[-1]
            p = BSpline.Point(x, last_spline.get_value(x))
            del self.splines[-1]
            last_spline = self.splines[-1]
            del self.splines[-1]
            if abs(last_spline.end_x - x) < EPS:
                self.splines.append(BSpline.Spline([last_spline.points[0], last_spline.points[1]], self.splines[-1].get_last_control_point()))
                return

            self.splines.append(BSpline.Spline([last_spline.points[0], last_spline.points[1], p], self.splines[-1].get_last_control_point()))
            self.splines.append(BSpline.Spline([last_spline.points[1], p], self.splines[-1].get_last_control_point()))

    def draw(self, fig, time_stamps, fcn, cl):
        values = []

        for time in time_stamps:
            values.append(getattr(self, fcn)(time))

        plt.plot(time_stamps, values, cl)
        plt.draw()

def main():
    min_time = 0
    max_time = 10
    min_thr = -2.0
    max_thr = 2.0

#    points = [[0, 0], [1, 1], [2, 2]]

    points = [[p, np.cos(p * np.pi)] for p in np.arange(0, max_time, 0.1)]

    bezier_spline = BSpline([BSpline.Point(p[0], p[1]) for p in points])

    del_time = 3.28
    bezier_spline.eraseUpper(del_time)
    max_time = del_time

    min_del_time = 1.0
    bezier_spline.erase(min_del_time)
    min_time = min_del_time

    fig = plt.figure()
    ax1 = fig.add_subplot(111)

    bezier_spline.draw(fig, np.arange(min_time, max_time, 0.01), 'get_value', 'y')
    bezier_spline.draw(fig, np.arange(min_time, max_time, 0.01), 'get_der_value', 'r')
    bezier_spline.draw(fig, np.arange(min_time, max_time, 0.01), 'get_der_2_value', 'g')
#    plt.plot([p[0] for p in points], [p[1] for p in points], 'ro')
    control_points = []
    for spline in bezier_spline.splines:
        control_points.extend(spline.get_control_points())
    plt.plot([p.x for p in control_points], [p.y for p in control_points], 'bo')
    plt.show()

if __name__ == "__main__":
    main()
