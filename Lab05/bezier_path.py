"""
Path planning with Bezier curve.
author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import numpy as np
#import scipy.special
import math

show_animation = True
################################################################################################
class BezierSpline2D:
    """
        Compute control points and path given start and end position.
        :param sx: (float) x-coordinate of the starting point
        :param sy: (float) y-coordinate of the starting point
        :param syaw: (float) yaw angle at start
        :param ex: (float) x-coordinate of the ending point
        :param ey: (float) y-coordinate of the ending point
        :param eyaw: (float) yaw angle at the end
        :param offset: (float) of 2nd  (and 3rd) control points as a fraction of distance (e.g. 3)
        :return: (numpy array, numpy array)
        """

    def __init__(self, cp0, cp1, cp2, cp3):
       self.number_pts = 20  # number of discrete spline points
       self.control_points = np.array([
            [cp0[0], cp0[1]], [cp1[0], cp1[1]],
            [cp2[0], cp2[1]], [cp3[0], cp3[1]] ])
       self.pathlength = np.hypot(cp0[0] - cp3[0], cp0[1] - cp3[1])
       print('pl: ', self.pathlength )

    #############################################################################
    def calc_bezier_path_items(self):
        """
        Compute points (variable x), 1st and 2nd derivatives (variable dx and ddx) and
        curvature on bezier spline for the given given control points.
        :param n_points: (int) number of points in the trajectory
        :return: 5 numpy arrays
        """

        rx, ry, ryaw, rk = [], [], [], []
        derivatives_cp = self.bezier_derivatives_control_points(2)  # build 1st and 2nd derivative
        s = np.linspace(0, 1.0, self.number_pts)
        for i_s in s:
            x  = self.bezier(i_s, self.control_points)
            dx  = self.bezier(i_s, derivatives_cp[1])
            ddx = self.bezier(i_s, derivatives_cp[2])
            rx.append(x[0])
            ry.append(x[1])
            ryaw.append(self.calc_yaw(dx))
            rk.append(self.curvature(dx[0], dx[1], ddx[0], ddx[1]))

        lv = [math.sqrt((rx[i] - rx[i - 1]) ** 2 + (ry[i] - ry[i - 1]) ** 2) for i in range(1, len(rx))]
        self.pathlength = sum(lv)

        print('path length: ', self.pathlength)
        return rx, ry, ryaw, rk, s



    #############################################################################
    def calc_bezier_path(self):
        """
        Compute path, points (variable x), 1st and 2nd derivatives (variable dx and ddx) and
        curvature on bezier spline for the given given control points.
        :param n_points: (int) number of points in the trajectory
        :return: 6 numpy arrays
        """
        path = []
        rx, ry, ryaw, rk = [], [], [], []
        derivatives_cp = self.bezier_derivatives_control_points(2)  # build 1st and 2nd derivative
        s = np.linspace(0, 1, self.number_pts)
        for i_s in s:
            x  = self.bezier(i_s, self.control_points)
            dx  = self.bezier(i_s, derivatives_cp[1])
            ddx = self.bezier(i_s, derivatives_cp[2])
            path.append(x)
            rx.append(x[0])
            ry.append(x[1])
            ryaw.append(self.calc_yaw(dx))
            rk.append(self.curvature(dx[0], dx[1], ddx[0], ddx[1]))

        return np.array(path), np.array(rx), np.array(ry), np.array(ryaw), np.array(rk), np.array(s)

    #########################################################################################
    def calc_bezier_path_only_points(self):
        """
        Compute (only) points on bezier spline for the given control points.
        :param control_points: (numpy array)
        :param n_points: (int) number of points in the trajectory
        :return: (numpy array)
        """
        traj = []
        for t in np.linspace(0, 1, self.number_pts):
            traj.append(self.bezier(t, self.control_points))
        return np.array(traj)

    ##################################################################
    def bernstein_poly(self, n, i, t):
        """
        Bernstein polynom.
        :param n: (int) polynom degree
        :param i: (int)
        :param t: (float)
        :return: (float)
        """
        return math.comb(n, i) * t ** i * (1 - t) ** (n - i)
        #return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)

    ###################################################################################
    def bezier(self, t, ctrl_points):
        """
        Return one point on the bezier curve.
        :param t: (float) number in [0, 1]
        :param control_points: (numpy array)
        :return: (numpy array) Coordinates of the point
        """
        n = len(ctrl_points) - 1
        return np.sum([self.bernstein_poly(n, i, t) * ctrl_points[i] for i in range(n + 1)], axis=0)

    #########################################################################################
    def calc_yaw(self, dx):
        """
        calc yaw
        Parameters
        ----------
        dx : float
            tangent on spline
        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        yaw = math.atan2(dx[1], dx[0])
        return yaw

    ##################################################################################
    def bezier_derivatives_control_points(self, n_derivatives):
        """
        Compute control points of the successive derivatives of a given bezier curve.
        A derivative of a bezier curve is a bezier curve.
        See https://pomax.github.io/bezierinfo/#derivatives
        for detailed explanations
        :param control_points: (numpy array)
        :param n_derivatives: (int)
        e.g., n_derivatives=2 -> compute control points for first and second derivatives
        :return: ([numpy array])
        """
        w = {0: self.control_points}
        #print (w)
        for i in range(n_derivatives):
            n = len(w[i])
            w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                                 for j in range(n - 1)])
        return w
##############################################################################

    def curvature(self, dx, dy, ddx, ddy):
        """
        Compute curvature at one point given first and second derivatives.
        :param dx: (float) First derivative along x axis
        :param dy: (float)
        :param ddx: (float) Second derivative along x axis
        :param ddy: (float)
        :return: (float)
        """
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)
#####################################################################################
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

##############################################################################
def main():
    """Plot an example bezier curve."""
    dist = 0.5  # distance of second control point from first
    cp0 = [0.0, 0.0]  # [m]
    s_yaw = np.radians(0.0)  # [rad]
    cp3 = [10.0, 3.0]  # [m]
    e_yaw = np.radians(0.0)  # [rad]
    dist = np.hypot(cp0[0] - cp3[0], cp0[1] - cp3[1])/3

    cp1 = [cp0[0] + dist * np.cos(s_yaw), cp0[1] + dist * np.sin(s_yaw)]
    cp2 = [cp3[0] - dist * np.cos(e_yaw), cp3[1] - dist * np.sin(s_yaw)]
    # cp0 = [0.0, 0.0]
    # cp1 = [0.5, 0.0]
    # cp2 = [0.5, 0.5]
    # cp3 = [1.0, 0.5]
    sp = BezierSpline2D(cp0, cp1, cp2, cp3)
    sp.number_pts = 20
    path, x, y, yaw, c, s = sp.calc_bezier_path()

    t = 0.65  # index of spline --Number in [0, 1]

    x_target, y_target = sp.bezier(t, sp.control_points)
    derivatives_cp = sp.bezier_derivatives_control_points(2)
    point = sp.bezier(t, sp.control_points)
    dt = sp.bezier(t, derivatives_cp[1])
    ddt = sp.bezier(t, derivatives_cp[2])
    curvature = sp.curvature(dt[0], dt[1], ddt[0], ddt[1])
    # Radius of curvature
    radius = 1 / curvature
    # Normalize derivative
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [- dt[1], dt[0]]])
    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
    circle = plt.Circle(tuple(curvature_center), radius, color=(0, 0.8, 0.8), fill=False, linewidth=1,label="Circle of curvature")

    print("path: ", path)
    print("path transposed: \n", path.T)
    if show_animation:  # pragma: no cover
        fig, ax = plt.subplots()
        ax.plot(path.T[0], path.T[1], label="Bezier Path")
        ax.plot(sp.control_points.T[0], sp.control_points.T[1], '--o', label="Control Points")
        ax.plot(x_target, y_target)
        ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
        ax.plot(normal[:, 0], normal[:, 1], label="Normal")
        ax.add_artist(circle)
        #plot_arrow(cp0[0], cp0[1], s_yaw)
        #plot_arrow(cp3[0], cp3[1], e_yaw)
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        plt.show()

##########################################################################################
def main2():
    dist = 5  # distance of second control point from first
    cp0 = [0.0, 0.0]  # [m]
    s_yaw = np.radians(0.0)  # [rad]
    cp3 = [5.0, 3.]  # [m]
    e_yaw = np.radians(0.0)  # [rad]

    cp1 = [cp0[0] + dist * np.cos(s_yaw), cp0[1] + dist * np.sin(s_yaw)]
    cp2 = [cp3[0] - dist * np.cos(e_yaw), cp3[1] - dist * np.sin(s_yaw)]

    # cp0 = [0.0, 0.0]
    # cp1 = [0.5, 0.0]
    # cp2 = [0.5, 0.5]
    # cp3 = [1.0, 0.5]

    sp = BezierSpline2D(cp0, cp1, cp2, cp3)
    sp.number_pts = 20

    # c = curvature on bezier spline (number)
    # s = evenly spaced numbers over interval (ndarray)
    path, x, y, yaw, c, s = sp.calc_bezier_path()

    if show_animation:  # pragma: no cover
        plt.plot(path.T[0], path.T[1], label="Spline Path")

        plot_arrow(cp0[0], cp0[1], s_yaw)
        plot_arrow(cp3[0], cp3[1], e_yaw)
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()

######################################################################
if __name__ == '__main__':
    main()
    #main2()