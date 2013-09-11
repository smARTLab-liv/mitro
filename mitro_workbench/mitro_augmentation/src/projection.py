#!/usr/bin/env python
from math import pi, cos, sin
from numpy import matrix, array

def ROS2OpenCV(x, y, z):
    return (z, -x, -y)

def proj(x, y, z):
    (x, y, z) = ROS2OpenCV(x, y, z)
    # rotation
    th = 60 / 180.0 * pi
    R = matrix("1 0 0; 0 %f %f; 0 %f %f"%(cos(th), -sin(th), sin(th), cos(th)))
    P = matrix([x y z]).transpose()
    (x, y, z) = R*P
    

    # distortion
    (k1, k2, p1, p2, k3) = (-0.62021, 0.28664, -0.0204, -0.01703, 0.0)

    # projection
    (fx, fy, cx, cy) = (534.76375, 550.33991, 330.75069, 260.24456)
    
    x_p = x / z
    y_p = y / z
    
    r2 = x_p * x_p + y_p * y_p

    x_pp = x_p * (1 + k1 * r2 + k2 * (r2 * r2) + k3 * (r2 * r2 * r2)) + 2 * p1 * x_p * y_p + p2 * (r2 + 2 * x_p * x_p)
    y_pp = y_p * (1 + k1 * r2 + k2 * (r2 * r2) + k3 * (r2 * r2 * r2)) + p1 * (r2 + 2 * y_p * y_p) + 2 * p2 * x_p * y_p

    u = fx * x_pp + cx
    v = fy * y_pp + cy
    print "%f %f %f -> %d %d"%(x, y, z, u, v)

if __name__ == '__main__':
    proj(0, 0, 2)

