
import matplotlib.pyplot as plt
import numpy as np
import cv2
import pptk

#     fx          cx               fy      cy
P = [320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
w = 640
h = 480
fx = P[0]
fy = P[5]
cx = P[2]
cy = P[6]
tx = P[3]
ty = P[7]
depthScale = 1


def resizeImage(image, scale):  # e.g 50 for 50%
    # calculate the 50 percent of original dimensions
    width = int(image.shape[1] * scale / 100)
    height = int(image.shape[0] * scale / 100)
    # dsize
    dsize = (width, height)
    # resize image
    output = cv2.resize(image, dsize)


def calcZ(d):
    z = d / depthScale
    return z


def calcX(u, z):
    CX = cx
    Z = z
    FX = fx
    X = (u - CX)*Z/FX
    return X


def calcY(v, z):
    CY = cy
    Z = z
    FY = fy
    Y = (v - CY)*Z/FY
    return Y


def xyz_array(dep_array):
    array_xyz = []
    for i in range(0, h):
        for j in range(0, w):
            z = calcZ(dep_array[i+j])
            y = calcY(i, z)
            x = calcX(j, z)
            arr = [x, y, z]
            array_xyz.append(arr)
    return array_xyz


if __name__ == "__main__":

    depth_arr = np.genfromtxt("dataset/data/left/depth_1.txt", delimiter=',')
    depth_arr.dtype
    print(depth_arr)
    xyz = xyz_array(depth_arr)
    print(xyz)  # takes long time to execute
    # v = pptk.viewer(xyz_array)  # not working yet, documentation: https://heremaps.github.io/pptk/tutorials/viewer/tanks_and_temples.html
