
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import cv2

w = 640
h = 480
fx = 320.
fy = 320.
cx = 320.
cy = 240.
depthScale = 1


def resizeImage(image, scale):  # e.g 50 for 50%
    # calculate the 50 percent of original dimensions
    width = int(image.shape[1] * scale / 100)
    height = int(image.shape[0] * scale / 100)
    # dsize
    dsize = (width, height)
    # resize image
    output = cv2.resize(image, dsize)


def showImages(color_raw, depth_raw):
    plt.subplot(1, 2, 1)
    plt.title('color_raw')
    plt.imshow(color_raw)
    plt.subplot(1, 2, 2)
    plt.title('depth_raw')
    plt.imshow(depth_raw)
    plt.show()


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


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.2989, 0.5870, 0.1140])


def xyzarray(dep_array):
    array_xyz = []
    for i in range(0, len(dep_array)):
        z = calcZ(dep_array[i])
        y = calcY(dep_array[i], z)
        x = calcX(dep_array[i], z)
        arr = [x, y, z]
        array_xyz.append(arr)
    return array_xyz


if __name__ == "__main__":
    color_raw = cv2.imread("dataset/data/left/scene_1.jpg")
    #depth_raw = open("dataset/data/left/depth_1.txt", "r").read()
    depth_raw = cv2.imread("dataset/data/left/depth_1.jpg")
    #color_raw = rgb2gray(color_raw)
    resizeImage(color_raw, 1)
    resizeImage(depth_raw, 1)

    # cv2.imshow("color_raw", color_raw)
    # cv2.imshow("depth_raw", depth_raw)
    #showImages(color_raw, depth_raw)

    color_array = np.asarray(color_raw)
    depth_array = np.asarray(depth_raw)
    # print(color_array)
    # print(depth_array)
    print(color_array[h-1][w-1])
    # for item in depth_array:
    #     print(item[639])
    depth_arr = np.genfromtxt("depth_1.txt", delimiter=',')
    depth_arr.dtype
    print(depth_arr)
    xyz_array = xyzarray(depth_arr)


# intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx,fy, cx, cy)
# intrinsic.intrinsic_matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
# cam = o3d.camera.PinholeCameraParameters()
# cam.intrinsic = intrinsic
# #cam.extrinsic = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 1.]])
# pcd = o3d.geometry.create_point_cloud_from_rgbd_image(
#     rgbd_image, cam.intrinsic)#, cam.extrinsic)
# #np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 1.]]))
#
# # Flip it, otherwise the pointcloud will be upside down
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])
