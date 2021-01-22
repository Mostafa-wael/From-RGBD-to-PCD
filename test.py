
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    print("Start\n")
    color_raw = o3d.io.read_image("dataset/data/left/scene_1.jpg")
    #depth_raw = open("dataset/data/left/depth_1.txt", "r").read()
    depth_raw = o3d.io.read_image("dataset/data/left/depth_1.jpg")
    print(np.asarray(depth_raw))

    rgbd_image = o3d.open3d.geometry.create_rgbd_image_from_color_and_depth(color_raw, depth_raw)
    print(rgbd_image)

    plt.subplot(1, 2, 1)
    plt.title('grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()

    w = 640
    h = 480
    fx = 320.
    fy = 320.
    cx = 320.
    cy = 240.
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx,fy, cx, cy)
    intrinsic.intrinsic_matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    cam = o3d.camera.PinholeCameraParameters()
    cam.intrinsic = intrinsic
    #cam.extrinsic = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 1.]])
    pcd = o3d.geometry.create_point_cloud_from_rgbd_image(
        rgbd_image, cam.intrinsic)#, cam.extrinsic)
    #np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 1.]]))

    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])
