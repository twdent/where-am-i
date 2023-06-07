import open3d as o3d
import open3d.visualization.rendering as rendering
import open3d.visualization as vis
import os
from matplotlib import pyplot as plt
import time
import pprint
import numpy as np
from scipy.spatial.transform import Rotation as R

from ros_data import RosData
from SuperGluePretrainedNetwork.superglue_pairs import get_matching_points

import cv2
def render_RGBBXYZ(render_position, render_lookat, render_up=[0,0,1]):
    img_width, img_height = 640, 480
    render = rendering.OffscreenRenderer(img_width, img_height)
    model_path = os.path.expanduser("~")+"/Documents/models3D/obj/ETHmodel.obj"
    model = o3d.io.read_triangle_model(model_path)

    # vis.draw([model],
    #          show_ui=True,
    #          )


    for idx, mesh in enumerate(model.meshes):
        norm_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh.mesh)
        norm_mesh=norm_mesh.compute_vertex_normals()
        render.scene.add_geometry("mesh"+str(idx), norm_mesh,model.materials[idx])

    scene_z_vector = np.array([0, 0, 1])
    scene_x_vector = np.array([1, 0, 0])

    camera_fov = 60.0
    camera_lookat = render_lookat
    camera_pos = render_position
    camera_up = np.array([0, 0, 1], dtype=np.float32)
    render.setup_camera(camera_fov, camera_lookat,camera_pos, camera_up)

    camera_z_vector = camera_lookat - camera_pos
    camera_z_vector /= np.linalg.norm(camera_z_vector)

    camera_x_vector = np.cross(camera_z_vector, camera_up)
    camera_x_vector /= np.linalg.norm(camera_x_vector)
    
    R_camera_to_scene = R.align_vectors(np.vstack([scene_x_vector, scene_z_vector]),
                    np.vstack([camera_x_vector, camera_z_vector]))[0]

    
    rot_matrix = R_camera_to_scene.as_matrix()
    Rt_cs = np.eye(4)
    Rt_cs[:3, :3] = rot_matrix
    Rt_cs[:3, 3] = camera_pos

    print("Rt_matrix: ", Rt_cs)
    print("camera_rot_axis: ", R_camera_to_scene.as_rotvec)
    render.scene.scene.set_sun_light([0.707, 0.0, -.707], [1.0, 1.0, 1.0],
                                     75000)
    render.scene.scene.enable_sun_light(True)

    cimg = render.render_to_image()
    print("Saving rendered img")
    render_dir = 'render_imgs/'
    img_name = 'render_img.png'
    o3d.io.write_image(render_dir + img_name, cimg, 9)

    dimg = render.render_to_depth_image(z_in_view_space=True)
    dimg = np.array(dimg, dtype=np.float32)
    actual_depth = dimg.copy()

    # compute XYZ_camera coordinates of each pixel from the depth image
    # compute the camera intrinsics
    fx = img_width / (2 * np.tan(camera_fov * np.pi / 360))
    fy = img_height / (2 * np.tan(camera_fov * np.pi / 360))
    cx = img_width / 2
    cy = img_height / 2

    # compute the XYZ coordinates
    x, y = np.meshgrid(np.arange(img_width), np.arange(img_height))
    x3 = (x - cx) * dimg / fx
    y3 = (y - cy) * dimg / fy
    z3 = dimg

    xyz_camera = np.stack([x3, y3, z3], axis=2)

    # convert to world coordinates
    xyz_world = xyz_camera @ np.linalg.inv(Rt_cs[:3, :3]).T + Rt_cs[:3, 3]
  
    RBGXYZ_img = np.dstack([cimg, xyz_world])
    return RBGXYZ_img


def get_pose(match_3d_pts, match_2d_pts, camera_intrinsics):
    #use PnP to get pose
    retval, rvec, tvec, inliers = cv2.solvePnPRansac(match_3d_pts, match_2d_pts, camera_intrinsics, None)
    print("retval: ", retval)
    print("rvec: ", rvec)
    print("tvec: ", tvec)
    return retval, rvec, tvec, inliers
    


if __name__ == "__main__":
    pts = np.load('img_pairs/ego_img_render_img_matches.npz')
    
    start = time.time()
    # #get ros imgs and candidate positions
    ros_data = RosData(prod=False)
    timestamp=1679918012.472778
    egocentric_imgs = ros_data.get_ego_imgs(timestamp)
    candidate_position = ros_data.get_candidate_position(timestamp)
    camera_intrinsics = ros_data.get_camera_intrinsics()
    #save ego img
    cv2.imwrite('render_imgs/ego_img.png', egocentric_imgs)

    #render candidate images
    render_position = candidate_position
    render_lookat = np.array([0,0,0], dtype=np.float32)
    img_and_points = render_RGBBXYZ(render_position, render_lookat)

    # #get matching points
    mkpts0, mkpts1, mconf = get_matching_points(img_and_points, egocentric_imgs)

    #get 3d points of mkpts1 from img_and_points
    all_3d_pts = img_and_points[:, :, 3:]
    match_3d_pts = np.array([all_3d_pts[int(mkpt[1]), int(mkpt[0])] for mkpt in mkpts1])

    plt.figure(3)
    ax = plt.axes(projection='3d')
    ax.scatter(match_3d_pts[:, 0], match_3d_pts[:, 1], match_3d_pts[:, 2], c=np.array(mconf), cmap='viridis')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #equal axis scaling
    ax.set_aspect('equal', 'box')

    #get pose with PnP
    pose = get_pose(match_3d_pts, mkpts0, camera_intrinsics)

    ax.scatter(pose[2][0], pose[2][1], pose[2][2], c='r', marker='o')


    #set view
    # ax.view_init(elev=0, azim=np.pi/2)
    #save figure
    plt.savefig('render_imgs/3d_pts.png')
    