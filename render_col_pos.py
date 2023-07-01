import open3d as o3d
import open3d.visualization.rendering as rendering
import open3d.visualization as vis
import os
from matplotlib import pyplot as plt

import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy
from ros_data import RosData

IMAGE_SIZE = [1440,1080]


def render_RGBBXYZ(render,render_position, render_lookat, render_up=[0,0,1],img_size=[1440,1080],camera_fov = 120.0):
    img_width, img_height = img_size


    scene_z_vector = np.array([0, 0, 1])
    scene_x_vector = np.array([1, 0, 0])

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

    render.scene.scene.set_sun_light([0.707, 0.0, -.707], [1.0, 1.0, 1.0],
                                     100000)
    render.scene.scene.enable_sun_light(True)

    cimg = render.render_to_image()
    print("Saving rendered img")
    render_dir = 'render_imgs/'
    img_name = 'render_img.png'
    o3d.io.write_image(render_dir + img_name, cimg, 9)

    dimg = render.render_to_depth_image(z_in_view_space=True)
    dimg = np.array(dimg, dtype=np.float32)

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


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


if __name__ == "__main__":
    #render load
    render = rendering.OffscreenRenderer(IMAGE_SIZE[0], IMAGE_SIZE[1])
    model_path = os.path.expanduser("~")+"/Documents/models3D/obj/ETHmodel.obj"
    model = o3d.io.read_triangle_model(model_path)

    for idx, mesh in enumerate(model.meshes):
        norm_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh.mesh)
        norm_mesh=norm_mesh.compute_vertex_normals()
        render.scene.add_geometry("mesh"+str(idx), norm_mesh,model.materials[idx])

    # vis.draw([model], width=IMAGE_SIZE[0], height=IMAGE_SIZE[1],)
             
    
    render_position = np.array([-7.0, 0.8, 0.3], dtype=np.float32)
    render_lookat = np.array([0,-0.5,0], dtype=np.float32)

    rgbxyz = render_RGBBXYZ(render,render_position=render_position, 
                            render_lookat=render_lookat, img_size=IMAGE_SIZE,camera_fov=120.0)
    rgb_img = rgbxyz[:,:,:3]/max(rgbxyz[:,:,:3].flatten())

    # plt.imshow(rgb_img)
    # plt.show()

    #every 50th point
    rgbxyz = rgbxyz.reshape(-1,6)
    rgbxyz = rgbxyz[::10,:]
    print(rgbxyz.shape)
    plt.figure(1)
    ax = plt.axes(projection='3d')
    ax.scatter3D(rgbxyz[:,3], rgbxyz[:,4], rgbxyz[:,5], c=rgbxyz[:,:3]/255, marker='.')
    set_axes_equal(ax)

    plt.show()