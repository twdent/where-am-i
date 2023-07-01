import open3d as o3d
import open3d.visualization.rendering as rendering
import open3d.visualization as vis
import os
from matplotlib import pyplot as plt

import numpy as np

from ros_data import RosData
import SuperGluePretrainedNetwork.superglue_pairs as sg
import render_col_pos as rcp
import cv2

from cv_bridge import CvBridge
import rospy


REND_IMAGE_SIZE = [1440,1080]

TARGET_EGO_IMG_SIZE = [640,480]


scale_x = TARGET_EGO_IMG_SIZE[0]/REND_IMAGE_SIZE[0]
scale_y = TARGET_EGO_IMG_SIZE[1]/REND_IMAGE_SIZE[1]


def get_pose(match_3d_pts, match_2d_pts, camera_intrinsics):
    #use PnP to get pose
    retval, rvec, tvec, inliers = cv2.solvePnPRansac(match_3d_pts, match_2d_pts, camera_intrinsics, None)

    return retval, rvec, tvec, inliers
    
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

def save_plots():
    plt.figure(1)
    ax = plt.axes(projection='3d')

    # plot some points from the model
    model_path = os.path.expanduser("~")+"/Documents/models3D/obj/ETHmodel.obj"
    mesh = o3d.io.read_triangle_mesh(model_path)

    pts = np.asarray(mesh.vertices)[0::50]
    # ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='k', marker='.')

    ax.scatter(match_3d_pts[:, 0], match_3d_pts[:, 1], match_3d_pts[:, 2])#, c=np.array(mconf), cmap='viridis')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.scatter(match_3d_pts[:,0], match_3d_pts[:,1], match_3d_pts[:,2], c='b', marker='o')

    #plot camera pos in world frame
    ax.scatter(w_t_wc[0], w_t_wc[1], w_t_wc[2], c='r', marker='o')

    #plot camera z axis in world frame
    c_camera_z_axis = np.array([0, 0, 130])
    w_camera_z_axis = np.matmul(R_wc, c_camera_z_axis)
    ax.quiver(w_t_wc[0], w_t_wc[1], w_t_wc[2], w_camera_z_axis[0], w_camera_z_axis[1], w_camera_z_axis[2], length=1, normalize=True, color='r')
    #plot camera x axis in world frame
    c_camera_x_axis = np.array([1, 0, 0])
    w_camera_x_axis = np.matmul(R_wc, c_camera_x_axis)
    ax.quiver(w_t_wc[0], w_t_wc[1], w_t_wc[2], w_camera_x_axis[0], w_camera_x_axis[1], w_camera_x_axis[2], length=1, normalize=True, color='b')
    #plot camera y axis in world frame
    c_camera_y_axis = np.array([0, 1, 0])
    w_camera_y_axis = np.matmul(R_wc, c_camera_y_axis)
    ax.quiver(w_t_wc[0], w_t_wc[1], w_t_wc[2], w_camera_y_axis[0], w_camera_y_axis[1], w_camera_y_axis[2], length=1, normalize=True, color='g')


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    set_axes_equal(ax)

    #save figure
    plt.savefig('render_imgs/3d_pts.png')

    ax.view_init(elev=0, azim=180)
    plt.savefig('render_imgs/3d_pts2.png')

    ax.view_init(elev=90, azim=0)
    plt.savefig('render_imgs/3d_pts3.png')



if __name__ == "__main__":    
    print("Loading bag file...")
    ros_data = RosData(prod=True,sub_bag=0)
    ros_bag = ros_data.bag

    start = rospy.Time(1679918010)
    end =   rospy.Time(1679918038)
    
    msg_gen = ros_bag.read_messages(start_time=start, end_time=end)
    
    bridge = CvBridge()

    camera_intrinsics = ros_data.get_camera_intrinsics()
    #scale camera intrinsics for TARGET_EGO_IMG_SIZE
    camera_intrinsics = np.array([[camera_intrinsics[0,0]*scale_x, 0, camera_intrinsics[0,2]*scale_x],
                                [0, camera_intrinsics[1,1]*scale_y, camera_intrinsics[1,2]*scale_y],
                                [0, 0, 1]], dtype=np.float32)
    print("scaled camera_intrinsics ", camera_intrinsics)

    candidate_position = None

    #render load
    render = rendering.OffscreenRenderer(REND_IMAGE_SIZE[0], REND_IMAGE_SIZE[1])
    model_path = "./models3D/obj/ETHmodel.obj"
    model = o3d.io.read_triangle_model(model_path)

    for idx, mesh in enumerate(model.meshes):
        norm_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh.mesh)
        norm_mesh=norm_mesh.compute_vertex_normals()
        render.scene.add_geometry("mesh"+str(idx), norm_mesh,model.materials[idx])

    #pure mesh for vis only
    mesh = o3d.io.read_triangle_mesh(model_path,enable_post_processing=True)
    # Show pose in open3d model
    visualizer = vis.Visualizer()
    visualizer.create_window()
    visualizer.add_geometry(mesh)

    camera_point = None

    match_imgs_list = []
    cnt = 0

    xy_pnp_pos = []
    # loop through all the messages in the bag file
    time_since_last_pnp = rospy.Duration(0)
    prev_t = start
    num_matches = []
    while not rospy.is_shutdown():
        
        #Handling errors in corrupt rosbag files
        try:
            topic, msg, t = next(msg_gen)
        except ValueError as e:
            print(e)
            # Try to get the next message
            print(t.to_sec(), "error time")
            print("last topic", topic)
            topic, msg, t = next(msg_gen)
        except StopIteration:
            # This happens when there are no more messages left
            print("StopIteration")
            print("last time", t.to_sec())
            
            break
        except OSError as e:
            print("OSError", e)
            print("Attempting to reload bag file 1s later")
            msg_gen = ros_bag.read_messages(start_time=t+rospy.Duration(1), end_time=end)
            continue

        # if topic == '/alphasense_driver_ros/cam3/color_rect/image/compressed':
        if topic == '/alphasense_driver_ros/cam4/color_rect/image/compressed':
            msg = bridge.compressed_imgmsg_to_cv2(msg)

            #resize to IMAGE_SIZE
            msg = cv2.resize(msg, (TARGET_EGO_IMG_SIZE[0], TARGET_EGO_IMG_SIZE[1]), interpolation=cv2.INTER_AREA)
           
            cv2.imwrite('render_imgs/ego_img.png', msg)
        else:
            continue
        # elif topic == '/alphasense_driver_ros/cam3/color_rect/image/compressed':
        #     msg = bridge.compressed_imgmsg_to_cv2(msg)

        #     cv2.imshow('cam3', msg)
        #     cv2.waitKey(1)

        if time_since_last_pnp < rospy.Duration(0.5):
            time_since_last_pnp += t - prev_t
            prev_t = t
            continue
        else:
            time_since_last_pnp = rospy.Duration(0)
            prev_t = t

        if candidate_position is None:
            candidate_position = np.array([-5.0, 1.2, 0.5], dtype=np.float32)


        #render candidate images

        lookats = [candidate_position + np.array([1, 0.1, -0.2]),
                    #  candidate_position + np.array([0, 1, -0.2]),
                    #     candidate_position + np.array([-1, 0, -0.20]),
                    #     candidate_position + np.array([0, -1, -0.20]),
                        ]
        max_points = 0
        for render_lookat in lookats:
            try:
                img_and_points = rcp.render_RGBBXYZ(render, candidate_position, render_lookat, img_size=REND_IMAGE_SIZE,camera_fov = 120)
            except np.linalg.LinAlgError as e:
                print(e)
                continue
            # #get matching points
            mkpts0, mkpts1, mconf, match_imgs = sg.get_matching_points(img_and_points, msg)
            #save match_imgs
            cv2.imwrite('img_pairs/match_seq/match_img'+str(cnt).zfill(3)+'.png', match_imgs)
            cv2.imshow('matches', match_imgs)
            cv2.waitKey(1)

            cnt += 1

            if len(mkpts0) >= max_points:
                max_points = len(mkpts0)
                best_img_and_points = img_and_points
                best_mkpts0 = mkpts0
                best_mkpts1 = mkpts1
                if len(best_mkpts0) > 10:
                    break

        if len(best_mkpts0) > 4:
            #scale best_mkpts to REND_IMAGE_SIZE
            best_mkpts1 = best_mkpts1 / np.array([scale_x, scale_y])
            #get 3d points of mkpts1 from img_and_points
            all_3d_pts = best_img_and_points[:, :, 3:]
            match_3d_pts = np.array([all_3d_pts[int(mkpt[1]), int(mkpt[0])] for mkpt in best_mkpts1])
            
            #get pose with PnP

            retval, rvec, tvec, inliers  = get_pose(match_3d_pts, best_mkpts0, camera_intrinsics)
            if not retval:
                print("PnP failed, continuing")
                continue

            Rot_mat, _ = cv2.Rodrigues(rvec)

            # Convert to world coordinate system
            C_t_cw = tvec
            C_t_wc = -C_t_cw
            R_cw = Rot_mat
            R_wc = R_cw.T
            #camera translation in world frame
            w_t_wc = np.matmul(R_wc, C_t_wc).flatten()



            #outlier rejection (could be improved by sensor fusion/ kalman filter for ex)
            last_pos = xy_pnp_pos[-1] if len(xy_pnp_pos) > 0 else None
            if last_pos is not None:
                if np.linalg.norm(w_t_wc - last_pos) > 3: #max speed for example
                    print("Rejecting outlier pose")
                    print("max jump", np.linalg.norm(w_t_wc - last_pos))
                    print()
                    continue

            
            #save pose
            xy_pnp_pos.append(w_t_wc)
            
            # #plot in 3D

            if camera_point is not None:
                visualizer.remove_geometry(camera_point)
            camera_point = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=w_t_wc + np.array([0.0,0.0,0.8]))
            camera_point.rotate(R_wc, center=w_t_wc + np.array([0.0,0.0,0.8]))
            visualizer.add_geometry(camera_point)
            visualizer.poll_events()
            visualizer.update_renderer()

            #save the visualizer image
            vis_img = visualizer.capture_screen_float_buffer()
            vis_img = np.asarray(vis_img)
            vis_img = (vis_img * 255).astype(np.uint8)
            vis_img = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)


            #update candidate position, with a vertical offset
            candidate_position = w_t_wc + np.array([0.0, 0, 0.4], dtype=np.float32)
            print("candidate_position ", candidate_position)

            
            #save as npz file
            # np.savez('./ros_anymal/ETH_Cerberus_localization/xy_pnp_pos.npz',
            #         xy_pnp_pos = np.array(xy_pnp_pos))
        
        else:
            print("Not enough matches are found - %d/%d" % (len(best_mkpts0), 5))


        print()
    

    ros_bag.close()

    

    
