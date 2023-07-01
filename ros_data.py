import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy

class RosData:

    def __init__(self, prod=False,sub_bag=False):
        self.rosbag_folder = './ros_anymal/ETH_Cerberus_localization/'

        if sub_bag is False:
            self.sub_folder = 'Processed_bag/' 
            self.bag_name =  'mergedBag.bag'
        else:
            print("loading sub bag: ", sub_bag)
            self.sub_folder ='jetson/' 
            self.bag_name = f'2023-03-27-13-48-57_anymal-cerberus-jetson_mission_{sub_bag}.bag'

        

        self.camera_topics = ['/alphasense_driver_ros/cam3/color_rect/image/compressed', 
                              '/alphasense_driver_ros/cam4/color_rect/image/compressed', 
                              '/alphasense_driver_ros/cam5/color_rect/image/compressed', 
                             ]
        self.camera_info_topics = ['/alphasense_driver_ros/cam3/color_rect/camera_info',
                                    '/alphasense_driver_ros/cam4/color_rect/camera_info',
                                    '/alphasense_driver_ros/cam5/color_rect/camera_info',
                                    ]
        self.odometry_topics = ['/compslam_lio/odometry',
                              '/state_estimator/odometry',
                              ]
        self.rtk_topics = ['/rtk_gps_driver/position_receiver_0/ros/pos_ecef',
                            ]   
        
        self.intrinsics = np.array([377.1323930964233, 0.0, 704.90459865174, 
                                    0.0, 376.76480664462014, 523.3273863186171, 
                                    0.0, 0.0, 1.0]).reshape(3,3)
        self.prod = prod
        if self.prod:                        
            self.bag : rosbag.Bag = rosbag.Bag(self.rosbag_folder + self.sub_folder + self.bag_name)
        
    def generate_imgs(self):

        bridge = CvBridge()
        topic = [
        # '/alphasense_driver_ros/cam4/color/camera_info',
        #  '/alphasense_driver_ros/cam4/color/image/compressed', 
        # '/alphasense_driver_ros/cam4/color_rect/camera_info', 
        '/alphasense_driver_ros/cam4/color_rect/image/compressed', 
        # '/alphasense_driver_ros/cam4/debayered/camera_info', '/alphasense_driver_ros/cam4/debayered/image/compressed', 
        # '/elevation_mapping/elevation_map_raw', '/elevation_mapping/semantic_map_raw', '/elevation_mapping/statistics']
        ]
        # for topic, msg, t in bag.read_messages(topics=[topic]):
        bag_messages = self.bag.read_messages(topics=topic)
        for topic, msg, t in bag_messages:
            #view the compressed image
            
                
            msg = bridge.compressed_imgmsg_to_cv2(msg)
            cv2.imwrite(f'./ros_anymal/ETH_Cerberus_localization/cerberus_imgs_4/eth_cerberus{t.to_sec()}.png', msg)

        self.bag.close()

    def get_ego_imgs(self,timestamp):
        #presaved_imgs
        containing_folder = './ros_anymal/ETH_Cerberus_localization/cerberus_imgs/'
        return cv2.imread(containing_folder + f'eth_cerberus{timestamp}.png')
        

    def get_candidate_position(self,timestamp):
        return np.array([-7, 0, 0.5], dtype=np.float32)
    
    def get_camera_intrinsics(self):
        if not self.prod:
            return self.intrinsics
        msgs = self.bag.read_messages(topics=['/alphasense_driver_ros/cam4/color_rect/camera_info'])
        # read the K matrix from first message header
        for topic, msg, t in msgs:
            self.instrinsics = np.array(msg.K).reshape(3,3)
            print(self.instrinsics)
            break
        return self.instrinsics
    
    def get_camera_fov(self,img_size):
        #calculate fov from intrinsics and img_size
        fx = self.intrinsics[0,0]
        fy = self.intrinsics[1,1]
        fov_x = 2*np.arctan(img_size[0]/(2*fx)) * 180/np.pi
        fov_y = 2*np.arctan(img_size[1]/(2*fy)) * 180/np.pi
        return fov_x, fov_y
    
    def get_positions(self):
        xy_lio_odom = []
        xy_state_odom = []
        xy_rtk = []
        bag_messages = self.bag.read_messages(topics=self.odometry_topics + self.rtk_topics)
        for topic, msg, t in bag_messages:
            if topic == self.odometry_topics[0]:
                xy_lio_odom.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
            elif topic == self.odometry_topics[1]:
                xy_state_odom.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
            elif topic == self.rtk_topics[0]:
                xy_rtk.append([msg.point.x, msg.point.y])

        #save as npz file
        np.savez('./ros_anymal/ETH_Cerberus_localization/positions.npz',
                xy_lio_odom = np.array(xy_lio_odom),
                xy_state_odom = np.array(xy_state_odom),
                xy_rtk = np.array(xy_rtk)
                )
        
        return xy_lio_odom, xy_state_odom, xy_rtk

         
if __name__ == "__main__":
    # ros_bag = RosData(prod=True)
    # ros_bag.get_camera_intrinsics()
    # ros_bag.get_positions()


    # Load the ROS bag file

    print("Loading bag file...")
    ros_bag = RosData(prod=True,sub_bag=2).bag

    start = rospy.Time(1679917935)
    end =   rospy.Time(1679917978)
    msg_gen = ros_bag.read_messages(start_time=start)#, end_time=end)
    
    bridge = CvBridge()
    compslam_lio_odom = []
    state_est_odom = []
    while not rospy.is_shutdown():
        try:
            topic, msg, t = next(msg_gen)
        except ValueError as e:
            print(e)
            # Try to get the next message
            print(t, "error time")
            print("last topic", topic)
            topic, msg, t = next(msg_gen)
        except StopIteration:
            # This happens when there are no more messages left
            print("No more messages in the bag file")
            print("last time", t)
            break
        except OSError as e:
            print("OSError", e)
            print("Attempting to reload bag file 1s later")
            msg_gen = ros_bag.read_messages(start_time=t+rospy.Duration(1), end_time=end)
            continue
        
        if topic == '/alphasense_driver_ros/cam4/color_rect/image/compressed':
            msg = bridge.compressed_imgmsg_to_cv2(msg)
            # print("cam4 time",(t.to_sec()), )
            # cv2.imshow('cam4', msg)
            # cv2.waitKey(1)

        elif topic == '/alphasense_driver_ros/cam3/color_rect/image/compressed':
            msg = bridge.compressed_imgmsg_to_cv2(msg)
            print("cam3 time",(t.to_sec()), )
            cv2.imshow('cam3', msg)
            cv2.waitKey(1)

        elif topic == '/alphasense_driver_ros/cam5/color_rect/image/compressed':
            pass

        elif topic == '/compslam_lio/odometry':
            print("lio time",(t.to_sec()), )
            compslam_lio_odom.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

        elif topic == '/state_estimator/odometry':
            # print("state time",(t.to_sec()), )
            state_est_odom.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
            


    ros_bag.close()
    np.savez('./ros_anymal/ETH_Cerberus_localization/odometry.npz',
                    compslam_lio_odom = np.array(compslam_lio_odom),
                    state_est_odom = np.array(state_est_odom),
                    )
    compslam_lio_odom = np.array(compslam_lio_odom)
    state_est_odom = np.array(state_est_odom)
    plt.figure(1)
    plt.plot(compslam_lio_odom[:,0], compslam_lio_odom[:,1], '-o', label='compslam_lio_odom')
    plt.plot(state_est_odom[:,0], state_est_odom[:,1], '-o', label='state_est_odom')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()


