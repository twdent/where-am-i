import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RosData:

    def __init__(self, prod=False):
        rosbag_folder = './ros_anymal/ETH_Cerberus_localization/'
        sub_folder ='jetson/' 
        bag_name = '2023-03-27-13-48-57_anymal-cerberus-jetson_mission_5.bag'

        sub_folder = 'Processed_bag/' 
        bag_name =  'mergedBag.bag'
        

        self.camera_topics = ['/alphasense_driver_ros/cam3/color_rect/image/compressed', 
                              '/alphasense_driver_ros/cam4/color_rect/image/compressed', 
                              '/alphasense_driver_ros/cam5/color_rect/image/compressed', 
                             ]
        self.prod = prod
        if self.prod:                        
            self.bag = rosbag.Bag(rosbag_folder + sub_folder + bag_name)
        
    # def generate_imgs(self):

    #     bridge = CvBridge()
    #     topic = [
    #     # '/alphasense_driver_ros/cam4/color/camera_info',
    #     #  '/alphasense_driver_ros/cam4/color/image/compressed', 
    #     # '/alphasense_driver_ros/cam4/color_rect/camera_info', 
    #     '/alphasense_driver_ros/cam4/color_rect/image/compressed', 
    #     # '/alphasense_driver_ros/cam4/debayered/camera_info', '/alphasense_driver_ros/cam4/debayered/image/compressed', 
    #     # '/elevation_mapping/elevation_map_raw', '/elevation_mapping/semantic_map_raw', '/elevation_mapping/statistics']
    #     ]
    #     # for topic, msg, t in bag.read_messages(topics=[topic]):
    #     bag_messages = self.bag.read_messages(topics=topic)
    #     for topic, msg, t in bag_messages:
    #         #view the compressed image
    #         if t.to_sec() > 1679918009 and t.to_sec() < 1679918025:
                
    #             msg = bridge.compressed_imgmsg_to_cv2(msg)
    #             cv2.imwrite(f'./ros_anymal/ETH_Cerberus_localization/cerberus_imgs/eth_cerberus{t.to_sec()}.png', msg)

    #     self.bag.close()

    def get_ego_imgs(self,timestamp):
        #presaved_imgs
        containing_folder = './ros_anymal/ETH_Cerberus_localization/cerberus_imgs/'
        return cv2.imread(containing_folder + f'eth_cerberus{timestamp}.png')
        

    def get_candidate_position(self,timestamp):
        return np.array([-7, 2, 0.5], dtype=np.float32)
    
    def get_camera_intrinsics(self):
        if not self.prod:
            return np.array([377.1323930964233, 0.0, 704.90459865174, 
                            0.0, 376.76480664462014, 523.3273863186171, 
                            0.0, 0.0, 1.0]).reshape(3,3)
        msgs = self.bag.read_messages(topics=['/alphasense_driver_ros/cam4/color_rect/camera_info'])
        # read the K matrix from first message header
        for topic, msg, t in msgs:
            instrinsics = msg.message.K
            break
        return instrinsics
         
        
if __name__ == "__main__":
    RosData().get_camera_intrinsics()