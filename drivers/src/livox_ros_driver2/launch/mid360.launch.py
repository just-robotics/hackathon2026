
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PI = 3.141592653589793

xfer_format     = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic     = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src        = 0    # 0-lidar, others-Invalid data src
publish_freq    = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type     = 0
livox_frame_id  = 'livox'
lvx_file_path   = '/workspace/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'


cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_tb_config.json')


livox_ros2_params = [ {"xfer_format": xfer_format},
                      {"multi_topic": multi_topic},
                      {"data_src": data_src},
                      {"publish_freq": publish_freq},
                      {"output_data_type": output_type},
                      {"frame_id": livox_frame_id},
                      {"lvx_file_path": lvx_file_path},
                      {"user_config_path": user_config_path},
                      {"cmdline_input_bd_code": cmdline_bd_code} ]


def generate_launch_description():

    livox_driver = Node(package    = 'livox_ros_driver2',
                        executable = 'livox_ros_driver2_node',
                        name       = 'livox_lidar_publisher',
                        output     = 'screen',
                        parameters = livox_ros2_params)

    tf_static_pub = Node(package    = "tf2_ros",
                         executable = "static_transform_publisher",
                         arguments  = ["0", "0", "0.3", str(PI/2), str(PI), "0", "base_link", livox_frame_id])

    return LaunchDescription([

        tf_static_pub,
        livox_driver
    ])
