
import os

from launch import LaunchDescription
from launch_ros.actions import Node


cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'tb_and_livox.rviz')


def generate_launch_description():

    livox_rviz = Node(package    = 'rviz2',
                      executable = 'rviz2',
                      output     = 'screen',
                      arguments  = ['--display-config', rviz_config_path])

    return LaunchDescription([livox_rviz])
