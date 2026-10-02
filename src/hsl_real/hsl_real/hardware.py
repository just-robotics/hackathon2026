"""Hardware actions shared by autonomous and keyboard recording modes."""
from pathlib import Path
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def hardware_nodes(cfg, mission, drivers=True, map_anchor=True):
    processes = []
    def add(package, executable, parameters=None, **kwargs):
        processes.append(Node(package=package, executable=executable, output='screen',
            parameters=[{'use_sim_time':False}] + (parameters or []), **kwargs))
    def static(parent, child, pose):
        x,y,z,roll,pitch,yaw = pose
        add('tf2_ros','static_transform_publisher',arguments=[
            '--x',str(x),'--y',str(y),'--z',str(z),'--roll',str(roll),
            '--pitch',str(pitch),'--yaw',str(yaw),'--frame-id',parent,'--child-frame-id',child])
    if drivers:
        add('kobuki_node','kobuki_ros_node', parameters=[{
            'device_port':cfg['kobuki_port'], 'odom_frame':'odom', 'base_frame':'base_footprint',
            'publish_tf':True, 'use_imu_heading':True, 'acceleration_limiter':True,
            'cmd_vel_timeout_sec':.6}], remappings=[('commands/velocity','/cmd_vel'),('odom',cfg['odom_topic'])])
        add('livox_ros_driver2','livox_ros_driver2_node', name='livox_lidar_publisher',
            parameters=[{'xfer_format':0, 'multi_topic':0, 'data_src':0, 'publish_freq':10.,
                         'output_data_type':0, 'frame_id':'livox', 'user_config_path':cfg['livox_config'],
                         'lvx_file_path':'', 'cmdline_input_bd_code':'livox0000000001'}],
            remappings=[('/livox/lidar',cfg['lidar_topic'])])
    description = Path(get_package_share_directory('kobuki_description'))/'urdf/kobuki_standalone.urdf.xacro'
    add('robot_state_publisher','robot_state_publisher', parameters=[{'robot_description':Command(['xacro ',str(description)])}],
        remappings=[('joint_states','/joint_states')])
    static('base_link','livox',cfg['lidar_mount'])
    if map_anchor and cfg['localization'] == 'odometry':
        x,y,yaw = mission['robot']['start']
        static('map','odom',[x,y,0.,0.,0.,yaw])
    return processes
