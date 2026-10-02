"""Hardware, keyboard gate and raw bag, without autonomous control."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from hsl_real.config import load_config,load_recording
from hsl_real.hardware import hardware_nodes


def nodes(context):
    path=Path(LaunchConfiguration('config_file').perform(context))
    cfg,mission=load_config(path)
    rec=load_recording(path.parent/'recording.yaml')
    directory=Path(LaunchConfiguration('session_dir').perform(context)).resolve()
    if not directory.is_relative_to('/records'):raise ValueError('session_dir must be under /records')
    directory.mkdir(parents=True,exist_ok=True)
    drivers=LaunchConfiguration('drivers_enabled').perform(context).lower()=='true'
    processes=hardware_nodes(cfg,mission,drivers=drivers,map_anchor=True)
    processes.append(Node(package='hsl_real',executable='real_manual_gate',parameters=[{
        'use_sim_time':False,'timeout_s':rec['keyboard_timeout_s'],
        'max_speed':mission['motion']['max_speed'],'max_angular_speed':mission['motion']['max_angular_speed']}]))
    topics=list(rec['bag_topics'])
    for topic in (cfg['odom_topic'],cfg['lidar_topic']):
        if topic not in topics:topics.append(topic)
    processes.append(ExecuteProcess(cmd=['ros2','bag','record','-s','mcap','-o',str(directory/'bag')]+topics,
        output='screen',sigterm_timeout='60',sigkill_timeout='10'))
    handlers=[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Recording process exited'))])) for p in processes]
    return processes+handlers


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file',default_value='/config/real.yaml'),
        DeclareLaunchArgument('session_dir',default_value='/records/session'),
        DeclareLaunchArgument('drivers_enabled',default_value='true'),
        OpaqueFunction(function=nodes)])
