"""Drivers and the existing autonomous stack on wall time; initially stopped."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from hsl_real.config import load_config, start_polygon
from hsl_real.hardware import hardware_nodes


def nodes(context):
    cfg, mission = load_config(LaunchConfiguration('config_file').perform(context))
    drivers = LaunchConfiguration('drivers_enabled').perform(context).lower() == 'true'
    role = mission['robot']['role']
    motion = mission['motion']
    reverse = role == 'explorer' and motion['allow_reverse']
    speed, angular = motion['max_speed'], motion['max_angular_speed']
    processes = []
    def add(package, executable, parameters=None, **kwargs):
        node = Node(package=package, executable=executable, output='screen',
                    parameters=[{'use_sim_time': False}] + (parameters or []), **kwargs)
        processes.append(node)
    processes.extend(hardware_nodes(cfg,mission,drivers=drivers))
    add('hsl_real','real_observations', parameters=[{'odom_topic':cfg['odom_topic'],'lidar_topic':cfg['lidar_topic'],'require_localization':cfg['localization']=='amcl'}])
    add('hsl_perception','opponent_detector', parameters=[{'opponent_max_height':float(mission['perception']['opponent_max_height'])}])
    add('hsl_decision','decision_manager', parameters=[{'role':role,'own_max_speed':speed,
        'own_start':start_polygon(mission['robot']),'opponent_start':start_polygon(mission['opponent'])}])
    add('hsl_planning','trajectory_planner',parameters=[{'role':role,'max_speed':speed,
        'random_seed':mission['match']['seed'],'arena_bounds':cfg['arena_bounds'],'require_match_active':True}])
    native = str(Path(get_package_share_directory('hsl_nav2_control'))/'config/native_mppi.yaml')
    add('hsl_nav2_control','native_mppi',parameters=[native, {'use_sim_time':False,
        'role':role,'random_seed':mission['match']['seed'], 'MPPI.vx_max':speed,
        'MPPI.vx_min':-speed if reverse else 0., 'MPPI.wz_max':angular,
        'MPPI.PathAngleCritic.forward_preference':not reverse,
        'MPPI.PreferForwardCritic.enabled':False,'MPPI.GoalAngleCritic.enabled':False,
        'MPPI.GoalCritic.cost_weight':15. if role=='guardian' else 5.,
        'costmap.plugins':['static_layer','obstacle_layer','inflation_layer'] if cfg['map_file'] else ['obstacle_layer','inflation_layer']}])
    add('hsl_debug_control','motion_gate',parameters=[{'require_match_active':True}],name='hsl_motion_gate')
    add('hsl_real','real_match',parameters=[{'active_seconds':float(mission['match']['active_seconds']),'require_localization':cfg['localization']=='amcl','role':role,'goal_center':mission['opponent']['start'][:2]}])
    if cfg['localization']=='amcl':
        localization = cfg['localization_file']
        add('pointcloud_to_laserscan','pointcloud_to_laserscan_node',name='localization_scan',
            parameters=[localization],remappings=[('cloud_in',cfg['lidar_topic']),('scan','/localization/scan')])
        x,y,yaw = mission['robot']['start']
        add('nav2_amcl','amcl',name='amcl',parameters=[localization, {
            'initial_pose.x':x,'initial_pose.y':y,'initial_pose.z':0.,'initial_pose.yaw':yaw}],
            remappings=[('scan','/localization/scan')])
        add('hsl_real','localization_monitor',parameters=[localization,{'odom_topic':cfg['odom_topic']}])
    if cfg['map_file']:
        add('nav2_map_server','map_server', name='map_server', parameters=[{'yaml_filename':cfg['map_file']}])
        add('nav2_lifecycle_manager','lifecycle_manager', name='map_lifecycle_manager',parameters=[{
            'autostart':True,'node_names':['map_server','amcl'] if cfg['localization']=='amcl' else ['map_server']}])
    if cfg['rviz']:
        rviz = str(Path(get_package_share_directory('hsl_real'))/'config/robot.rviz')
        # RViz is optional; closing it must not terminate the robot stack.
        viewer = Node(package='rviz2',executable='rviz2',arguments=['-d',rviz],parameters=[{'use_sim_time':False}])
    session = LaunchConfiguration('session_dir').perform(context)
    if session:
        directory=Path(session).resolve()
        if not directory.is_relative_to('/records'):raise ValueError('session_dir must be under /records')
        directory.mkdir(parents=True,exist_ok=True)
        topics=[cfg['odom_topic'],cfg['lidar_topic'],'/livox/imu','/tf','/tf_static','/map',
            '/amcl_pose','/initialpose','/localization/scan','/localization/ready','/localization/status',
            '/navigation/self','/navigation/opponent','/navigation/opponent_visible',
            '/navigation/intent','/navigation/behavior','/navigation/indication','/navigation/global_path','/navigation/nav2_reference',
            '/navigation/local_path','/navigation/global_status','/navigation/mppi_diagnostics',
            '/navigation/planning_diagnostics','/navigation/native_ready','/navigation/planner_status','/navigation/mppi_cmd_vel','/navigation/native_mppi_cycle_ms',
            '/native_mppi/costmap','/native_mppi/costmap_updates','/native_mppi/costmap_raw',
            '/match/allowed','/match/active','/real/match_finished','/cmd_vel','/diagnostics']
        processes.append(ExecuteProcess(cmd=['ros2','bag','record','-s','mcap','-o',str(directory/'bag')]+topics,
            output='screen',sigterm_timeout='60',sigkill_timeout='10'))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=p,
        on_exit=[EmitEvent(event=Shutdown(reason='Real robot process exited'))])) for p in processes]
    return processes + handlers + ([viewer] if cfg['rviz'] else [])


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file',default_value='/config/real.yaml'),
        DeclareLaunchArgument('drivers_enabled',default_value='true'),
        DeclareLaunchArgument('session_dir',default_value=''),OpaqueFunction(function=nodes)])
