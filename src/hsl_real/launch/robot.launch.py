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
from hsl_planning.configuration import load_planning


def nodes(context):
    cfg, mission = load_config(LaunchConfiguration('config_file').perform(context))
    drivers = LaunchConfiguration('drivers_enabled').perform(context).lower() == 'true'
    role = mission['robot']['role']
    tuning = load_planning(cfg.get('planning_file'))
    motion = mission['motion']
    reverse = role == 'explorer' and motion['allow_reverse']
    speed, angular = motion['max_speed'], motion['max_angular_speed']
    localized = cfg['localization'] in ('amcl', 'fastlio')
    processes = []
    def add(package, executable, parameters=None, **kwargs):
        node = Node(package=package, executable=executable, output='screen',
                    parameters=[{'use_sim_time': False}] + (parameters or []), **kwargs)
        processes.append(node)
    processes.extend(hardware_nodes(cfg,mission,drivers=drivers))
    add('hsl_lidar_filter','real_lidar_filter', parameters=(
        [cfg['lidar_filter_file']] if cfg.get('lidar_filter_file') else []) + [{'lidar_topic':cfg['lidar_topic']}])
    add('hsl_real','real_observations', parameters=[{'odom_topic':'/localization/kinematic_state' if cfg['localization']=='fastlio' else cfg['odom_topic'],
        'lidar_topic':'/sensing/lidar/points_filtered','require_localization':localized}])
    # Detector input: own robot body and floor (robot_body_filter, +-0.25 m so
    # a close opponent survives), then near-robot noise (dbscan_filter). The
    # cloud stays in the LiDAR frame: the detector casts rays from its origin.
    perception = Path(get_package_share_directory('jr_launch'))/'config'
    body = 0.25
    add('robot_body_filter','robot_body_filter_node',name='crop_box_filter_node',parameters=[
        str(perception/'sensing/crop.param.yaml'),{'keep_input_frame':True,
        'static_boxes.body.min_x':-body,'static_boxes.body.min_y':-body,
        'static_boxes.body.max_x':body,'static_boxes.body.max_y':body}],
        remappings=[('~/input/pointcloud','/sensing/lidar/points_filtered'),
                    ('~/output/pointcloud','/sensing/lidar/cropped/pointcloud')])
    add('dbscan_filter','dbscan_filter_node',parameters=[
        str(Path(get_package_share_directory('dbscan_filter'))/'config/dbscan.param.yaml')],
        remappings=[('~/input/pointcloud','/sensing/lidar/cropped/pointcloud'),
                    ('~/output/pointcloud','/sensing/lidar/dbscan/pointcloud')])
    add('jr_perception','robot_detector.py',parameters=[
        str(perception/'perception/robot_detector_fastlio.param.yaml'),
        {'pose_topic':'/localization/kinematic_state' if cfg['localization']=='fastlio' else '/navigation/self'}])
    add('hsl_decision','decision_manager', parameters=[{'role':role,'own_max_speed':speed,
        'own_start':start_polygon(mission['robot']),'opponent_start':start_polygon(mission['opponent'])}])
    add('hsl_planning','trajectory_planner',parameters=[tuning['global'], {'role':role,'max_speed':speed,
        'random_seed':mission['match']['seed'],'arena_bounds':cfg['arena_bounds'],'require_match_active':True}])
    native = str(Path(get_package_share_directory('hsl_nav2_control'))/'config/native_mppi.yaml')
    add('hsl_nav2_control','native_mppi',parameters=[native, tuning['local'], {'use_sim_time':False,
        'role':role,'random_seed':mission['match']['seed'], 'MPPI.vx_max':speed,
        'MPPI.vx_min':-speed if reverse else 0., 'MPPI.wz_max':angular,
        'MPPI.PathAngleCritic.forward_preference':not reverse,
        'MPPI.PreferForwardCritic.enabled':False,'MPPI.GoalAngleCritic.enabled':False,
        'MPPI.GoalCritic.cost_weight':15. if role=='guardian' else 5.,
        'costmap.plugins':['static_layer','inflation_layer']}])
    add('hsl_debug_control','motion_gate',parameters=[{'require_match_active':True}],name='hsl_motion_gate')
    add('hsl_real','real_match',parameters=[{'active_seconds':float(mission['match']['active_seconds']),'require_localization':localized,'role':role,'goal_center':mission['opponent']['start'][:2]}])
    if cfg['localization']=='amcl':
        localization = cfg['localization_file']
        add('pointcloud_to_laserscan','pointcloud_to_laserscan_node',name='localization_scan',
            parameters=[localization],remappings=[('cloud_in','/sensing/lidar/points_filtered'),('scan','/localization/scan')])
        x,y,yaw = mission['robot']['start']
        add('nav2_amcl','amcl',name='amcl',parameters=[localization, {
            'set_initial_pose':False,'initial_pose.x':x,'initial_pose.y':y,'initial_pose.z':0.,'initial_pose.yaw':yaw,
            'odom_frame_id':'odom'}],
            remappings=[('scan','/localization/scan')])
        add('hsl_real','localization_monitor',parameters=[localization,{'odom_topic':cfg['odom_topic'],
            'initial_x':x,'initial_y':y,'initial_yaw':yaw}])
    if cfg['localization']=='fastlio':
        # FAST-LIO2 alone, anchored at robot.start: the bridge publishes the
        # base pose in map as /localization/kinematic_state, TF map -> odom
        # and /localization/ready. No AMCL correction.
        x,y,yaw = mission['robot']['start']
        add('jr_perception','livox_custom.py',name='livox_custom',
            parameters=[{'cloud_topic':cfg['lidar_topic']}])
        add('fast_lio','fastlio_mapping',name='laser_mapping',parameters=[cfg['fastlio_file']])
        add('jr_perception','fastlio_bridge.py',name='fastlio_bridge',parameters=[{
            'x':float(x),'y':float(y),'yaw':float(yaw),'publish_tf':True,'publish_ready':True,
            'map_frame':'map','odometry_topic':'/localization/kinematic_state'}])
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
            '/Odometry','/localization/kinematic_state','/livox/lidar_custom',
            '/amcl_pose','/initialpose','/localization/scan','/localization/ready','/localization/status',
            '/navigation/self','/navigation/observation_diagnostics','/navigation/scan','/navigation/obstacle_grid','/opponent/odom','/opponent/markers','/navigation/opponent_visible','/navigation/detector_diagnostics',
            '/navigation/intent','/navigation/behavior','/navigation/indication','/navigation/global_path','/navigation/nav2_reference',
            '/navigation/local_path','/navigation/global_status','/navigation/mppi_diagnostics',
            '/navigation/planning_diagnostics','/navigation/native_ready','/navigation/planner_status','/navigation/mppi_cmd_vel','/navigation/native_mppi_cycle_ms',
            '/native_mppi/costmap','/native_mppi/costmap_updates','/native_mppi/costmap_raw',
            '/sensing/lidar/points_filtered','/sensing/lidar/filter_diagnostics','/sensing/lidar/dbscan/pointcloud',
            '/opponent/robot_markers','/opponent/foreground',
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
