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
    processes = []
    def add(package, executable, parameters=None, **kwargs):
        node = Node(package=package, executable=executable, output='screen',
                    parameters=[{'use_sim_time': False}] + (parameters or []), **kwargs)
        processes.append(node)
    processes.extend(hardware_nodes(cfg,mission,drivers=drivers))
    add('hsl_lidar_filter','real_lidar_filter', parameters=(
        [cfg['lidar_filter_file']] if cfg.get('lidar_filter_file') else []) + [{'lidar_topic':cfg['lidar_topic']}])
    add('hsl_real','real_observations', parameters=[{'odom_topic':cfg['odom_topic'],'lidar_topic':'/sensing/lidar/points_filtered','require_localization':cfg['localization'] in ('amcl','fastlio')}])
    # Облако до детектора: фильтры (свой робот, высота, дальность) в
    # base_footprint, затем сегментация земли linefit; детектор берёт облако
    # препятствий, и его собственные фильтры выключены.
    add('jr_perception','cloud_prefilter.py',parameters=[{'cloud_topic':cfg['lidar_topic']}])
    add('linefit_ground_segmentation_ros','ground_segmentation_node',name='ground_segmentation',
        parameters=[str(Path(get_package_share_directory('jr_launch'))/'config/perception/ground_segmentation.yaml')])
    add('jr_perception','robot_detector.py',parameters=[
        str(Path(get_package_share_directory('jr_perception'))/'config/real.yaml'),
        {'world':'','background_topic':'/map','world_frame':'map','base_frame':'base_footprint',
         'pose_topic':'navigation/self','cloud_topic':'/perception/obstacle_cloud',
         'self_range':0.0,'floor_z':-1.0,'floor_noise':0.0,'ceiling_z':10.0,'max_range':0.0}])
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
    add('hsl_real','real_match',parameters=[{'active_seconds':float(mission['match']['active_seconds']),'require_localization':cfg['localization'] in ('amcl','fastlio'),'role':role,'goal_center':mission['opponent']['start'][:2]}])
    if cfg['localization']=='amcl':
        localization = cfg['localization_file']
        add('pointcloud_to_laserscan','pointcloud_to_laserscan_node',name='localization_scan',
            parameters=[localization],remappings=[('cloud_in','/sensing/lidar/points_filtered'),('scan','/localization/scan')])
        x,y,yaw = mission['robot']['start']
        add('nav2_amcl','amcl',name='amcl',parameters=[localization, {
            'initial_pose.x':x,'initial_pose.y':y,'initial_pose.z':0.,'initial_pose.yaw':yaw}],
            remappings=[('scan','/localization/scan')])
        add('hsl_real','localization_monitor',parameters=[localization,{'odom_topic':cfg['odom_topic']}])
    if cfg['localization']=='fastlio':
        # FAST-LIO2 вместо AMCL: облако в CustomMsg на лету, лидар-инерциальная
        # одометрия, мост переводит её в позу base_footprint в map от старта
        # робота и публикует map -> odom (как AMCL), а с ним и готовность
        # /localization/ready вместо localization_monitor.
        launch_share = Path(get_package_share_directory('jr_launch'))
        add('jr_perception','livox_custom.py',parameters=[{'cloud_topic':cfg['lidar_topic'],'custom_topic':'/livox/lidar_custom'}])
        add('fast_lio','fastlio_mapping',name='laser_mapping',
            parameters=[str(launch_share/'config/perception/fastlio.param.yaml')])
        x,y,yaw = mission['robot']['start']
        add('jr_perception','fastlio_bridge.py',parameters=[{'x':float(x),'y':float(y),'yaw':float(yaw),
            'base_frame':'base_footprint','lidar_frame':'livox','publish_tf':True,'publish_ready':True}])
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
            '/Odometry','/localization/fastlio/odometry',
            '/navigation/self','/navigation/observation_diagnostics','/navigation/scan','/navigation/obstacle_grid','/opponent/odom','/opponent/markers','/opponent/foreground','/perception/obstacle_cloud','/navigation/opponent_visible','/navigation/detector_diagnostics',
            '/navigation/intent','/navigation/behavior','/navigation/indication','/navigation/global_path','/navigation/nav2_reference',
            '/navigation/local_path','/navigation/global_status','/navigation/mppi_diagnostics',
            '/navigation/planning_diagnostics','/navigation/native_ready','/navigation/planner_status','/navigation/mppi_cmd_vel','/navigation/native_mppi_cycle_ms',
            '/native_mppi/costmap','/native_mppi/costmap_updates','/native_mppi/costmap_raw',
            '/sensing/lidar/points_filtered','/sensing/lidar/filter_diagnostics',
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
