import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def launch_setup(context, *args, **kwargs):
    # External sensors (D435 RGBD, front camera, gpu_lidar). The IMU and foot
    # force sensors are not affected -- the controllers need them either way.
    sensors = context.launch_configurations['sensors'].lower() in ('true', '1', 'yes')
    # lidar_only: keep the lidar, drop the cameras and the D435. Cameras dominate
    # the cost under software rendering, and mapping only needs the lidar.
    lidar_only = context.launch_configurations['lidar_only'].lower() in ('true', '1', 'yes')
    # velodyne: swap the built-in L1 head lidar (200 deg, 10 m) for a VLP-16
    # (360 deg, 131 m). Same topic, so nothing downstream changes.
    velodyne = context.launch_configurations['velodyne'].lower() in ('true', '1', 'yes')

    # Physics engine. gz-sim does NOT read <physics type="..."> from the world --
    # that is Gazebo Classic syntax. The engine comes from --physics-engine (here)
    # or the Physics system plugin's <engine><filename>. Leave empty for the
    # default (dartsim). Useful because the pipeline terrain is a triangle mesh,
    # and mesh contact handling differs a lot between engines.
    engine = context.launch_configurations['physics_engine'].strip()
    engine_arg = f'--physics-engine gz-physics-{engine}-plugin ' if engine else ''

    # Server-side render engine. This is NOT cosmetic: the sensor pipeline
    # (gpu_lidar, cameras) initialises the render engine inside the gz server,
    # and ogre2 segfaults under this box's software GL (virgl):
    #     Loading plugin [gz-rendering-ogre2] ... Segmentation fault
    # So sensors:=true kills the server outright unless we force ogre.
    # clearpath_ws/garisani_bringup/launch/gz_world.launch.py does the same.
    render = context.launch_configurations['render_engine'].strip()
    render_arg = f'--render-engine-server {render} ' if render else ''

    # Gazebo World
    world = context.launch_configurations['world']
    # 절대경로(또는 .sdf 로 끝나는 경로)를 그대로 받는다.
    #
    # 원래는 이 패키지의 worlds/ 안에서만 찾았다. 그래서 다른 패키지의 월드를
    # 쓰려면 install 디렉토리에 **손으로 심볼릭 링크를 걸어야** 했는데, 그건
    # 어디에도 기록이 안 남는다 — 새로 받은 사람은 "Fuel world download failed"
    # 만 보고 원인을 알 수가 없다. 실제로 그렇게 막혔다.
    if world.endswith('.sdf') or os.path.isabs(world):
        default_sdf_path = world
    else:
        default_sdf_path = os.path.join(
            get_package_share_directory('gz_quadruped_playground'), 'worlds', world + '.sdf')
    print(default_sdf_path)

    # Init Height When spawn the model
    init_height = context.launch_configurations['height']
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'robot', '-allow_renaming', 'true', '-z', init_height],
    )

    # Robot Description
    pkg_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(pkg_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'GAZEBO': 'true',
        'EXTERNAL_SENSORS': 'true' if sensors else 'false',
        'LIDAR_ONLY': 'true' if lidar_only else 'false',
        'VELODYNE': 'true' if velodyne else 'false'
    }).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    # Controllers
    controller = context.launch_configurations['controller']
    if controller == 'ocs2':
        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gz_quadruped_playground'),
                                                                 'launch',
                                                                 'ocs2.launch.py'])])
        )
        rviz_config_file = os.path.join(get_package_share_directory('gz_quadruped_playground'), "config", "ocs2.rviz")
    else:
        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gz_quadruped_playground'),
                                                                 'launch',
                                                                 'unitree_guide.launch.py'])])
        )
        rviz_config_file = os.path.join(get_package_share_directory('gz_quadruped_playground'), "config", "rviz.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    # /clock is always bridged; the sensor topics only exist when the sensors do.
    # odom_gz 는 항법이 쓰는 위치다. SLAM 과 분리해야 한다 — SLAM 을 항법의
    # 위치원으로 쓰면 scan_gate 를 닫는 순간 로봇이 자기 위치를 잃는다
    # (gazebo.xacro 의 OdometryPublisher 설명 참고).
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/odom_gz@nav_msgs/msg/Odometry[gz.msgs.Odometry"]
    if sensors:
        bridge_args += [
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ]
    if sensors and not lidar_only:
        bridge_args += [
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/rgbd_d435/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            # "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            # "/odom_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
            # "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ]

    # ros_ns 를 주면 **ROS 쪽 이름만** 접두사를 붙인다. gz 쪽 이름은 그대로다.
    #
    # 지상 로봇 둘을 동시에 돌리려면 필요하다 — 안 나누면 A200 과 Go2 가 같은
    # /scan/points, /odom_gz 를 쓰게 되어 서로의 센서를 본다.
    #
    # **remap 으로 해야 한다.** 브리지 노드에 namespace 를 주면 gz 쪽 이름까지
    # 같이 바뀌어서 연결이 끊긴다. arguments 의 왼쪽 이름이 gz 토픽이기 때문이다.
    # /clock 은 전역으로 둔다 — 시뮬 전체가 하나의 시계를 쓴다.
    ros_ns = context.launch_configurations.get('ros_ns', '').strip('/')
    bridge_remaps = []
    if ros_ns:
        for a in bridge_args:
            t = a.split('@')[0]
            if t == '/clock':
                continue
            bridge_remaps.append((t, f'/{ros_ns}{t}'))

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        remappings=bridge_remaps,
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # RViz is optional. Under software rendering it is the single most
    # expensive process in this launch -- measured at 143% CPU on a 4-core box,
    # more than the gz server itself. Since this controller runs its gait on
    # wall-clock time, that cost turns straight into falls. Off by default here;
    # pass rviz:=true when you actually need it.
    show_rviz = context.launch_configurations['rviz'].lower() in ('true', '1', 'yes')

    nodes = [
        robot_state_publisher,
        gz_spawn_entity,
        gz_bridge_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 ', engine_arg, render_arg,
                                          default_sdf_path])]),
        controller_launch
    ]
    if show_rviz:
        nodes.insert(0, rviz)

    if sensors and not lidar_only:
        nodes.append(Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=[
                "/camera/image",
                '/rgbd_d435/depth_image',
                '/rgbd_d435/image',
            ],
            output="screen",
            parameters=[
                {'use_sim_time': True,
                 'camera.image.compressed.jpeg_quality': 75},
            ],
        ))

    return nodes


def generate_launch_description():
    world = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='The world to load'
    )

    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    height = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Init height in simulation'
    )

    controller = DeclareLaunchArgument(
        'controller',
        default_value='unitree_guide',
        description='The ROS2-Control Controllers'
    )

    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz. Expensive under software rendering -- it '
                    'costs more CPU than the gz server and drags RTF down.'
    )

    render_engine = DeclareLaunchArgument(
        'render_engine',
        default_value=os.environ.get('GZ_RENDER_ENGINE_SERVER', 'ogre'),
        description='Server-side render engine. ogre2 segfaults under software '
                    'GL, which takes the whole server down as soon as any '
                    'sensor is enabled. Empty to leave it to gz.'
    )

    physics_engine = DeclareLaunchArgument(
        'physics_engine',
        default_value='',
        description='dartsim | bullet-featherstone | bullet | tpe. '
                    'Empty uses the gz default (dartsim).'
    )

    velodyne = DeclareLaunchArgument(
        'velodyne',
        default_value='false',
        description='Use a Velodyne VLP-16 (360 deg, 131 m) instead of the '
                    'built-in L1 head lidar (200 deg, 10 m). Same topic.'
    )

    lidar_only = DeclareLaunchArgument(
        'lidar_only',
        default_value='false',
        description='With sensors:=true, keep only the lidar (no cameras, no '
                    'D435). Cameras dominate cost under software rendering.'
    )

    sensors = DeclareLaunchArgument(
        'sensors',
        default_value='true',
        description='Enable the external sensors (D435 RGBD, front camera, lidar)'
    )

    # 지상 로봇 둘을 동시에 돌릴 때 ROS 토픽을 나눈다. 비우면 예전과 같다.
    ros_ns = DeclareLaunchArgument(
        'ros_ns', default_value='',
        description='ROS 토픽 접두사 (예: go2). gz 쪽 이름은 안 바뀐다.'
    )

    return LaunchDescription([
        world,
        pkg_description,
        height,
        controller,
        sensors,
        ros_ns,
        lidar_only,
        velodyne,
        physics_engine,
        render_engine,
        rviz,
        OpaqueFunction(function=launch_setup),
    ])
