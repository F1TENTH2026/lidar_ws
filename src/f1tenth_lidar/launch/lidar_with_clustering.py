"""
F1Tenth LiDAR 통합 Launch
===========================
사용 예시:
  # Ethernet (기본)
  ros2 launch f1tenth_lidar lidar_with_clustering.py

  # Serial
  ros2 launch f1tenth_lidar lidar_with_clustering.py sensor_interface:=serial

  # 클러스터링 파라미터 오버라이드
  ros2 launch f1tenth_lidar lidar_with_clustering.py \
      cluster_tolerance:=0.2 min_cluster_size:=5 max_range:=8.0

  # RViz2 없이
  ros2 launch f1tenth_lidar lidar_with_clustering.py rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('f1tenth_lidar')

    urdf_path        = os.path.join(pkg, 'urdf',   'hokuyo_standalone.urdf')
    rviz_config      = os.path.join(pkg, 'config', 'lidar_clustering.rviz')
    clustering_yaml  = os.path.join(pkg, 'config', 'clustering_params.yaml')

    # ── Launch Arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('sensor_interface', default_value='ethernet',
            description='LiDAR 연결 방식: ethernet | serial'),
        DeclareLaunchArgument('rviz', default_value='true',
            description='RViz2 실행 여부: true | false'),

        # 클러스터링 파라미터
        DeclareLaunchArgument('scan_topic',            default_value='/scan'),
        DeclareLaunchArgument('clusters_marker_topic', default_value='/lidar_clusters'),
        DeclareLaunchArgument('min_range',             default_value='0.05',
            description='유효 최소 거리 (m)'),
        DeclareLaunchArgument('max_range',             default_value='10.0',
            description='유효 최대 거리 (m)'),
        DeclareLaunchArgument('cluster_tolerance',     default_value='0.15',
            description='DBSCAN eps: 이웃 반경 (m)'),
        DeclareLaunchArgument('min_cluster_size',      default_value='3',
            description='클러스터 최소 포인트 수'),
        DeclareLaunchArgument('max_cluster_size',      default_value='500',
            description='클러스터 최대 포인트 수'),
        DeclareLaunchArgument('marker_lifetime',       default_value='0.2'),
        DeclareLaunchArgument('show_bbox',             default_value='true'),
        DeclareLaunchArgument('show_centroid',         default_value='true'),
        DeclareLaunchArgument('show_id_text',          default_value='true'),
    ]

    # ── LiDAR YAML 동적 선택 ────────────────────────────────────────────────
    def pick_lidar_yaml(context):
        interface = context.launch_configurations['sensor_interface']
        yaml_file = os.path.join(pkg, 'config', f'lidar_{interface}.yaml')
        if not os.path.exists(yaml_file):
            raise RuntimeError(
                f"파라미터 파일 없음: {yaml_file}\n"
                f"sensor_interface는 'ethernet' 또는 'serial' 이어야 합니다.")
        return [SetLaunchConfiguration('lidar_yaml', yaml_file)]

    # ── 노드들 ─────────────────────────────────────────────────────────────

    # 1. robot_state_publisher — Hokuyo STL 3D 모델 TF 브로드캐스트
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. lidar_driver — Hokuyo URG/UST 드라이버
    lidar_driver = Node(
        package='f1tenth_lidar',
        executable='urg_node',
        name='urg_node',
        output='screen',
        parameters=[LaunchConfiguration('lidar_yaml')]
    )

    # 3. clustering_node — DBSCAN 클러스터링
    clustering = Node(
        package='f1tenth_lidar',
        executable='lidar_clustering',
        name='lidar_clustering',
        output='screen',
        emulate_tty=True,
        parameters=[
            clustering_yaml,
            {
                'scan_topic':            LaunchConfiguration('scan_topic'),
                'clusters_marker_topic': LaunchConfiguration('clusters_marker_topic'),
                'min_range':             LaunchConfiguration('min_range'),
                'max_range':             LaunchConfiguration('max_range'),
                'cluster_tolerance':     LaunchConfiguration('cluster_tolerance'),
                'min_cluster_size':      LaunchConfiguration('min_cluster_size'),
                'max_cluster_size':      LaunchConfiguration('max_cluster_size'),
                'marker_lifetime':       LaunchConfiguration('marker_lifetime'),
                'show_bbox':             LaunchConfiguration('show_bbox'),
                'show_centroid':         LaunchConfiguration('show_centroid'),
                'show_id_text':          LaunchConfiguration('show_id_text'),
            }
        ]
    )

    # 4. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription(args)
    ld.add_action(OpaqueFunction(function=pick_lidar_yaml))
    ld.add_action(robot_state_pub)
    ld.add_action(lidar_driver)
    ld.add_action(clustering)
    ld.add_action(rviz)
    return ld
