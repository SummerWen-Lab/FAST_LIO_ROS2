import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取 FAST-LIO 配置
    fast_lio_config = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'avia.yaml'
    )
    
    # 获取数据包路径
    bag_path = os.path.expanduser('bags/databag01_ros2/07-05-10-37/07-05-10-37_0.db3')
    
    # 获取 QoS 覆写文件路径 
    qos_config_path = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'qos_overrides.yaml'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('fast_lio'), 
        'rviz',
        'fastlio.rviz' 
    )

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) time if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        # 1. 启动 FAST-LIO
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            parameters=[
                fast_lio_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        
        # 2. 启动数据播放器（数据包）
            ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', 
                bag_path,
                '--clock',  # 提供仿真时间
                '--qos-profile-overrides-path', qos_config_path  # 解决QoS兼容性问题
            ],
            output='screen'
        ),

        # 3. 启动 RVIZ2 节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file], 
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])