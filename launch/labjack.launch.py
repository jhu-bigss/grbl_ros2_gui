import launch
from launch.substitutions import LaunchConfiguration
import launch_ros


def generate_launch_description():
    labjack_stream_node = launch_ros.actions.Node(
        package='grbl_ros2_gui',
        executable='labjack_stream_device',
    )
    labjack_process_data_node = launch_ros.actions.Node(
        package='grbl_ros2_gui',
        executable='labjack_process_data',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('raw'))
    )
    labjack_pointcloud_publisher_node = launch_ros.actions.Node(
        package='grbl_ros2_gui',
        executable='labjack_pointcould2_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('raw'))
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='raw', default_value='False',
                                             description='Flag to stream LabJack raw data without processing'),
        labjack_stream_node,
        labjack_process_data_node,
        labjack_pointcloud_publisher_node
    ])
