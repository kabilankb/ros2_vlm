from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'param1',
            default_value='param1_default',
            description='Description of param1'),
        DeclareLaunchArgument(
            'param2',
            default_value='param2_default',
            description='Description of param2'),
    ]

    # Your model parameters 
    param1 = LaunchConfiguration('param1')
    param2 = LaunchConfiguration('param2')

    # Model node
    model_node = Node(
            package='vlm_ros2',  # Make sure your package is named correctly
            executable='ros2_vila',  # Ensure this matches your entry point
            parameters=[{
                'param1': param1, 
                'param2': param2,
            }]
    )

    final_launch_description = launch_args + [model_node]

    return LaunchDescription(final_launch_description)

