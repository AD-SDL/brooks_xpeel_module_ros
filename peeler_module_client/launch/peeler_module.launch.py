from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    peeler_port = LaunchConfiguration("peeler_port")

    
    declare_use_peeler_port_cmd = DeclareLaunchArgument(
        name = "peeler_port",
        default_value= "/dev/ttyUSB1",
        description= "Flag to accept peeler port"
        )

    peeler=Node(
        package='peeler_module_client',
        namespace = 'std_ns',
        executable='peeler_client',
        name='PeelerNode',
        parameters = [{"peeler_port":peeler_port}],
        emulate_tty=True

    )


    ld.add_action(declare_use_peeler_port_cmd)
    ld.add_action(peeler)

    return ld
