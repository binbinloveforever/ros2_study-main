import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from os.path import join


def generate_launch_description():
    robot_name_in_model = "balance_car"
    package_share = get_package_share_directory("robot_description_pkg")

    default_model_path = join(package_share, "urdf", "balance_car.urdf.xacro")

    declare_model = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to the robot xacro file.",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments=[("verbose", "true"), ("pause", "true")],
    )

    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", robot_name_in_model],
    )

    load_joint_state_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_joint_state_broadcaster"],
    )

    load_lqr_effort_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lqr_effort_controller"],
    )

    unpause_gazebo = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_model,
            robot_state_publisher,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_lqr_effort_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_lqr_effort_controller,
                    on_exit=[unpause_gazebo],
                )
            ),
        ]
    )
