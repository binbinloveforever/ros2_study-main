from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    robot_description_share = get_package_share_directory("robot_description_pkg")
    balance_nav_share = get_package_share_directory("balance_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(robot_description_share, "launch", "robot_display.launch.py")
        )
    )

    nav_params = join(balance_nav_share, "config", "nav_params.yaml")
    rviz_config = join(robot_description_share, "config", "rviz2_default_config.rviz")

    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")

    nav2_launch = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(nav2_bringup_share, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "slam": slam,
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": nav_params,
                    "autostart": autostart,
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
        ]
    )

    odometry_node = Node(
        package="balance_nav",
        executable="odometry_node",
        name="balance_odometry",
        output="screen",
        parameters=[nav_params, {"use_sim_time": use_sim_time}],
    )

    cmd_vel_gate_node = Node(
        package="balance_nav",
        executable="cmd_vel_gate_node",
        name="cmd_vel_gate",
        output="screen",
        parameters=[nav_params, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    launch_description = LaunchDescription()
    launch_description.add_action(
        DeclareLaunchArgument(
            "slam",
            default_value="True",
            description="Use slam_toolbox to provide map->odom and online mapping.",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Static map yaml file to use when slam:=False.",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use Gazebo simulation clock.",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "autostart",
            default_value="True",
            description="Autostart the Nav2 lifecycle nodes.",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Launch RViz with a navigation-oriented config.",
        )
    )

    launch_description.add_action(robot_launch)
    launch_description.add_action(odometry_node)
    launch_description.add_action(cmd_vel_gate_node)
    launch_description.add_action(nav2_launch)
    launch_description.add_action(rviz_node)

    return launch_description
