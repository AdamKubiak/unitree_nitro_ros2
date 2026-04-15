"""
nav2.launch.py — One-command autonomous navigation for Unitree Go1

Brings up the complete Nav2 stack:
  [1] Go1 Robot Base
      - robot_state_publisher  → /robot_description, /tf_static (URDF joints)
      - udp_high               → /high_state pub, /cmd_vel sub  → Go1 UDP
      - jsp_high               → /joint_states
      - odom_publisher         → /odom, odom→base_link TF

  [2] Slamtec Mapper Bridge
      - /scan  (LaserScan)
      - /map   (OccupancyGrid, transient_local)
      - /mapper/pose (PoseStamped)
      - map→odom TF  ← critical for Nav2 TF chain

  [3] Nav2 Navigation Stack  (no AMCL — slamware handles localisation)
      - controller_server   → /cmd_vel  (Regulated Pure Pursuit)
      - planner_server      (NavFn A*)
      - behavior_server     (spin / backup / wait recoveries)
      - bt_navigator        → /navigate_to_pose action server
      - waypoint_follower   → /navigate_through_poses action server
      - lifecycle_manager_navigation

Full TF chain at runtime:
  map ──(slamware_bridge)──► odom ──(odom_publisher)──► base_link

Usage:
  ros2 launch unitree_legged_real nav2.launch.py
  ros2 launch unitree_legged_real nav2.launch.py mapper_ip:=192.168.11.1
  ros2 launch unitree_legged_real nav2.launch.py use_rviz:=true log_level:=debug

Send a goal from the command line:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}}}"

Or use RViz2 → "Nav2 Goal" button (2D Nav Goal tool).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('unitree_legged_real'), 'config', 'nav2_params.yaml'
    ])

    return LaunchDescription([

        # ----------------------------------------------------------------
        # Arguments
        # ----------------------------------------------------------------
        DeclareLaunchArgument(
            'mapper_ip',
            default_value='192.168.11.1',
            description='IP address of the Slamtec Mapper M2M2'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Launch RViz2 for visualisation'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (always false for real robot)'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start Nav2 lifecycle nodes on launch'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 node logging level (debug|info|warn|error)'
        ),

        # ----------------------------------------------------------------
        # [1]  Go1 Robot Base
        #      Provides: /robot_description, /joint_states, /high_state,
        #                /odom, odom→base_link TF, base_link→* static TFs
        # ----------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('unitree_legged_real'), 'launch', 'high.launch.py'
                ])
            ]),
            launch_arguments={
                'use_rviz':    LaunchConfiguration('use_rviz'),
                # Keep map as fixed frame so RViz costmaps render correctly
                'fixed_frame': 'map',
            }.items(),
        ),

        # ----------------------------------------------------------------
        # [2]  Slamtec Mapper Bridge
        #      Provides: /scan, /map, /mapper/pose, map→odom TF
        #      use_internal_nav is no longer a parameter — internal nav has been removed.
        # ----------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slamware_bridge'), 'launch', 'slamware_bridge.launch.py'
                ])
            ]),
            launch_arguments={
                'mapper_ip': LaunchConfiguration('mapper_ip'),
            }.items(),
        ),

        # ----------------------------------------------------------------
        # [3]  Nav2 Navigation Stack
        #      navigation_launch.py starts (no AMCL, no map_server):
        #        controller_server, planner_server, behavior_server,
        #        bt_navigator, waypoint_follower, lifecycle_manager_navigation
        #
        #      map_subscribe_transient_local=True matches the QoS that
        #      slamware_bridge uses when publishing /map.
        # ----------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time':                  LaunchConfiguration('use_sim_time'),
                'autostart':                     LaunchConfiguration('autostart'),
                'params_file':                   nav2_params_file,
                'use_composition':               'False',
                'map_subscribe_transient_local': 'True',
                'log_level':                     LaunchConfiguration('log_level'),
            }.items(),
        ),

    ])
