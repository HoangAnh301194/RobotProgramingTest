"""Gazebo Classic + official MoveIt Panda + physical ros2_control.

The upstream Panda xacro is loaded from moveit_resources_panda_description.
Only a controller/plugin overlay is added to the expanded XML in memory.
No Panda kinematic or geometry URDF is maintained by this package.
"""
import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def add_tag(parent, tag, value=None, **attrs):
    item = ET.SubElement(parent, tag, attrs)
    if value is not None:
        item.text = value
    return item


def make_robot_description(controllers_path):
    panda_dir = get_package_share_directory(
    "moveit_resources_panda_description"
    )

    panda_xacro = os.path.join(
        panda_dir, "urdf", "panda.urdf.xacro"
    )

    panda_urdf = os.path.join(
        panda_dir, "urdf", "panda.urdf"
    )

    if os.path.isfile(panda_xacro):
        print("[PANDA] Using XACRO:", panda_xacro)
        root = ET.fromstring(
            xacro.process_file(panda_xacro).toxml()
        )

    elif os.path.isfile(panda_urdf):
        print("[PANDA] Using URDF:", panda_urdf)
        root = ET.parse(panda_urdf).getroot()

    else:
        raise FileNotFoundError(
            f"No Panda URDF or XACRO found in {panda_dir}/urdf"
        )

    # Gazebo must constrain the base to the inertial world.
    add_tag(root, "link", name="world")
    fixed = add_tag(root, "joint", name="world_to_panda", type="fixed")
    add_tag(fixed, "parent", link="world")
    add_tag(fixed, "child", link="panda_link0")

    control = add_tag(root, "ros2_control", name="PandaGazeboSystem", type="system")
    hardware = add_tag(control, "hardware")
    add_tag(hardware, "plugin", "gazebo_ros2_control/GazeboSystem")
    # Initial Panda configuration: MoveIt SRDF "ready"
    initial_positions = {
        "panda_joint1": 0.0,
        "panda_joint2": -0.785,
        "panda_joint3": 0.0,
        "panda_joint4": -2.356,
        "panda_joint5": 0.0,
        "panda_joint6": 1.571,
        "panda_joint7": 0.785,
        "panda_finger_joint1": 0.035,
    }

    for joint_name in [f"panda_joint{i}" for i in range(1, 8)] + ["panda_finger_joint1"]:
        joint = add_tag(control, "joint", name=joint_name)
        add_tag(joint, "command_interface", name="position")
        position_state = add_tag(
            joint, "state_interface", name="position"
        )
        add_tag(
            position_state,
            "param",
            str(initial_positions[joint_name]),
            name="initial_value",
        )
        add_tag(joint, "state_interface", name="velocity")
    # The second gripper finger is mechanically coupled to the first.
    finger2 = add_tag(control, "joint", name="panda_finger_joint2")
    add_tag(finger2, "state_interface", name="position")
    add_tag(finger2, "state_interface", name="velocity")
    # Explicit mimic fields for gazebo_ros2_control.
    add_tag(finger2, "param", "panda_finger_joint1", name="mimic")
    add_tag(finger2, "param", "1.0", name="multiplier")

    gazebo = add_tag(root, "gazebo")
    plugin = add_tag(gazebo, "plugin", name="gazebo_ros2_control",
                     filename="libgazebo_ros2_control.so")
    add_tag(plugin, "robot_param", "robot_description")
    add_tag(plugin, "robot_param_node", "robot_state_publisher")
    add_tag(plugin, "parameters", controllers_path)
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    pkg_dir = get_package_share_directory("panda_pick_place")
    config_dir = get_package_share_directory("moveit_resources_panda_moveit_config")
    gazebo_dir = get_package_share_directory("gazebo_ros")
    controllers_path = os.path.join(pkg_dir, "config", "controllers.yaml")
    world_path = os.path.join(pkg_dir, "worlds", "panda_pick_place.world")
    robot_description = make_robot_description(controllers_path)
    use_sim_time = {"use_sim_time": True}

    # Reuse the vendor-provided SRDF, kinematics, limits, and OMPL config.
    upstream = (
        MoveItConfigsBuilder(
            "moveit_resources_panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    moveit_params = upstream.to_dict()
    moveit_params["robot_description"] = robot_description
    # SRDF upstream defines a floating virtual joint rooted at panda_link0;
    # Gazebo adds a fixed world_to_panda root, so the virtual joint may be
    # ignored by MoveIt. The actual physical base remains fixed.

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "false"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[use_sim_time, {"robot_description": robot_description}],
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_panda",
        output="screen",
        arguments=["-entity", "panda", "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0"],
        parameters=[use_sim_time],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, use_sim_time],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(config_dir, "launch", "moveit.rviz")],
        parameters=[
            use_sim_time,
            {"robot_description": robot_description},
            upstream.robot_description_semantic,
            upstream.robot_description_kinematics,
            upstream.planning_pipelines,
            upstream.joint_limits,
        ],
    )

    def controller(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager",
                       "--controller-manager-timeout", "120"],
            output="screen",
        )

    task = Node(
        package="panda_pick_place",
        executable="pick_place_node",
        output="screen",
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("run_task")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "run_task", default_value="false",
            description="Start pick/place automatically (after the controllers)."
        ),
        gazebo,
        rsp,
        spawn_robot,
        move_group,
        rviz,
        TimerAction(period=12.0, actions=[
            controller("joint_state_broadcaster"),
            controller("panda_arm_controller"),
            controller("panda_hand_controller"),
        ]),
        TimerAction(period=35.0, actions=[task]),
    ])
