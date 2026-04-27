import os
import shutil
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def create_nodes(context: LaunchContext):
    import shutil

    namespace = ""

    # Create mujoco directory
    mujoco_model_path = "/tmp/mujoco"
    if os.path.exists(mujoco_model_path):
        shutil.rmtree(mujoco_model_path)
    os.makedirs(mujoco_model_path, exist_ok=True)

    # ----------------------------
    # Launch arguments
    # ----------------------------
    ur_type = LaunchConfiguration("ur_type")
    rviz = LaunchConfiguration("use_rviz")
    show_gui = LaunchConfiguration("show_gui")
    use_pose_broadcaster = LaunchConfiguration("use_pose_broadcaster")
    gravity = LaunchConfiguration("gravity")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    mujoco_joint_armature = LaunchConfiguration("mujoco_joint_armature")
    mujoco_joint_damping = LaunchConfiguration("mujoco_joint_damping")
    mujoco_joint_frictionloss = LaunchConfiguration("mujoco_joint_frictionloss")

    ur_type_str = context.perform_substitution(ur_type)
    gravity_str = context.perform_substitution(gravity)
    initial_positions_file_str = context.perform_substitution(initial_positions_file)
    mujoco_joint_armature_str = context.perform_substitution(mujoco_joint_armature)
    mujoco_joint_damping_str = context.perform_substitution(mujoco_joint_damping)
    mujoco_joint_frictionloss_str = context.perform_substitution(mujoco_joint_frictionloss)

    # ----------------------------
    # Paths
    # ----------------------------
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    pkg_mujoco = get_package_share_directory("crisp_ur_mujoco")
    pkg_mujoco_ros2_control = get_package_share_directory("mujoco_ros2_control")

    ur_xacro_filepath = os.path.join(pkg_mujoco, "urdf", "ur.urdf_mujoco.xacro")
    ros2_control_params_file = os.path.join(
        pkg_mujoco, "config", "controllers_mujoco.yaml"
    )
    rviz_config_file = os.path.join(pkg_mujoco, "config", "rviz_view.rviz")
    # ----------------------------
    # Build robot_description
    # ----------------------------
    robot_description_str = xacro.process_file(
        ur_xacro_filepath,
        mappings={
            "name": "ur",
            "ur_type": ur_type_str,
            "mujoco": "true",
            "gravity": gravity_str,
            "initial_positions_file": initial_positions_file_str,
        },
    ).toprettyxml(indent="  ")

    robot_description = {"robot_description": robot_description_str}

    robot_description_controller_params = os.path.join(
        "/tmp",
        "cartesian_impedance_robot_description.yaml",
    )

    with open(robot_description_controller_params, "w") as f:
        f.write("cartesian_impedance_controller:\n")
        f.write("  ros__parameters:\n")
        f.write("    robot_description: |\n")
        for line in robot_description_str.splitlines():
            f.write(f"      {line}\n")

    # ----------------------------
    # Additional MuJoCo scene files
    # ----------------------------
    additional_files = [
        os.path.join(pkg_mujoco_ros2_control, "mjcf", "scene.xml"),
    ]

    # ----------------------------
    # Generate MJCF at launch time
    # ----------------------------
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description_str]},
            {"input_files": additional_files},
            {"output_file": mujoco_model_file},
            {"mujoco_files_path": mujoco_model_path},
        ],
        output="screen",
    )

    def patch_mjcf_model(_context: LaunchContext):
        joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        }

        def collect_mjcf_files(path: str, visited: set[str] | None = None):
            if visited is None:
                visited = set()
            path = os.path.abspath(path)
            if path in visited or not os.path.exists(path):
                return []
            visited.add(path)

            files = [path]
            root = ET.parse(path).getroot()
            base_dir = os.path.dirname(path)
            for include in root.iter("include"):
                include_file = include.get("file")
                if include_file:
                    files.extend(
                        collect_mjcf_files(os.path.join(base_dir, include_file), visited)
                    )
            return files

        patched_joints = 0
        patched_motors = 0
        patched_geoms = 0
        joint_summaries = []

        for mjcf_file in collect_mjcf_files(mujoco_model_file):
            tree = ET.parse(mjcf_file)
            root = tree.getroot()
            changed = False
            contains_ur_joints = False

            for joint in root.iter("joint"):
                if joint.get("name") in joint_names:
                    contains_ur_joints = True
                    joint.set("armature", mujoco_joint_armature_str)
                    joint.set("damping", mujoco_joint_damping_str)
                    joint.set("frictionloss", mujoco_joint_frictionloss_str)
                    patched_joints += 1
                    changed = True
                    joint_summaries.append(
                        f"{joint.get('name')}: axis={joint.get('axis')}"
                    )

            for motor in root.iter("motor"):
                if motor.get("joint") in joint_names:
                    limit = motor.get("forcerange") or motor.get("ctrlrange")
                    if limit:
                        motor.set("ctrlrange", limit)
                        motor.set("forcerange", limit)
                    motor.set("ctrllimited", "true")
                    motor.set("forcelimited", "true")
                    patched_motors += 1
                    changed = True

            if contains_ur_joints:
                for geom in root.iter("geom"):
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
                    patched_geoms += 1
                    changed = True

            if changed:
                tree.write(mjcf_file, encoding="unicode", xml_declaration=True)

        return [
            LogInfo(
                msg=(
                    "[1/4] MJCF patched with UR joint passive dynamics "
                    f"(armature={mujoco_joint_armature_str}, "
                    f"damping={mujoco_joint_damping_str}, "
                    f"frictionloss={mujoco_joint_frictionloss_str}) "
                    f"on {patched_joints} joints and {patched_motors} motors; "
                    f"disabled contacts on {patched_geoms} robot geoms."
                )
            ),
            LogInfo(msg="[1/4] Patched MJCF joints: " + ", ".join(joint_summaries)),
        ]

    # ----------------------------
    # Robot state publisher
    # ----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description],
        output="screen",
    )

    # ----------------------------
    # MuJoCo ros2_control backend
    # Pass robot_description directly
    # ----------------------------
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        parameters=[
            robot_description,  # robot_description passed here
            ros2_control_params_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": show_gui},
            {"use_sim_time": True},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # ----------------------------
    # Controllers
    # ----------------------------
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            ros2_control_params_file,
            "--controller-manager-timeout",
            "60",
            "--service-call-timeout",
            "20",
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    load_cartesian_impedance_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_cartesian_impedance_controller",
        arguments=[
            "cartesian_impedance_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
            "--param-file",
            ros2_control_params_file,
            "--param-file",
            robot_description_controller_params,
            "--controller-manager-timeout",
            "20",
            "--service-call-timeout",
            "20",
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    load_pose_broadcaster = Node(
        condition=IfCondition(use_pose_broadcaster),
        package="controller_manager",
        executable="spawner",
        name="spawner_pose_broadcaster",
        arguments=[
            "pose_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            ros2_control_params_file,
            "--controller-manager-timeout",
            "60",
            "--service-call-timeout",
            "20",
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # Optional RViz
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    # ----------------------------
    # Event handlers for proper startup sequence
    # ----------------------------
    return [
        xacro2mjcf,
        RegisterEventHandler(
            OnProcessExit(
                target_action=xacro2mjcf,
                on_exit=[
                    OpaqueFunction(function=patch_mjcf_model),
                    LogInfo(
                        msg="[1/4] MJCF generated. Starting robot_state_publisher..."
                    ),
                    robot_state_publisher,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[
                    LogInfo(
                        msg="[2/4] Robot state publisher ready. Starting mujoco_ros2_control..."
                    ),
                    mujoco,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=mujoco,
                on_start=[
                    LogInfo(
                        msg="[3/5] MuJoCo process started. Waiting for controller_manager..."
                    ),
                    TimerAction(
                        period=2.0,
                        actions=[
                            LogInfo(msg="[4/5] Starting joint_state_broadcaster..."),
                            load_joint_state_broadcaster,
                        ],
                    ),
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                    LogInfo(
                        msg="[5/6] joint_state_broadcaster finished. Waiting briefly for joint states/TF before starting cartesian_impedance_controller and RViz..."
                    ),
                    TimerAction(
                        period=2.0,
                        actions=[
                            load_cartesian_impedance_controller,
                            rviz_node,
                        ],
                    ),
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_cartesian_impedance_controller,
                on_exit=[
                    LogInfo(
                        msg="[6/6] cartesian_impedance_controller finished. Starting pose_broadcaster if enabled..."
                    ),
                    load_pose_broadcaster,
                ],
            )
        ),
    ]


def generate_launch_description():
    pkg_mujoco = get_package_share_directory("crisp_ur_mujoco")
    initial_positions_file_default = os.path.join(
        pkg_mujoco, "config", "initial_positions.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur30",
                description="UR robot type",
                choices=[
                    "ur3",
                    "ur3e",
                    "ur5",
                    "ur5e",
                    "ur10",
                    "ur10e",
                    "ur16e",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz",
            ),
            DeclareLaunchArgument(
                "show_gui",
                default_value="true",
                description="Show MuJoCo GUI",
            ),
            DeclareLaunchArgument(
                "gravity",
                default_value="0 0 0",
                description="MuJoCo gravity vector",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=initial_positions_file_default,
                description="YAML file with initial joint positions for MuJoCo",
            ),
            DeclareLaunchArgument(
                "mujoco_joint_armature",
                default_value="0.1",
                description="MuJoCo armature value applied to UR joints.",
            ),
            DeclareLaunchArgument(
                "mujoco_joint_damping",
                default_value="1.0",
                description="MuJoCo passive damping value applied to UR joints.",
            ),
            DeclareLaunchArgument(
                "mujoco_joint_frictionloss",
                default_value="0.1",
                description="MuJoCo frictionloss value applied to UR joints.",
            ),
            DeclareLaunchArgument(
                "use_pose_broadcaster",
                default_value="true",
                description="Start the custom CRISP pose_broadcaster (can hang controller_manager in MuJoCo)",
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
