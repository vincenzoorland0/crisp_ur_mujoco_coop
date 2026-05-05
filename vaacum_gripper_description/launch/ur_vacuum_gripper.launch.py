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


def _append_fixed_joint(robot_root, parent_link, child_link, xyz, rpy):
    joint = ET.Element("joint", {"name": "ur_flange_to_vacuum_gripper", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent_link})
    ET.SubElement(joint, "child", {"link": child_link})
    robot_root.append(joint)


def _root_link_name(robot_root):
    links = [link.get("name") for link in robot_root.findall("link")]
    children = {
        child.get("link")
        for joint in robot_root.findall("joint")
        for child in [joint.find("child")]
        if child is not None
    }
    roots = [link for link in links if link not in children]
    if not roots:
        raise RuntimeError("Vacuum gripper URDF has no root link.")
    return roots[0]


def _merge_urdf_documents(ur_xml, gripper_urdf_path, parent_link, mount_xyz, mount_rpy):
    ur_root = ET.fromstring(ur_xml)
    gripper_root = ET.parse(gripper_urdf_path).getroot()
    gripper_base_link = _root_link_name(gripper_root)

    for element in list(gripper_root):
        ur_root.append(element)

    _append_fixed_joint(
        ur_root,
        parent_link=parent_link,
        child_link=gripper_base_link,
        xyz=mount_xyz,
        rpy=mount_rpy,
    )

    return ET.tostring(ur_root, encoding="unicode"), gripper_base_link


def create_nodes(context: LaunchContext):
    namespace = ""

    mujoco_model_path = "/tmp/mujoco"
    if os.path.exists(mujoco_model_path):
        shutil.rmtree(mujoco_model_path)
    os.makedirs(mujoco_model_path, exist_ok=True)

    ur_type = LaunchConfiguration("ur_type")
    rviz = LaunchConfiguration("use_rviz")
    show_gui = LaunchConfiguration("show_gui")
    use_pose_broadcaster = LaunchConfiguration("use_pose_broadcaster")
    gravity = LaunchConfiguration("gravity")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    mujoco_joint_armature = LaunchConfiguration("mujoco_joint_armature")
    mujoco_joint_damping = LaunchConfiguration("mujoco_joint_damping")
    mujoco_joint_frictionloss = LaunchConfiguration("mujoco_joint_frictionloss")
    gripper_parent_link = LaunchConfiguration("gripper_parent_link")
    gripper_mount_xyz = LaunchConfiguration("gripper_mount_xyz")
    gripper_mount_rpy = LaunchConfiguration("gripper_mount_rpy")

    ur_type_str = context.perform_substitution(ur_type)
    gravity_str = context.perform_substitution(gravity)
    initial_positions_file_str = context.perform_substitution(initial_positions_file)
    mujoco_joint_armature_str = context.perform_substitution(mujoco_joint_armature)
    mujoco_joint_damping_str = context.perform_substitution(mujoco_joint_damping)
    mujoco_joint_frictionloss_str = context.perform_substitution(mujoco_joint_frictionloss)
    gripper_parent_link_str = context.perform_substitution(gripper_parent_link)
    gripper_mount_xyz_str = context.perform_substitution(gripper_mount_xyz)
    gripper_mount_rpy_str = context.perform_substitution(gripper_mount_rpy)

    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    pkg_mujoco = get_package_share_directory("crisp_ur_mujoco")
    pkg_mujoco_ros2_control = get_package_share_directory("mujoco_ros2_control")
    pkg_gripper = get_package_share_directory("vaacum_gripper_description")

    ur_xacro_filepath = os.path.join(pkg_mujoco, "urdf", "ur.urdf_mujoco.xacro")
    gripper_urdf_filepath = os.path.join(
        pkg_gripper, "vacuum_gripper", "vacuum_gripper.urdf"
    )
    ros2_control_params_file = os.path.join(
        pkg_mujoco, "config", "controllers_mujoco.yaml"
    )
    rviz_config_file = os.path.join(pkg_mujoco, "config", "rviz_view.rviz")

    ur_description_str = xacro.process_file(
        ur_xacro_filepath,
        mappings={
            "name": "ur_vacuum_gripper",
            "ur_type": ur_type_str,
            "mujoco": "true",
            "gravity": gravity_str,
            "initial_positions_file": initial_positions_file_str,
        },
    ).toprettyxml(indent="  ")

    robot_description_str, gripper_base_link = _merge_urdf_documents(
        ur_description_str,
        gripper_urdf_filepath,
        parent_link=gripper_parent_link_str,
        mount_xyz=gripper_mount_xyz_str,
        mount_rpy=gripper_mount_rpy_str,
    )
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

    additional_files = [
        os.path.join(pkg_mujoco_ros2_control, "mjcf", "scene.xml"),
    ]

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
                    f"disabled contacts on {patched_geoms} UR geoms."
                )
            ),
            LogInfo(msg="[1/4] Patched MJCF joints: " + ", ".join(joint_summaries)),
        ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description],
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        parameters=[
            robot_description,
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

    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    return [
        LogInfo(
            msg=(
                "Attaching vacuum gripper root link "
                f"'{gripper_base_link}' to UR link '{gripper_parent_link_str}' "
                f"with xyz='{gripper_mount_xyz_str}' rpy='{gripper_mount_rpy_str}'."
            )
        ),
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
                default_value="true",
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
                description="Start the custom CRISP pose_broadcaster.",
            ),
            DeclareLaunchArgument(
                "gripper_parent_link",
                default_value="flange",
                description="UR link used as the fixed parent of the gripper.",
            ),
            DeclareLaunchArgument(
                "gripper_mount_xyz",
                default_value="0 0 0",
                description="Fixed transform from UR flange to gripper root link.",
            ),
            DeclareLaunchArgument(
                "gripper_mount_rpy",
                default_value="0 0 0",
                description="Fixed transform rotation from UR flange to gripper root link.",
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
