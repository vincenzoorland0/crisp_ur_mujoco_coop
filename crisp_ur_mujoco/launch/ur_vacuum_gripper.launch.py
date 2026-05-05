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
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


GRIPPER_SPRING_JOINTS = ("spring_flexlink", "spring_flexlink2")
GRIPPER_NON_COLLIDING_LINKS = ("spring", "spring_2")
FLANGE_FT_BODY_NAME = "flange"
FLANGE_FT_SITE_NAME = "flange_ft_site"
WRIST_CAMERA_BODY_NAME = "tool0"
WRIST_CAMERA_NAME = "wrist_camera"
EXTERNAL_CAMERA_BODY_NAME = "external_camera_frame"
EXTERNAL_CAMERA_NAME = "external_camera"
UR_GRAVCOMP_BODY_NAMES = {
    "base",
    "base_link",
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "flange",
    "tool0",
}


def _append_flange_ft_sensor(root):
    flange_body = None
    for body in root.iter("body"):
        if body.get("name") == FLANGE_FT_BODY_NAME:
            flange_body = body
            break

    if flange_body is None:
        return False, 0

    if not any(site.get("name") == FLANGE_FT_SITE_NAME for site in flange_body.findall("site")):
        ET.SubElement(
            flange_body,
            "site",
            {
                "name": FLANGE_FT_SITE_NAME,
                "pos": "0 0 0",
                "size": "0.01",
                "rgba": "0.1 0.8 0.1 0.6",
            },
        )

    sensor_root = root.find("sensor")
    if sensor_root is None:
        sensor_root = ET.SubElement(root, "sensor")

    added_sensors = 0
    existing_names = {
        sensor.get("name") for sensor in sensor_root if sensor.get("name") is not None
    }
    if "flange_ft_site_force" not in existing_names:
        ET.SubElement(
            sensor_root,
            "force",
            {"name": "flange_ft_site_force", "site": FLANGE_FT_SITE_NAME},
        )
        added_sensors += 1
    if "flange_ft_site_torque" not in existing_names:
        ET.SubElement(
            sensor_root,
            "torque",
            {"name": "flange_ft_site_torque", "site": FLANGE_FT_SITE_NAME},
        )
        added_sensors += 1

    return True, added_sensors


def _apply_ur_gravity_compensation(root, joint_names):
    patched_bodies = 0

    for body in root.iter("body"):
        body_name = body.get("name", "")
        body_has_ur_joint = any(
            joint.get("name") in joint_names for joint in body.findall("joint")
        )

        if body_name in UR_GRAVCOMP_BODY_NAMES or body_has_ur_joint:
            body.set("gravcomp", "1")
            patched_bodies += 1

    return patched_bodies


def _disable_direct_body_geoms(root, body_names):
    patched_geoms = 0
    body_names = set(body_names)

    for body in root.iter("body"):
        if body.get("name") not in body_names:
            continue

        for geom in body.findall("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            patched_geoms += 1

    return patched_geoms


def _append_camera(parent, name, pos, xyaxes, fovy):
    for camera in parent.findall("camera"):
        if camera.get("name") == name:
            return False

    ET.SubElement(
        parent,
        "camera",
        {
            "name": name,
            "mode": "fixed",
            "pos": pos,
            "xyaxes": xyaxes,
            "fovy": fovy,
        },
    )
    return True


def _append_mujoco_cameras(
    root,
    wrist_camera_pos,
    wrist_camera_xyaxes,
    wrist_camera_fovy,
    include_external_camera=True,
):
    added_cameras = 0

    wrist_body = None
    for body in root.iter("body"):
        if body.get("name") == WRIST_CAMERA_BODY_NAME:
            wrist_body = body
            break

    if wrist_body is not None:
        if _append_camera(
            wrist_body,
            WRIST_CAMERA_NAME,
            pos=wrist_camera_pos,
            xyaxes=wrist_camera_xyaxes,
            fovy=wrist_camera_fovy,
        ):
            added_cameras += 1

    worldbody = root.find("worldbody") if include_external_camera else None
    if worldbody is not None:
        external_body = None
        for body in worldbody.findall("body"):
            if body.get("name") == EXTERNAL_CAMERA_BODY_NAME:
                external_body = body
                break

        if external_body is None:
            external_body = ET.SubElement(
                worldbody,
                "body",
                {
                    "name": EXTERNAL_CAMERA_BODY_NAME,
                    "pos": "1.2 -1.4 0.9",
                },
            )

        if _append_camera(
            external_body,
            EXTERNAL_CAMERA_NAME,
            pos="0 0 0",
            xyaxes="1 0 0 0 0 1",
            fovy="45",
        ):
            added_cameras += 1

    return added_cameras


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


def _append_gripper_state_interfaces(robot_root):
    ros2_control = robot_root.find("ros2_control")
    if ros2_control is None:
        raise RuntimeError("Combined URDF has no ros2_control block.")

    existing_joints = {
        joint.get("name")
        for joint in ros2_control.findall("joint")
        if joint.get("name") is not None
    }

    for joint_name in GRIPPER_SPRING_JOINTS:
        if joint_name in existing_joints:
            continue
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})


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

    _append_gripper_state_interfaces(ur_root)

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
    gripper_spring_stiffness = LaunchConfiguration("gripper_spring_stiffness")
    gripper_spring_damping = LaunchConfiguration("gripper_spring_damping")
    gripper_spring_frictionloss = LaunchConfiguration("gripper_spring_frictionloss")
    gripper_springref = LaunchConfiguration("gripper_springref")
    wrist_camera_pos = LaunchConfiguration("wrist_camera_pos")
    wrist_camera_xyaxes = LaunchConfiguration("wrist_camera_xyaxes")
    wrist_camera_fovy = LaunchConfiguration("wrist_camera_fovy")

    ur_type_str = context.perform_substitution(ur_type)
    gravity_str = context.perform_substitution(gravity)
    initial_positions_file_str = context.perform_substitution(initial_positions_file)
    mujoco_joint_armature_str = context.perform_substitution(mujoco_joint_armature)
    mujoco_joint_damping_str = context.perform_substitution(mujoco_joint_damping)
    mujoco_joint_frictionloss_str = context.perform_substitution(mujoco_joint_frictionloss)
    gripper_parent_link_str = context.perform_substitution(gripper_parent_link)
    gripper_mount_xyz_str = context.perform_substitution(gripper_mount_xyz)
    gripper_mount_rpy_str = context.perform_substitution(gripper_mount_rpy)
    gripper_spring_stiffness_str = context.perform_substitution(
        gripper_spring_stiffness
    )
    gripper_spring_damping_str = context.perform_substitution(gripper_spring_damping)
    gripper_spring_frictionloss_str = context.perform_substitution(
        gripper_spring_frictionloss
    )
    gripper_springref_str = context.perform_substitution(gripper_springref)
    wrist_camera_pos_str = context.perform_substitution(wrist_camera_pos)
    wrist_camera_xyaxes_str = context.perform_substitution(wrist_camera_xyaxes)
    wrist_camera_fovy_str = context.perform_substitution(wrist_camera_fovy)

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
        ur_joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        }
        gripper_spring_joint_names = set(GRIPPER_SPRING_JOINTS)

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
        patched_ft_site = False
        patched_ft_sensors = 0
        patched_gravcomp_bodies = 0
        patched_gripper_spring_geoms = 0
        patched_cameras = 0
        external_camera_added = False
        joint_summaries = []

        for mjcf_file in collect_mjcf_files(mujoco_model_file):
            tree = ET.parse(mjcf_file)
            root = tree.getroot()
            changed = False
            contains_ur_joints = False

            added_cameras = _append_mujoco_cameras(
                root,
                wrist_camera_pos=wrist_camera_pos_str,
                wrist_camera_xyaxes=wrist_camera_xyaxes_str,
                wrist_camera_fovy=wrist_camera_fovy_str,
                include_external_camera=not external_camera_added,
            )
            if added_cameras:
                patched_cameras += added_cameras
                external_camera_added = any(
                    camera.get("name") == EXTERNAL_CAMERA_NAME
                    for camera in root.iter("camera")
                )
                changed = True

            ft_site_added, ft_sensors_added = _append_flange_ft_sensor(root)
            if ft_site_added:
                patched_ft_site = True
                patched_ft_sensors += ft_sensors_added
                changed = True

            gravcomp_bodies = _apply_ur_gravity_compensation(root, ur_joint_names)
            if gravcomp_bodies:
                patched_gravcomp_bodies += gravcomp_bodies
                changed = True

            spring_geoms = _disable_direct_body_geoms(
                root, GRIPPER_NON_COLLIDING_LINKS
            )
            if spring_geoms:
                patched_gripper_spring_geoms += spring_geoms
                changed = True

            for joint in root.iter("joint"):
                if joint.get("name") in ur_joint_names:
                    contains_ur_joints = True
                    joint.set("armature", mujoco_joint_armature_str)
                    joint.set("damping", mujoco_joint_damping_str)
                    joint.set("frictionloss", mujoco_joint_frictionloss_str)
                    patched_joints += 1
                    changed = True
                    joint_summaries.append(
                        f"{joint.get('name')}: axis={joint.get('axis')}"
                    )
                elif joint.get("name") in gripper_spring_joint_names:
                    joint.set("stiffness", gripper_spring_stiffness_str)
                    joint.set("damping", gripper_spring_damping_str)
                    joint.set("frictionloss", gripper_spring_frictionloss_str)
                    joint.set("springref", gripper_springref_str)
                    patched_joints += 1
                    changed = True
                    joint_summaries.append(
                        f"{joint.get('name')}: passive spring axis={joint.get('axis')}"
                    )

            for motor in root.iter("motor"):
                if motor.get("joint") in ur_joint_names:
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
                    f"disabled contacts on {patched_geoms} UR geoms; "
                    f"flange F/T site={'added' if patched_ft_site else 'not found'}, "
                    f"MuJoCo sensors added={patched_ft_sensors}; "
                    f"UR gravcomp bodies={patched_gravcomp_bodies}; "
                    f"disabled spring collision geoms={patched_gripper_spring_geoms}; "
                    f"cameras added={patched_cameras}."
                )
            ),
            LogInfo(
                msg=(
                    "[1/4] Passive gripper springs "
                    f"({', '.join(GRIPPER_SPRING_JOINTS)}) configured with "
                    f"stiffness={gripper_spring_stiffness_str}, "
                    f"damping={gripper_spring_damping_str}, "
                    f"frictionloss={gripper_spring_frictionloss_str}, "
                    f"springref={gripper_springref_str}."
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

    load_force_torque_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_force_torque_sensor_broadcaster",
        arguments=[
            "force_torque_sensor_broadcaster",
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
                        msg="[3/5] MuJoCo process started. Starting broadcasters and active cartesian_impedance_controller hold..."
                    ),
                    load_joint_state_broadcaster,
                    load_force_torque_sensor_broadcaster,
                    load_cartesian_impedance_controller,
                    rviz_node,
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
        pkg_mujoco, "config", "initial_positions_ur_vacuum_gripper.yaml"
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
                default_value="0 0 -9.81",
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
                default_value="60.0",
                description="MuJoCo passive damping value applied to UR joints.",
            ),
            DeclareLaunchArgument(
                "mujoco_joint_frictionloss",
                default_value="20.0",
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
                default_value="0 -1.5708 0",
                description="Fixed transform rotation from UR flange to gripper root link.",
            ),
            DeclareLaunchArgument(
                "gripper_spring_stiffness",
                default_value="1070.0",
                description="Passive MuJoCo stiffness for the gripper prismatic spring joints.",
            ),
            DeclareLaunchArgument(
                "gripper_spring_damping",
                default_value="20.0",
                description="Passive MuJoCo damping for the gripper prismatic spring joints.",
            ),
            DeclareLaunchArgument(
                "gripper_spring_frictionloss",
                default_value="0.0",
                description="Passive MuJoCo frictionloss for the gripper prismatic spring joints.",
            ),
            DeclareLaunchArgument(
                "gripper_springref",
                default_value="0.0",
                description="Passive MuJoCo spring reference position for the gripper prismatic joints.",
            ),
            DeclareLaunchArgument(
                "wrist_camera_pos",
                default_value="0.08 0 0.06",
                description="Position of the wrist camera in the tool0 frame.",
            ),
            DeclareLaunchArgument(
                "wrist_camera_xyaxes",
                default_value="1 0 0 0 -1 0",
                description="MuJoCo xyaxes orientation of the wrist camera in the tool0 frame.",
            ),
            DeclareLaunchArgument(
                "wrist_camera_fovy",
                default_value="75",
                description="Vertical field of view of the wrist camera in degrees.",
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
