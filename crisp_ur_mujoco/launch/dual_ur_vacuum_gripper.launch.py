import copy
import math
import os
import shutil
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


UR_JOINT_BASENAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_SPRING_JOINTS = ("spring_flexlink", "spring_flexlink2")
GRIPPER_NON_COLLIDING_LINKS = ("spring", "spring_2")


def _read_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _quat_y(angle):
    return f"{math.cos(angle / 2.0)} 0 {math.sin(angle / 2.0)} 0"


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


def _prefix_gripper_element(element, prefix):
    cloned = copy.deepcopy(element)

    for node in cloned.iter():
        if node.tag in ("link", "joint", "material") and node.get("name"):
            node.set("name", prefix + node.get("name"))
        if node.tag in ("parent", "child") and node.get("link"):
            node.set("link", prefix + node.get("link"))
        if node.tag == "mimic" and node.get("joint"):
            node.set("joint", prefix + node.get("joint"))

    return cloned


def _append_fixed_joint(robot_root, name, parent_link, child_link, xyz, rpy):
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent_link})
    ET.SubElement(joint, "child", {"link": child_link})
    robot_root.append(joint)


def _base_poses(prefixes, layout):
    distance = float(layout["distance"])
    height = float(layout["height"])
    base_angle = float(layout["base_connection_axis_rotation"])
    return {
        prefixes[0]: (f"0 {-distance / 2.0} {height}", f"0 {base_angle} 0"),
        prefixes[1]: (f"0 {distance / 2.0} {height}", f"0 {base_angle} 0"),
    }


def _apply_urdf_base_pose(robot_root, prefix, xyz, rpy):
    base_link = prefix + "base_link"
    for joint in robot_root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is not None
            and child is not None
            and parent.get("link") == "world"
            and child.get("link") == base_link
        ):
            origin = joint.find("origin")
            if origin is None:
                origin = ET.SubElement(joint, "origin")
            origin.set("xyz", xyz)
            origin.set("rpy", rpy)
            return
    raise RuntimeError(f"Could not find fixed world joint for {base_link}.")


def _append_gripper_state_interfaces(robot_root, prefix):
    ros2_control = robot_root.find("ros2_control")
    if ros2_control is None:
        raise RuntimeError("Combined URDF has no ros2_control block.")

    existing_joints = {
        joint.get("name")
        for joint in ros2_control.findall("joint")
        if joint.get("name") is not None
    }
    for joint_name in GRIPPER_SPRING_JOINTS:
        prefixed_joint = prefix + joint_name
        if prefixed_joint in existing_joints:
            continue
        joint = ET.SubElement(ros2_control, "joint", {"name": prefixed_joint})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})


def _attach_prefixed_gripper(robot_root, gripper_path, prefix, parent_link, mount_xyz, mount_rpy):
    gripper_root = ET.parse(gripper_path).getroot()
    gripper_base_link = _root_link_name(gripper_root)

    for element in list(gripper_root):
        robot_root.append(_prefix_gripper_element(element, prefix))

    _append_fixed_joint(
        robot_root,
        name=prefix + "ur_flange_to_vacuum_gripper",
        parent_link=parent_link,
        child_link=prefix + gripper_base_link,
        xyz=mount_xyz,
        rpy=mount_rpy,
    )
    _append_gripper_state_interfaces(robot_root, prefix)


def _merge_mujoco_blocks(target_root, source_root):
    target_mujoco = target_root.find("mujoco")
    source_mujoco = source_root.find("mujoco")
    if target_mujoco is None or source_mujoco is None:
        return

    target_actuator = target_mujoco.find("actuator")
    for child in list(source_mujoco):
        if child.tag in ("compiler", "option"):
            continue
        if child.tag == "actuator" and target_actuator is not None:
            for actuator_child in list(child):
                target_actuator.append(copy.deepcopy(actuator_child))
        else:
            target_mujoco.append(copy.deepcopy(child))


def _combine_robot_descriptions(robot_roots):
    combined = robot_roots[0]

    for source_root in robot_roots[1:]:
        _merge_mujoco_blocks(combined, source_root)
        for element in list(source_root):
            if element.tag == "link" and element.get("name") == "world":
                continue
            if element.tag == "mujoco":
                continue
            combined.append(copy.deepcopy(element))

    combined.set("name", "dual_ur_vacuum_gripper")
    return ET.tostring(combined, encoding="unicode")


def _append_camera(parent, name, pos, xyaxes, fovy):
    if any(camera.get("name") == name for camera in parent.findall("camera")):
        return False
    ET.SubElement(
        parent,
        "camera",
        {"name": name, "mode": "fixed", "pos": pos, "xyaxes": xyaxes, "fovy": fovy},
    )
    return True


def _append_cameras(root, prefix, add_external):
    added = 0
    for body in root.iter("body"):
        if body.get("name") == prefix + "tool0":
            added += int(
                _append_camera(
                    body,
                    prefix + "wrist_camera",
                    pos="0.08 0 0.06",
                    xyaxes="1 0 0 0 -1 0",
                    fovy="75",
                )
            )
            break

    if add_external:
        worldbody = root.find("worldbody")
        if worldbody is not None:
            body = ET.SubElement(
                worldbody,
                "body",
                {"name": "external_camera_frame", "pos": "1.2 -1.4 0.9"},
            )
            added += int(
                _append_camera(
                    body,
                    "external_camera",
                    pos="0 0 0",
                    xyaxes="1 0 0 0 0 1",
                    fovy="45",
                )
            )
    return added


def _append_flange_ft_sensor(root, prefix):
    flange_body = None
    for body in root.iter("body"):
        if body.get("name") == prefix + "flange":
            flange_body = body
            break
    if flange_body is None:
        return False, 0

    site_name = prefix + "flange_ft_site"
    if not any(site.get("name") == site_name for site in flange_body.findall("site")):
        ET.SubElement(
            flange_body,
            "site",
            {"name": site_name, "pos": "0 0 0", "size": "0.01", "rgba": "0.1 0.8 0.1 0.6"},
        )

    sensor_root = root.find("sensor")
    if sensor_root is None:
        sensor_root = ET.SubElement(root, "sensor")

    added_sensors = 0
    existing_names = {sensor.get("name") for sensor in sensor_root if sensor.get("name")}
    for tag, suffix in (("force", "force"), ("torque", "torque")):
        sensor_name = prefix + "flange_ft_site_" + suffix
        if sensor_name not in existing_names:
            ET.SubElement(sensor_root, tag, {"name": sensor_name, "site": site_name})
            added_sensors += 1
    return True, added_sensors


def _patch_mjcf(root, prefixes, layout, mujoco_joint_armature, mujoco_joint_damping,
                mujoco_joint_frictionloss, gripper_spring_stiffness, gripper_spring_damping,
                gripper_spring_frictionloss, gripper_springref, add_external_camera):
    patched = {"joints": 0, "motors": 0, "spring_geoms": 0, "gravcomp": 0, "sensors": 0, "cameras": 0}
    base_poses = _base_poses(prefixes, layout)
    base_angle = float(layout["base_connection_axis_rotation"])
    base_quat = _quat_y(base_angle)

    for prefix in prefixes:
        ur_joint_names = {prefix + name for name in UR_JOINT_BASENAMES}
        gripper_spring_names = {prefix + name for name in GRIPPER_SPRING_JOINTS}

        for body in root.iter("body"):
            body_name = body.get("name", "")
            if body_name == prefix + "base_link":
                body.set("pos", base_poses[prefix][0])
                body.set("quat", base_quat)
            if body_name in {
                prefix + "base",
                prefix + "base_link",
                prefix + "base_link_inertia",
                prefix + "shoulder_link",
                prefix + "upper_arm_link",
                prefix + "forearm_link",
                prefix + "wrist_1_link",
                prefix + "wrist_2_link",
                prefix + "wrist_3_link",
                prefix + "flange",
                prefix + "tool0",
            }:
                body.set("gravcomp", "1")
                patched["gravcomp"] += 1

            if body_name in {prefix + name for name in GRIPPER_NON_COLLIDING_LINKS}:
                for geom in body.findall("geom"):
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
                    patched["spring_geoms"] += 1

        for joint in root.iter("joint"):
            joint_name = joint.get("name")
            if joint_name in ur_joint_names:
                joint.set("armature", mujoco_joint_armature)
                joint.set("damping", mujoco_joint_damping)
                joint.set("frictionloss", mujoco_joint_frictionloss)
                patched["joints"] += 1
            elif joint_name in gripper_spring_names:
                joint.set("stiffness", gripper_spring_stiffness)
                joint.set("damping", gripper_spring_damping)
                joint.set("frictionloss", gripper_spring_frictionloss)
                joint.set("springref", gripper_springref)
                patched["joints"] += 1

        for motor in root.iter("motor"):
            if motor.get("joint") in ur_joint_names:
                limit = motor.get("forcerange") or motor.get("ctrlrange")
                if limit:
                    motor.set("ctrlrange", limit)
                    motor.set("forcerange", limit)
                motor.set("ctrllimited", "true")
                motor.set("forcelimited", "true")
                patched["motors"] += 1

        _, sensor_count = _append_flange_ft_sensor(root, prefix)
        patched["sensors"] += sensor_count

    for idx, prefix in enumerate(prefixes):
        patched["cameras"] += _append_cameras(
            root, prefix, add_external=(idx == 0 and add_external_camera)
        )

    return patched


def _controller_block(prefix, robot_description_str):
    joints = [prefix + name for name in UR_JOINT_BASENAMES]
    ns = prefix.rstrip("_")
    controller_name = ns + "_cartesian_impedance_controller"
    ft_name = ns + "_force_torque_sensor_broadcaster"

    return {
        controller_name: {
            "ros__parameters": {
                "joints": joints,
                "end_effector_frame": prefix + "tool0",
                "base_frame": prefix + "base_link",
                "filter": {"q": 0.5, "dq": 0.5, "output_torque": 0.5, "target_pose": 0.1},
                "task": {
                    "k_pos_x": 10000.0,
                    "k_pos_y": 10000.0,
                    "k_pos_z": 10000.0,
                    "k_rot_x": 500.0,
                    "k_rot_y": 500.0,
                    "k_rot_z": 500.0,
                },
                "nullspace": {"stiffness": 0.0, "damping": 0.0, "max_tau": 2.0},
                "use_friction": False,
                "use_coriolis_compensation": True,
                "use_local_jacobian": True,
                "limit_torques": False,
                "robot_description": robot_description_str,
                "target_pose_topic": f"{ns}/target_pose",
                "target_joint_topic": f"{ns}/target_joint",
                "target_wrench_topic": f"{ns}/target_wrench",
            }
        },
        ft_name: {
            "ros__parameters": {
                "sensor_name": prefix + "flange_ft_sensor",
                "state_interface_names": ["force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"],
                "frame_id": prefix + "flange",
                "topic_name": "wrench",
            }
        },
    }


def _write_controller_yaml(path, prefixes, robot_description_str):
    controller_types = {"joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"}}
    for prefix in prefixes:
        ns = prefix.rstrip("_")
        controller_types[f"{ns}_cartesian_impedance_controller"] = {"type": "crisp_controllers/CartesianController"}
        controller_types[f"{ns}_force_torque_sensor_broadcaster"] = {"type": "force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster"}

    data = {
        "controller_manager": {"ros__parameters": {"update_rate": 500, "enforce_command_limits": False, **controller_types}},
        "joint_state_broadcaster": {"ros__parameters": {"use_local_topics": False}},
    }
    for prefix in prefixes:
        data.update(_controller_block(prefix, robot_description_str))

    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def _collect_mjcf_files(path, visited=None):
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
            files.extend(_collect_mjcf_files(os.path.join(base_dir, include_file), visited))
    return files


def create_nodes(context: LaunchContext):
    mujoco_model_path = "/tmp/mujoco"
    if os.path.exists(mujoco_model_path):
        shutil.rmtree(mujoco_model_path)
    os.makedirs(mujoco_model_path, exist_ok=True)

    ur_type = context.perform_substitution(LaunchConfiguration("ur_type"))
    rviz = LaunchConfiguration("use_rviz")
    show_gui = LaunchConfiguration("show_gui")
    gravity = context.perform_substitution(LaunchConfiguration("gravity"))
    layout_file = context.perform_substitution(LaunchConfiguration("layout_file"))
    initial_positions_file = context.perform_substitution(LaunchConfiguration("initial_positions_file"))
    gripper_mount_xyz = context.perform_substitution(LaunchConfiguration("gripper_mount_xyz"))
    gripper_mount_rpy = context.perform_substitution(LaunchConfiguration("gripper_mount_rpy"))

    mujoco_joint_armature = context.perform_substitution(LaunchConfiguration("mujoco_joint_armature"))
    mujoco_joint_damping = context.perform_substitution(LaunchConfiguration("mujoco_joint_damping"))
    mujoco_joint_frictionloss = context.perform_substitution(LaunchConfiguration("mujoco_joint_frictionloss"))
    gripper_spring_stiffness = context.perform_substitution(LaunchConfiguration("gripper_spring_stiffness"))
    gripper_spring_damping = context.perform_substitution(LaunchConfiguration("gripper_spring_damping"))
    gripper_spring_frictionloss = context.perform_substitution(LaunchConfiguration("gripper_spring_frictionloss"))
    gripper_springref = context.perform_substitution(LaunchConfiguration("gripper_springref"))

    layout_config = _read_yaml(layout_file)
    prefixes = [
        layout_config["robots"]["left"].get("prefix", "left_"),
        layout_config["robots"]["right"].get("prefix", "right_"),
    ]
    layout = layout_config.get("layout", {})
    base_poses = _base_poses(prefixes, layout)

    pkg_mujoco = get_package_share_directory("crisp_ur_mujoco")
    pkg_mujoco_ros2_control = get_package_share_directory("mujoco_ros2_control")
    pkg_gripper = get_package_share_directory("vaacum_gripper_description")

    ur_xacro = os.path.join(pkg_mujoco, "urdf", "ur.urdf_mujoco.xacro")
    gripper_urdf = os.path.join(pkg_gripper, "vacuum_gripper", "vacuum_gripper.urdf")
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")
    controllers_yaml = os.path.join("/tmp", "dual_ur_vacuum_gripper_controllers.yaml")
    rviz_config_file = os.path.join(pkg_mujoco, "config", "rviz_view.rviz")

    robot_roots = []
    for prefix in prefixes:
        ur_xml = xacro.process_file(
            ur_xacro,
            mappings={
                "name": prefix.rstrip("_") + "_ur",
                "tf_prefix": prefix,
                "ur_type": ur_type,
                "mujoco": "true",
                "gravity": gravity,
                "initial_positions_file": initial_positions_file,
            },
        ).toprettyxml(indent="  ")
        root = ET.fromstring(ur_xml)
        _apply_urdf_base_pose(root, prefix, *base_poses[prefix])
        _attach_prefixed_gripper(
            root,
            gripper_urdf,
            prefix=prefix,
            parent_link=prefix + "flange",
            mount_xyz=gripper_mount_xyz,
            mount_rpy=gripper_mount_rpy,
        )
        robot_roots.append(root)

    robot_description_str = _combine_robot_descriptions(robot_roots)
    robot_description = {"robot_description": robot_description_str}
    _write_controller_yaml(controllers_yaml, prefixes, robot_description_str)

    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description_str]},
            {"input_files": [os.path.join(pkg_mujoco_ros2_control, "mjcf", "scene.xml")]},
            {"output_file": mujoco_model_file},
            {"mujoco_files_path": mujoco_model_path},
        ],
        output="screen",
    )

    def patch_mjcf_model(_context):
        patched_total = {"joints": 0, "motors": 0, "spring_geoms": 0, "gravcomp": 0, "sensors": 0, "cameras": 0}
        external_camera_added = False

        for mjcf_file in _collect_mjcf_files(mujoco_model_file):
            tree = ET.parse(mjcf_file)
            root = tree.getroot()
            patched = _patch_mjcf(
                root, prefixes, layout, mujoco_joint_armature, mujoco_joint_damping,
                mujoco_joint_frictionloss, gripper_spring_stiffness, gripper_spring_damping,
                gripper_spring_frictionloss, gripper_springref,
                add_external_camera=not external_camera_added,
            )
            external_camera_added = external_camera_added or any(
                camera.get("name") == "external_camera" for camera in root.iter("camera")
            )
            for key, value in patched.items():
                patched_total[key] += value
            if any(patched.values()):
                tree.write(mjcf_file, encoding="unicode", xml_declaration=True)
        return [LogInfo(msg=f"[dual] MJCF patched: {patched_total}")]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers_yaml,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": show_gui},
            {"use_sim_time": True},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    controller_loaders = [
        ExecuteProcess(
            cmd=[
                "ros2",
                "control",
                "load_controller",
                "--set-state",
                "active",
                "-c",
                "/controller_manager",
                "joint_state_broadcaster",
                controllers_yaml,
            ],
            output="screen",
        )
    ]
    for prefix in prefixes:
        ns = prefix.rstrip("_")
        controller_loaders.extend(
            [
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "control",
                        "load_controller",
                        "--set-state",
                        "active",
                        "-c",
                        "/controller_manager",
                        f"{ns}_force_torque_sensor_broadcaster",
                        controllers_yaml,
                    ],
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "control",
                        "load_controller",
                        "--set-state",
                        "active",
                        "-c",
                        "/controller_manager",
                        f"{ns}_cartesian_impedance_controller",
                        controllers_yaml,
                    ],
                    output="screen",
                ),
            ]
        )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return [
        LogInfo(msg=f"[dual] prefixes={prefixes}, layout_file={layout_file}"),
        xacro2mjcf,
        RegisterEventHandler(
            OnProcessExit(
                target_action=xacro2mjcf,
                on_exit=[OpaqueFunction(function=patch_mjcf_model), robot_state_publisher],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=robot_state_publisher, on_start=[mujoco])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=mujoco, on_start=controller_loaders + [rviz_node])
        ),
    ]


def generate_launch_description():
    pkg_mujoco = get_package_share_directory("crisp_ur_mujoco")
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur30"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("show_gui", default_value="true"),
            DeclareLaunchArgument("gravity", default_value="0 0 -9.81"),
            DeclareLaunchArgument(
                "layout_file",
                default_value=os.path.join(pkg_mujoco, "config", "dual_ur_vacuum_gripper.yaml"),
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=os.path.join(pkg_mujoco, "config", "initial_positions_ur_vacuum_gripper.yaml"),
            ),
            DeclareLaunchArgument("gripper_mount_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("gripper_mount_rpy", default_value="0 -1.5708 0"),
            DeclareLaunchArgument("mujoco_joint_armature", default_value="0.1"),
            DeclareLaunchArgument("mujoco_joint_damping", default_value="60.0"),
            DeclareLaunchArgument("mujoco_joint_frictionloss", default_value="20.0"),
            DeclareLaunchArgument("gripper_spring_stiffness", default_value="1070.0"),
            DeclareLaunchArgument("gripper_spring_damping", default_value="20.0"),
            DeclareLaunchArgument("gripper_spring_frictionloss", default_value="0.0"),
            DeclareLaunchArgument("gripper_springref", default_value="0.0"),
            OpaqueFunction(function=create_nodes),
        ]
    )
