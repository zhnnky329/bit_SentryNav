# Copyright 2026 bw_nav contributors
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("arguments", [
    {},
    {"config": "/maps/custom.yaml"},
    {"odom_frame": '""'},
    {"config": "/maps/custom.yaml", "automatic": "false",
     "body_frame": "imu_link", "odom_frame": "", "surface_map": "/maps/with space.pcd"},
])
def test_launch_only_overrides_explicit_parameters(monkeypatch, tmp_path, arguments):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "roslog"))
    module = load_module("relocalizer_launch", PACKAGE_ROOT /
                         "launch/relocalizer.launch.py")
    monkeypatch.setattr(module, "get_package_share_directory",
                        lambda _: str(PACKAGE_ROOT))
    node_factory = Mock()
    monkeypatch.setattr(module, "Node", node_factory)
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for action in module.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, OpaqueFunction)):
            action.execute(context)
    parameter_file, overrides = node_factory.call_args.kwargs["parameters"]
    assert parameter_file.perform(context) == arguments.get(
        "config", str(PACKAGE_ROOT / "config/relocalizer.yaml"))
    evaluated = {name: value.evaluate(context) for name, value in overrides.items()}
    expected = {name: value for name, value in arguments.items() if name != "config"}
    if "automatic" in expected:
        expected["automatic"] = False
    if expected.get("odom_frame") == '""':
        expected["odom_frame"] = ""
    assert evaluated == expected


def test_validation_reports_missing_inputs(monkeypatch, tmp_path, capsys):
    module = load_module("run_validation", PACKAGE_ROOT / "tools/run_validation.py")
    prefix = tmp_path / "unrelated_install/relocalizer"
    monkeypatch.setattr(module, "get_package_prefix", lambda _: str(prefix))
    with pytest.raises(SystemExit) as exception:
        module.main(["--recordings", str(tmp_path), "--plan", str(tmp_path / "map.yaml"),
                     "--surface", str(tmp_path / "map.pcd")])
    assert exception.value.code == 2
    error = capsys.readouterr().err
    assert "missing input files" in error
    assert str(prefix / "lib/relocalizer/relocalizer_probe") in error
    assert str(tmp_path / "map.yaml") in error
    assert "x:0_y:0_yaw:90.npz" in error


@pytest.mark.parametrize("load_relocalizer_setup", [True, False])
def test_pipeline_uses_selected_jazzy_environment(tmp_path, load_relocalizer_setup):
    # Exercise the whole startup/cleanup sequence with local ROS command doubles.
    # No driver, ROS node or hardware is started.
    commands = tmp_path / "commands.jsonl"
    ros2 = tmp_path / "ros2"
    ros2.write_text("""#!/usr/bin/env python3
import json, os, sys, time
args = sys.argv[1:]
with open(os.environ['PIPELINE_TEST_COMMANDS'], 'a') as stream:
    stream.write(json.dumps(args) + '\\n')
if args[:2] == ['pkg', 'prefix']:
    print(os.environ['PIPELINE_TEST_PREFIX'])
elif args[:2] == ['topic', 'list']:
    print('/livox/lidar\\n/livox/imu')
elif args[:2] == ['topic', 'info']:
    print('Type: sensor_msgs/msg/PointCloud2\\nPublisher count: 1')
elif args[:2] == ['topic', 'echo']:
    pass
elif args[:2] != ['launch', 'relocalizer']:
    time.sleep(30)
""")
    ros2.chmod(0o755)
    setup = tmp_path / "setup.zsh"
    setup.write_text("export ROS_DISTRO=jazzy\n")
    plan = tmp_path / "map.yaml"
    surface = tmp_path / "map.pcd"
    fastlio = tmp_path / "fastlio.yaml"
    filter_config = tmp_path / "filter.yaml"
    for path in (plan, surface, fastlio, filter_config):
        path.touch()
    env = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "ROS_DISTRO": "jazzy", "ROS_SETUP": str(setup),
        "RELOCALIZATION_SETUP": str(setup) if load_relocalizer_setup else "",
        "NAV_WS": "", "NAV_SETUP": "",
        "PIPELINE_TEST_COMMANDS": str(commands), "PIPELINE_TEST_PREFIX": str(tmp_path),
    }
    result = subprocess.run([
        "zsh", str(PACKAGE_ROOT / "tools/start_pipeline.zsh"),
        "--nav-setup", str(setup), "--plan", str(plan), "--surface", str(surface),
        "--fastlio-params", str(fastlio), "--filter-params", str(filter_config),
        "--livox-launch", "msg_MID360_launch.py", "--sweep-topic", "/test_cloud",
        "--body-frame", "imu_link", "--world-frame", "site", "--odom-frame", "odom",
        "--base-transform", "0.1", "0.2", "-0.3", "0", "0", "0.4",
        "--no-rviz", "--log-dir", str(tmp_path / "logs"),
    ], env=env, text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    recorded = [json.loads(line) for line in commands.read_text().splitlines()]
    assert ["launch", "livox_ros_driver2", "msg_MID360_launch.py"] in recorded
    fastlio_call = next(call for call in recorded if call[:2] == ["run", "fast_lio"])
    assert str(fastlio) in fastlio_call
    assert "/cloud_registered_body:=/test_cloud" in fastlio_call
    filter_call = next(call for call in recorded if call[:2] == ["run", "bw_fastlio_nav2"])
    assert str(filter_config) in filter_call
    assert "input_topic:=/test_cloud" in filter_call
    base_call = next(call for call in recorded if call[:2] == ["run", "tf2_ros"])
    assert base_call[base_call.index("--frame-id") + 1] == "imu_link"
    assert base_call[base_call.index("--yaw") + 1] == "0.4"
    launch_call = next(call for call in recorded if call[:2] == ["launch", "relocalizer"])
    assert "odom_frame:=odom" in launch_call
    assert "body_frame:=imu_link" in launch_call
    assert "world_frame:=site" in launch_call
    assert "rviz:=false" in launch_call


def test_pipeline_rejects_conflicting_tf_before_startup():
    result = subprocess.run([
        "zsh", str(PACKAGE_ROOT / "tools/start_pipeline.zsh"), "--odom-frame", "",
    ], env={**os.environ, "ROS_DISTRO": "jazzy"}, text=True, capture_output=True)
    assert result.returncode != 0
    assert "distinct, non-empty --odom-frame" in result.stderr


def test_pipeline_rejects_mixed_underlays(tmp_path):
    setup = tmp_path / "jazzy.zsh"
    setup.write_text("export ROS_DISTRO=jazzy\n")
    upstream = tmp_path / "old_overlay.zsh"
    upstream.write_text("export AMENT_PREFIX_PATH=/opt/ros/humble\n")
    plan = tmp_path / "map.yaml"
    surface = tmp_path / "map.pcd"
    plan.touch()
    surface.touch()
    result = subprocess.run([
        "zsh", str(PACKAGE_ROOT / "tools/start_pipeline.zsh"),
        "--nav-setup", str(upstream), "--plan", str(plan), "--surface", str(surface),
        "--log-dir", str(tmp_path / "logs"),
    ], env={**os.environ, "ROS_DISTRO": "jazzy", "ROS_SETUP": str(setup),
            "RELOCALIZATION_SETUP": str(setup)}, text=True, capture_output=True)
    assert result.returncode != 0
    assert "without a Humble underlay" in result.stderr


def test_smoke_checks_composed_indirect_tf():
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    import math

    module = load_module("ros_smoke_test", PACKAGE_ROOT / "tools/ros_smoke_test.py")
    world_to_odom = TransformStamped()
    world_to_odom.header.frame_id = "site"
    world_to_odom.header.stamp.sec = 1
    world_to_odom.child_frame_id = "odom"
    world_to_odom.transform.translation.x = 2.0
    world_to_odom.transform.translation.y = 3.0
    world_to_odom.transform.rotation.z = math.sin(math.pi / 4)
    world_to_odom.transform.rotation.w = math.cos(math.pi / 4)
    odom_to_body = TransformStamped()
    odom_to_body.header.frame_id = "odom"
    odom_to_body.header.stamp.sec = 1
    odom_to_body.child_frame_id = "robot"
    odom_to_body.transform.translation.x = 1.0
    odom_to_body.transform.rotation.z = -math.sin(math.pi / 4)
    odom_to_body.transform.rotation.w = math.cos(math.pi / 4)
    buffer = Buffer()
    buffer.set_transform(world_to_odom, "relocalizer_test")
    buffer.set_transform(odom_to_body, "odometry_test")
    pose = PoseStamped()
    pose.header.frame_id = "site"
    state = SimpleNamespace(
        world_frame="site", body_frame="robot", odom_frame="odom", output_edge=None,
        transform=buffer.lookup_transform("site", "robot", Time(seconds=1)),
        pose=pose, proposals=PoseArray(poses=[Pose()]), initial_pose=None, initial_sent=False,
        status={"state": "TRACKING"}, service_result={"success": True},
        expected=(2.0, 4.0, 0.0),
    )
    module.SmokeNode.on_tf(state, TFMessage(transforms=[world_to_odom, odom_to_body]))
    assert state.output_edge == world_to_odom
    report = module.SmokeNode.report(state)
    assert report["ok"], report
    assert report["output_child_frame"] == "odom"
    assert report["transform"]["distance_error"] < 1e-9


def test_smoke_rejects_synthetic_odom_without_frame():
    module = load_module("ros_smoke_test", PACKAGE_ROOT / "tools/ros_smoke_test.py")
    with pytest.raises(SystemExit) as exception:
        module.main(["--sweep", "/not-used.pcd", "--odom-pose", "1", "2", "30"])
    assert exception.value.code == 2
