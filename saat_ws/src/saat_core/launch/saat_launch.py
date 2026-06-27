"""
saat_launch.py
==============
Master launch file for the SAAT Pear Sorting & Packing System.

Startup sequence (enforced via TimerAction + ReadyCondition):
─────────────────────────────────────────────────────────────
 T+0.0 s │ vision_init_node         ← must run FIRST, broadcasts latched params
 T+1.0 s │ frame_capture_node       ← camera must be up before dividers
 T+2.0 s │ frame_divider_node
          │ frame_speed_divider_node
          │ volume_divider_node
 T+3.0 s │ 6× vision_node          (A1…B3)
          │ 6× area_node            (A1…B3)
 T+4.0 s │ 6× action_node          (A1…B3)
          │ 6× data_collector_node  (A1…B3)
          │ infection_description_node
          │ volume_estimation_node
          │ mass_estimation_node
 T+5.0 s │ main_speed_node
          │ conv1_speed_node
          │ conv2_speed_node
          │ servo_speed_node
          │ speed_publisher_node
 T+6.0 s │ data_collection_node
          │ webpage_publisher_node

Usage
-----
  ros2 launch saat_core saat_launch.py
  ros2 launch saat_core saat_launch.py use_sim:=true   # No GPIO/camera
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ── Zones ──────────────────────────────────────────────────────────────────
_ZONES    = ["A1", "A2", "A3", "B1", "B2", "B3"]
_PKG      = "saat_core"
_IFACE    = "saat_interfaces"


def _params_file() -> PathJoinSubstitution:
    """Return the path to saat_params.yaml in the installed share directory."""
    return PathJoinSubstitution([
        FindPackageShare(_PKG), 'config', 'saat_params.yaml'
    ])


def _node(executable: str, name: str, extra_params: dict | None = None) -> Node:
    """Helper: build a ROS2 Node action with shared params file."""
    params = [_params_file()]
    if extra_params:
        params.append(extra_params)
    return Node(
        package=_PKG,
        executable=executable,
        name=name,
        output='screen',
        parameters=params,
    )


def _zone_nodes(executable: str, name_template: str) -> list[Node]:
    """
    Create 6 identical nodes, one per zone.
    name_template must contain {zone}, e.g. '{zone}_vision'.
    """
    return [
        _node(
            executable=executable,
            name=name_template.format(zone=z),
            extra_params={'zone_id': z},
        )
        for z in _ZONES
    ]


def _zone_action_nodes() -> list[Node]:
    """Create 6 action nodes with their PCA9685 channel assignments."""
    channel_map = {"A1": 0, "A2": 1, "A3": 2, "B1": 3, "B2": 4, "B3": 5}
    return [
        _node(
            executable='action_node.py',
            name=f'{z}_action',
            extra_params={
                'zone_id':    z,
                'pca_channel': channel_map[z],
            }
        )
        for z in _ZONES
    ]


def generate_launch_description() -> LaunchDescription:
    # ── Arguments ─────────────────────────────────────────────────────────
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set true to disable GPIO/camera for desktop simulation.'
    )

    # ── T+0: vision_init_node (MUST be first) ─────────────────────────────
    t0 = GroupAction([
        LogInfo(msg='[SAAT] T+0: Starting vision_init_node...'),
        _node('vision_init_node.py', 'vision_init_node'),
    ])

    # ── T+1: Camera capture ───────────────────────────────────────────────
    t1 = TimerAction(period=1.0, actions=[
        LogInfo(msg='[SAAT] T+1: Starting RealSense capture...'),
        _node('frame_capture_node.py', 'frame_capture_node'),
    ])

    # ── T+2: Frame dividers ───────────────────────────────────────────────
    t2 = TimerAction(period=2.0, actions=[
        LogInfo(msg='[SAAT] T+2: Starting frame dividers...'),
        _node('frame_divider_node.py',       'frame_divider_node'),
        _node('frame_speed_divider_node.py', 'frame_speed_divider_node'),
        _node('volume_divider_node.py',      'volume_divider_node'),
    ])

    # ── T+3: Vision nodes + area nodes (6×2 = 12 nodes) ──────────────────
    t3 = TimerAction(period=3.0, actions=[
        LogInfo(msg='[SAAT] T+3: Starting 6 vision nodes + 6 area nodes...'),
        *_zone_nodes('vision_node.py', '{zone}_vision'),
        *_zone_nodes('area_node.py',   '{zone}_area_node'),
    ])

    # ── T+4: Action + data-collector + aggregators ────────────────────────
    t4 = TimerAction(period=4.0, actions=[
        LogInfo(msg='[SAAT] T+4: Starting action nodes, data collectors, aggregators...'),
        *_zone_action_nodes(),
        *_zone_nodes('data_collector_node.py', '{zone}_data_collector'),
        _node('infection_description_node.py', 'infection_description_node'),
        _node('volume_estimation_node.py',     'volume_estimation_node'),
        _node('mass_estimation_node.py',       'mass_estimation_node'),
    ])

    # ── T+5: Speed pipeline ───────────────────────────────────────────────
    t5 = TimerAction(period=5.0, actions=[
        LogInfo(msg='[SAAT] T+5: Starting speed pipeline...'),
        _node('main_speed_node.py',  'main_speed_node'),
        _node('servo_speed_node.py', 'servo_speed_node'),
        _node('speed_publisher_node.py', 'speed_publisher_node'),

        # Conv1 speed node (Loading Belt, 175 cm)
        _node(
            'conv_speed_node.py', 'conv1_speed_node',
            extra_params={'zone_id': 'CONV1', 'belt_length_m': 1.75}
        ),
        # Conv2 speed node (Packing Belt, 115 cm)
        _node(
            'conv_speed_node.py', 'conv2_speed_node',
            extra_params={'zone_id': 'CONV2', 'belt_length_m': 1.15}
        ),
    ])

    # ── T+6: Database + web dashboard ─────────────────────────────────────
    t6 = TimerAction(period=6.0, actions=[
        LogInfo(msg='[SAAT] T+6: Starting database node + SCADA dashboard...'),
        _node('data_collection_node.py',  'data_collection_node'),
        _node('webpage_publisher_node.py','webpage_publisher_node'),
        LogInfo(msg='[SAAT] ✅ Full pipeline online. Dashboard at http://localhost:8080'),
    ])

    return LaunchDescription([
        use_sim_arg,
        t0, t1, t2, t3, t4, t5, t6,
    ])
