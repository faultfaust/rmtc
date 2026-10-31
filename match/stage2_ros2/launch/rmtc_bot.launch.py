#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rmtc_bot.launch.py —— 启动 RMTC 第二阶段节点。

用法：
    source install/setup.bash
    export RMTC_AI_URL='http://<gateway>:8000/v1/chat/completions'
    export RMTC_AI_TOKEN='<本场席位令牌>'
    ros2 launch rmtc_stage2_tm rmtc_bot.launch.py seat:=A team_name:="My Team"

AI 网关/令牌通过环境变量传入，避免写进 launch 文件或仓库。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    seat = LaunchConfiguration('seat')
    team_name = LaunchConfiguration('team_name')
    params_file = os.path.join(
        get_package_share_directory('rmtc_stage2_tm'), 'config', 'rmtc_bot.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'seat', default_value='A',
            description='席位 A/B/C/D（对应 /team_<seat>/... 话题）'),
        DeclareLaunchArgument(
            'team_name', default_value='RMTC Bot',
            description='注册队名，1-96 字节 UTF-8，不含换行'),
        Node(
            package='rmtc_stage2_tm',
            executable='rmtc_bot',
            name='rmtc_bot',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {
                'seat': seat,
                'team_name': team_name,
            }],
        ),
    ])
