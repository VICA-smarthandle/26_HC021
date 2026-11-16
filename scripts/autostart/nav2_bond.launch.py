"""자동 기동 유닛용 Nav2 — vica_nav2 의 nav2_map_test.launch.py 를 그대로 띄우되,
Nav2 lifecycle 관리자의 bond_timeout 을 4 s → 8 s 로 늘린다(사용자 11-16).

    ros2 launch nav2_bond.launch.py map:=<지도 yaml> use_route:=true   (인자는 그대로 넘어간다)

왜: 2026-11-14·16 실기에서 controller_server 등이 관리자와 bond 4.00 s 안에 연결되지 않아 기동이
중단됐다("was unable to be reached after 4.00s by bond", 원래 방식 14번 중 1번). 정상 회차에서도
연결에 5.9 s 걸린 적이 있다 — 8 s 는 그보다 2 s 여유.
대가: Nav2 노드가 멈췄을 때 관리자가 알아채는 데 최대 4 s 더 걸린다. controller 가 죽으면 safety 가
0.5 s 안에 세우지만, bt_navigator·planner 가 멈추면 그동안 controller 가 받아 둔 경로를 계속 따라간다
(장애물 회피·collision_monitor 는 그대로).

왜 감싸나: 이 값은 nav2_params.yaml 로는 안 바뀐다. nav2_bringup 이 관리자에게 파일을 넘기지 않고
use_sim_time·autostart·node_names 만 준다. SetParameter 는 이 묶음 안에서 뜨는 모든 노드에
전역 파라미터로 얹히고, bond_timeout 을 선언한 노드(lifecycle_manager)만 쓴다.
칸에서 띄우는 ⑨ nav2 는 이 파일을 쓰지 않는다 — 지금 그대로다.

인자: 바깥 launch 의 명령줄 인자는 안쪽 include 로 저절로 넘어가지 않는다(2026-11-16 실기 —
"Included launch description missing required argument 'map'"). 여기서 선언하고 직접 넘긴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

BOND_TIMEOUT_SEC = 8.0


def generate_launch_description() -> LaunchDescription:
    inner = os.path.join(
        get_package_share_directory("vica_nav2"), "launch", "nav2_map_test.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="지도 yaml(안쪽 nav2_map_test 의 필수 인자)"),
            DeclareLaunchArgument("use_route", default_value="true"),
            GroupAction(
                [
                    SetParameter(name="bond_timeout", value=BOND_TIMEOUT_SEC),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(inner),
                        launch_arguments={
                            "map": LaunchConfiguration("map"),
                            "use_route": LaunchConfiguration("use_route"),
                        }.items(),
                    ),
                ]
            ),
        ]
    )
