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
use_sim_time·autostart·node_names 만 준다. 그래서 관리자 이름만 적은 파라미터 파일을 이 묶음의
전역 파라미터로 얹는다(SetParametersFromFile).

**모든 노드 공통(/**) 으로 얹지 않는다(2026-11-16 실기).** 처음엔 SetParameter(bond_timeout) 를 썼는데,
그것은 모든 노드 명령줄 맨 앞에 `-p`(= /** 항목)를 만든다. ROS 2 는 파라미터의 노드 항목을 처음 나온
순서대로 적용하므로, launch 가 뒤에 /** 로 덧씌우는 값 — route_server 의 graph_filepath(레일)와 keepout
마스크 서버의 yaml_filename(금지구역) — 이 nav2_params.yaml 의 빈 값('')에 졌다. 그날 유닛 Nav2 는
레일·금지구역 없이 달렸다('No graph set!', 'yaml-filename parameter is empty'). 칸 시절에는 /** 가 맨
뒤에 생겨 덧씌우기가 이겼다. 관리자 이름만 적으면 /** 가 생기지 않아 칸 시절 순서 그대로다.
칸에서 띄우는 ⑨ nav2 는 이 파일을 쓰지 않는다 — 지금 그대로다.

인자: 바깥 launch 의 명령줄 인자는 안쪽 include 로 저절로 넘어가지 않는다(2026-11-16 실기 —
"Included launch description missing required argument 'map'"). 여기서 선언하고 직접 넘긴다.
시험용: 환경 변수 VICA_NAV2_BOND_TIMEOUT 이 있으면 그 값을 쓴다(자동 재시도를 실기로 확인하려고
일부러 중단을 만들 때만). 없거나 숫자가 아니면 8 s.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParametersFromFile

BOND_TIMEOUT_SEC = 8.0
# nav2_map_test.launch.py 가 띄우는 lifecycle 관리자(2026-11-16 저널에서 확인한 다섯).
LIFECYCLE_MANAGERS = (
    "lifecycle_manager_navigation",
    "lifecycle_manager_localization",
    "lifecycle_manager_collision_monitor",
    "lifecycle_manager_keepout",
    "lifecycle_manager_route",
)


def bond_timeout_sec(env=os.environ) -> float:
    try:
        return float(env.get("VICA_NAV2_BOND_TIMEOUT", BOND_TIMEOUT_SEC))
    except ValueError:
        return BOND_TIMEOUT_SEC


def bond_params_file(timeout: float) -> str:
    """관리자 이름만 적은 파라미터 파일을 만들고 경로를 돌려준다. /** 항목은 만들지 않는다."""
    fd, path = tempfile.mkstemp(prefix="vica_nav2_bond_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as out:
        for name in LIFECYCLE_MANAGERS:
            out.write(f"{name}:\n  ros__parameters:\n    bond_timeout: {timeout}\n")
    return path


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
                    SetParametersFromFile(bond_params_file(bond_timeout_sec())),
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
