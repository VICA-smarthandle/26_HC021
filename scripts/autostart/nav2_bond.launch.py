"""자동 기동 유닛용 Nav2 — vica_nav2 의 nav2_map_test.launch.py 를 그대로 띄우되,"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParametersFromFile

BOND_TIMEOUT_SEC = 8.0
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
