"""모드 노드 voice_on|off 서비스와 /cmd_vel_safe 움직임 판정(앱 마이크 점검, 2026-11-19).

노드는 ROS 가 있어야 돌아가므로 원문 문자열만 확인한다.
"""
import unittest
from pathlib import Path

NODE = (Path(__file__).resolve().parents[1] / "autostart" / "vica_mode_node.py").read_text(encoding="utf-8")


class VoiceMicNode(unittest.TestCase):
    def test_services_registered(self):
        self.assertIn('"/vica/mode/voice_on"', NODE)
        self.assertIn('"/vica/mode/voice_off"', NODE)

    def test_cmd_vel_safe_subscribed_with_twist(self):
        self.assertIn('"/cmd_vel_safe"', NODE)
        self.assertIn("from geometry_msgs.msg import Twist", NODE)
        self.assertIn("ml.twist_moving", NODE)

    def test_refusal_and_check_unit_used(self):
        self.assertIn("ml.voice_node_refusal", NODE)
        self.assertIn("ml.MIC_CHECK_UNIT", NODE)
        self.assertIn("ml.CMD_MOVING_SEC", NODE)
        self.assertIn("cmd_moving=", NODE)

    def test_response_texts(self):
        self.assertIn("음성 노드를 켭니다", NODE)
        self.assertIn("음성 노드를 껐습니다", NODE)


if __name__ == "__main__":
    unittest.main()
