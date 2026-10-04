# ファイルの役割: 接触コールバックを直接呼び、床は許容し、壁・箱・ポールでは停止が保持されることを検証する。
"""Verify obstacle contacts latch the real safety node and cannot be resumed."""
import os
os.environ['ROS_DOMAIN_ID'] = '47'
import unittest
import rclpy
from loop_controller import SafetyGate
from ros_gz_interfaces.msg import Contact, Contacts
from std_srvs.srv import SetBool


# 関連する検証ケースをまとめ、失敗条件をassertで確認するテストクラス。
class ContactsTest(unittest.TestCase):
    # 床・壁・箱・ポールを区別し、障害物との接触後は通常の解除で再開できないことを確認する。
    def test_contact_classification_and_latch(self):
        rclpy.init()
        node = SafetyGate()
        try:
            for name, blocked in [('ground::collision', False),
                                  ('obstacle_box_inner::collision', True),
                                  ('obstacle_pylon_outer::collision', True),
                                  ('outer_wall::wall_0', True)]:
                with self.subTest(name=name):
                    node.collision = False
                    contact = Contact()
                    contact.collision1.name = 'loop_cart::base_link::body_collision'
                    contact.collision2.name = name
                    node.contacts(Contacts(contacts=[contact]))
                    self.assertEqual(node.collision, blocked)
                    response = node.enable(SetBool.Request(data=True), SetBool.Response())
                    self.assertEqual(response.success, not blocked)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
