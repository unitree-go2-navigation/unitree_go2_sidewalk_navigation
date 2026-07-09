"""collision_oracle_node의 clearance 수식/다중 actor/CSV 기록 테스트."""

import pytest
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from perception_avoidance.collision_oracle_node import CollisionOracleNode


def tf_msg(entries):
    msg = TFMessage()
    for name, x, y in entries:
        t = TransformStamped()
        t.child_frame_id = name
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        msg.transforms.append(t)
    return msg


@pytest.fixture()
def node():
    n = CollisionOracleNode()
    yield n
    n.destroy_node()


def test_clearance_formula_exact(node):
    # center_dist 2.0 - r_robot 0.25 - r_actor 0.30 = 1.45
    node.pose_cb(tf_msg([('go2', 0.0, 0.0), ('ped', 2.0, 0.0)]))
    node.check_clearance()
    assert node.min_clearance == pytest.approx(1.45, abs=1e-6)


def test_multi_actor_min_is_nearest(node):
    node.pose_cb(tf_msg([('go2', 0.0, 0.0), ('far', 5.0, 0.0), ('near', 1.0, 0.0)]))
    node.check_clearance()
    assert node.min_clearance == pytest.approx(1.0 - 0.55, abs=1e-6)
    assert node.min_clearance_actor == 'near'


def test_collision_counted_once_per_entry(node):
    node.pose_cb(tf_msg([('go2', 0.0, 0.0), ('ped', 0.3, 0.0)]))
    node.check_clearance()
    node.check_clearance()          # still inside band → no double count
    assert node.collision_count == 1
    # leave then re-enter → counted again
    node.pose_cb(tf_msg([('go2', 0.0, 0.0), ('ped', 3.0, 0.0)]))
    node.check_clearance()
    node.pose_cb(tf_msg([('go2', 0.0, 0.0), ('ped', 0.3, 0.0)]))
    node.check_clearance()
    assert node.collision_count == 2


def test_csv_written_with_rows_and_summary(tmp_path):
    from rclpy.parameter import Parameter
    csv_path = str(tmp_path / 'run.csv')
    node = CollisionOracleNode(
        parameter_overrides=[Parameter('csv_path', value=csv_path)])
    node.pose_cb(tf_msg([('go2', 0.0, 0.0), ('ped', 0.3, 0.0)]))
    node.check_clearance()
    node.destroy_node()

    with open(csv_path) as f:
        content = f.read()
    assert content.startswith('t,actor,clearance')      # header
    assert ',ped,-0.2500,' in content                   # per-tick row
    assert '# summary collisions=1' in content          # shutdown summary
    assert 'min_clearance=-0.2500' in content
