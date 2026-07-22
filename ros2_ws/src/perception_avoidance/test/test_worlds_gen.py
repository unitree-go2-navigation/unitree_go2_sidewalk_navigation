"""verification/worlds.py 월드 생성 가드 테스트.

gz <actor>는 headless 센서 렌더링에서 loop=false면 sim을 wedge시킴
(2026-07-09~10) — 이동 보행자 actor가 안전 레시피(loop=true + 준정지 hold +
tension=1.0 + 비매칭 trajectory type)로 생성되고, 퇴화 궤적(초저속/제자리
세그먼트)이 생성 단계에서 거부되는지 고정한다.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[4] / 'verification'))
import worlds  # noqa: E402


def test_degenerate_hold_segment_rejected():
    spec = {'name': 'ped', 'waypoints': [[0, 10.0, 5.2, 0], [9999, 10.001, 5.2, 0]]}
    with pytest.raises(ValueError, match='degenerate'):
        worlds.moving_ped_xml(spec)


def test_pause_segment_rejected():
    spec = {'name': 'ped', 'waypoints': [
        [0, 12.0, 2.0, 1.57], [6, 12.0, 5.2, 1.57], [16, 12.0, 5.2, 1.57]]}
    with pytest.raises(ValueError, match='degenerate'):
        worlds.moving_ped_xml(spec)


def test_normal_trajectory_accepted():
    spec = {'name': 'ped', 'loop': False,
            'waypoints': [[0, 4.0, 5.2, 0], [12, 10.0, 5.2, 0]]}
    xml = worlds.moving_ped_xml(spec)
    assert '<actor name="ped">' in xml
    # headless 안전 레시피 4종 고정
    assert '<loop>true</loop>' in xml           # loop=false는 wedge
    assert 'interpolate_x' not in xml           # 있으면 시간 무시(거리 재타이밍)
    assert 'tension="1.0"' in xml               # 낮으면 스플라인 오버슈트
    assert 'type="route"' in xml                # animation 이름과 매칭 금지
    # spec loop=false → 준정지 hold 세그먼트 자동 추가 (2 + 1)
    assert xml.count('<waypoint>') == 3
    assert '<time>10011.0</time>' in xml        # 12 + HOLD_DURATION


def test_bike_trajectory_uses_exported_glb_model():
    spec = {'name': 'bike_head_on', 'loop': False,
            'waypoints': [[0, -14.0, 5.2, 0], [8, -12.0, 5.2, 0]]}
    xml = worlds.moving_bike_xml(spec)
    assert '<model name="bike_head_on">' in xml
    assert '<actor' not in xml
    assert 'midday_ride/meshes/midday_ride_gazebo.glb' in xml
    assert 'filename="WaypointMover"' in xml
    assert '<z>0.16</z>' in xml
    assert '<loop>false</loop>' in xml
    assert '<waypoint>0 -14.0 5.2 0</waypoint>' in xml
    # Y-up→Z-up으로 세우고 원본 -Y 전방을 시나리오 +X 진행축에 맞춘다.
    assert '<pose>0 0 0 1.57079632679 0 1.57079632679</pose>' in xml


def test_bike_scenario_tracks_model_in_oracle(tmp_path):
    base = tmp_path / 'base.sdf'
    out = tmp_path / 'world.sdf'
    base.write_text('''<sdf version="1.10"><world name="default">
    <plugin name="go2_simulation::ActorPosePublisher" filename="ActorPosePublisher">
      <topic>/world/default/actor_pose/info</topic>
    </plugin>
    <actor name="placeholder"><script/></actor>
</world></sdf>''')
    spec = {'name': 'bike_pass', 'loop': False,
            'waypoints': [[0, -14.0, 6.7, 0], [8, -12.0, 6.7, 0]]}
    worlds.generate_world(
        base, [spec], out, scenario_name='bike_pass')
    xml = out.read_text()
    assert '<actor' not in xml
    assert '<model name="bike_pass">' in xml
    assert '<model_name>bike_pass</model_name>' in xml


def test_single_waypoint_rejected():
    spec = {'name': 'ped', 'waypoints': [[0, 4.0, 5.2, 0]]}
    with pytest.raises(ValueError, match='static'):
        worlds.moving_ped_xml(spec)


def test_static_ped_is_model_not_actor():
    xml = worlds.static_ped_xml({'name': 'ped_s', 'x': 10.0, 'y': 5.2})
    assert '<model name="ped_s">' in xml
    assert '<static>true</static>' in xml
    assert '<actor' not in xml
    # 사람 형태 visual (실린더 대체) + oracle actor_radius와 일치하는 충돌 실린더
    assert 'MaleVisitorOnPhone/meshes/MaleVisitorStatic.obj' in xml
    assert '<cylinder><radius>0.3</radius>' in xml


def test_box_obstacle_generated(tmp_path):
    base = tmp_path / 'base.sdf'
    out = tmp_path / 'w.sdf'
    base.write_text("""<sdf version="1.10"><world name="default">
    <plugin name="go2_simulation::ActorPosePublisher" filename="ActorPosePublisher">
      <topic>/t</topic>
    </plugin>
</world></sdf>""")
    worlds.generate_world(
        base, [{'name': 'b1', 'box': True, 'x': 10.0, 'y': 4.9,
                'size': [0.4, 0.5, 1.0]}], str(out), scenario_name='squeeze')
    sdf = out.read_text()
    assert '<box><size>0.4 0.5 1.0</size></box>' in sdf
    assert '10.0 4.9 0.66' in sdf          # z = 0.16 + sz/2
    assert 'b1' in sdf
