"""시나리오 월드 생성: small_city.sdf의 actor 블록을 시나리오 명세로 교체.

기본 월드(커밋본)를 그대로 두고 actor/정적 보행자만 갈아끼운 변형 SDF를
생성한다. 5.5k줄 월드 사본을 커밋하지 않기 위한 결정적(문자열 치환) 생성.

actor 명세:
  {name, waypoints: [[t, x, y, yaw], ...], loop: bool}          # 이동 보행자
  {name, static: true, x, y}                                    # 정지 보행자

⚠ 퇴화 궤적 금지: 세그먼트 이동거리 < 0.05m 또는 속도 < 0.02 m/s인 waypoint
쌍은 거부한다. 근거(2026-07-09 디버깅): 퇴화 궤적 actor는 센서 렌더링이 활성인
시뮬에서 gz sim 스레드를 wedge시킨다 (스켈레탈 애니메이션 시간 계산 발산 추정,
전 토픽/서비스 무응답 + "SceneBroadcaster: Timed out waiting for state" 증상).
정지 보행자는 actor가 아니라 static 실린더 모델(static: true)로 표현할 것 —
plugin <model_name> 주입으로 oracle이 동일하게 추적한다.
"""

import math
import re

ACTOR_TEMPLATE = """    <actor name="{name}">
      <skin>
        <filename>model://actor_walking/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://actor_walking/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>{loop}</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.6">
{waypoints}
        </trajectory>
      </script>
    </actor>
"""

WAYPOINT_TEMPLATE = """          <waypoint>
            <time>{t}</time>
            <pose>{x} {y} 1.15 0 0 {yaw}</pose>
          </waypoint>"""

# 정지 보행자: 사람 크기 실린더 (r=0.3, h=1.7). 실린더 중심 z = 인도면(0.16)+0.85.
STATIC_PED_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 1.01 0 0 0</pose>
      <link name="body">
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>1.7</length></cylinder></geometry>
          <material><ambient>0.2 0.7 0.2 1</ambient><diffuse>0.2 0.7 0.2 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>1.7</length></cylinder></geometry>
        </collision>
      </link>
    </model>
"""

ACTOR_BLOCK_RE = re.compile(r'[ \t]*<actor\b.*?</actor>\n', re.DOTALL)

MIN_SEGMENT_DIST = 0.05   # m
MIN_SEGMENT_SPEED = 0.02  # m/s


def validate_waypoints(name, waypoints):
    if len(waypoints) < 2:
        raise ValueError(f'{name}: trajectory needs >= 2 waypoints '
                         f'(정지 보행자는 static: true 사용)')
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(waypoints, waypoints[1:]):
        dt = float(t1) - float(t0)
        dist = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        if dt <= 0:
            raise ValueError(f'{name}: waypoint time not increasing ({t0}→{t1})')
        if dist < MIN_SEGMENT_DIST or dist / dt < MIN_SEGMENT_SPEED:
            raise ValueError(
                f'{name}: degenerate segment t={t0}→{t1} dist={dist:.3f}m '
                f'speed={dist / dt:.4f}m/s — gz actor 애니메이션이 sim을 '
                f'wedge시킴. 정지는 static: true, 종료 유지는 loop=false '
                f'마지막 waypoint로 표현할 것')


def actor_xml(spec):
    validate_waypoints(spec['name'], spec['waypoints'])
    waypoints = '\n'.join(
        WAYPOINT_TEMPLATE.format(t=w[0], x=w[1], y=w[2], yaw=w[3])
        for w in spec['waypoints'])
    return ACTOR_TEMPLATE.format(
        name=spec['name'],
        loop='true' if spec.get('loop', False) else 'false',
        waypoints=waypoints)


def static_ped_xml(spec):
    return STATIC_PED_TEMPLATE.format(
        name=spec['name'], x=spec['x'], y=spec['y'])


def generate_world(base_sdf_path, actors, out_path):
    with open(base_sdf_path) as f:
        sdf = f.read()

    n_removed = len(ACTOR_BLOCK_RE.findall(sdf))
    sdf = ACTOR_BLOCK_RE.sub('', sdf)

    insert = ''
    static_names = []
    for spec in actors:
        if spec.get('static'):
            insert += static_ped_xml(spec)
            static_names.append(spec['name'])
        else:
            insert += actor_xml(spec)

    idx = sdf.rindex('</world>')
    sdf = sdf[:idx] + insert + sdf[idx:]

    # 정적 보행자 모델을 oracle이 추적하도록 plugin에 <model_name> 주입
    if static_names:
        m = re.search(
            r'(<plugin[^>]*ActorPosePublisher.*?)(\n[ \t]*</plugin>)',
            sdf, re.DOTALL)
        if not m:
            raise ValueError('ActorPosePublisher plugin block not found in base world')
        extra = ''.join(
            f'\n      <model_name>{n}</model_name>' for n in static_names)
        sdf = sdf[:m.end(1)] + extra + sdf[m.end(1):]

    with open(out_path, 'w') as f:
        f.write(sdf)
    return n_removed, len(actors)
