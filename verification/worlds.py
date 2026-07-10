"""시나리오 월드 생성: small_city.sdf의 actor 블록을 시나리오 명세로 교체.

기본 월드(커밋본)를 그대로 두고 actor/정적 보행자만 갈아끼운 변형 SDF를
생성한다. 5.5k줄 월드 사본을 커밋하지 않기 위한 결정적(문자열 치환) 생성.

actor 명세:
  {name, waypoints: [[t, x, y, yaw], ...], loop: bool}          # 이동 보행자
  {name, static: true, x, y}                                    # 정지 보행자

⚠ gz <actor>는 headless(-s) 센서 렌더링과 같이 쓸 때 함정이 4개 있다
(2026-07-09~10 디버깅 — ACTOR_TEMPLATE 위 주석 참조). 특히 <loop>false</loop>는
sim 루프를 wedge시키므로(전 토픽/서비스 무응답) 템플릿은 항상 loop=true를
내보내고, "이동 후 정지"는 준정지 hold 세그먼트로 표현한다.
퇴화 궤적 가드(사용자 세그먼트 이동거리 < 0.05m 또는 속도 < 0.02 m/s 거부)는
방어적으로 유지 — 정지는 static: true가 올바른 표현이다.
정지 보행자 모델은 plugin <model_name> 주입으로, 이동 보행자(actor)는
oracle이 자동으로 추적한다.
"""

import math
import re

# 이동 보행자: 사람 스킨(walk.dae) actor. headless(-s) 센서 렌더링과 함께
# 쓰려면 아래 4가지가 전부 필요하다 (2026-07-10 A/B 실험으로 각각 확정):
#   1) <loop>true</loop> — loop=false는 센서 씬 초기화 시 sim 루프 wedge
#   2) <interpolate_x> 없음 — 있으면 waypoint 시간 무시(거리 기반 재타이밍)
#   3) tension="1.0" — 낮으면 불균등 세그먼트에서 스플라인 오버슈트(수 m 진동)
#   4) trajectory type ≠ animation 이름 — 매칭되면 애니메이션 길이가 타이밍 오염
# "이동 후 정지"(spec loop=false)는 loop=true + 초장주기 준정지 hold 세그먼트로
# 표현한다 (generate 시 자동 추가 — HOLD_* 상수 참조).
ACTOR_TEMPLATE = """    <actor name="{name}">
      <skin>
        <filename>model://actor_walking/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://actor_walking/meshes/walk.dae</filename>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="route" tension="1.0">
{waypoints}
        </trajectory>
      </script>
    </actor>
"""

WAYPOINT_TEMPLATE = """          <waypoint>
            <time>{t}</time>
            <pose>{x} {y} 1.15 0 0 {yaw}</pose>
          </waypoint>"""

# 준정지 hold: 관측 창보다 훨씬 긴 주기로 미세 이동 → 사실상 그 자리에 정지.
# (제자리 0거리 세그먼트 대신 미세 이동을 쓰는 것은 방어적 선택)
HOLD_DIST = 0.06      # m
HOLD_DURATION = 9999  # s

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
                f'speed={dist / dt:.4f}m/s — 정지는 static: true, '
                f'종료 유지는 loop=false 마지막 waypoint로 표현할 것')


def moving_ped_xml(spec):
    validate_waypoints(spec['name'], spec['waypoints'])
    waypoints = list(spec['waypoints'])
    if not spec.get('loop', False):
        # spec loop=false("이동 후 정지") → 준정지 hold를 붙여 loop=true로 표현.
        # (사용자 waypoint 검증 후 내부적으로 추가하므로 퇴화 가드 대상 아님)
        (t0, x0, y0, _), (t1, x1, y1, yaw1) = waypoints[-2], waypoints[-1]
        seg = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        ux, uy = (float(x1) - float(x0)) / seg, (float(y1) - float(y0)) / seg
        waypoints.append([float(t1) + HOLD_DURATION,
                          float(x1) + ux * HOLD_DIST,
                          float(y1) + uy * HOLD_DIST, yaw1])
    waypoint_xml = '\n'.join(
        WAYPOINT_TEMPLATE.format(t=w[0], x=w[1], y=w[2], yaw=w[3])
        for w in waypoints)
    return ACTOR_TEMPLATE.format(name=spec['name'], waypoints=waypoint_xml)


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
            insert += moving_ped_xml(spec)   # actor는 oracle이 자동 추적

    idx = sdf.rindex('</world>')
    sdf = sdf[:idx] + insert + sdf[idx:]

    # 정지 보행자 모델을 oracle이 추적하도록 plugin에 <model_name> 주입
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
