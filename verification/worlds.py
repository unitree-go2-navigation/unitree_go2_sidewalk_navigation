"""시나리오 월드 생성: small_city.sdf의 actor 블록을 시나리오 명세로 교체.

기본 월드(커밋본)를 그대로 두고 actor/정적 보행자만 갈아끼운 변형 SDF를
생성한다. 5.5k줄 월드 사본을 커밋하지 않기 위한 결정적(문자열 치환) 생성.

actor / 이동 모델 명세:
  {name, waypoints: [[t, x, y, yaw], ...], loop: bool}          # 이동 보행자
  {name, static: true, x, y}                                    # 정지 보행자
  {name, box: true, x, y, size: [sx, sy, sz]}                   # 정적 박스 (스퀴즈)

시나리오 이름에 ``bike``가 포함되면 이동 개체를 skeletal actor 대신
``midday_ride`` GLB visual + WaypointMover 모델로 생성한다. 일반 보행자
시나리오는 기존 actor 경로를 그대로 사용한다.

⚠ gz <actor>는 headless(-s) 센서 렌더링과 같이 쓸 때 함정이 4개 있다
(2026-07-09~10 디버깅 — ACTOR_TEMPLATE 위 주석 참조). 특히 <loop>false</loop>는
sim 루프를 wedge시키므로(전 토픽/서비스 무응답) 템플릿은 항상 loop=true를
내보내고, "이동 후 정지"는 준정지 hold 세그먼트로 표현한다.
퇴화 궤적 가드(사용자 세그먼트 이동거리 < 0.05m 또는 속도 < 0.02 m/s 거부)는
방어적으로 유지 — 정지는 static: true가 올바른 표현이다.
정지 보행자와 bike 이동 모델은 plugin <model_name> 주입으로, 일반
이동 보행자(actor)는 oracle이 자동으로 추적한다.
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

# bike 시나리오: 정적 GLB visual을 model-scoped WaypointMover로 이동한다.
# GLB의 Y-up을 Gazebo Z-up으로 바꾸는 roll +90deg 뒤, Blender 원본의 전방
# -Y를 +X로 맞추는 yaw +90deg를 적용한다. 메시 바닥 Z=0을 인도면 0.16m에
# 두므로 waypoint yaw=0인 bike는 +X로 달려 yaw=pi인 Go2와 마주 본다.
BIKE_WAYPOINT_TEMPLATE = """        <waypoint>{t} {x} {y} {yaw}</waypoint>"""

BIKE_MODEL_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x0} {y0} 0.16 0 0 {yaw0}</pose>
      <link name="body">
        <visual name="visual">
          <pose>0 0 0 1.57079632679 0 1.57079632679</pose>
          <geometry>
            <mesh>
              <uri>model://midday_ride/meshes/midday_ride_gazebo.glb</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
      <plugin filename="WaypointMover" name="go2_simulation::WaypointMover">
        <z>0.16</z>
        <loop>{loop}</loop>
{waypoints}
      </plugin>
    </model>
"""

# 준정지 hold: 관측 창보다 훨씬 긴 주기로 미세 이동 → 사실상 그 자리에 정지.
# (제자리 0거리 세그먼트 대신 미세 이동을 쓰는 것은 방어적 선택)
HOLD_DIST = 0.06      # m
HOLD_DURATION = 9999  # s

# 정지 보행자: 서 있는 사람 메시 (MaleVisitorOnPhone, 키 1.74m, 발 = 메시 원점).
# 라이다(GPU)는 visual을 보므로 사람 형태 그대로 감지된다. 충돌은 oracle의
# actor_radius(0.3)와 일치하는 실린더 유지. 모델 pose z = 인도면(0.16).
STATIC_PED_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.16 0 0 0</pose>
      <link name="body">
        <visual name="visual">
          <geometry><mesh><uri>model://MaleVisitorOnPhone/meshes/MaleVisitorStatic.obj</uri></mesh></geometry>
        </visual>
        <collision name="collision">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><cylinder><radius>0.3</radius><length>1.7</length></cylinder></geometry>
        </collision>
      </link>
    </model>
"""

BOX_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="body">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.6 0.45 0.3 1</ambient>
            <diffuse>0.6 0.45 0.3 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
      </link>
    </model>
"""

ACTOR_BLOCK_RE = re.compile(r'[ \t]*<actor\b.*?</actor>\n', re.DOTALL)

# 회귀 운영점 (generate_world 주석 참조).
# ⚠ 이 값은 라이다가 아니라 **보행 제어 충실도**가 결정한다 — 올리면 심시간
# 기준 gait 제어율이 그만큼 떨어져 거동이 바뀐다. 0.45 등가성 검증 실패
# (l1_to_l2_delta §3.2). 바꾸려면 전 시나리오 재기준화가 필요하다.
RTF_CAP = 0.25

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


def moving_bike_xml(spec):
    """Render a bike scenario mover as the exported cyclist GLB model."""
    validate_waypoints(spec['name'], spec['waypoints'])
    waypoints = list(spec['waypoints'])
    waypoint_xml = '\n'.join(
        BIKE_WAYPOINT_TEMPLATE.format(t=w[0], x=w[1], y=w[2], yaw=w[3])
        for w in waypoints)
    x0, y0, yaw0 = waypoints[0][1], waypoints[0][2], waypoints[0][3]
    return BIKE_MODEL_TEMPLATE.format(
        name=spec['name'], x0=x0, y0=y0, yaw0=yaw0,
        loop=str(bool(spec.get('loop', False))).lower(),
        waypoints=waypoint_xml)


def static_ped_xml(spec):
    return STATIC_PED_TEMPLATE.format(
        name=spec['name'], x=spec['x'], y=spec['y'])


def box_xml(spec):
    sx, sy, sz = spec.get('size', [0.4, 0.4, 1.0])
    # 보도 상판(z=0.16) 위에 올려놓는다
    return BOX_TEMPLATE.format(
        name=spec['name'], x=spec['x'], y=spec['y'],
        z=0.16 + 0.5 * sz, sx=sx, sy=sy, sz=sz)


def generate_world(base_sdf_path, actors, out_path, scenario_name=''):
    with open(base_sdf_path) as f:
        sdf = f.read()

    # RTF 캡 주입 (Phase 3.5 RTF 운영점 재유도, §6.6) — 목적은 **운영점 고정**.
    #
    # 출발점은 센서였다: 라이다 update_rate 5.55 는 심 시간 기준 설정값이고,
    # 게이트 파라미터(TTC·sensor_timeout·fast_trig_frames)도 전부 심 시간 위에서
    # 돌며, 실기 L2 가 벽시계 5.55 Hz 이므로 시뮬도 심 시간 기준 5.55 Hz 여야 한다.
    # 그러나 실측해 보니 **캡을 결정하는 것은 센서가 아니라 로코모션**이었다
    # (측정 1 vs 측정 2).
    #
    # 측정 1 — 라이다는 캡을 요구하지 않는다
    # (2026-08-05, 클린, bike_pass 월드 = GLB 메시 + actor, 최대 부하):
    #     설정 RTF   실효 RTF   심시간 주기 median/p90
    #       0.25       0.249        0.180 / 0.180
    #       0.45       0.442        0.180 / 0.180
    #       0.60       0.560        0.180 / 0.180
    #       1.0(무제한) 0.672        0.180 / 0.180
    # 어느 지점에서도 주기가 열화되지 않는다. gz 는 렌더가 밀리면 프레임을
    # 버리는 게 아니라 **시뮬 전체를 늦춰** 센서 스케줄을 지키므로, 한계가
    # "주기 열화"가 아니라 "RTF 포화(이 머신 0.672)"로 나타난다.
    #
    # 측정 2 — 🔴 캡을 결정하는 것은 **보행 제어 충실도**다
    # CHAMP quadruped_controller 는 joint 명령을 **벽시계 고정 ~200 Hz** 로
    # 낸다 (심 시간 기준이 아니다). 따라서 심시간 실효 제어율이 RTF 에 반비례한다:
    #     RTF 0.25 → 벽시계 200.3 Hz → 심시간 809 Hz
    #     RTF 0.45 → 벽시계 201.3 Hz → 심시간 462 Hz     (거의 절반)
    # 게이트 루프(/safety/state)는 두 지점 모두 심시간 50 Hz 로 멀쩡하다 —
    # 즉 바뀌는 것은 지각·판단이 아니라 **로코모션**이다.
    #
    # 0.45 등가성 검증 실패 (N=5, RTF 0.25 baseline 대비):
    #     head_on       3/5 — baseline 에 없던 실패 유형(robot_y 밴드 이탈
    #                   6.35/6.36): 양보 기동이 오버슈트해 연석을 넘는다
    #     social_yield  2/5 (baseline 8/10)
    #     bike_head_on_4ms 5/5 (유지 — fast-class 는 판단 지배적이라 둔감)
    # → 0.25 유지. 회귀 96 시행 실효 RTF 는 최소 0.243·중앙 0.248 로 머신이
    #   아니라 캡이 구속이었지만, 그 구속이 P4/P5 튜닝의 전제였다.
    #   캡을 바꾸려면 전 시나리오 재기준화가 필요하다.
    #
    # ⚠ 정정 이력: 이전 판은 "캡 없으면 심시간 1.4 Hz 붕괴 / 캡이 게이트 성립
    # 조건"이라 적었으나 둘 다 오염된 측정이었다(정리 목록에 ekf_node·
    # state_estimation_node 누락 → 유령 노드 생존). 완전 정리 후 A/B
    # (static_stop N=3): 캡 O 0/3(0.292~0.353) · 캡 X 0/3(0.302~0.343) —
    # 구분 불가. 캡은 게이트 성립 조건이 아니다 (l1_to_l2_delta §3.2).
    #
    # P35_RTF_CAP: '0' 이면 캡 해제(대조 실험용), 그 외에는 RTF 값으로 해석.
    import os as _os
    _cap = _os.environ.get('P35_RTF_CAP', str(RTF_CAP))
    if _cap != '0':
        _rtf = float(_cap)
        sdf = sdf.replace('<real_time_factor>1</real_time_factor>',
                          f'<real_time_factor>{_rtf}</real_time_factor>')
        sdf = sdf.replace('<real_time_update_rate>1000</real_time_update_rate>',
                          f'<real_time_update_rate>{int(round(_rtf * 1000))}'
                          f'</real_time_update_rate>')

    n_removed = len(ACTOR_BLOCK_RE.findall(sdf))
    sdf = ACTOR_BLOCK_RE.sub('', sdf)

    insert = ''
    tracked_model_names = []
    is_bike_scenario = 'bike' in scenario_name.lower()
    for spec in actors:
        if spec.get('box'):
            insert += box_xml(spec)
            tracked_model_names.append(spec['name'])
        elif spec.get('static'):
            insert += static_ped_xml(spec)
            tracked_model_names.append(spec['name'])
        elif is_bike_scenario:
            insert += moving_bike_xml(spec)
            tracked_model_names.append(spec['name'])
        else:
            insert += moving_ped_xml(spec)   # actor는 oracle이 자동 추적

    idx = sdf.rindex('</world>')
    sdf = sdf[:idx] + insert + sdf[idx:]

    # actor가 아닌 model 기반 개체를 oracle이 추적하도록 이름을 주입
    if tracked_model_names:
        m = re.search(
            r'(<plugin[^>]*ActorPosePublisher.*?)(\n[ \t]*</plugin>)',
            sdf, re.DOTALL)
        if not m:
            raise ValueError('ActorPosePublisher plugin block not found in base world')
        extra = ''.join(
            f'\n      <model_name>{n}</model_name>' for n in tracked_model_names)
        sdf = sdf[:m.end(1)] + extra + sdf[m.end(1):]

    with open(out_path, 'w') as f:
        f.write(sdf)
    return n_removed, len(actors)
