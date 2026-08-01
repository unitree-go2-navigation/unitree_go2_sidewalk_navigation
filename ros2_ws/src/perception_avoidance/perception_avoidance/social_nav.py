"""Phase 5a gap 판단 순수 함수 — ROS 비의존 (pytest 직접 실행).

좌표는 base_link 횡축 y (REP-103: +y=좌측). 우측 통과 관습 = 낮은 y 선호.
근거 수치 (계획 v2 리서치 노트 — Kirby '10, Pacchierotti '06, Hall proxemics):
Go2 유효폭 0.31 기준 gap 시도 하한 0.7m / 기본(사람 관여) 1.0m / 이동 보행자
1.2m. 통과 시 사람 측면 여유 ≥0.45m + 속도 ≤0.3m/s. 정적 장애물-보행자
사이는 정적 쪽 밀착(보행자 여유 최대화).
"""

from collections import namedtuple

# kind: 'static' | 'person' | 'person_moving' (경계는 gap 쪽에서 'edge')
Blocker = namedtuple('Blocker', 'y_min y_max kind')
# lo_kind/hi_kind: y_min/y_max 쪽 경계의 종류 ('edge' 포함)
Gap = namedtuple('Gap', 'y_min y_max lo_kind hi_kind')

_KIND_PRIORITY = {'static': 0, 'person': 1, 'person_moving': 2}


class GapRules:
    """gap 판단 파라미터. 기본값은 계획 v2 확정 수치."""

    def __init__(self, robot_width=0.31, attempt_min=0.8, default_min=1.0,
                 moving_min=1.2, person_margin=0.45, static_margin=0.25,
                 edge_margin=0.30, pass_speed_cap=0.3):
        self.robot_width = robot_width
        # 0.8 (설계검토, 0.7→0.8): 0.7 squeeze는 몸 측면이 코리도 게이트
        # (corridor_half 0.335)와 클러스터 노이즈 폭 안에서 물려 STOP 채터링.
        self.attempt_min = attempt_min      # 정적-정적 squeeze 하한
        self.default_min = default_min      # 정지 보행자 관여 gap 하한
        self.moving_min = moving_min        # 이동 보행자 관여 gap 하한
        self.person_margin = person_margin  # 사람 측면 여유 (단일 통과 시)
        # 0.25 (0.10→0.20→0.25): 시스템 자신의 횡 불확실 예산(0.18)+박스
        # 클러스터 폭 과소측정 — 대각 박스 8.8cm 스침 실측 후 상향.
        self.static_margin = static_margin  # 정적 밀착 시 최소 여유
        # polygon 경계는 물리 미확인(연석/펜스 가능) + odom 드리프트 + 요 sway
        # 지렛대 오차를 흡수해야 하므로 정적 물체보다 보수적으로.
        self.edge_margin = edge_margin
        # 사람 여유의 comfort 목표: 공간이 있으면 이만큼, 없으면 하드
        # person_margin(0.45)까지 양보. '정적 밀착 최대화'를 모든 gap에
        # 적용하면 넓은 gap에서 과도 우회 (프로브 실측: 필요 0.4m에 0.9m).
        self.person_comfort = 0.60
        self.pass_speed_cap = pass_speed_cap


def merge_blockers(blockers):
    """겹치거나 맞닿은 blocker 구간 병합. kind는 우선순위 높은 쪽 유지."""
    items = sorted((b for b in blockers if b.y_max > b.y_min),
                   key=lambda b: b.y_min)
    merged = []
    for b in items:
        if merged and b.y_min <= merged[-1].y_max:
            prev = merged[-1]
            kind = max(prev.kind, b.kind, key=lambda k: _KIND_PRIORITY[k])
            merged[-1] = Blocker(prev.y_min, max(prev.y_max, b.y_max), kind)
        else:
            merged.append(b)
    return merged


def compute_gaps(blockers, y_lo, y_hi):
    """[y_lo, y_hi] 밴드 안의 gap 목록. 밴드 경계 쪽은 kind='edge'."""
    clipped = [Blocker(max(b.y_min, y_lo), min(b.y_max, y_hi), b.kind)
               for b in blockers
               if b.y_max > y_lo and b.y_min < y_hi]
    merged = merge_blockers(clipped)
    gaps = []
    cursor, cursor_kind = y_lo, 'edge'
    for b in merged:
        if b.y_min > cursor:
            gaps.append(Gap(cursor, b.y_min, cursor_kind, b.kind))
        cursor, cursor_kind = b.y_max, b.kind
    if y_hi > cursor:
        gaps.append(Gap(cursor, y_hi, cursor_kind, 'edge'))
    return gaps


def classify_gap(gap, rules):
    """(통과 가능, 목표 y, 속도 상한|None). 불가면 (False, None, None).

    목표 y 배치 규칙 (최소 이탈 원칙):
    - 사람|정적(또는 경계): 사람에게서 comfort(0.6) 여유 지점 — 공간이
      부족하면 반대쪽 margin까지 밀착(정적 밀착)하되 하드 person_margin
      (0.45)은 항상 우선. 넓은 gap에서 반대편 최대 밀착은 과도 우회.
    - 사람|사람: 중앙 (양쪽 균등이 최선)
    - 정적/경계만: 중앙. 빈 밴드(edge|edge)는 무편향(0) — 관습적 우측
      편향은 측 선택 동률에서만 적용 (게이트 몫).
    """
    w = gap.y_max - gap.y_min
    lo_person = gap.lo_kind in ('person', 'person_moving')
    hi_person = gap.hi_kind in ('person', 'person_moving')
    if 'person_moving' in (gap.lo_kind, gap.hi_kind):
        required = rules.moving_min
    elif lo_person or hi_person:
        required = rules.default_min
    else:
        required = rules.attempt_min
    if w < required:
        return False, None, None

    def side_margin(kind):
        return rules.edge_margin if kind == 'edge' else rules.static_margin

    half = rules.robot_width / 2.0
    if lo_person and hi_person:
        target = (gap.y_min + gap.y_max) / 2.0
    elif lo_person:
        # 사람이 y_min 쪽 — comfort 여유 지점, 반대쪽 margin 한도 내,
        # 하드 person_margin은 항상 우선
        target = min(gap.y_min + rules.person_comfort + half,
                     gap.y_max - side_margin(gap.hi_kind) - half)
        target = max(target, gap.y_min + rules.person_margin + half)
    elif hi_person:
        target = max(gap.y_max - rules.person_comfort - half,
                     gap.y_min + side_margin(gap.lo_kind) + half)
        target = min(target, gap.y_max - rules.person_margin - half)
    elif gap.lo_kind == 'edge' and gap.hi_kind == 'edge':
        target = 0.0                # 빈 밴드 — 무편향 (사행 방지)
    else:
        target = (gap.y_min + gap.y_max) / 2.0

    slow = lo_person or hi_person or w < rules.default_min
    return True, target, rules.pass_speed_cap if slow else None


def choose_gap(gaps, rules):
    """통과 가능한 gap 중 우측 통과 관습(목표 y 최소) 선택.

    반환: (gap, target_y, speed_cap) 또는 None (전부 불가 → YIELD 후보).
    """
    best = None
    for g in gaps:
        ok, target, cap = classify_gap(g, rules)
        if ok and (best is None or target < best[1]):
            best = (g, target, cap)
    return best


def _clip_halfplane(pts, inside, intersect):
    """Sutherland-Hodgman 한 단계: inside(p) 판정과 교점 함수로 클리핑."""
    out = []
    for i, cur in enumerate(pts):
        prev = pts[i - 1]
        cur_in, prev_in = inside(cur), inside(prev)
        if cur_in:
            if not prev_in:
                out.append(intersect(prev, cur))
            out.append(cur)
        elif prev_in:
            out.append(intersect(prev, cur))
    return out


def lateral_band(poly_pts, x0, x1):
    """볼록 polygon(base_link [(x,y),...])을 x∈[x0,x1] 스트립으로 클리핑한
    영역의 (y_min, y_max). 스트립과 겹치지 않으면 None."""
    def cut(pts, xb, keep_ge):
        sign = 1.0 if keep_ge else -1.0
        return _clip_halfplane(
            pts,
            lambda p: sign * (p[0] - xb) >= 0.0,
            lambda a, b: (xb, a[1] + (b[1] - a[1])
                          * (xb - a[0]) / (b[0] - a[0])))
    pts = cut(list(poly_pts), x0, True)
    if len(pts) >= 3:
        pts = cut(pts, x1, False)
    if len(pts) < 3:
        return None
    ys = [p[1] for p in pts]
    return min(ys), max(ys)


def classify_kind(moving_now, ever_moved, bbox_top_z, compact):
    """track → blocker kind. 보수 원칙: 애매하면 사람 쪽으로.

    - 확장 구조물(비컴팩트)은 이동 판정 무관 'static' — 진행 방향과 평행한
      울타리/벽은 가림 경계가 로봇과 함께 슬라이딩해 상대속도 ≈0, 지상
      속도 ≈ 로봇 속도의 유사 이동체로 보인다 (프로브 실측: moving 킬스위치
      영구 래치). P4 fast_max_extent와 동일 원리. 사람/자전거는 컴팩트.
    - 현재 이동 중 + 키 큼 → 'person_moving' (YIELD 대상은 사람 형상만 —
      낮은 track의 부트스트랩 속도 스파이크가 person_moving이 되면 박스
      상대 YIELD 오발, narrow_gap_refuse 실측)
    - 과거에 이동했던 track(멈춘 보행자) 또는 키 큰 정지 클러스터 →
      'person' (사람/입간판 구분 불가 — 사람 취급이 마진만 늘리는 안전 방향)
    - 그 외(낮은 박스 — 이동 스파이크 포함) → 'static'
    """
    if not compact:
        return 'static'
    if moving_now and bbox_top_z:
        return 'person_moving'
    if ever_moved or bbox_top_z:
        return 'person'
    return 'static'


def blockers_from_detections(dets, x_min, x_max, inflate):
    """전방 창과 겹치는 탐지 → 횡방향 Blocker 목록.

    dets: (cx, cy, sx, sy, kind) — 게이트가 kind 분류를 마친 평면 튜플.
    inflate: 클러스터 edge 과소추정 보상 (양쪽 각각).
    """
    out = []
    for cx, cy, sx, sy, kind in dets:
        if cx + 0.5 * sx < x_min or cx - 0.5 * sx > x_max:
            continue
        out.append(Blocker(cy - 0.5 * sy - inflate,
                           cy + 0.5 * sy + inflate, kind))
    return out
