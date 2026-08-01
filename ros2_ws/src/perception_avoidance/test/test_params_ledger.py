"""params_ledger.yaml ↔ 코드 값 정합 (계획 v3.1.1 §6.3, C7·원칙 8).

대장이 죽은 문서가 되지 않도록 고정한다:
- binding 이 있는 레코드는 실제 코드 값(yaml 파라미터 / 모듈 상수 /
  GapRules 기본값)과 일치해야 한다 — 값을 바꾸면 대장도 갱신해야 실패하지 않는다.
- 스키마: status 는 §4.1 정의 집합, basis_tier 4(센서 사양 의존)는
  sensor_generation 필수 — 센서 세대 변경 시 전수 재계산 검색의 전제.
"""

import importlib
from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).parents[1] / 'config'
VALID_STATUS = {'CONFIRMED', 'PENDING', 'INVALID', 'PROVISIONAL', 'UNVERIFIED'}


def load_ledger():
    with open(CONFIG_DIR / 'params_ledger.yaml') as f:
        return yaml.safe_load(f)['parameters']


def resolve_binding(binding):
    """binding → {키: 실제 코드 값}."""
    if 'file' in binding:
        with open(CONFIG_DIR / binding['file']) as f:
            params = yaml.safe_load(f)[binding['node']]['ros__parameters']
        keys = binding['params'] if 'params' in binding else [binding['param']]
        return {k: params[k] for k in keys}
    mod = importlib.import_module(binding['module'])
    if 'const' in binding:
        return {binding['const']: getattr(mod, binding['const'])}
    obj = getattr(mod, binding['ctor'])()
    return {binding['attr']: getattr(obj, binding['attr'])}


def test_ledger_schema():
    problems = []
    for name, rec in load_ledger().items():
        if rec.get('status') not in VALID_STATUS:
            problems.append(f'{name}: status={rec.get("status")!r}')
        tier = rec.get('basis_tier')
        if tier not in (1, 2, 3, 4):
            problems.append(f'{name}: basis_tier={tier!r}')
        if tier == 4 and 'sensor_generation' not in rec:
            problems.append(f'{name}: tier 4 인데 sensor_generation 없음 (원칙 8)')
    assert not problems, '대장 스키마 위반:\n' + '\n'.join(problems)


def test_ledger_matches_code():
    mismatches = []
    for name, rec in load_ledger().items():
        binding = rec.get('binding')
        if binding is None:
            continue
        actual = resolve_binding(binding)
        expected = rec['value']
        if not isinstance(expected, dict):
            expected = {next(iter(actual)): expected}
        for key, exp in expected.items():
            act = actual[key]
            ok = (float(act) == pytest.approx(float(exp))
                  if isinstance(exp, (int, float)) else act == exp)
            if not ok:
                mismatches.append(f'{name}.{key}: 대장={exp!r} 코드={act!r}')
    assert not mismatches, '대장↔코드 드리프트:\n' + '\n'.join(mismatches)
