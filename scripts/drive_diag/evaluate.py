"""x·y·θ 오차 평가 — 구간 자동 분할(정지·전진·후진·회전)과 항목별 판정(좋음·주의·나쁨)·문장.

판정 기준은 THRESHOLDS 한 곳에 근거와 함께 둔다. 전부 [잠정](2026-11-01 run 몇 개 기준) — 데이터가 쌓이면 고친다.
입력 z 는 analyze_loc.py 캐시(npz 를 dict 로).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import locerr as le  # noqa: E402
import metrics  # noqa: E402

D = math.degrees

# (좋음 상한, 주의 상한) — 값이 작을수록 좋음. score 만 클수록 좋음(좋음 하한, 주의 하한).
THRESHOLDS = {
    'loc_lat_p95_cm': (5.0, 10.0, '도착 옆 허용 end_straight_lateral 10 cm — 위치 오차가 그 절반을 넘으면 주의'),
    'loc_yaw_p95_deg': (1.5, 3.0, '방향 1° 틀리면 1 m 당 1.7 cm 옆으로 감'),
    'score_pct': (75.0, 65.0, '11-01 정지 중 정합 78~80 %. 65 % 미만은 지도와 실제가 달라 ① 을 믿기 어려움'),
    'odom_lat_cm_per_m': (1.0, 3.0, '참고만(판정 안 함) — 라이다 잣대 영향이 섞여 EKF 도 1.4~1.7 cm/m 로 나옴 [미검증]'),
    'odom_yaw_deg_per_m': (0.1, 0.25, '방향 오차의 거리 기울기(직선 맞춤). 10 m 가면 옆으로 9 cm(0.1°/m) · 22 cm(0.25°/m)'),
    'wobble_deg': (3.5, 5.0, '직선 차체 방향 흔들림 폭(5~95 %). 맨바닥 2.4~3.5°, 카펫 6.4~7.2°(10-15~24 실측)'),
}
GRADE_KO = {'good': '좋음', 'warn': '주의', 'bad': '나쁨', None: '—'}


def grade(value, good, warn, higher_better=False):
    if value is None or not np.isfinite(value):
        return None
    if higher_better:
        return 'good' if value >= good else ('warn' if value >= warn else 'bad')
    return 'good' if value <= good else ('warn' if value <= warn else 'bad')


def worst(grades):
    order = {'good': 0, 'warn': 1, 'bad': 2}
    gs = [g for g in grades if g in order]
    return max(gs, key=order.get) if gs else None


def _g(key, value):
    good, warn, _ = THRESHOLDS[key]
    return grade(value, good, warn, higher_better=(key == 'score_pct'))


# ── 구간 자동 분할 ───────────────────────────────────────

def segment_motion(t_v, vx, t_g, gz, dt=0.05, smooth=1.0, v_th=0.1, w_th=0.15, min_len=1.5):
    """정지·전진·후진·회전으로 나눈다(1 s 평균). min_len 보다 짧은 조각은 앞 구간에 붙인다. 시각은 입력 그대로."""
    t = np.arange(max(t_v[0], t_g[0]), min(t_v[-1], t_g[-1]), dt)
    if len(t) < 2:
        return []
    k = max(1, int(round(smooth / dt)))
    sm = lambda x: np.convolve(x, np.ones(k) / k, 'same')
    v = sm(np.interp(t, t_v, vx))
    w = sm(np.interp(t, t_g, gz))
    lab = np.where(np.abs(w) >= w_th, '회전', np.where(v > v_th, '전진', np.where(v < -v_th, '후진', '정지')))
    runs, s = [], 0
    for i in range(1, len(t) + 1):
        if i == len(t) or lab[i] != lab[s]:
            runs.append([str(lab[s]), float(t[s]), float(t[i - 1] + dt)])
            s = i
    merged = []
    for r in runs:
        if merged and (r[2] - r[1] < min_len):
            merged[-1][2] = r[2]
        elif merged and merged[-1][0] == r[0]:
            merged[-1][2] = r[2]
        else:
            merged.append(r)
    if len(merged) > 1 and merged[0][2] - merged[0][1] < min_len:  # 맨 앞 조각은 뒤에 붙인다
        merged[1][1] = merged[0][1]
        merged.pop(0)
    out = []
    for r in merged:
        if out and out[-1]['kind'] == r[0]:
            out[-1]['t1'] = r[2]
        else:
            out.append(dict(kind=r[0], t0=r[1], t1=r[2]))
    return out


# ── 구간 평가 ────────────────────────────────────────────

def _loc_stats(z, a, b):
    if 'm_t' not in z or not len(z['m_t']):
        return None
    s = (z['m_t'] >= a) & (z['m_t'] <= b)
    ok = s & (z['m_edge'] < 0.5)
    if ok.sum() < 3:
        return dict(n=int(s.sum()), edge=int(s.sum() - ok.sum())) if s.sum() else None
    lat, fwd, yaw = z['m_lat'][ok] * 100, z['m_fwd'][ok] * 100, D(1) * z['m_dyaw'][ok]
    return dict(n=int(s.sum()), edge=int(s.sum() - ok.sum()),
                lat_mean=float(lat.mean()), lat_p95=float(np.percentile(np.abs(lat), 95)),
                fwd_mean=float(fwd.mean()), yaw_mean=float(yaw.mean()), yaw_p95=float(np.percentile(np.abs(yaw), 95)),
                score_amcl=float(z['m_score_amcl'][ok].mean()), score_best=float(z['m_score_best'][ok].mean()))


def _odom_stats(z, a, b, key):
    """구간 [a, b] 의 오도메트리 오차. 판정은 방향 오차의 거리 기울기(°/m, 직선 맞춤) — 끝점 값은 구간 시작 기준 방향의
    ICP 잡음(키프레임 하나 ≈0.5°)에 끌려 11 m 에 옆 10 cm 씩 흔들리므로 표시만 한다."""
    tt, P = (z['odom_t'], z['odom']) if key == 'ekf' else (z.get('wheel_t'), z.get('wheel'))
    if tt is None:
        return None
    lt, lid = z['lid_t'], z['lid']
    sel = (lt >= a) & (lt <= b)
    if sel.sum() < 2:
        return None
    p = lid[sel]
    d = np.r_[0.0, np.cumsum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])))]
    e = le.odom_error(lt, lid, tt, P, a)
    s = e['t'] <= b
    yaw, lat = D(1) * e['dyaw'][s], e['lat'][s] * 100
    n = min(len(d), len(yaw))
    out = dict(dist=float(d[-1]), lat=float(lat[-1]), yaw=float(yaw[-1]), yaw_slope=None, lat_slope=None)
    if d[-1] >= 2.0 and n >= 5:
        out['yaw_slope'] = float(np.polyfit(d[:n], yaw[:n], 1)[0])
        out['lat_slope'] = float(np.polyfit(d[:n], lat[:n], 1)[0])
    return out


def _wobble(z, a, b):
    if 'gz' not in z or b - a < 4.0:
        return None
    t = np.arange(a, b, 0.05)
    g = np.interp(t, z['imu_t'], z['gz'])
    v = np.interp(t, z['vx_t'], z['vx'])
    if np.mean(np.abs(v) > 0.15) < 0.8 or np.mean(np.abs(g)) > 0.15:
        return None  # 직선 주행 구간에서만
    return metrics.wobble_amp_deg(np.cumsum(g) * 0.05, 0.05)


def solutions(items):
    """판정 결과 → 처방 목록 [{key, grade, title, why, actions}]. 순서 = 먼저 할 일(지도 → AMCL → EKF → 바퀴 → 흔들림).

    근거는 2026-11-01 진단(라이다 장착각·후진 치우침·카펫 흔들림)이다. 지도 정합이 나쁘면 ① 은 믿을 수 없어 따로 처방하지 않는다.
    """
    I = {i['key']: i for i in items}
    bad = lambda k: k in I and I[k]['grade'] in ('warn', 'bad')
    out = []
    if bad('score'):
        v = I['score']['values']
        out.append(dict(key='map', grade=I['score']['grade'], title='지도와 실제 공간이 다름',
                        why=f"지도 정합 점수 평균 {v['score']:.0f} % (좋음 ≥ 75 %, 65 % 미만이면 ① 신뢰 낮음)",
                        actions=['이 장소 지도를 지금 상태로 다시 만들기 — 라이다 장착각(−1.98°)·엔코더 시각을 고친 뒤라 더 정확하게 나옴',
                                 '가구·문·장애물처럼 바뀐 곳 확인 — 구간 표에서 지도 점수가 낮은 구간부터',
                                 '지도를 바꾸기 전까지 ① 위치추정 판정은 참고만']))
    if bad('loc') and not (('score' in I) and I['score']['grade'] == 'bad'):
        v = I['loc']['values']
        biased = abs(v.get('lat_mean', 0)) >= 3.0 or abs(v.get('yaw_mean', 0)) >= 1.0
        acts = ['멈춘 자리에서 초기 위치 다시 잡기(rviz2 2D Pose Estimate)' +
                (' — 평균이 한쪽으로 치우쳐 AMCL 이 틀린 자리를 믿는 중' if biased else ''),
                '라이다 장착각 확인 — laser_yaw.py drive 의 잔여(실측 − TF)가 0.3° 를 넘으면 재측정',
                '계속되면 AMCL 센서 모델(sigma_hit·z_hit·z_rand) 점검 — 한 번에 하나씩, docs/nav2_backlog.md §11 먼저']
        if v.get('edge', 0) > 0.2 * max(v.get('n', 1), 1):
            acts.insert(0, '탐색 범위 밖이 많음 — AMCL 이 25 cm·6° 넘게 틀렸거나 위치를 잃음, 초기 위치부터 다시')
        out.append(dict(key='loc', grade=I['loc']['grade'], title='AMCL 위치가 실제와 어긋남',
                        why=(f"|옆| 95 % {v['lat_p95']:.1f} cm · |θ| 95 % {v['yaw_p95']:.2f}° "
                             f"(평균 옆 {v.get('lat_mean', 0):+.1f} cm · θ {v.get('yaw_mean', 0):+.2f}°)"), actions=acts))
    if bad('odom_ekf'):
        v = I['odom_ekf']['values']
        out.append(dict(key='odom_ekf', grade=I['odom_ekf']['grade'], title='EKF 방향이 샘(자이로 쪽)',
                        why=f"EKF 방향 표류 {v['yaw_slope']:+.2f}°/m (좋음 ≤ 0.1°/m)",
                        actions=['정지 상태 자이로 편향 확인 — 10-20 기준 ±13°/h 안',
                                 'IMU 어댑터의 정지 보정(ZUPT)이 동작하는지 확인',
                                 '카메라 마스트 떨림이 자이로에 섞이는지 확인 — 카펫에서 자이로 흔들림이 바퀴보다 컸음']))
    if bad('odom_wheel'):
        v = I['odom_wheel']['values']
        ekf = I.get('odom_ekf', {}).get('values', {}).get('yaw_slope')
        out.append(dict(key='odom_wheel', grade=I['odom_wheel']['grade'], title='바퀴만으로는 방향이 샘(미끄럼 회전)',
                        why=(f"바퀴만 방향 표류 {v['yaw_slope']:+.2f}°/m" +
                             (f" · EKF {ekf:+.2f}°/m" if ekf is not None else '') + ' (좋음 ≤ 0.1°/m)'),
                        actions=['캐스터: 손으로 돌려 스위블·굴림이 좌우 같은지, 이물·한쪽 들림 확인',
                                 '구동 타이어 마모·공기압과 축 유격 확인',
                                 '바닥에 끌리는 케이블, 한쪽으로 쏠린 짐 확인',
                                 'Nav2 주행은 자이로가 방향을 잡아 영향이 작음 — teleop·곧게 가기에서 치우침으로 보임']))
    if bad('wobble'):
        v = I['wobble']['values']
        out.append(dict(key='wobble', grade=I['wobble']['grade'], title='직선 주행 좌우 흔들림',
                        why=f"차체 방향 흔들림 폭 {v['amp']:.1f}° (맨바닥 2.4~3.5°, 카펫 6.4~7.2°)",
                        actions=['캐스터 — 카펫에서 스위블이 늦고 굴림 저항이 큼',
                                 '구동륜 하중·무게 배분 — 짐을 실으면 덜 흔들림(사용자 관찰)',
                                 '카펫용 컨트롤러 설정을 환경 프로필로 — docs/nav2_backlog.md §11 먼저, 한 번에 하나씩',
                                 '모터 드라이버 속도 루프 설정(제조사 매뉴얼)']))
    return out


def evaluate_range(z, a_rel, b_rel):
    """상대 시각 [a, b] 구간의 항목별 판정과 문장."""
    T0 = float(z['odom_t'][0])
    a, b = T0 + a_rel, T0 + b_rel
    items = []
    L = _loc_stats(z, a, b)
    sc_grade = None
    if L and 'score_best' in L:
        sc_grade = _g('score_pct', L['score_best'])
        items.append(dict(key='score', label='지도 정합', grade=sc_grade, values=dict(score=L['score_best']),
                          text=f"지도 정합 점수 평균 {L['score_best']:.0f} % (AMCL 자세 {L['score_amcl']:.0f} %)"
                               + (' — 지도와 실제가 달라 ① 결과 신뢰 낮음' if sc_grade == 'bad' else '')))
        g = worst([_g('loc_lat_p95_cm', L['lat_p95']), _g('loc_yaw_p95_deg', L['yaw_p95'])])
        txt = (f"AMCL 위치 오차: 옆 평균 {L['lat_mean']:+.1f} cm (95 % {L['lat_p95']:.1f}) · 앞 평균 {L['fwd_mean']:+.1f} cm · "
               f"방향 평균 {L['yaw_mean']:+.2f}° (95 % {L['yaw_p95']:.2f}) · 대조 {L['n']}장(범위 밖 {L['edge']})")
        if sc_grade == 'bad':
            txt += ' — 지도 정합이 낮아 신뢰 낮음'
        items.append(dict(key='loc', label='① 위치추정', grade=g, values=L, text=txt))
    elif L:
        items.append(dict(key='loc', label='① 위치추정', grade=None, values=L,
                          text=f"지도 대조 {L['n']}장 중 범위 밖 {L['edge']}장 — 판정 불가"))
    for key, name, short in (('ekf', 'EKF(바퀴 거리 + 자이로 방향)', '② EKF'), ('wheel', '바퀴만', '② 바퀴만')):
        O = _odom_stats(z, a, b, key)
        if not O:
            continue
        g = _g('odom_yaw_deg_per_m', abs(O['yaw_slope'])) if O['yaw_slope'] is not None else None
        per = (f" — 방향 표류 {O['yaw_slope']:+.2f}°/m · 옆 {O['lat_slope']:+.1f} cm/m(참고)" if O['yaw_slope'] is not None
               else ' (2 m 미만이라 판정 안 함)')
        items.append(dict(key='odom_' + key, label=short, grade=g, values=O,
                          text=f"{name} 오도메트리: 라이다 {O['dist']:.1f} m 동안 구간 끝 옆 {O['lat']:+.1f} cm · 방향 {O['yaw']:+.2f}°{per}"))
    w = _wobble(z, a, b)
    if w is not None:
        items.append(dict(key='wobble', label='흔들림', grade=_g('wobble_deg', w), values=dict(amp=w),
                          text=f"차체 방향 흔들림 폭 {w:.1f}° (맨바닥 2.4~3.5°, 카펫 6.4~7.2°)"))
    return dict(t0=a_rel, t1=b_rel, grade=worst([i['grade'] for i in items]), items=items, solutions=solutions(items))


def evaluate_bag(z):
    """구간 자동 분할 + 구간별 판정 표 + 전체 요약."""
    T0 = float(z['odom_t'][0])
    g_t, g_w = (z['imu_t'], z['gz']) if 'gz' in z else (z['odom_t'], np.zeros(len(z['odom_t'])))
    rows = []
    for s in segment_motion(z['vx_t'], z['vx'], g_t, g_w):
        a, b = s['t0'] - T0, s['t1'] - T0
        r = evaluate_range(z, a, b)
        cell = {i['key']: i for i in r['items']}
        rows.append(dict(kind=s['kind'], t0=round(a, 2), t1=round(b, 2), grade=r['grade'],
                         cells={k: dict(grade=v['grade'], values=v['values']) for k, v in cell.items()}))
    return dict(segments=rows, overall=evaluate_range(z, 0.0, float(z['odom_t'][-1] - T0)),
                thresholds={k: dict(good=v[0], warn=v[1], basis=v[2]) for k, v in THRESHOLDS.items()},
                grade_ko=GRADE_KO)
