"""주행 흔들림·치우침 판정 계산(순수 numpy·scipy). bag 읽기와 화면은 bagio.py·serve.py.

부호: 회전·옆은 + = 왼쪽(반시계). 각도 출력은 °, 각속도는 rad/s.
배경: devlog·메모 2026-11-01 직진 치우침·흔들림 분석(run97~111). 기준값은 REF 에 출처와 함께 둔다.
"""
import math

import numpy as np
from scipy.signal import butter, filtfilt, welch

WHEEL_RADIUS_M = 0.065   # vica_localization/config/encoder.yaml · motor_core.hpp 와 같은 값
WHEEL_BASE_M = 0.37
HAND_ARM_M = 0.415       # 손 횡속도 = |w| × 0.415 (devlog 2026-08-29)


# ── 기본 신호 ──────────────────────────────────────────────

def quantized_w(v, w, radius=WHEEL_RADIUS_M, base=WHEEL_BASE_M):
    """모터 노드(motor_core.hpp 485-498)가 실제로 보내는 좌우 정수 rpm 으로 환산한 각속도.

    np.round 는 파이썬 round 와 같이 .5 를 짝수 쪽으로 보낸다(py_round).
    """
    circ = 2 * math.pi * radius
    rr = np.round((np.asarray(v) + np.asarray(w) * base / 2) / circ * 60)
    rl = np.round((np.asarray(v) - np.asarray(w) * base / 2) / circ * 60)
    return (rr - rl) * circ / 60 / base


def movavg(x, n):
    n = max(1, int(n))
    return np.convolve(x, np.ones(n) / n, 'same')


def runs_of(mask, t, min_len):
    """mask 가 참인 연속 구간 중 길이(초) ≥ min_len 인 (첫 인덱스, 마지막 인덱스)."""
    out, s = [], None
    for i, on in enumerate(mask):
        if on and s is None:
            s = i
        if (not on or i == len(mask) - 1) and s is not None:
            e = i if on else i - 1
            if t[e] - t[s] >= min_len:
                out.append((s, e))
            s = None
    return out


def straight_segments(t, vx, gz, v_min=0.2, w_max=0.15, min_len=4.0, smooth_s=1.0):
    """직선 주행 구간: smooth_s 평균 vx > v_min, |자이로 wz| < w_max 가 min_len 초 이상."""
    dt = float(np.median(np.diff(t)))
    k = round(smooth_s / dt)
    mask = (movavg(vx, k) > v_min) & (np.abs(movavg(gz, k)) < w_max)
    return runs_of(mask, t, min_len)


def highpass(x, dt, fc=0.15):
    b, a = butter(2, fc * 2 * dt, 'high')
    return filtfilt(b, a, x)


def wobble_amp_deg(yaw, dt, fc=0.15, trim_s=1.0):
    """방향 흔들림 폭(°): fc 고역통과한 방향의 5~95 % 폭. 끝 trim_s 초는 필터 과도분이라 뺀다."""
    h = highpass(np.asarray(yaw, float), dt, fc)
    k = int(trim_s / dt)
    if len(h) > 2 * k + 10:
        h = h[k:-k]
    return math.degrees(np.percentile(h, 95) - np.percentile(h, 5))


def band_rms(x, dt, lo, hi):
    """lo~hi Hz 대역 RMS(Welch)."""
    x = np.asarray(x, float)
    f, p = welch(x - x.mean(), fs=1 / dt, nperseg=min(len(x), int(8 / dt)))
    sel = (f >= lo) & (f < hi)
    if sel.sum() < 2:
        return float('nan')
    return math.sqrt(np.trapezoid(p[sel], f[sel]))


def fit_delay_gain(cmd, y, dt, max_lag_s=0.6):
    """y ≈ 이득 × cmd(t − 지연) + 상수. 잔차 표준편차가 가장 작은 지연을 고른다 → (지연 s, 이득, 잔차)."""
    best = None
    for lag in range(int(max_lag_s / dt) + 1):
        x = cmd[:len(cmd) - lag] if lag else cmd
        yy = y[lag:]
        X = np.c_[x, np.ones(len(x))]
        coef, *_ = np.linalg.lstsq(X, yy, rcond=None)
        r = float(np.std(yy - X @ coef))
        if best is None or r < best[2]:
            best = (lag * dt, float(coef[0]), r)
    return best


# ── 위치추정 ─────────────────────────────────────────────

def _compose(row, x, y, yaw):
    c, s = math.cos(row[3]), math.sin(row[3])
    return row[1] + c * x - s * y, row[2] + s * x + c * y, row[3] + yaw


def amcl_corrections(mo, ekf, max_jump=0.5):
    """map→odom 이 바뀐 순간마다 로봇 자리에서 본 보정량. 옆(+ 왼쪽)·앞·방향.

    mo: (t, x, y, yaw, …) map→odom, ekf: (t, x, y, yaw, …) /odom.
    """
    if len(mo) < 2:
        return []
    eyaw = np.unwrap(ekf[:, 3])
    ch = np.where((np.abs(np.diff(mo[:, 1])) > 1e-6) | (np.abs(np.diff(mo[:, 2])) > 1e-6)
                  | (np.abs(np.diff(mo[:, 3])) > 1e-7))[0] + 1
    out = []
    for k in ch:
        tc = mo[k, 0]
        if tc < ekf[0, 0] or tc > ekf[-1, 0]:
            continue
        p = (float(np.interp(tc, ekf[:, 0], ekf[:, 1])), float(np.interp(tc, ekf[:, 0], ekf[:, 2])),
             float(np.interp(tc, ekf[:, 0], eyaw)))
        bx, by, byaw = _compose(mo[k - 1], *p)
        ax, ay, ayaw = _compose(mo[k], *p)
        dx, dy = ax - bx, ay - by
        if math.hypot(dx, dy) > max_jump:
            continue
        out.append(dict(t=tc, side=-dx * math.sin(byaw) + dy * math.cos(byaw),
                        fwd=dx * math.cos(byaw) + dy * math.sin(byaw),
                        dyaw=math.atan2(math.sin(ayaw - byaw), math.cos(ayaw - byaw)), heading=byaw))
    return out


def heading_name(yaw):
    return ('동', '북', '서', '남')[int((math.degrees(yaw) + 45) % 360 // 90)]


# ── 명령 ─────────────────────────────────────────────────

def open_loop_segments(cmd, min_len=2.0, v_min=0.1, max_gap=0.3):
    """명령 w 가 정확히 0 이고 v ≥ v_min 인 연속 구간(teleop·곧게 가기). cmd: (t, v, w). → [(t0, t1)]."""
    t = cmd[:, 0]
    ok = (np.abs(cmd[:, 2]) < 1e-9) & (cmd[:, 1] >= v_min)
    out, s = [], None
    for i in range(len(t)):
        brk = (not ok[i]) or (s is not None and t[i] - t[i - 1] > max_gap)
        if brk and s is not None:
            if t[i - 1] - t[s] >= min_len:
                out.append((float(t[s]), float(t[i - 1])))
            s = None
        if ok[i] and s is None:
            s = i
    if s is not None and t[-1] - t[s] >= min_len:
        out.append((float(t[s]), float(t[-1])))
    return out


# ── 판정 ─────────────────────────────────────────────────

def verdict_c1(cmd_amp, gyro_amp, gyro_high=4.5, ratio_high=2.0, cmd_high=3.0):
    """명령이 흔드나 차체가 흔드나."""
    ratio = gyro_amp / max(cmd_amp, 0.1)
    if gyro_amp >= gyro_high and ratio >= ratio_high:
        who, text = 'body', '차체 쪽 — 같은 명령에 차체가 더 흔들림(바퀴·바닥·하중을 볼 것)'
    elif cmd_amp >= cmd_high:
        who, text = 'command', '명령 쪽 — 명령 자체가 크게 흔들림(컨트롤러·위치추정·센서를 볼 것)'
    else:
        who, text = 'normal', '정상 범위'
    return dict(who=who, ratio=ratio, text=text)


def verdict_c5(by_dir, min_dist=5.0, min_abs=0.5):
    """AMCL 옆 보정(cm/m) 의 주행 방향별 부호. by_dir: {방향: (cm/m, 거리 m)}."""
    vals = [v for v, d in by_dir.values() if d >= min_dist]
    if len(vals) < 2:
        return dict(kind='unknown', text='방향 2개 이상(각 5 m 이상) 필요')
    if max(abs(v) for v in vals) < min_abs:
        return dict(kind='none', text='옆 보정 작음')
    if all(v > 0 for v in vals) or all(v < 0 for v in vals):
        side = '왼' if vals[0] > 0 else '오른'
        return dict(kind='robot', text=f'로봇에 붙은 오차 — 어느 방향으로 가도 {side}쪽으로 보정(라이다 장착각·구동축 정렬 의심)')
    return dict(kind='place', text='장소에 붙은 오차 — 방향 따라 부호 반대(바닥 기울기·카펫 결·지도 왜곡 의심)')


# ── 구간 점검 1~5 ───────────────────────────────────────

# 기준값: 도구 자신의 계산으로 기준 run 에서 잰 값(README 의 '기준값 다시 재기'). [잠정] run 7개.
REF = {
    'note': '맨바닥 = run97(10-15)·run105(10-23), 카펫 = run107·109·110(10-24). 판정 문턱은 두 무리 사이 [잠정]',
}


def _grid(d, dt=0.05):
    w, e, imu, cmd = d['wheel'], d['ekf'], d['imu'], d['cmd']
    off = float(np.median(w[:, 6] - w[:, 0])) if w.shape[1] > 6 else 0.0
    ct = cmd[:, 0] - off
    t = np.arange(max(w[0, 0], e[0, 0], imu[0, 0], ct[0]), min(w[-1, 0], e[-1, 0], imu[-1, 0], ct[-1]), dt)
    ci = np.clip(np.searchsorted(ct, t + 1e-6, side='right') - 1, 0, len(ct) - 1)  # 부동소수 오차로 한 칸 앞 명령을 집지 않게
    stale = (t - ct[ci]) > 0.5
    cv, cw = cmd[ci, 1].copy(), cmd[ci, 2].copy()
    cv[stale] = 0.0
    cw[stale] = 0.0
    g = dict(t=t, dt=dt, off=off, ct=ct, cv=cv, cw=cw, wq=quantized_w(cv, cw),
             vx=np.interp(t, e[:, 0], e[:, 4]), gz=np.interp(t, imu[:, 0], imu[:, 1]),
             wyaw=np.interp(t, w[:, 0], np.unwrap(w[:, 3])), eyaw=np.interp(t, e[:, 0], np.unwrap(e[:, 3])),
             ex=np.interp(t, e[:, 0], e[:, 1]), ey=np.interp(t, e[:, 0], e[:, 2]))
    g['gyaw'] = np.cumsum(g['gz']) * dt
    g['cyaw'] = np.cumsum(cw) * dt
    mo = d.get('mapodom')
    if mo is not None and len(mo) > 10:
        mi = np.clip(np.searchsorted(mo[:, 0], t, side='right') - 1, 0, len(mo) - 1)
        c, s = np.cos(mo[mi, 3]), np.sin(mo[mi, 3])
        g['X'] = mo[mi, 1] + c * g['ex'] - s * g['ey']
        g['Y'] = mo[mi, 2] + s * g['ex'] + c * g['ey']
        g['myaw'] = mo[mi, 3] + g['eyaw']
        g['has_map'] = bool(t[-1] > mo[0, 0])
    else:
        g['X'], g['Y'], g['myaw'], g['has_map'] = g['ex'], g['ey'], g['eyaw'], False
    k = int(0.25 / dt)
    ww = np.zeros_like(t)
    ww[k:-k] = (g['wyaw'][2 * k:] - g['wyaw'][:-2 * k]) / (2 * k * dt)
    g['wwz'] = ww
    return g


def _segs_in_range(g, t0, t1, min_len):
    t = g['t']
    segs = straight_segments(t, g['vx'], g['gz'], min_len=min_len)
    lo = 0 if t0 is None else int(np.searchsorted(t, t0))
    hi = len(t) - 1 if t1 is None else int(np.searchsorted(t, t1)) - 1
    out = []
    for a, b in segs:
        a, b = max(a, lo), min(b, hi)
        if b > a and t[b] - t[a] >= min_len:
            out.append((a, b))
    return out


def _pooled_fit(segs, x, y, dt, max_lag_s=0.6):
    best = None
    for lag in range(int(max_lag_s / dt) + 1):
        xs = np.concatenate([x[a:b + 1 - lag] if lag else x[a:b + 1] for a, b in segs])
        ys = np.concatenate([y[a + lag:b + 1] for a, b in segs])
        X = np.c_[xs, np.ones(len(xs))]
        coef, *_ = np.linalg.lstsq(X, ys, rcond=None)
        r = float(np.std(ys - X @ coef))
        if best is None or r < best[2]:
            best = (lag * dt, float(coef[0]), r, float(np.std(ys)))
    return best


def _event_response(g, segs, events, before=1.0, after=2.0):
    """사건 시각마다 부호 s 를 곱한 명령 w 의 +0.2~1.0 s 평균(rad/s)."""
    t, dt, cw = g['t'], g['dt'], g['cw']
    nb, na = int(before / dt), int(after / dt)
    inside = np.zeros(len(t), bool)
    for a, b in segs:
        inside[a:b + 1] = True
    vals = []
    for te, s in events:
        i = int(np.searchsorted(t, te))
        if i - nb < 0 or i + na >= len(t) or not inside[i - nb:i + na].all():
            continue
        vals.append(s * cw[i + int(0.2 / dt):i + int(1.0 / dt)].mean())
    if len(vals) < 5:
        return dict(n=len(vals), mean=None, se=None)
    return dict(n=len(vals), mean=float(np.mean(vals)), se=float(np.std(vals) / math.sqrt(len(vals))))


def run_checks(d, t0=None, t1=None, seg_min=4.0):
    """구간 [t0, t1](센서 시각, None = 전체) 의 직선 구간으로 점검 1~5 를 계산한다."""
    g = _grid(d)
    t, dt = g['t'], g['dt']
    segs = _segs_in_range(g, t0, t1, seg_min)
    res = dict(range=[float(t[0] if t0 is None else t0), float(t[-1] if t1 is None else t1)], ref=REF,
               has_map=g['has_map'], segments=[])
    if not segs:
        res['empty'] = f'직선 구간 없음(v > 0.2 m/s, |wz| < 0.15 rad/s 가 {seg_min:.0f} s 이상)'
        return res
    dist = lambda a, b: float(np.sum(np.hypot(np.diff(g['ex'][a:b + 1]), np.diff(g['ey'][a:b + 1]))))
    rows = []
    for a, b in segs:
        L = dist(a, b)
        amp = {k: wobble_amp_deg(g[k][a:b + 1], dt) for k in ('cyaw', 'wyaw', 'gyaw')}
        hw = highpass(g['wyaw'][a:b + 1], dt)
        hg = highpass(g['gyaw'][a:b + 1], dt)
        rows.append(dict(a=a, b=b, t0=float(t[a]), t1=float(t[b]), dist=L, v=float(np.median(g['vx'][a:b + 1])),
                         heading=heading_name(float(np.median(g['myaw'][a:b + 1]))), amp=amp,
                         corr=float(np.corrcoef(hw, hg)[0, 1]),
                         band=[band_rms(g['gz'][a:b + 1], dt, lo, hi) for lo, hi in ((0.2, 1), (1, 3), (3, 10))],
                         wb=(g['wyaw'][b] - g['wyaw'][a]) - g['wq'][a:b + 1].sum() * dt,
                         gw=(g['gyaw'][b] - g['gyaw'][a]) - (g['wyaw'][b] - g['wyaw'][a]),
                         cmean=g['cyaw'][b] - g['cyaw'][a]))
    L = sum(r['dist'] for r in rows)
    med = lambda f: float(np.median([f(r) for r in rows]))
    res['segments'] = [dict(t0=r['t0'], t1=r['t1'], dist=r['dist'], v=r['v'], heading=r['heading'],
                            cmd_amp=r['amp']['cyaw'], wheel_amp=r['amp']['wyaw'], gyro_amp=r['amp']['gyaw'],
                            bias=math.degrees(r['wb']) / r['dist']) for r in rows]

    # 1 명령 vs 차체
    c1 = dict(cmd_amp=med(lambda r: r['amp']['cyaw']), wheel_amp=med(lambda r: r['amp']['wyaw']),
              gyro_amp=med(lambda r: r['amp']['gyaw']), n_seg=len(rows), dist_m=L,
              v=float(np.median([r['v'] for r in rows])))
    c1.update(verdict_c1(c1['cmd_amp'], c1['gyro_amp']))
    c1['per_m'] = 1.0 / max(c1['v'] * 2.5, 1e-6)  # 주기 ≈2.5 s 가정 시 1 m 당 흔들림 횟수(참고)
    res['c1'] = c1

    # 2 차체 쪽: 바퀴 vs 자이로, 대역, 지연
    fw = _pooled_fit(segs, movavg(g['wq'], 11), g['wwz'], dt)  # 홀수 창 = 정확히 가운데
    fg = _pooled_fit(segs, movavg(g['wq'], 11), movavg(g['gz'], 11), dt)
    band = {k: med(lambda r, i=i: r['band'][i]) for i, k in enumerate(('0.2-1', '1-3', '3-10'))}
    c2 = dict(corr=med(lambda r: r['corr']),
              gyro_over_wheel=med(lambda r: r['amp']['gyaw'] / max(r['amp']['wyaw'], 1e-3)), band=band,
              delay_wheel=fw[0], gain_wheel=fw[1], resid_wheel=fw[2], delay_gyro=fg[0], gain_gyro=fg[1], resid_gyro=fg[2])
    notes = []
    notes.append('바퀴 틱도 같이 흔들림 → 바퀴 단계(드라이버·접지·하중)' if c2['corr'] >= 0.8
                 else '바퀴와 자이로 흔들림이 따로 놂 → 미끄럼·마스트 떨림 또는 바퀴 틱 분해능')
    if c2['gyro_over_wheel'] >= 1.5:
        notes.append('자이로가 바퀴보다 1.5배 이상 → 미끄럼·마스트 떨림 몫 큼')
    if band['1-3'] + band['3-10'] > 0.03:
        notes.append('1 Hz 이상 떨림 큼 → 기구 진동(마스트·캐스터) 확인')
    c2['text'] = ' · '.join(notes)
    res['c2'] = c2

    # 3 명령 쪽: 무엇이 명령을 흔드나
    cw = g['cw']
    flips = []
    for a, b in segs:
        idx = np.where(np.abs(cw[a:b + 1]) > 0.03)[0] + a
        for i0, i1 in zip(idx[:-1], idx[1:]):
            if np.sign(cw[i0]) != np.sign(cw[i1]) and i1 - i0 < int(2 / dt):
                flips.append(t[i1])
    c3 = dict(flips_per_m=len(flips) / L, lane_frac=None)
    vcc = d.get('vcc')
    if vcc is not None and len(vcc) > 10:
        vt = vcc[:, 0] - g['off']
        chg = vt[1:][np.abs(np.diff(vcc[:, 1])) > 0.02]
        if flips:
            c3['lane_frac'] = float(np.mean([np.min(np.abs(chg - tf)) < 1.0 if len(chg) else False for tf in flips]))
    corr = amcl_corrections(d['mapodom'], d['ekf']) if g['has_map'] else []
    c3['amcl_side'] = _event_response(g, segs, [(c['t'], math.copysign(1, c['side'])) for c in corr
                                                if abs(c['side']) >= 0.005])
    c3['amcl_yaw'] = _event_response(g, segs, [(c['t'], math.copysign(1, c['dyaw'])) for c in corr
                                               if abs(c['dyaw']) >= math.radians(0.3)])
    notes = []
    if c3['lane_frac'] is not None and c3['lane_frac'] >= 0.4:
        notes.append('명령 좌우 바뀜의 40 % 이상이 VCC 차선 바꿈과 겹침')
    for k, name in (('amcl_side', 'AMCL 옆 보정'), ('amcl_yaw', 'AMCL 방향 보정')):
        r = c3[k]
        if r['mean'] is not None and r['mean'] <= -0.03:
            notes.append(f'{name} 직후 반대로 꺾음 {r["mean"]:+.3f} rad/s → 위치추정 튐을 쫓음')
    c3['text'] = ' · '.join(notes) or '뚜렷한 명령 원인 없음(차선 바꿈·AMCL 보정 영향 작음)'
    res['c3'] = c3

    # 4 치우침
    per = [math.degrees(r['wb']) / r['dist'] for r in rows]
    c4 = dict(wheel_minus_rpm=math.degrees(sum(r['wb'] for r in rows)) / L,
              se=float(np.std(per) / math.sqrt(len(per))) if len(per) > 1 else None,
              gyro_minus_wheel=math.degrees(sum(r['gw'] for r in rows)) / L,
              cmd_mean=math.degrees(sum(r['cmean'] for r in rows)) / L, by_dir={})
    for h in ('동', '북', '서', '남'):
        rr = [r for r in rows if r['heading'] == h]
        if rr:
            Lh = sum(r['dist'] for r in rr)
            c4['by_dir'][h] = (math.degrees(sum(r['wb'] for r in rr)) / Lh, Lh)
    ol = []
    for a_t, b_t in open_loop_segments(np.c_[g['ct'] + 0.0, d['cmd'][:, 1], d['cmd'][:, 2]]):
        a_t += 0.3
        if (t0 is not None and a_t < t0) or (t1 is not None and b_t > t1) or b_t - a_t < 1.5:
            continue
        a, b = int(np.searchsorted(t, a_t)), int(np.searchsorted(t, b_t)) - 1
        if b <= a:
            continue
        Lo = dist(a, b)
        if Lo < 0.3:
            continue
        ol.append(dict(t0=float(t[a]), t1=float(t[b]), dist=Lo, wheel=math.degrees(g['wyaw'][b] - g['wyaw'][a]) / Lo,
                       gyro=math.degrees(g['gyaw'][b] - g['gyaw'][a]) / Lo))
    c4['open_loop'] = ol
    b, se = c4['wheel_minus_rpm'], c4['se'] or 0.0
    if abs(b) >= 0.3 and abs(b) > 2 * se:
        side = '왼' if b > 0 else '오른'
        c4['text'] = f'{side}쪽 치우침 {b:+.2f}°/m — 10 m 에 옆 약 {abs(math.radians(b)) * 100 / 2 * 100:.0f} cm (바퀴 단계)'
    else:
        c4['text'] = f'치우침 작거나 잡음 안({b:+.2f} ± {se:.2f}°/m). 확정은 명령 w=0 직진(teleop) 구간으로'
    if abs(c4['gyro_minus_wheel']) >= 0.3:
        c4['text'] += f' · 자이로−바퀴 {c4["gyro_minus_wheel"]:+.2f}°/m → 미끄럼·바퀴 지름 차'
    res['c4'] = c4

    # 5 AMCL 옆 보정 방향별
    c5 = dict(by_dir={})
    if g['has_map']:
        inside = np.zeros(len(t), bool)
        for a, b in segs:
            inside[a:b + 1] = True
        for h in ('동', '북', '서', '남'):
            rr = [r for r in rows if r['heading'] == h]
            if not rr:
                continue
            Lh = sum(r['dist'] for r in rr)
            s = 0.0
            for c in corr:
                i = int(np.searchsorted(t, c['t']))
                if 0 <= i < len(t) and inside[i] and heading_name(c['heading']) == h:
                    s += c['side']
            c5['by_dir'][h] = (s / Lh * 100, Lh)
        c5.update(verdict_c5(c5['by_dir']))
    else:
        c5.update(kind='unknown', text='map→odom 없음(AMCL 미실행)')
    res['c5'] = c5
    return res
