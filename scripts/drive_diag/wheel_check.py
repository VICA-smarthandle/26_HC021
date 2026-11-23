#!/usr/bin/env python3
"""바퀴 점검 — 직진 명령 rpm ↔ 엔코더(C5) 비교 · 한 바퀴 틱 확인."""
import argparse
import bisect
import math
import os
import re
import statistics
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

CMD_ID = 0x001
RESP_ID = 0x701
PID_PNT_VEL_CMD = 0xCF
PID_POSI_DATA = 0xC5
PID_PNT_IO_MONITOR = 0xF1
RIGHT, LEFT = 0, 1
NAMES = {RIGHT: "R", LEFT: "L"}

RATIO_OK = (0.98, 1.02)
TPR_OK_PCT = 1.0
STRAIGHT_MIN_RPM = 20
KNOB_SAME_RATIO = 0.01
KNOB_GROUP_PCT = 5

KNOB_MIN_PCT = 55
KNOB_MAX_PCT = 98
KNOB_DEADZONE_PCT = 5
MAX_LINEAR_MPS = 1.0

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ENCODER_YAML = os.path.join(REPO, "vica_ros2_ws", "src", "vica_localization", "config", "encoder.yaml")
DEFAULT_TPR = 61.2
DEFAULT_RADIUS = 0.065

_LINE = re.compile(r"^\((\d+(?:\.\d+)?)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s*$")


def parse_line(line: str) -> Optional[Tuple[float, int, bytes]]:
    """candump -L 한 줄 → (시각 s, CAN id, data). 형식이 다르면 None."""
    m = _LINE.match(line.strip())
    if not m or len(m.group(4)) % 2:
        return None
    return float(m.group(1)), int(m.group(3), 16), bytes.fromhex(m.group(4))


def format_line(t: float, iface: str, can_id: int, data: bytes) -> str:
    """candump -L 형식 한 줄(줄바꿈 포함)."""
    return "(%.6f) %s %03X#%s\n" % (t, iface, can_id, data.hex().upper())


def decode_cf(data: bytes) -> Optional[Tuple[int, int]]:
    """속도 명령 → (오른쪽 rpm, 왼쪽 rpm)."""
    if len(data) < 7 or data[0] != PID_PNT_VEL_CMD:
        return None
    return struct.unpack_from("<h", data, 2)[0], struct.unpack_from("<h", data, 5)[0]


def decode_c5(data: bytes) -> Optional[Tuple[int, int]]:
    """위치 응답 → (motor index, 위치 틱)."""
    if len(data) < 6 or data[0] != PID_POSI_DATA or data[1] not in (RIGHT, LEFT):
        return None
    return data[1], struct.unpack_from("<i", data, 2)[0]


def decode_f1(data: bytes) -> Optional[int]:
    """IO 모니터(첫 묶음 d[1] == 0) → 가변저항 원값 % (모터 노드처럼 d[6], 0~100 으로 자름)."""
    if len(data) != 8 or data[0] != PID_PNT_IO_MONITOR or data[1] != 0:
        return None
    return max(0, min(100, data[6]))


def knob_limit(raw: int, kmin=KNOB_MIN_PCT, kmax=KNOB_MAX_PCT, deadzone=KNOB_DEADZONE_PCT,
               max_linear=MAX_LINEAR_MPS) -> Tuple[int, float]:
    """모터 노드와 같은 계산: 원값 → (정규화 %, 직진 속도 상한 m/s). motor_core.hpp normalize_knob_pct·motor_speed_ratio."""
    norm = raw if kmax <= kmin else max(0, min(100, round((raw - kmin) * 100.0 / (kmax - kmin))))
    return norm, (0.0 if norm <= deadzone else max_linear * norm / 100.0)


def int32_delta(current: int, previous: int) -> int:
    """int32 카운터 넘김을 건너는 부호 있는 차(encoder_feedback.int32_delta 와 같은 식)."""
    delta = int(current) - int(previous)
    if delta > 0x7FFFFFFF:
        delta -= 0x100000000
    elif delta < -0x80000000:
        delta += 0x100000000
    return delta


@dataclass
class Tracker:
    """프레임을 차례로 받아 명령 rpm 과 누적 위치(첫 응답 = 0, 넘김 펼침)를 쌓는다."""
    cmd: List[Tuple[float, int, int]] = field(default_factory=list)
    pos: Dict[int, List[Tuple[float, int]]] = field(default_factory=lambda: {RIGHT: [], LEFT: []})
    knob: List[Tuple[float, int]] = field(default_factory=list)
    cmd_changed_t: Optional[float] = None
    _raw: Dict[int, int] = field(default_factory=dict)
    _cum: Dict[int, int] = field(default_factory=lambda: {RIGHT: 0, LEFT: 0})

    def feed(self, t: float, can_id: int, data: bytes) -> None:
        if can_id == CMD_ID:
            c = decode_cf(data)
            if c is None:
                return
            if not self.cmd or self.cmd[-1][1:] != c:
                self.cmd_changed_t = t
            self.cmd.append((t, c[0], c[1]))
        elif can_id == RESP_ID:
            k = decode_f1(data)
            if k is not None:
                self.knob.append((t, k))
                return
            p = decode_c5(data)
            if p is None:
                return
            m, raw = p
            if m in self._raw:
                self._cum[m] += int32_delta(raw, self._raw[m])
            self._raw[m] = raw
            self.pos[m].append((t, self._cum[m]))


def build_trace(frames) -> Tracker:
    tr = Tracker()
    for t, cid, data in frames:
        tr.feed(t, cid, data)
    return tr


def read_log(path: str) -> Tracker:
    tr = Tracker()
    with open(path, errors="replace") as f:
        for line in f:
            p = parse_line(line)
            if p is not None:
                tr.feed(*p)
    return tr


def fit_rate(samples: List[Tuple[float, int]], a: float, b: float) -> Optional[float]:
    """[a, b] 안 (시각, 틱) 직선 맞춤 기울기 = 틱/s. 표본 3개 미만·0.3 s 미만이면 None."""
    i = bisect.bisect_left(samples, a, key=lambda s: s[0])
    j = bisect.bisect_right(samples, b, key=lambda s: s[0])
    pts = samples[i:j]
    if len(pts) < 3 or pts[-1][0] - pts[0][0] < 0.3:
        return None
    tm = sum(t for t, _ in pts) / len(pts)
    vm = sum(v for _, v in pts) / len(pts)
    den = sum((t - tm) ** 2 for t, _ in pts)
    return sum((t - tm) * (v - vm) for t, v in pts) / den if den > 0 else None


def pos_at(samples: List[Tuple[float, int]], t: float) -> Optional[int]:
    """t 이전 마지막 위치(없으면 첫 위치)."""
    if not samples:
        return None
    i = bisect.bisect_right(samples, t, key=lambda s: s[0])
    return samples[max(i - 1, 0)][1]


@dataclass
class Segment:
    t0: float
    t1: float
    a: float
    cmd_r: float
    cmd_l: float
    rate: Dict[int, Optional[float]] = field(default_factory=dict)


def steady_segments(cmd, min_s=2.0, skip_s=0.7, tol_rpm=3, max_gap=0.3) -> List[Segment]:
    """명령이 좌우 각각 tol_rpm 폭 안에 머문 0 아닌 구간(≥ min_s). teleop 은 명령이 정확히 같다."""
    segs: List[Segment] = []
    n, i = len(cmd), 0
    while i < n:
        rmin = rmax = cmd[i][1]
        lmin = lmax = cmd[i][2]
        j = i
        while j + 1 < n and cmd[j + 1][0] - cmd[j][0] <= max_gap:
            r, l = cmd[j + 1][1], cmd[j + 1][2]
            if max(rmax, r) - min(rmin, r) > tol_rpm or max(lmax, l) - min(lmin, l) > tol_rpm:
                break
            rmin, rmax, lmin, lmax = min(rmin, r), max(rmax, r), min(lmin, l), max(lmax, l)
            j += 1
        t0, t1 = cmd[i][0], cmd[j][0]
        part = cmd[i:j + 1]
        mr = statistics.fmean(c[1] for c in part)
        ml = statistics.fmean(c[2] for c in part)
        if (abs(mr) >= 1 or abs(ml) >= 1) and t1 - t0 >= min_s:
            segs.append(Segment(t0, t1, t0 + skip_s, mr, ml))
        i = j + 1
    return segs


@dataclass
class Run:
    t0: float
    t1: float
    sum_r: float = 0.0
    sum_l: float = 0.0
    delta: Dict[int, Optional[int]] = field(default_factory=dict)

    @property
    def direction(self) -> str:
        if self.sum_r > 0 and self.sum_l > 0:
            return "전진"
        if self.sum_r < 0 and self.sum_l < 0:
            return "후진"
        return "회전"


def motion_runs(tr: Tracker, merge_s=1.0, settle_s=1.5) -> List[Run]:
    """명령이 0 아닌 덩어리(0 이 merge_s 미만이면 이어 붙임). Δ틱 = 시작 직전 위치 → 멈춘 뒤 위치."""
    runs: List[Run] = []
    for t, r, l in tr.cmd:
        if r == 0 and l == 0:
            continue
        if runs and t - runs[-1].t1 <= merge_s:
            runs[-1].t1 = t
        else:
            runs.append(Run(t, t))
        runs[-1].sum_r += r
        runs[-1].sum_l += l
    for k, run in enumerate(runs):
        end = run.t1 + settle_s
        if k + 1 < len(runs):
            end = min(end, runs[k + 1].t0)
        for m in (RIGHT, LEFT):
            s = tr.pos[m]
            p0, p1 = pos_at(s, run.t0), pos_at(s, end)
            run.delta[m] = None if p0 is None or p1 is None else p1 - p0
    return runs


def segment_rows(tr: Tracker, segs: List[Segment], tpr: float) -> List[dict]:
    rows = []
    for s in segs:
        row = {"seg": s, "knob": knob_range(tr.knob, s.a, s.t1)}
        for m, c in ((RIGHT, s.cmd_r), (LEFT, s.cmd_l)):
            rate = fit_rate(tr.pos[m], s.a, s.t1)
            s.rate[m] = rate
            meas = None if rate is None else rate * 60.0 / tpr
            row[m] = {
                "cmd": c,
                "meas": meas,
                "ratio": None if meas is None or abs(c) < 1 else meas / c,
                "tpr_speed": None if rate is None or abs(c) < 1 else rate * 60.0 / c,
            }
        rows.append(row)
    return rows


def knob_range(knob: List[Tuple[float, int]], a: float, b: float) -> Optional[Tuple[int, int, float]]:
    """[a, b] 안 가변저항 원값 (최소, 최대, 중앙). 프레임이 없으면 None."""
    i = bisect.bisect_left(knob, a, key=lambda s: s[0])
    j = bisect.bisect_right(knob, b, key=lambda s: s[0])
    vals = [v for _, v in knob[i:j]]
    return (min(vals), max(vals), statistics.median(vals)) if vals else None


def is_straight(row: dict) -> bool:
    s = row["seg"]
    return (abs(s.cmd_r) >= STRAIGHT_MIN_RPM and abs(s.cmd_l) >= STRAIGHT_MIN_RPM
            and s.cmd_r * s.cmd_l > 0)


def weighted(rows: List[dict]) -> Dict[int, Optional[dict]]:
    """구간 길이 가중 평균 비율·한 바퀴 틱(좌우 각각)."""
    out = {}
    for m in (RIGHT, LEFT):
        w = r = tp = 0.0
        n = 0
        for row in rows:
            if row[m]["ratio"] is None:
                continue
            d = row["seg"].t1 - row["seg"].a
            w += d
            r += row[m]["ratio"] * d
            tp += row[m]["tpr_speed"] * d
            n += 1
        out[m] = None if w <= 0 else {"n": n, "secs": w, "ratio": r / w, "tpr_speed": tp / w}
    return out


def straight_summary(rows: List[dict]) -> Dict[int, Optional[dict]]:
    """좌우 같은 부호·|명령| ≥ STRAIGHT_MIN_RPM 구간의 길이 가중 평균 비율·한 바퀴 틱."""
    return weighted([row for row in rows if is_straight(row)])


def knob_groups(rows: List[dict]) -> List[dict]:
    """직진 구간을 가변저항 위치(중앙 원값, KNOB_GROUP_PCT 안이면 같은 위치)로 묶어 비율을 낸다."""
    items = sorted(((row["knob"][2], row) for row in rows if is_straight(row) and row["knob"] is not None),
                   key=lambda kr: kr[0])
    groups: List[List] = []
    for k, row in items:
        if groups and k - groups[-1][0][0] < KNOB_GROUP_PCT:
            groups[-1].append((k, row))
        else:
            groups.append([(k, row)])
    out = []
    for g in groups:
        ks = [k for k, _ in g]
        out.append({"knob_lo": min(ks), "knob_hi": max(ks), "agg": weighted([row for _, row in g])})
    return out


def compare_measured(delta: Dict[int, Optional[int]], tpr: float, radius: float,
                     revs: Dict[int, Optional[float]], tape_m: Optional[float]) -> List[str]:
    """타이어 마크 바퀴 수·줄자 거리와 비교한 문장들."""
    lines = []
    circ = 2 * math.pi * radius
    for m in (RIGHT, LEFT):
        n = revs.get(m)
        d = delta.get(m)
        if n is None or d is None or n <= 0:
            continue
        meas = abs(d) / n
        pct = (meas - tpr) / tpr * 100
        verdict = "맞음" if abs(pct) <= TPR_OK_PCT else "다름"
        lines.append(f"  타이어 마크 {NAMES[m]} {n:.2f} 바퀴 ↔ Δ틱 {abs(d)} → 한 바퀴 {meas:.2f} 틱"
                     f" (설정 {tpr:g} 대비 {pct:+.2f} %, {verdict})")
    if tape_m is not None and tape_m > 0:
        ds = [abs(delta[m]) for m in (RIGHT, LEFT) if delta.get(m) is not None]
        if ds:
            mean = sum(ds) / len(ds)
            wheel = mean / tpr * circ
            pct = (wheel - tape_m) / tape_m * 100
            tpr_tape = mean / (tape_m / circ)
            lines.append(f"  줄자 {tape_m:.3f} m ↔ 바퀴 거리 {wheel:.3f} m ({pct:+.2f} %)."
                         f" 반지름 {radius:g} m 가 맞다면 한 바퀴 {tpr_tape:.2f} 틱")
    return lines


def _f(v, fmt):
    """값이 없으면 같은 폭의 '–'(표 열이 밀리지 않게)."""
    if v is not None:
        return format(v, fmt)
    m = re.match(r"[+\- ]?(\d+)", fmt)
    return "–".rjust(int(m.group(1)) if m else 1)


def print_summary(tr: Tracker, tpr: float, radius: float, args) -> None:
    if not tr.cmd or not tr.pos[RIGHT] or not tr.pos[LEFT]:
        print("\n명령(0x001 CF) 또는 위치(0x701 C5) 프레임이 없습니다 — 모터 노드가 떠 있었는지 확인하세요.")
        return
    t_ref = min(tr.cmd[0][0], tr.pos[RIGHT][0][0], tr.pos[LEFT][0][0])
    circ = 2 * math.pi * radius

    segs = steady_segments(tr.cmd, min_s=args.min_s, skip_s=args.skip_s, tol_rpm=args.tol_rpm)
    rows = segment_rows(tr, segs, tpr)
    print(f"\n== 일정 명령 구간 (좌우 각각 ±{args.tol_rpm} rpm 안 ≥ {args.min_s:g} s, 앞 {args.skip_s:g} s 제외) ==")
    if not rows:
        print("  없음 — teleop 으로 같은 속도를 2 s 넘게 유지해야 비율이 나옵니다.")
    else:
        print("   #    시작     길이   명령 R / L rpm    측정 R / L rpm     비율 R / L      속도 기준 한 바퀴 틱 R / L   가변저항 원값")
        for k, row in enumerate(rows, 1):
            s = row["seg"]
            R, L = row[RIGHT], row[LEFT]
            kn = row["knob"]
            ktxt = "–" if kn is None else (f"{kn[0]} %" if kn[0] == kn[1] else f"{kn[0]}~{kn[1]} %")
            print(f"  {k:2d}  {s.t0 - t_ref:+7.1f} s {s.t1 - s.a:4.1f} s  {R['cmd']:+6.1f} / {L['cmd']:+6.1f}"
                  f"   {_f(R['meas'], '+6.1f')} / {_f(L['meas'], '+6.1f')}"
                  f"   {_f(R['ratio'], '6.3f')} / {_f(L['ratio'], '6.3f')}"
                  f"    {_f(R['tpr_speed'], '6.2f')} / {_f(L['tpr_speed'], '6.2f')}     {ktxt}")
    agg = straight_summary(rows)
    print(f"\n== 직진 결론 (좌우 같은 부호·|명령| ≥ {STRAIGHT_MIN_RPM} rpm 구간, 길이 가중) ==")
    for m in (RIGHT, LEFT):
        a = agg[m]
        if a is None:
            print(f"  {NAMES[m]}: 해당 구간 없음")
            continue
        ok = RATIO_OK[0] <= a["ratio"] <= RATIO_OK[1]
        print(f"  {NAMES[m]}: 측정/명령 {a['ratio']:.3f} ({'같음' if ok else '다름'}, 기준 {RATIO_OK[0]}~{RATIO_OK[1]})"
              f" · 속도 기준 한 바퀴 {a['tpr_speed']:.2f} 틱 (설정 {tpr:g}, {(a['tpr_speed'] - tpr) / tpr * 100:+.2f} %)"
              f" · {a['n']}구간 {a['secs']:.1f} s")
    if agg[RIGHT] and agg[LEFT]:
        lr = (agg[RIGHT]["ratio"] - agg[LEFT]["ratio"]) / agg[LEFT]["ratio"] * 100
        print(f"  좌우 차(R − L) {lr:+.2f} %")

    print_knob_section(tr, rows, args)

    runs = motion_runs(tr)
    print(f"\n== 주행 덩어리 (명령 0 아님, 멈춘 뒤 1.5 s 까지 Δ틱) — 바퀴 수 = Δ틱 ÷ {tpr:g} ==")
    if not runs:
        print("  없음")
    for k, run in enumerate(runs, 1):
        dr, dl = run.delta[RIGHT], run.delta[LEFT]
        rv = [None if d is None else d / tpr for d in (dr, dl)]
        dist = [None if v is None else v * circ for v in rv]
        print(f"  {k:2d}  {run.t0 - t_ref:+7.1f} s  {run.t1 - run.t0:5.1f} s  {run.direction}"
              f"  Δ틱 R {_f(dr, '+d')} L {_f(dl, '+d')}"
              f"  바퀴 R {_f(rv[0], '+.2f')} L {_f(rv[1], '+.2f')}"
              f"  거리 R {_f(dist[0], '+.3f')} L {_f(dist[1], '+.3f')} m")

    revs = {RIGHT: args.revs_r if args.revs_r is not None else args.revs,
            LEFT: args.revs_l if args.revs_l is not None else args.revs}
    if any(v is not None for v in revs.values()) or args.tape_m is not None:
        if not runs:
            print("\n실측 비교: 주행 덩어리가 없어 못 합니다.")
            return
        k = args.run if args.run else 1
        if not 1 <= k <= len(runs):
            print(f"\n실측 비교: --run {k} 이 없습니다(1~{len(runs)}).")
            return
        print(f"\n== 실측 비교 (주행 덩어리 {k}{'' if args.run else ', --run 으로 바꿈'}) ==")
        for line in compare_measured(runs[k - 1].delta, tpr, radius, revs, args.tape_m):
            print(line)


def print_knob_section(tr: Tracker, rows: List[dict], args) -> None:
    """가변저항: 소프트웨어 상한이 명령을 잘랐는지 + 위치별 비율(드라이버가 가변저항을 쓰는지)."""
    print("\n== 가변저항 (701#F1 d[6]) ==")
    if not tr.knob:
        print("  F1 프레임 없음 — 가변저항 값을 모릅니다.")
        return
    vals = [v for _, v in tr.knob]
    lo, hi = min(vals), max(vals)
    if getattr(args, "ignore_knob", False):
        print(f"  원값 {lo}~{hi} % — 모터 노드 ignore_knob:=true 라 속도에 안 씀(직진 상한 = max_linear {args.max_linear:g} m/s)")
    else:
        for raw in sorted({lo, hi}):
            norm, lim = knob_limit(raw, args.knob_min, args.knob_max, args.deadzone, args.max_linear)
            print(f"  원값 {raw} % → 모터 노드 정규화 {norm} % → 직진 상한 {lim:.2f} m/s"
                  f" (knob_min {args.knob_min}·max {args.knob_max}·max_linear {args.max_linear:g})")
    print("  비율은 실제로 나간 CAN 명령 대비라 소프트웨어 상한은 비율에 안 들어갑니다.")
    groups = knob_groups(rows)
    if not groups:
        print("  직진 구간이 없어 가변저항 위치별 비율을 못 냅니다.")
        return
    if len(groups) < 2:
        where = "–" if not groups else (f"{groups[0]['knob_lo']:g}" if groups[0]["knob_lo"] == groups[0]["knob_hi"]
                                        else f"{groups[0]['knob_lo']:g}~{groups[0]['knob_hi']:g}")
        print(f"  직진 구간의 가변저항 위치가 하나({where} %)뿐 — 같은 속도로 위치만 바꿔(예: 98 → 80) 한 번 더 달리면"
              " 드라이버가 가변저항을 쓰는지 나옵니다.")
        return
    print("  위치별 직진 비율:")
    ratios = {RIGHT: [], LEFT: []}
    for g in groups:
        a = g["agg"]
        where = f"{g['knob_lo']:g}" if g["knob_lo"] == g["knob_hi"] else f"{g['knob_lo']:g}~{g['knob_hi']:g}"
        parts = []
        for m in (RIGHT, LEFT):
            if a[m] is not None:
                ratios[m].append(a[m]["ratio"])
                parts.append(f"{NAMES[m]} {a[m]['ratio']:.3f}")
        secs = max((a[m]["secs"] for m in (RIGHT, LEFT) if a[m]), default=0.0)
        print(f"    원값 {where:>7} % : {' · '.join(parts)}  ({secs:.1f} s)")
    spread = max((max(v) - min(v)) for v in ratios.values() if len(v) >= 2)
    if spread < KNOB_SAME_RATIO:
        print(f"  → 위치를 바꿔도 비율 차 {spread:.3f} (< {KNOB_SAME_RATIO}) — 드라이버는 가변저항으로 속도를 바꾸지 않음."
              " 가변저항은 원인이 아님")
    else:
        print(f"  → 위치에 따라 비율이 {spread:.3f} 달라짐 (≥ {KNOB_SAME_RATIO}) — 드라이버가 가변저항으로 속도를 바꾸는 것으로 보임")


def live_line(tr: Tracker, t_now: float, t_ref: float, tpr: float, radius: float, window: float,
              knob=(KNOB_MIN_PCT, KNOB_MAX_PCT, KNOB_DEADZONE_PCT, MAX_LINEAR_MPS)) -> str:
    circ = 2 * math.pi * radius
    cr, cl = (tr.cmd[-1][1], tr.cmd[-1][2]) if tr.cmd else (0, 0)
    settled = tr.cmd_changed_t is not None and t_now - tr.cmd_changed_t >= window + 0.5
    meas, ratio, cum, rv = {}, {}, {}, {}
    for m, c in ((RIGHT, cr), (LEFT, cl)):
        rate = fit_rate(tr.pos[m], t_now - window, t_now)
        meas[m] = None if rate is None else rate * 60.0 / tpr
        ratio[m] = meas[m] / c if (settled and c != 0 and meas[m] is not None) else None
        cum[m] = tr.pos[m][-1][1] if tr.pos[m] else None
        rv[m] = None if cum[m] is None else cum[m] / tpr
    dist = None if rv[RIGHT] is None or rv[LEFT] is None else (rv[RIGHT] + rv[LEFT]) / 2 * circ
    ignore = len(knob) > 4 and knob[4]
    if tr.knob:
        raw = tr.knob[-1][1]
        if ignore:
            ktxt = f" | 가변저항 {raw:3d} % (모터 노드 무시, 상한 {knob[3]:.2f} m/s)"
        else:
            norm, lim = knob_limit(raw, *knob[:4])
            ktxt = f" | 가변저항 {raw:3d} %→{norm:3d} % 상한 {lim:.2f} m/s"
    else:
        ktxt = " | 가변저항 –"
    return (f"{t_now - t_ref:7.1f} s | 명령 R {cr:+4d} L {cl:+4d} rpm"
            f" | 측정 R {_f(meas[RIGHT], '+6.1f')} L {_f(meas[LEFT], '+6.1f')}"
            f" | 비율 R {_f(ratio[RIGHT], '5.3f')} L {_f(ratio[LEFT], '5.3f')}"
            f" | 누적 틱 R {_f(cum[RIGHT], '+6d')} L {_f(cum[LEFT], '+6d')}"
            f" | 바퀴 R {_f(rv[RIGHT], '+6.2f')} L {_f(rv[LEFT], '+6.2f')}"
            f" | 거리 {_f(dist, '+6.3f')} m{ktxt}")


def load_config(path: str = ENCODER_YAML) -> Tuple[float, float, str]:
    """encoder.yaml 의 ticks_per_rev·wheel_radius_m. 못 읽으면 코드 기본값."""
    try:
        import yaml
        with open(path) as f:
            p = yaml.safe_load(f)["encoder_feedback"]["ros__parameters"]
        return float(p["ticks_per_rev"]), float(p["wheel_radius_m"]), os.path.relpath(path, REPO)
    except Exception as e:  # noqa: BLE001 — 설정 파일이 없어도 도구는 돈다
        return DEFAULT_TPR, DEFAULT_RADIUS, f"코드 기본값({type(e).__name__}: encoder.yaml 못 읽음)"


def run_live(args, tpr: float, radius: float) -> None:
    import can
    bus = can.Bus(interface="socketcan", channel=args.iface, receive_own_messages=False,
                  can_filters=[{"can_id": CMD_ID, "can_mask": 0x7FF},
                               {"can_id": RESP_ID, "can_mask": 0x7FF}])
    save = None
    if args.save:
        path = os.path.expanduser(args.save)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        save = open(path, "w")
        print(f"원본 프레임 저장: {path}")
    tr = Tracker()
    t_ref = t_now = None
    last_print = 0.0
    last_rx = time.monotonic()
    print("Ctrl-C 로 끝내면 요약을 냅니다.\n")
    try:
        while True:
            msg = bus.recv(timeout=0.1)
            mono = time.monotonic()
            if msg is not None and not msg.is_error_frame and not msg.is_remote_frame:
                data = bytes(msg.data)
                tr.feed(msg.timestamp, msg.arbitration_id, data)
                if save:
                    save.write(format_line(msg.timestamp, args.iface, msg.arbitration_id, data))
                t_now = msg.timestamp
                t_ref = t_ref if t_ref is not None else t_now
                last_rx = mono
            if mono - last_print >= 0.5:
                last_print = mono
                if mono - last_rx > 2.0:
                    print(f"{args.iface} 0x001·0x701 프레임이 2 s 없음 — 모터 노드(명령 송신)가 떠 있는지 확인", flush=True)
                elif t_now is not None:
                    print(live_line(tr, t_now, t_ref, tpr, radius, args.window,
                                    (args.knob_min, args.knob_max, args.deadzone, args.max_linear,
                                     args.ignore_knob)), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        if save:
            save.close()
    print_summary(tr, tpr, radius, args)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="직진 명령 rpm ↔ 엔코더(C5) 비교 · 한 바퀴 틱 확인 (CAN 받기만)")
    sub = ap.add_subparsers(dest="mode", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--tpr", type=float, help="한 바퀴 틱(기본: encoder.yaml ticks_per_rev)")
    common.add_argument("--radius", type=float, help="바퀴 반지름 m(기본: encoder.yaml wheel_radius_m)")
    common.add_argument("--min-s", type=float, default=2.0, help="일정 명령 구간 최소 길이 s (기본 2.0)")
    common.add_argument("--skip-s", type=float, default=0.7, help="구간 앞 응답 지연으로 빼는 시간 s (기본 0.7)")
    common.add_argument("--tol-rpm", type=int, default=3, help="일정 명령으로 보는 rpm 폭(기본 3, teleop 은 0 이어도 같음)")
    common.add_argument("--revs", type=float, help="타이어 마크로 센 바퀴 수(좌우 같을 때)")
    common.add_argument("--revs-r", type=float, help="오른쪽 바퀴 수")
    common.add_argument("--revs-l", type=float, help="왼쪽 바퀴 수")
    common.add_argument("--tape-m", type=float, help="줄자로 잰 이동 거리 m")
    common.add_argument("--run", type=int, help="실측 비교에 쓸 주행 덩어리 번호(기본 1)")
    common.add_argument("--knob-min", type=int, default=KNOB_MIN_PCT, help="모터 노드 knob_min_pct(기본 55)")
    common.add_argument("--knob-max", type=int, default=KNOB_MAX_PCT, help="모터 노드 knob_max_pct(기본 98)")
    common.add_argument("--deadzone", type=int, default=KNOB_DEADZONE_PCT, help="모터 노드 deadzone_pct(기본 5)")
    common.add_argument("--max-linear", type=float, default=MAX_LINEAR_MPS, help="모터 노드 max_linear_mps(기본 1.0)")
    common.add_argument("--ignore-knob", action="store_true",
                        help="모터 노드를 ignore_knob:=true 로 띄웠을 때(가변저항 무시) — 화면의 상한 표시를 맞춘다")
    lv = sub.add_parser("live", parents=[common], help="CAN 실시간 보기 → Ctrl-C 요약")
    lv.add_argument("--iface", default="can1")
    lv.add_argument("--window", type=float, default=1.0, help="측정 rpm 창 s (기본 1.0)")
    lv.add_argument("--save", help="받은 프레임을 candump -L 형식으로 저장할 파일")
    fl = sub.add_parser("file", parents=[common], help="candump -L 기록 요약")
    fl.add_argument("log")
    args = ap.parse_args(argv)

    tpr_cfg, radius_cfg, src = load_config()
    tpr = args.tpr if args.tpr else tpr_cfg
    radius = args.radius if args.radius else radius_cfg
    print(f"설정값: 한 바퀴 {tpr_cfg:g} 틱 · 반지름 {radius_cfg:g} m ({src})")
    if (tpr, radius) != (tpr_cfg, radius_cfg):
        print(f"이번 계산: 한 바퀴 {tpr:g} 틱 · 반지름 {radius:g} m (인자로 덮어씀)")
    print(f"명령 rpm 은 모터 노드가 wheel_radius_m 으로 계산 — 0.3 m/s = {0.3 / (2 * math.pi * radius) * 60:.1f} rpm"
          f" → 기대 {0.3 / (2 * math.pi * radius) * tpr:.1f} 틱/s")
    if args.mode == "live":
        run_live(args, tpr, radius)
    else:
        print_summary(read_log(os.path.expanduser(args.log)), tpr, radius, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
