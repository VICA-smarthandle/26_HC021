"""지도 수정(지우기·부분 스캔) 보고서 그림과 원자료 CSV — 11-19 실기·시험 기록에서.

python3 make_figures.py  → figs/*.png(220 dpi), data/*.csv  — 젯슨 ~/vica_data/map_fix_tests/2026-11-19_dock/report/ 에서 돈다(../field/check_ws.jsonl 을 읽음)
원자료: ../field/check_ws.jsonl(위치 맞춤 확인 14건), data/loadmap_timeline_1119.csv(저널),
data/lidar_near_points_1119.csv(라이다 1 m 안 반사점 30장), 대화 중 잰 메모리·적용 기록(아래 표).
"""
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon

HERE = Path(__file__).resolve().parent
DATA, FIGS = HERE / "data", HERE / "figs"
DATA.mkdir(exist_ok=True); FIGS.mkdir(exist_ok=True)
for f in ("NanumSquare", "Noto Sans CJK KR", "Noto Sans CJK TC"):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = f
        break
plt.rcParams["axes.unicode_minus"] = False
INK, MUTED, GRID, ACC, ACC2, WARN, ORANGE = "#1f2a2e", "#5f6f73", "#d9e1e2", "#1d7a5f", "#2f6db0", "#c2563d", "#c2743a"
DPI = 220


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)


def title(ax, text):
    ax.set_title(text, fontsize=11, color=INK, loc="left")


def write_csv(name, header, rows):
    with open(DATA / name, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


# ---------------------------------------------------------------------------
# 1. 실기 위치 맞춤 확인(scan_check) 14건 — RMS·남긴 빔·AMCL 과의 차이
# ---------------------------------------------------------------------------
checks = []
for line in open(HERE.parent / "field" / "check_ws.jsonl", encoding="utf-8"):
    row = json.loads(line)
    if row.get("action") != "scan_check" or not row.get("result_json"):
        continue
    r = json.loads(row["result_json"])
    if "rms_m" not in r:
        continue
    checks.append([row["t"], r["ok"], r["reason"], r["score"], r["used_beams"], r["kept_beams"],
                   r["rms_m"], r["moved_m"], r["moved_deg"], r["x"], r["y"], r["yaw"], row.get("_sec")])
write_csv("scan_check_field_1119.csv",
          ["time", "ok", "reason", "score_pct", "used_beams", "kept_beams", "rms_m", "moved_m", "moved_deg",
           "x", "y", "yaw_rad", "answer_sec"], checks)

fig, (a1, a2) = plt.subplots(1, 2, figsize=(8.4, 4.1), dpi=DPI, gridspec_kw={"width_ratios": [1.4, 1]})
xs = range(1, len(checks) + 1)
rms = [c[6] * 100 for c in checks]
a1.axhspan(3, 5, color="#fbe9e4", zorder=0)
a1.text(8.45, 4.0, "옛 기준이면 탈락,\n새 기준이면 통과", fontsize=7.5, color=WARN, ha="right", va="center")
a1.axhline(3, color=WARN, lw=1.2, ls="--", label="처음 기준 3 cm(시험 데이터로 정함)")
a1.axhline(5, color=ACC, lw=1.4, label="바꾼 기준 5 cm(지도 한 칸)")
a1.axhline(0.94, color=ACC2, lw=1.0, ls=":", label="가짜 스캔(잡음 5 mm) 0.94 cm")
a1.scatter(xs, rms, s=36, color=INK, zorder=3, label="실기 측정")
a1.set_ylim(0, 6); a1.set_xlabel("위치 맞춤 확인(시간 순)", color=INK, fontsize=9)
a1.set_ylabel("남긴 빔의 벽까지 거리 RMS (cm)", color=INK, fontsize=9)
style(a1); a1.legend(fontsize=7.2, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
title(a1, "위치가 맞는데도 RMS 3.3~3.7 cm — 기준을 한 칸으로")
kept = [c[5] / c[4] * 100 for c in checks]
moved = [c[7] * 100 for c in checks]
a2.scatter(moved, kept, s=36, color=INK)
a2.axhline(50, color=WARN, ls="--", lw=1.0)
a2.axvline(30, color=WARN, ls="--", lw=1.0)
a2.text(31, 52, "통과 범위\n(빔 ≥ 50 %, 옮긴 양 ≤ 30 cm)", fontsize=7.5, color=WARN, va="bottom")
a2.set_xlim(0, 40); a2.set_ylim(40, 80)
a2.set_xlabel("AMCL 자세에서 옮긴 양 (cm)", color=INK, fontsize=9)
a2.set_ylabel("벽에 맞은 빔 비율 (%)", color=INK, fontsize=9)
style(a2); title(a2, "다른 조건은 여유 있게 통과")
fig.tight_layout(); fig.savefig(FIGS / "fig01_rms_field.png"); plt.close(fig)

# ---------------------------------------------------------------------------
# 2. 몸체 반사점과 몸체 + 사방 20 cm 거르기
# ---------------------------------------------------------------------------
FOOT = [(0.151, 0.225), (0.151, -0.225), (-0.459, -0.225), (-0.569, -0.035), (-0.569, 0.035), (-0.459, 0.225)]
SENSOR = (0.031, 0.0, math.radians(-1.98))   # 노드가 실기 때 쓴 라이다 자리(앞 3.1 cm)·설정 각도
near = []
for r in csv.DictReader(open(DATA / "lidar_near_points_1119.csv", encoding="utf-8")):
    a, d = math.radians(float(r["angle_deg"])), float(r["range_m"])
    near.append((SENSOR[0] + d * math.cos(a + SENSOR[2]), SENSOR[1] + d * math.sin(a + SENSOR[2])))
# 21:05 부분 스캔이 로봇 1 m 안에 더한 10칸(로봇 기준 앞 +, 왼쪽 +) — 사용자가 지우기로 지움
front = [(0.37, 0.26), (0.41, 0.16), (0.42, 0.21), (0.46, 0.16), (0.56, 0.15), (0.58, 0.40),
         (0.61, 0.10), (0.66, 0.10), (0.71, 0.05), (0.71, 0.10)]
write_csv("front_cells_2105.csv", ["forward_m", "left_m"], front)


def inflate(poly, margin, steps=24):
    """다각형 둘레 margin(m) — 그림용 근사(꼭짓점마다 원호)."""
    pts = []
    n = len(poly)
    for i in range(n):
        px, py = poly[i]
        for k in range(steps):
            t = 2 * math.pi * k / steps
            pts.append((px + margin * math.cos(t), py + margin * math.sin(t)))
    # 볼록 껍질
    pts = sorted(set(pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


fig, ax = plt.subplots(figsize=(6.6, 5.0), dpi=DPI)
ax.add_patch(Polygon(inflate(FOOT, 0.20), closed=True, facecolor="#eef4f3", edgecolor=ACC, lw=1.2, ls="--",
                     label="거르는 범위: 몸체 + 사방 20 cm"))
ax.add_patch(Polygon(FOOT, closed=True, facecolor="#dfe7e8", edgecolor=INK, lw=1.4, label="몸체(Nav2 footprint)"))
ax.scatter([p[0] for p in near], [p[1] for p in near], s=4, color=WARN, alpha=0.35,
           label=f"라이다 1 m 안 반사점(30장, {len(near)}점)")
ax.scatter([p[0] for p in front], [p[1] for p in front], s=40, marker="s", color=ORANGE, edgecolor="white",
           label="21:05 스캔이 더한 10칸(로봇 앞 사람 추정, 사용자가 지움)")
ax.scatter([0], [0], marker="+", s=120, color=INK)
ax.scatter([SENSOR[0]], [SENSOR[1]], marker="o", s=30, facecolor="white", edgecolor=INK, label="라이다")
ax.annotate("", xy=(0.45, -0.75), xytext=(0.0, -0.75), arrowprops=dict(arrowstyle="->", color=MUTED))
ax.text(0.47, -0.75, "앞(주행 방향)", va="center", fontsize=8, color=MUTED)
ax.set_aspect("equal"); ax.set_xlim(-1.05, 1.05); ax.set_ylim(-0.9, 0.9)
ax.set_xlabel("로봇 기준 앞뒤 (m)", color=INK, fontsize=9); ax.set_ylabel("로봇 기준 좌우 (m, 왼쪽 +)", color=INK, fontsize=9)
style(ax); ax.legend(fontsize=7.2, frameon=False, loc="upper right", bbox_to_anchor=(1.0, 1.0))
title(ax, "서 있는 로봇의 뒤쪽 차체가 매 스캔 같은 자리에 찍힌다")
fig.tight_layout(); fig.savefig(FIGS / "fig02_body_mask.png"); plt.close(fig)

# ---------------------------------------------------------------------------
# 3. 적용(load_map) 시간 흐름 — 19:35 응답 실종 vs 21:03~21:06
# ---------------------------------------------------------------------------
tl = list(csv.DictReader(open(DATA / "loadmap_timeline_1119.csv", encoding="utf-8")))
for r in tl:
    r["t"] = float(r["time"])
fig, (b1, b2) = plt.subplots(2, 1, figsize=(8.4, 4.6), dpi=DPI)
lanes = {"map_server": 2, "amcl": 1, "map_fix_node": 0}
colors = {"map_server": ACC2, "amcl": MUTED, "map_fix_node": INK}
for ax, (t0, t1, head) in ((b1, (1795084505, 1795084575, "19:35 적용 — 요청 7번을 모두 읽었는데 응답 6번이 오지 않았다(그동안 잠금을 쥔 채 재요청)")),
                           (b2, (1795089810, 1795090003, "21:03~21:06 고친 뒤 — 적용 6번, 매번 LoadMap 1번(4번은 /map 으로 먼저 확인)"))):
    for r in tl:
        if t0 <= r["t"] <= t1:
            y = lanes[r["source"]]
            c = WARN if "apply_timeout" in r["event"] else ("#1d7a5f" if "/map" in r["event"] else colors[r["source"]])
            ax.scatter(r["t"] - t0, y, s=34, color=c, zorder=3)
    ax.set_yticks([0, 1, 2]); ax.set_yticklabels(["지도 수정 노드\n(적용 확인)", "AMCL\n(새 지도 받음)", "map_server\n(LoadMap 받음)"], fontsize=8)
    ax.set_ylim(-0.6, 2.6); ax.set_xlim(-2, t1 - t0 + 2)
    style(ax); ax.set_title(head, fontsize=9.5, color=INK, loc="left")
b1.text(15.5, -0.45, "← 10 s 시간 초과(apply_timeout)", fontsize=7.5, color=WARN)
b2.set_xlabel("첫 요청부터 지난 시간 (s)", color=INK, fontsize=9)
fig.tight_layout(); fig.savefig(FIGS / "fig03_apply_timeline.png"); plt.close(fig)

# ---------------------------------------------------------------------------
# 4. AMCL 이 보는 자기 오차 vs 스캔으로 맞춘 위치와의 실제 차이
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.2, 3.0), dpi=DPI)
labels = ["AMCL σ 앞뒤", "AMCL σ 좌우", "스캔과 실제 차이\n(19:20 자리)", "스캔과 실제 차이\n(19:15 자리)"]
vals = [6.8, 7.3, max(c[7] for c in checks) * 100, min(c[7] for c in checks) * 100]
ax.barh(labels, vals, color=[ACC2, ACC2, WARN, MUTED])
ax.axvline(10, color=ACC, lw=1.2, ls="--")
ax.text(10.3, 3.25, "주행 위치를 쓰는 조건\nσ ≤ 10 cm", fontsize=7.5, color=ACC, va="center")
for i, v in enumerate(vals):
    ax.text(v + 0.3, i, f"{v:.1f} cm", va="center", fontsize=8, color=INK)
ax.set_xlim(0, 20); ax.invert_yaxis(); style(ax)
ax.set_xlabel("cm", color=INK, fontsize=9)
title(ax, "AMCL 은 자기 오차를 실제보다 작게 본다")
fig.tight_layout(); fig.savefig(FIGS / "fig04_amcl_vs_scan.png"); plt.close(fig)

# ---------------------------------------------------------------------------
# 5. 메모리 — LLM·Claude 세션 겹침 vs LLM 내림
# ---------------------------------------------------------------------------
mem = [  # 시각, 상황, RAM 사용(MB), 사용 가능(MB), 스왑 사용(MB), 부하(1분)
    ("19:03:53", "주행 켬 + LLM 올라감", 14903, 460, 6919, 38.7),
    ("19:06:22", "Claude 세션 3개 더 열림", 15191, 162, 7827, 19.4),
    ("19:12:46", "LLM 내림(ollama stop)", 6513, 8855, 2984, 8.8),
    ("20:57:27", "운영 반영 뒤 주행 켬·LLM 끔", 6256, 9097, 1224, 22.2),
]
write_csv("memory_1119.csv", ["time", "state", "ram_used_mb", "available_mb", "swap_used_mb", "load1"], mem)
fig, ax = plt.subplots(figsize=(7.4, 3.7), dpi=DPI)
ys = range(len(mem))
ax.barh([y + 0.18 for y in ys], [m[4] / 1024 for m in mem], height=0.34, color=WARN, label="스왑 사용 (GB, 최대 7.6)")
ax.barh([y - 0.18 for y in ys], [m[3] / 1024 for m in mem], height=0.34, color=ACC, label="사용 가능 RAM (GB)")
ax.set_yticks(list(ys)); ax.set_yticklabels([f"{m[0]}\n{m[1]}" for m in mem], fontsize=7.5)
ax.axvline(7.64, color=WARN, lw=0.8, ls=":")
ax.invert_yaxis(); style(ax); ax.legend(fontsize=7.5, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2)
ax.set_xlabel("GB (RAM 15.3 GB · 스왑 7.6 GB)", color=INK, fontsize=9)
title(ax, "LLM 을 올린 채 세션이 겹치자 스왑이 꽉 찼다(15:58 OOM 이력)")
fig.tight_layout(); fig.savefig(FIGS / "fig05_memory.png"); plt.close(fig)

# ---------------------------------------------------------------------------
# 6. 장소 점 크기 — 지도가 그려지는 배율에 따라
# ---------------------------------------------------------------------------
def dot(scale, base=1.0):
    size = 5 * base * min(1.0, max(0.5, scale / 1.2))
    return max(2.5, size)


xsx = [i / 100 for i in range(30, 201)]
fig, ax = plt.subplots(figsize=(6.2, 3.0), dpi=DPI)
ax.plot(xsx, [5 for _ in xsx], color=MUTED, ls="--", lw=1.0, label="고치기 전(웹) 5 px 고정")
ax.plot(xsx, [dot(x) for x in xsx], color=ACC, lw=2.0, label="고친 뒤(웹)")
for name, s in (("폰 폭 390", 0.60), ("태블릿 820", 1.36), ("노트북", 1.50)):
    ax.scatter([s], [dot(s)], color=INK, zorder=3)
    ax.text(s, dot(s) + 0.25, name, ha="center", fontsize=8, color=INK)
write_csv("marker_dot_size.csv", ["display_scale_px_per_cell", "dot_px_web"], [[round(x, 2), round(dot(x), 3)] for x in xsx[::10]])
ax.set_ylim(0, 6); ax.set_xlabel("지도가 그려지는 배율 (지도 1칸 = 화면 px)", color=INK, fontsize=9)
ax.set_ylabel("장소 점 지름 (px)", color=INK, fontsize=9)
style(ax); ax.legend(fontsize=7.5, frameon=False, loc="lower right")
title(ax, "좁은 화면일수록 점을 줄여 지도 위 크기를 맞춘다(폰 폭 1/2)")
fig.tight_layout(); fig.savefig(FIGS / "fig06_marker_scale.png"); plt.close(fig)

# ---------------------------------------------------------------------------
# 7. 적용 기록 표(원자료)
# ---------------------------------------------------------------------------
write_csv("applies_1119.csv",
          ["time", "where", "action", "radius_m", "added", "removed", "erased_black", "confirm", "note"],
          [["19:35:10", "격리 노드(고치기 전)", "scan_apply", 3.0, 29, 8, "", "응답 실종 → 10 s × 6 재요청, 19:36:11 확인",
            "로봇 뒤 0.4 m 차체 점 포함 → 되돌림"],
           ["19:44:47", "격리 노드", "undo", "", "", "", "", "응답", "원본 복원"],
           ["21:03:39", "운영", "scan_apply", 2.0, 14, 0, "", "/map(응답보다 먼저)", "몸 둘레 0칸, 1.55~1.83 m"],
           ["21:05:23", "운영", "undo", "", "", "", "", "/map", ""],
           ["21:05:46", "운영", "scan_apply", 2.0, 21, 0, "", "응답", "10칸이 로봇 앞 0.37~0.71 m"],
           ["21:06:04", "운영", "erase", "", "", "", 10, "/map", "그 10칸을 사각형 하나로 지움"],
           ["21:06:34", "운영", "undo", "", "", "", "", "/map", ""],
           ["21:06:37", "운영", "undo", "", "", "", "", "응답", "git 원본과 같아짐"]])
print("그림", sorted(p.name for p in FIGS.glob("*.png")))
print("자료", sorted(p.name for p in DATA.glob("*.csv")))
