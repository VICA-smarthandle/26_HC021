#!/usr/bin/env python3
"""도킹 보고서 그림 5~13(11-19 추가) — 기능을 정한 근거가 된 11-17·18 단독 시험과 11-19 실기 원자료에서.
각 그림의 원자료 경로는 그림 함수 주석에 적었다. 출력: fig05~fig13_*.png(300 dpi), data/*.csv"""
import csv, json, math, os, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in ('NanumSquare', 'Noto Sans CJK KR'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f; break
plt.rcParams['axes.unicode_minus'] = False
INK, MUTED, GRID, ACC, ACC2, WARN, GOLD = '#1f2a2e', '#5f6f73', '#d9e1e2', '#1d7a5f', '#2f6db0', '#c2563d', '#b98a12'
B = '/home/ji_w/vica_data/docking_plan'
os.makedirs('data', exist_ok=True)

def style(ax, grid=True):
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8.5)
    if grid: ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)

def title(ax, t): ax.set_title(t, fontsize=10.5, color=INK, loc='left')

def read_csv(p): return list(csv.DictReader(open(p, encoding='utf-8-sig')))

# ── 5. V자 구조물 실측 (scan_2026-11-17_dock_placed/scan_2008.npz, vfit_result_2008.txt) ──
def fig05():
    d = np.load(f'{B}/scan_2026-11-17_dock_placed/scan_2008.npz')
    R = d['ranges']; amin, inc = float(d['amin']), float(d['inc'])
    ang = amin + inc * np.arange(R.shape[1])
    fig, ax = plt.subplots(figsize=(6.2, 4.6), dpi=300)
    sel = np.abs(np.degrees(ang)) < 70
    for row in R[::4]:
        ok = np.isfinite(row) & (row > 0.15) & (row < 0.6) & sel
        ax.scatter(row[ok] * np.cos(ang[ok]), row[ok] * np.sin(ang[ok]), s=1.2, c=MUTED, alpha=0.35, lw=0)
    apex = (0.2813, 0.022); ra = (0.188, -0.158); la = (0.174, 0.189)
    ax.plot([ra[0], apex[0], la[0]], [ra[1], apex[1], la[1]], c=ACC, lw=2, label='직선 맞춤(팔 2개)')
    ax.scatter([apex[0]], [apex[1]], c=ACC, s=30, zorder=4)
    ax.scatter([0], [0], marker='^', c=ACC2, s=60, zorder=4, label='라이다')
    ax.annotate('꼭짓점 0.281 m\n내각 120.5° (±0.22)', apex, xytext=(0.33, 0.06), fontsize=8.5, color=INK, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.7))
    ax.annotate('팔 0.203 m', (0.235, -0.07), xytext=(0.29, -0.16), fontsize=8.5, color=INK, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.7))
    ax.annotate('팔 0.199 m', (0.228, 0.105), xytext=(0.29, 0.17), fontsize=8.5, color=INK, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.7))
    ax.set_aspect('equal'); ax.set_xlim(-0.03, 0.45); ax.set_ylim(-0.25, 0.25)
    ax.set_xlabel('라이다 앞 (m)', fontsize=9); ax.set_ylabel('왼쪽 (m)', fontsize=9); style(ax)
    title(ax, 'V자 구조물 라이다 실측 — 정지 스캔 80장 겹침(11-17)')
    ax.legend(fontsize=8, frameon=False, loc='lower left')
    ax.text(0.44, -0.235, '빔별 거리 표준편차 중앙 0.63 mm · 직선 잔차 최대 0.72 mm', ha='right', fontsize=7.5, color=MUTED)
    fig.tight_layout(); fig.savefig('fig05_v_structure.png'); plt.close(fig)

# ── 6. 거리별 V자 인식률 (README 3절 표, field_2026-11-17/*.npz, 11-18 정정) ──
def fig06():
    cats = [('0.22 m', '범퍼 9.8 cm', 40, 40, ''), ('0.28 m', '범퍼 16.2 cm', 80, 80, ''), ('1.13 m', '범퍼 100.7 cm', 70, 80, ''),
            ('1.32 m', '범퍼 120 cm', None, None, '11-18 실기에서\n안정 인식'), ('1.62 m', '범퍼 ~150 cm', 0, 80, ''),
            ('2.05 m', '범퍼 ~190 cm', 0, 80, ''), ('2.5 m', '범퍼 ~240 cm', 0, 80, '')]
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=300)
    for i, (d, bb, k, n, note) in enumerate(cats):
        if k is None:
            ax.bar(i, 100, 0.6, color='none', edgecolor=ACC2, hatch='///', lw=0.8)
            ax.text(i, 103, note, ha='center', fontsize=6.8, color=ACC2)
        else:
            v = 100 * k / n
            ax.bar(i, max(v, 0.8), 0.6, color=ACC if v > 50 else WARN)
            ax.text(i, v + 3, f'{k}/{n}', ha='center', fontsize=7.5, color=INK)
    ax.axvspan(3.5, 4.5, color='#f6efe0', zorder=0); ax.text(4, 70, '한계\n1.32~1.62 m', ha='center', fontsize=7.5, color=GOLD)
    ax.set_xticks(range(len(cats))); ax.set_xticklabels([f'{c[0]}\n{c[1]}' for c in cats], fontsize=7)
    ax.set_ylim(0, 122); ax.set_ylabel('V자 인식률 (%)', fontsize=9)
    ax.set_xlabel('라이다 → 꼭짓점 거리(범퍼 기준)', fontsize=8.5); style(ax)
    title(ax, '거리별 V자 인식률 — 정지 스캔(11-17·18)')
    fig.tight_layout(); fig.savefig('fig06_v_range.png'); plt.close(fig)

# ── 7·8·9. 11-18 전진 접근 v2.1 (field_2026-11-18/run0*/trace.csv, runs_summary.csv) ──
RUNS = ['run02_102759', 'run03_103214', 'run04_103453', 'run05_103709', 'run06_104005', 'run07_104126', 'run08_104441', 'run09_104612']
def fig07():
    fig, ax = plt.subplots(figsize=(6.4, 4.4), dpi=300)
    cmap = plt.get_cmap('viridis')
    for i, r in enumerate(RUNS):
        t = read_csv(f'{B}/field_2026-11-18/{r}/trace.csv')
        x = [float(a['axis_x_m']) - 0.151 for a in t if a['axis_x_m']]
        y = [float(a['y_left_m']) * 100 for a in t if a['axis_x_m']]
        col = WARN if r.startswith('run08') else cmap(i / (len(RUNS) - 1) * 0.85)
        ax.plot(x, y, c=col, lw=1.3, alpha=0.95)
        ax.scatter([x[0]], [y[0]], c=[col], s=18, zorder=3)
        if r.startswith('run08'):
            ax.text(x[-1] - 0.02, y[-1] - 2, 'run08 중단\n(V자 1.5 s 놓침)', fontsize=7.2, color=WARN, ha='right', va='top')
    ax.axvline(0.14, color=ACC, lw=1, ls='--'); ax.text(0.15, 33, '도착 14 cm', fontsize=7.5, color=ACC)
    ax.axvline(1.0, color=MUTED, lw=0.8, ls=':'); ax.text(1.01, 33, '준비 위치 1 m', fontsize=7.5, color=MUTED)
    ax.axhspan(-30, 30, color='#f2f6f5', zorder=0)
    ax.set_xlim(1.35, 0.0); ax.set_ylim(-52, 38)
    ax.set_xlabel('범퍼 → V자 꼭짓점 (m) — 오른쪽이 도크', fontsize=9); ax.set_ylabel('좌우 (cm, 왼쪽 +)', fontsize=9); style(ax)
    title(ax, '전진 접근 궤적 — 도크 기준으로 본 로봇 위치(11-18, 8회)')
    ax.text(0.02, -48, '옅은 띠 = 좌우 출발 한계 ±30 cm', fontsize=7.2, color=MUTED, ha='right')
    fig.tight_layout(); fig.savefig('fig07_approach_tracks.png'); plt.close(fig)

def fig08():
    rows = read_csv(f'{B}/field_2026-11-18/runs_summary.csv')
    rows = [r for r in rows if r['run'] != 'run01_102518']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(6.4, 3.3), dpi=300)
    for r in rows:
        y0, th0 = float(r['start_y_m']) * 100, float(r['start_th_deg'])
        ok = not r['aborted']
        a1.scatter([y0], [th0], s=34, c=ACC if ok else WARN, marker='o' if ok else 'x', zorder=3)
        dx, ha = (-1.5, 'right') if r['run'][3:5] in ('02', '08') else (1.5, 'left')
        a1.text(y0 + dx, th0 + 1.5, r['run'][3:5], fontsize=6.8, color=MUTED, ha=ha)
    a1.add_patch(plt.Rectangle((-30, -30), 60, 60, fill=False, ec=ACC, ls='--', lw=0.9))
    a1.set_xlim(-55, 40); a1.set_ylim(-40, 40)
    a1.set_xlabel('출발 좌우 (cm)', fontsize=8.5); a1.set_ylabel('출발 방향 (°)', fontsize=8.5); style(a1)
    title(a1, '출발 자리 (○ 도착 · × 중단)')
    ok = [r for r in rows if not r['aborted']]
    names = [r['run'][3:5] for r in ok]
    a2.bar(np.arange(len(ok)) - 0.2, [abs(float(r['err_y_cm'])) for r in ok], 0.4, color=ACC2, label='|좌우| cm')
    a2.bar(np.arange(len(ok)) + 0.2, [abs(float(r['err_th_deg'])) for r in ok], 0.4, color=GOLD, label='|방향| °')
    a2.axhline(1.5, color=ACC2, lw=0.8, ls='--'); a2.axhline(5, color=GOLD, lw=0.8, ls='--')
    a2.set_xticks(range(len(ok))); a2.set_xticklabels(names, fontsize=7.5); a2.set_xlabel('회차', fontsize=8.5); style(a2)
    a2.legend(fontsize=7.2, frameon=False); title(a2, '도착 오차(점선 = 성공 기준)')
    fig.tight_layout(); fig.savefig('fig08_start_vs_result.png'); plt.close(fig)

def fig09():
    t = read_csv(f'{B}/field_2026-11-18/run04_103453/trace.csv')
    ts = np.array([float(a['t_s']) for a in t]); ph = [a['phase'] for a in t]
    def col(k, s=1.0): return np.array([float(a[k]) * s if a[k] else np.nan for a in t])
    x = col('axis_x_m') - 0.151; y = col('y_left_m', 100); th = col('th_deg'); v = col('cmd_v'); w = col('cmd_w')
    fig, axs = plt.subplots(3, 1, figsize=(6.4, 5.2), dpi=300, sharex=True)
    shade = {'WAIT': '#eef1f2', 'ALIGN': '#e6eef8', 'APPROACH': '#e8f3ef', 'STOPPED': '#f6efe0'}
    spans = []; s0 = 0
    for i in range(1, len(ph) + 1):
        if i == len(ph) or ph[i] != ph[s0]:
            spans.append((ts[s0], ts[i - 1], ph[s0])); s0 = i
    for ax in axs:
        for a, b, p in spans: ax.axvspan(a, b, color=shade.get(p, '#fff'), lw=0)
        style(ax)
    for a, b, p in spans:
        axs[0].text((a + b) / 2, 1.12, {'WAIT': '대기', 'ALIGN': '정렬', 'APPROACH': '직진', 'STOPPED': '정지·측정'}.get(p, p), ha='center', fontsize=7.5, color=INK)
    axs[0].plot(ts, x, c=ACC, lw=1.4); axs[0].set_ylabel('범퍼→꼭짓점 (m)', fontsize=8); axs[0].set_ylim(0, 1.2)
    axs[1].plot(ts, y, c=ACC2, lw=1.2, label='좌우 (cm)'); axs[1].plot(ts, th, c=GOLD, lw=1.2, label='방향 (°)')
    axs[1].axhline(0, color=MUTED, lw=0.6); axs[1].legend(fontsize=7, frameon=False, ncol=2, loc='lower right'); axs[1].set_ylabel('좌우·방향', fontsize=8)
    axs[2].plot(ts, v * 100, c=ACC, lw=1.2, label='직진 (cm/s)'); axs[2].plot(ts, np.degrees(w), c=WARN, lw=1.0, label='회전 (°/s)')
    axs[2].legend(fontsize=7, frameon=False, ncol=2); axs[2].set_ylabel('명령', fontsize=8); axs[2].set_xlabel('시간 (s)', fontsize=8.5)
    axs[0].set_title('한 번의 접근 — 출발 좌우 +14.5 cm · 방향 -15.4° (11-18 run04)', fontsize=10.5, color=INK, loc='left', pad=16)
    fig.tight_layout(); fig.savefig('fig09_approach_timeline.png'); plt.close(fig)

# ── 10. 넘침과 정지 앞당김 (field_2026-11-17/runs_summary.csv, field_2026-11-18/runs_summary.csv, report/dock_results.csv) ──
def fig10():
    r17 = read_csv(f'{B}/field_2026-11-17/runs_summary.csv'); r18 = [r for r in read_csv(f'{B}/field_2026-11-18/runs_summary.csv') if r['overshoot_cm']]
    ov = [float(r['overshoot_cm']) for r in r17] + [float(r['overshoot_cm']) for r in r18 if r['run'] != 'run01_102518']
    node = [float(r['x_cm']) for r in read_csv('dock_results.csv') if r['kind'] == 'dock' and r['result'] == 'success']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(6.4, 3.2), dpi=300, gridspec_kw=dict(width_ratios=[1.1, 1]))
    a1.bar(range(len(ov)), ov, color=[MUTED] * len(r17) + [ACC2] * (len(ov) - len(r17)))
    m = float(np.mean(ov[len(r17):])); a1.axhline(m, color=ACC2, lw=1, ls='--'); a1.text(-0.4, 1.72, f'11-18 평균 {m:.2f} cm → 그만큼 일찍 정지 명령', ha='left', fontsize=7.5, color=ACC2)
    a1.set_xticks(range(len(ov))); a1.set_xticklabels(['17-1', '17-2'] + [r['run'][3:5] for r in r18 if r['run'] != 'run01_102518'], fontsize=6.8)
    a1.set_ylabel('정지 명령 뒤 더 간 거리 (cm)', fontsize=8); a1.set_ylim(0, 1.9); style(a1); title(a1, '넘침(0.05 m/s 에서)')
    a2.axhspan(-1.5, 1.5, color='#e8f3ef', zorder=0)
    e0 = float(r17[0]['err_x_cm'])
    a2.scatter([1], [e0], c=WARN, s=36, zorder=3); a2.text(1.12, e0, f'{e0:+.2f} cm', va='center', fontsize=7.5, color=WARN)
    a2.scatter(np.random.default_rng(1).normal(2, 0.05, len(node)), node, c=ACC, s=12, zorder=3)
    a2.text(2.18, 0.0, f'{min(node):+.2f} ~ {max(node):+.2f} cm', va='center', fontsize=7.5, color=ACC)
    a2.set_xlim(0.5, 2.9)
    a2.set_xticks([1, 2]); a2.set_xticklabels(['앞당김 없음\n(11-17 v1)', f'앞당김\n(도킹 노드 {len(node)}회)'], fontsize=7.5)
    a2.set_ylabel('앞뒤 오차 (cm)', fontsize=8); a2.set_ylim(-1.8, 0.8); style(a2); title(a2, '도착 앞뒤 오차(띠 = 성공 기준)')
    fig.tight_layout(); fig.savefig('fig10_overshoot_lead.png'); plt.close(fig)

# ── 11. 해제 10회 (field_2026-11-18/undock_summary.csv, undockNN/trace.csv) ──
def fig11():
    rows = read_csv(f'{B}/field_2026-11-18/undock_summary.csv')
    names, back, minus, note = [], [], [], []
    for r in rows:
        tr = read_csv(f"{B}/field_2026-11-18/{r['run']}/trace.csv")
        vals = []
        for a in tr:
            if a['phase'] in ('TURN', 'SETTLE_TURN'):
                for k in ('us_rear_right_m', 'us_rear_left_m', 'us_side_rear_right_m', 'us_side_rear_left_m'):
                    if a.get(k) and a[k] not in ('', 'inf'):
                        v = float(a[k])
                        if math.isfinite(v): vals.append(v)
        names.append(r['run'][6:8]); back.append(float(r['start_bumper_apex_cm']) + (float(r['back_traveled_cm']) if r['back_traveled_cm'] else 0))
        minus.append(min(vals) if vals else np.nan)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.4, 4.6), dpi=300, sharex=True)
    a1.bar(range(len(rows)), back, color=[ACC2 if b > 80 else MUTED for b in back])
    a1.axhline(85, color=ACC2, lw=0.9, ls='--'); a1.text(len(rows) - 0.5, 86.5, '결정 85 cm', ha='right', fontsize=7.5, color=ACC2)
    a1.set_ylabel('후진 뒤 범퍼→꼭짓점 (cm)', fontsize=8); a1.set_ylim(0, 112); style(a1)
    title(a1, '도크에서 나오기 10회 — 후진 거리와 회전 중 뒤 여유(11-18)')
    a2.bar(range(len(rows)), [m * 100 if m == m else 0 for m in minus], color=[WARN if (m == m and m < 0.25) else ACC for m in minus])
    a2.axhline(25, color=WARN, lw=0.9, ls='--'); a2.text(-0.45, 50, '빨강 = 회전 정지 기준 0.25 m 안으로 들어옴', ha='left', fontsize=7.5, color=WARN)
    for i, m in enumerate(minus):
        a2.text(i, (m * 100 if m == m else 0) + 1.5, '없음' if m != m else f'{m*100:.0f}', ha='center', fontsize=7, color=INK)
    a2.set_xticks(range(len(rows))); a2.set_xticklabels(names, fontsize=8); a2.set_xlabel('회차(undock)', fontsize=8.5)
    a2.set_ylabel('회전 중 초음파 최소 (cm)', fontsize=8); a2.set_ylim(0, 60); style(a2)
    fig.tight_layout(); fig.savefig('fig11_undock_trials.png'); plt.close(fig)
    with open('data/undock_trials_1118.csv', 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['회차', '출발_범퍼꼭짓점_cm', '후진_cm', '후진뒤_범퍼꼭짓점_cm', '회전_deg', '회전중_초음파최소_m', '중단'])
        for r, b, m in zip(rows, back, minus):
            w.writerow([r['run'], r['start_bumper_apex_cm'], r['back_traveled_cm'], f'{b:.1f}', r['turned_deg'], '' if m != m else f'{m:.3f}', r['aborted']])

# ── 12. 상자가 V자를 가린 순간 (field_2026-11-19_integ/run05_app/rec/scans.npz, events.csv: 284.1 s lost) ──
def fig12():
    d = np.load(f'{B}/field_2026-11-19_integ/run05_app/rec/scans.npz')
    t = d['t']; R = d['ranges']; amin, inc = float(d['amin']), float(d['inc'])
    ang = amin + inc * np.arange(R.shape[1]) - 0.0346
    def pts(ti):
        i = int(np.argmin(np.abs(t - ti))); r = R[i]; ok = np.isfinite(r) & (r > 0.15) & (r < 2.5)
        return 0.031 + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
    fig, axs = plt.subplots(1, 2, figsize=(6.6, 3.6), dpi=300, sharey=True)
    for ax, ti, lab in ((axs[0], 281.3, '281.3 s — V자 보임(직진 시작)'), (axs[1], 283.9, '283.9 s — 상자가 가림 → 3 s 뒤 lost')):
        x, y = pts(ti)
        ax.scatter(x, y, s=2, c=INK, lw=0)
        ax.add_patch(plt.Rectangle((-0.55, -0.225), 0.70, 0.45, fill=False, ec=ACC2, lw=1))
        ax.text(-0.2, 0, 'VICA', ha='center', va='center', fontsize=8, color=ACC2)
        ax.add_patch(plt.Rectangle((0.151, -0.225), 0.30, 0.45, fc='#f8e9e2', ec=WARN, lw=0.6, ls='--', alpha=0.6))
        ax.set_aspect('equal'); ax.set_xlim(-0.6, 1.6); ax.set_ylim(-0.9, 0.9); style(ax)
        ax.set_title(lab, fontsize=8.5, color=INK, loc='left'); ax.set_xlabel('로봇 앞 (m)', fontsize=8)
    axs[0].set_ylabel('왼쪽 (m)', fontsize=8)
    axs[1].text(0.16, -0.85, '빨간 칸 = 범퍼 앞 30 cm 감시 구역', fontsize=7, color=WARN)
    fig.suptitle('상자가 30 cm 밖에서 V자를 가린 장면(11-19 run05) — 이후 "가려짐 = 막힘"으로 바꿈', fontsize=9.5, color=INK, x=0.02, ha='left')
    fig.tight_layout(); fig.savefig('fig12_occlusion_scan.png'); plt.close(fig)

# ── 13. 지도 위 도크 위치 비교 (maps/vica_map_0630.*, 11-19 run07 로그·dock.yaml) ──
def fig13():
    import yaml
    my = yaml.safe_load(open('/home/ji_w/VICA-smarthandle/vica_ros2_ws/maps/vica_map_0630.yaml'))
    raw = open('/home/ji_w/VICA-smarthandle/vica_ros2_ws/maps/vica_map_0630.pgm', 'rb').read()
    parts = raw.split(b'\n', 4); w, h = map(int, parts[1].split() if not parts[1].startswith(b'#') else parts[2].split())
    img = np.frombuffer(raw[-w * h:], dtype=np.uint8).reshape(h, w)
    res = my['resolution']; ox, oy = my['origin'][:2]
    x0, x1, y0, y1 = -9.0, -5.0, 0.6, 4.6
    c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res); r0, r1 = h - int((y1 - oy) / res), h - int((y0 - oy) / res)
    fig, ax = plt.subplots(figsize=(6.0, 5.4), dpi=300)
    ax.imshow(img[r0:r1, c0:c1], cmap='gray', extent=(x0, x1, y0, y1), vmin=0, vmax=255)
    P = [('오전 확인된 자리(도킹 때 AMCL 일치)', -7.0345, 2.4297, 3.11278, ACC, 'o'),
         ('16:33 인식 저장(123 cm)', -7.2736, 3.4273, -3.0959, WARN, 's'),
         ('16:35 지도에서 지정', -6.89, 2.55, math.radians(-178.4), GOLD, 'D'),
         ('16:38 인식 저장(126 cm)', -6.991, 2.927, 3.01263, WARN, '^'),
         ('16:38 처음 도킹 때 고쳐 저장', -7.4246, 2.7116, 3.08832, ACC2, 'v')]
    for lab, x, y, f, c, m in P:
        ax.scatter([x], [y], c=c, marker=m, s=46, edgecolors='white', linewidths=0.6, zorder=4, label=lab)
        ax.annotate('', xy=(x, y), xytext=(x - 0.45 * math.cos(f), y - 0.45 * math.sin(f)), arrowprops=dict(arrowstyle='->', color=c, lw=1))
    ax.annotate('지도에 그려진 옛 도크', (-7.15, 2.25), xytext=(-6.6, 1.95), fontsize=7.5, color=INK, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.7))
    ax.annotate('벽', (-7.28, 3.95), xytext=(-6.9, 4.05), fontsize=7.5, color=INK, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.7))
    ax.set_xlabel('지도 x (m)', fontsize=8.5); ax.set_ylabel('지도 y (m)', fontsize=8.5); ax.tick_params(labelsize=8)
    ax.legend(fontsize=7, loc='lower right', framealpha=0.92)
    ax.set_title('지도 위 도크 꼭짓점 — 도크를 옮긴 뒤(11-19 오후)', fontsize=10.5, color=INK, loc='left')
    ax.text(x0 + 0.05, y1 - 0.12, '화살표 = 로봇이 도크를 보는 방향 · 바탕 = 주행 지도(옛 도크가 그려진 상태)', fontsize=6.8, color=INK, va='top',
            bbox=dict(fc='white', ec='none', alpha=0.8, pad=1.5))
    fig.tight_layout(); fig.savefig('fig13_dock_positions_map.png'); plt.close(fig)

# ── 표: 11-17·18 접근 회차 ──
def tables():
    import shutil
    shutil.copy(f'{B}/field_2026-11-17/runs_summary.csv', 'data/approach_runs_1117.csv')
    shutil.copy(f'{B}/field_2026-11-18/runs_summary.csv', 'data/approach_runs_1118.csv')
    shutil.copy('dock_results.csv', 'data/dock_results_1118_1119.csv')

for fn in (fig05, fig06, fig07, fig08, fig09, fig10, fig11, fig12, fig13, tables):
    try:
        fn(); print('ok', fn.__name__)
    except Exception as e:
        import traceback; traceback.print_exc(); print('FAIL', fn.__name__, e)
