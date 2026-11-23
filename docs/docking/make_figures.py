#!/usr/bin/env python3
"""보고서용 그림 4장(PNG, 300 dpi)과 요약 표(markdown) — dock_results.csv 와 README 11-18 표에서(11-19)."""
import csv, statistics as st
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in ('NanumSquare', 'Noto Sans CJK KR', 'Noto Sans CJK TC'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f; break
plt.rcParams['axes.unicode_minus'] = False
INK, MUTED, GRID, ACC, ACC2, WARN = '#1f2a2e', '#5f6f73', '#d9e1e2', '#1d7a5f', '#2f6db0', '#c2563d'
rows = list(csv.DictReader(open('dock_results.csv')))
dock = [r for r in rows if r['kind'] == 'dock' and r['result'] == 'success']
und = [r for r in rows if r['kind'] == 'undock' and r['result'] == 'success' and float(r['x_cm']) > 1]
# 11-18 단독 도구(전진 접근 v2.1, 같은 구조물) — README 8.1
tool = [(-0.18, -1.06, 1.52), (-0.04, -0.96, -0.22), (-0.30, -0.82, 0.77), (-0.22, -0.48, 0.58),
        (-0.33, -0.69, -0.36), (-0.14, -0.26, -0.85), (-0.08, -1.25, 4.09)]

def style(ax):
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9); ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)

# 그림 1: 도킹 최종 오차 — 좌우(cm) × 방향(°), 성공 기준 상자
fig, ax = plt.subplots(figsize=(6.2, 4.4), dpi=300)
ax.add_patch(plt.Rectangle((-1.5, -5), 3, 10, fill=True, fc='#e8f3ef', ec=ACC, lw=1, ls='--'))
ax.scatter([float(r['y_cm']) for r in dock], [float(r['th_deg']) for r in dock], s=34, c=ACC, label=f'도킹 노드(11-18~19, {len(dock)}회)', zorder=3)
ax.scatter([t[1] for t in tool], [t[2] for t in tool], s=34, marker='^', c=ACC2, label='단독 시험 도구 v2.1(11-18, 7회)', zorder=3)
ax.set_xlim(-2.2, 2.2); ax.set_ylim(-6, 6)
ax.set_xlabel('좌우 오차 (cm, 왼쪽 +)', color=INK, fontsize=10); ax.set_ylabel('방향 오차 (°, 반시계 +)', color=INK, fontsize=10)
ax.text(1.42, 4.75, '성공 기준\n좌우 ±1.5 cm · 방향 ±5°', ha='right', va='top', fontsize=8, color=ACC)
style(ax); ax.legend(fontsize=8, frameon=False, loc='lower right')
ax.set_title('도킹 최종 오차 — 범퍼→꼭짓점 14 cm 에서', fontsize=11, color=INK, loc='left')
fig.tight_layout(); fig.savefig('fig1_dock_error.png'); plt.close(fig)

# 그림 2: 앞뒤 오차(목표 14 cm 대비)
fig, ax = plt.subplots(figsize=(6.2, 2.8), dpi=300)
xs = [float(r['x_cm']) for r in dock]
ax.axvspan(-1.5, 1.5, color='#e8f3ef'); ax.axvline(0, color=MUTED, lw=0.8)
ax.scatter(xs, [1] * len(xs), s=40, c=ACC, alpha=0.8, zorder=3)
ax.scatter([t[0] for t in tool], [0] * len(tool), s=40, marker='^', c=ACC2, alpha=0.8, zorder=3)
ax.set_yticks([0, 1]); ax.set_yticklabels(['단독 도구 v2.1', '도킹 노드']); ax.set_ylim(-0.7, 1.7); ax.set_xlim(-1.7, 1.7)
ax.set_xlabel('앞뒤 오차 (cm, + = 목표보다 덜 들어감)', color=INK, fontsize=10); style(ax)
ax.set_title(f'앞뒤 오차 — 도킹 노드 평균 {st.mean(xs):+.2f} cm, 최대 |{max(abs(v) for v in xs):.2f}| cm', fontsize=11, color=INK, loc='left')
fig.tight_layout(); fig.savefig('fig2_dock_depth.png'); plt.close(fig)

# 그림 3: 걸린 시간
fig, ax = plt.subplots(figsize=(6.2, 3.2), dpi=300)
ds = [float(r['secs']) for r in dock if r['secs']]; us = [float(r['secs']) for r in und if r['secs']]
bp = ax.boxplot([ds, us], vert=False, widths=0.5, patch_artist=True, showfliers=False,
           whiskerprops=dict(color=MUTED), capprops=dict(color=MUTED))
for box, med, fc, ec in zip(bp['boxes'], bp['medians'], ('#e8f3ef', '#e6eef8'), (ACC, ACC2)):
    box.set(facecolor=fc, edgecolor=ec); med.set(color=ec, linewidth=2)
ax.scatter(ds, [1] * len(ds), s=14, c=ACC, zorder=3); ax.scatter(us, [2] * len(us), s=14, c=ACC2, zorder=3)
ax.set_yticks([1, 2]); ax.set_yticklabels([f'들어가기\n(준비 위치→14 cm, {len(ds)}회)', f'나오기\n(후진 71 cm+180°, {len(us)}회)'])
ax.set_xlabel('걸린 시간 (s) — 도킹 노드가 일을 받은 때부터 끝날 때까지', color=INK, fontsize=9); style(ax)
ax.set_title(f'걸린 시간 — 들어가기 중앙 {st.median(ds):.1f} s · 나오기 중앙 {st.median(us):.1f} s', fontsize=11, color=INK, loc='left')
fig.tight_layout(); fig.savefig('fig3_duration.png'); plt.close(fig)

# 그림 4: 나오기 — 후진 거리와 회전각
fig, (a1, a2) = plt.subplots(1, 2, figsize=(6.2, 3.0), dpi=300)
bx = [float(r['x_cm']) for r in und]; tt = [float(r['th_deg']) for r in und]
a1.scatter(range(1, len(bx) + 1), bx, c=ACC2, s=26); a1.axhline(71.0, color=MUTED, ls='--', lw=0.8)
a1.set_ylim(68, 74); a1.set_xlabel('회차', fontsize=9); a1.set_ylabel('후진 거리 (cm)', fontsize=9); style(a1)
a1.set_title(f'후진 평균 {st.mean(bx):.1f} cm', fontsize=10, color=INK, loc='left')
a2.scatter(range(1, len(tt) + 1), tt, c=ACC2, s=26); a2.axhline(180, color=MUTED, ls='--', lw=0.8)
a2.set_ylim(176, 184); a2.set_xlabel('회차', fontsize=9); a2.set_ylabel('회전각 (°)', fontsize=9); style(a2)
a2.set_title(f'회전 평균 {st.mean(tt):.1f}°', fontsize=10, color=INK, loc='left')
fig.suptitle('도크에서 나오기 — 범퍼→꼭짓점 85 cm 까지 후진 후 180° 회전', fontsize=11, color=INK, x=0.02, ha='left')
fig.tight_layout(); fig.savefig('fig4_undock.png'); plt.close(fig)

# 요약 표
def line(name, v, unit):
    return f'| {name} | {len(v)} | {st.mean(v):+.2f} {unit} | {st.pstdev(v):.2f} | {min(v):+.2f} ~ {max(v):+.2f} |'
md = ['# 도킹 실기 결과 요약(11-18~19, dock_results.csv 에서 자동 계산)', '',
      '| 항목 | 회수 | 평균 | 표준편차 | 범위 |', '| --- | --- | --- | --- | --- |',
      line('도킹 앞뒤 오차', xs, 'cm'), line('도킹 좌우 오차', [float(r['y_cm']) for r in dock], 'cm'),
      line('도킹 방향 오차', [float(r['th_deg']) for r in dock], '°'), line('도킹 걸린 시간', ds, 's'),
      line('나오기 후진 거리', bx, 'cm'), line('나오기 넘침', [float(r['y_cm']) for r in und], 'cm'),
      line('나오기 회전각', tt, '°'), line('나오기 걸린 시간', us, 's'), '',
      '실패·중단(성공 기준 밖이 아니라 멈춘 경우): ' + ', '.join(f"{r['day']} {r['time']} {r['reason']}" for r in rows if r['result'] == 'failed')]
open('summary_tables.md', 'w').write('\n'.join(md) + '\n')
print('\n'.join(md))
