#!/usr/bin/env python3
"""보고서용 그림 3장(PNG, 300 dpi) — 수동 주행·전방 초음파 스위치 격리 시험(11-18)과 초음파·VCC 실기(11-14).

그림 1 은 e2e_cmd_timeline.csv(격리 끝단 시험: 실제 앱 provider → rosbridge → 가짜 로봇, 도메인 42·별도
네트워크 공간)에서, 그림 2·3 은 11-14 보고서(run102·104·106·107 bag 재생 분석) 숫자에서 그립니다.
실행: python3 make_figures.py  (이 폴더에서, 결과는 ../images/manual_drive/)
"""
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in ('NanumSquare', 'NanumGothic', 'Noto Sans CJK KR', 'Noto Sans CJK TC'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f; break
plt.rcParams['axes.unicode_minus'] = False
INK, MUTED, GRID, ACC, ACC2, WARN = '#1f2a2e', '#5f6f73', '#d9e1e2', '#1d7a5f', '#2f6db0', '#c2563d'
OUT = '../images/manual_drive/'

def style(ax):
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9); ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)

# 그림 1: 앱이 누르고 뗄 때 로봇이 받은 /cmd_vel_req(격리 시험)
rows = list(csv.DictReader(open('e2e_cmd_timeline.csv')))
cmd = [r for r in rows if r['ev'] == 'cmd_vel_req']
t = [float(r['t']) for r in cmd]; v = [float(r['v']) for r in cmd]; w = [float(r['wz']) for r in cmd]
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 4.6), dpi=300, sharex=True,
                               gridspec_kw={'height_ratios': [3, 1.1]})
ax1.step(t, v, where='post', color=ACC, lw=1.6, label='직진 명령 (m/s)')
ax1.step(t, w, where='post', color=ACC2, lw=1.2, ls='--', label='회전 명령 (rad/s)')
ax1.plot(t, v, 'o', ms=2.2, color=ACC)
ax1.axhline(0, color=MUTED, lw=0.6)
notes = [(0.0, 0.78, '0.7 누름'), (0.63, 0.40, '0.3으로 줄임'), (0.93, -0.42, '뒤로'), (1.40, 0.10, '뗌'),
         (2.35, 0.60, '선회 누름'), (2.82, 0.10, '뗌'), (3.76, 0.40, '홈 조종판 0.3'), (4.12, 0.10, '뗌')]
for x, y, s in notes:
    ax1.annotate(s, (x, y), fontsize=7.5, color=INK)
ax1.set_ylim(-0.55, 0.9); ax1.set_ylabel('명령', color=INK, fontsize=9)
ax1.legend(fontsize=8, frameon=False, loc='upper right')
ax1.set_title('누르는 동안만 나가고, 출발은 0.15씩(가속 3.0), 줄이기·반대·떼기는 바로', fontsize=10.5, color=INK, loc='left')
style(ax1)
ev = {r['ev']: float(r['t']) for r in rows if r['src'] == 'robot' and r['ev'] in ('set_bool',)}
sb = [(float(r['t']), r['note']) for r in rows if r['ev'] == 'set_bool']
cl = [float(r['t']) for r in rows if r['ev'] == 'clear']
for x, n in sb:
    ax2.axvline(x, color=WARN if 'False' in n else ACC, lw=1.4)
    ax2.text(x + 0.05, 0.62, '초음파 끄기' if 'False' in n else '초음파 켜기', fontsize=7.5, color=INK)
for x in cl:
    ax2.axvline(x, color=MUTED, lw=1.0, ls=':')
if cl:
    ax2.text(cl[0] - 0.05, 0.18, f'costmap 비움 ×{len(cl)} (끈 뒤 {cl[0]-sb[0][0]:.2f} s)', fontsize=7.5, color=MUTED, ha='right')
ax2.set_yticks([]); ax2.set_ylim(0, 1); ax2.set_xlabel('시간 (s, 첫 누름 = 0)', color=INK, fontsize=9)
style(ax2); ax2.grid(False)
fig.tight_layout(); fig.savefig(OUT + 'fig1_e2e_cmd.png'); plt.close(fig)

# 그림 2: VCC 새 경로 누적 밀림 수정 전후(run106 → run107, 같은 지도 vica_map_0630)
metrics = [('레일에서 옆 거리 중앙 (m)', 0.32, 0.20), ('0.6 m 넘게 벗어난 시간 (%)', 29, 7.6),
           ('offset 다시 맞춤 (번/분)', 6.6, 2.9), ('차선 빈 뒤 복귀 중앙 (s)', 1.4, 0.8)]
fig, axes = plt.subplots(1, 4, figsize=(8.4, 2.6), dpi=300)
for ax, (name, a, b) in zip(axes, metrics):
    bars = ax.bar(['수정 전\nrun106', '수정 후\nrun107'], [a, b], color=[MUTED, ACC], width=0.6)
    for bar, val in zip(bars, (a, b)):
        ax.text(bar.get_x() + bar.get_width() / 2, val, f'{val:g}', ha='center', va='bottom', fontsize=8.5, color=INK)
    ax.set_title(name, fontsize=8.5, color=INK); ax.set_ylim(0, max(a, b) * 1.25); style(ax); ax.grid(axis='x', visible=False)
fig.suptitle('VCC 새 경로마다 목표 차선도 같이 옮김(6695638) — 실기 비교', fontsize=10.5, color=INK, x=0.01, ha='left')
fig.tight_layout(); fig.savefig(OUT + 'fig2_vcc_compare.png'); plt.close(fig)

# 그림 3: 앞 통로 장애물 칸·초 나눔(run102 실주행·run104 정지 시험, 초음파 층만 재생)
fig, ax = plt.subplots(figsize=(7.2, 2.5), dpi=300)
runs = [('run102 실주행 (2324 s)', 79.6, 3.0, 17.4), ('run104 정지 시험 (1234 s)', 81.2, 2.5, 16.3)]
for i, (name, a, b, c) in enumerate(runs):
    left = 0
    for val, col, lab in ((a, ACC, '감지 중'), (b, WARN, '빔 안 잔상'), (c, '#b9c4c6', '빔 밖 기억')):
        ax.barh(i, val, left=left, color=col, height=0.55, label=lab if i == 0 else None)
        if val >= 6:
            ax.text(left + val / 2, i, f'{val:g} %', ha='center', va='center', fontsize=8, color='white' if col != '#b9c4c6' else INK)
        else:  # 좁은 칸은 막대 위에
            ax.text(left + val / 2, i - 0.40, f'{val:g} %', ha='center', va='center', fontsize=8, color=WARN)
        left += val
ax.set_yticks(range(len(runs))); ax.set_yticklabels([r[0] for r in runs], fontsize=8.5, color=INK)
ax.set_xlim(0, 100); ax.set_xlabel('앞 통로 장애물 칸·초 중 비율 (%)', fontsize=9, color=INK)
ax.legend(fontsize=8, frameon=False, ncol=3, loc='upper center', bbox_to_anchor=(0.5, 1.35))
style(ax); ax.set_ylim(1.45, -0.75)
fig.tight_layout(); fig.savefig(OUT + 'fig3_us_ghost.png'); plt.close(fig)
print('ok')
