"""발표 트러블슈팅 1 — reSpeaker 칩 최대 증폭 ×31.6 → ×2 전후 비교 막대그래프.

숫자 출처: ~/vica_data/mic_bystander/20261015-*/results.jsonl (10-15 연구실 시험, 녹음 152개)를
rescore.py 가 시험기 채점기로 다시 채점한 rescored.json. 수음 문턱 0.012 (−38.4 dB) 기준.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(Path.home() / 'vica_data' / 'analysis' / 'final_report' / 'scripts'))
import style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

TH = 0.012
rows = json.load(open(HERE / 'rescored.json'))


def frac(preset, roles, test):
    rs = [r for r in rows if r['preset'] == preset and r['role'] in roles]
    return sum(test(r) for r in rs), len(rs)


NEAR = ('near1', 'near2', 'near3')
USER = ('user', 'userwake')
FAR = ('far1', 'far15')
KEEP = USER + FAR
kept = lambda r: r['rms'] >= TH          # noqa: E731
clipped = lambda r: r['clip'] > 0.001    # noqa: E731

panels = [
    ('줄어야 할 소리', '낮을수록 좋음', [
        ('옆사람 잡담이 귀에 들어감 (1·2·3 m)', NEAR, kept),
        ('받아야 할 목소리가 찢어짐', KEEP, clipped),
    ]),
    ('지켜야 할 소리', '높을수록 좋음', [
        ('손잡이를 잡은 사용자 말을 받음', USER, kept),
        ('1~1.5 m 앞에서 한 접근 대답을 받음', FAR, kept),
    ]),
]

style.setup()
fig, axes = plt.subplots(1, 2, figsize=(13.3, 5.6), dpi=150)
fig.subplots_adjust(left=0.03, right=0.985, top=0.70, bottom=0.17, wspace=0.12)
fig.suptitle('칩 최대 증폭을 ×31.6에서 ×2로 낮추자, 옆사람 잡담은 걸러지고 사용자 말은 그대로 들렸다',
             x=0.012, ha='left', fontsize=17, fontweight='bold', y=0.965)
fig.text(0.012, 0.865, 'reSpeaker 마이크 칩 설정 전후 · 같은 문턱(수음 최소 rms 0.012) · 10-15 연구실 시험 녹음',
         fontsize=11.5, color=style.INK2)
fig.legend(handles=[Patch(color=style.CONTEXT, label='적용 전  최대 증폭 ×31.6 (공장값)'),
                    Patch(color=style.ACCENT, label='적용 후  최대 증폭 ×2')],
           loc='upper left', bbox_to_anchor=(0.006, 0.84), ncol=2, fontsize=12, handlelength=1.2,
           handleheight=1.0, columnspacing=2.0)

for ax, (title, hint, items) in zip(axes, panels):
    ax.set_xlim(0, 100)
    ax.set_ylim(-0.55, len(items) - 0.35)
    style.clean(ax, grid='x')
    ax.set_yticks([])
    ax.set_xticks([0, 50, 100])
    ax.set_xticklabels(['0 %', '50 %', '100 %'], fontsize=10.5)
    ax.set_title(title, fontsize=14, pad=10)
    ax.text(1.0, 1.035, hint, transform=ax.transAxes, ha='right', va='bottom', fontsize=10.5, color=style.MUTED)
    fig.canvas.draw()
    for i, (label, roles, test) in enumerate(items):
        y = len(items) - 1 - i
        ax.text(0, y + 0.36, label, fontsize=12.5, color=style.INK, va='bottom', fontweight='bold')
        for preset, dy, color, ink, weight in (('base', 0.17, style.CONTEXT, style.INK2, 'normal'),
                                                ('gain2', -0.17, style.ACCENT, style.INK, 'bold')):
            k, n = frac(preset, roles, test)
            pct = 100 * k / n
            style.hbar(ax, y + dy, pct, color, thick_pt=15)
            tx, ha = (pct + 1.2, 'left') if pct < 80 else (pct - 1.2, 'right')
            tc = ink if pct < 80 else ('white' if preset == 'gain2' else style.INK)
            ax.text(tx, y + dy, f'{pct:.0f} %  ({k}/{n})', va='center', ha=ha, fontsize=11.5,
                    color=tc, fontweight=weight)

style.note(fig, '받음·들어감 = 녹음 소리 크기가 문턱 이상 · 찢어짐 = 녹음의 0.1 % 넘게 최대치에 닿음 · '
                '적용 후에도 감지 유지: "비카야" 3/3, "멈춰" 3/3 (시험기 채점기로 다시 채점, 표본 적음)', y=0.02)
for ext in ('png', 'svg'):
    fig.savefig(HERE / f'mic_gain_before_after.{ext}', dpi=300)
print('저장', HERE / 'mic_gain_before_after.png')
