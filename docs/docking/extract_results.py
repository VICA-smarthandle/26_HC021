#!/usr/bin/env python3
"""도킹·해제 결과 줄을 모든 실기 로그에서 모아 CSV 로 만든다(보고서용, 11-19).
도킹: '도킹 끝: 앞뒤 X cm 좌우 Y cm 방향 Z° (시도 N)' · 해제: '해제 끝: 후진 X cm(넘침 Y) 회전 Z°'
걸린 시간: 같은 일의 '시작: approach|undock' 부터 결과 줄까지. 실패: '<action> 실패: <사유>'."""
import csv, glob, os, re, sys, time
BASE = '/home/ji_w/vica_data/docking_plan'
SRC = sorted(glob.glob(f'{BASE}/field_2026-11-18_integ/run*/node.log') +
             glob.glob(f'{BASE}/field_2026-11-19_integ/run0*_*/mission.log') +
             [f'{BASE}/report/normal_run_1724.log'])
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
DOCK = re.compile(r'도킹 끝: 앞뒤 ([+-]?[\d.]+) cm 좌우 ([+-]?[\d.]+) cm 방향 ([+-]?[\d.]+)° \(시도 (\d)\)')
UND = re.compile(r'해제 끝: 후진 ([\d.]+) cm\(넘침 ([+-]?[\d.]+)\) 회전 ([\d.]+)°')
UND_RESUME = re.compile(r'해제 끝: 남은 회전')
START = re.compile(r'시작: (approach|undock|retreat)')
FAIL = re.compile(r'(dock|undock) 실패: (\w+)')
REFINE = re.compile(r'고쳐 저장\((\w+) 에서\): ([\d.]+) cm · ([\d.]+)°')
rows, seen = [], set()
for path in SRC:
    run = path.split('/')[-2] if 'normal_run' not in path else '평소 유닛(17:24)'
    day = '11-18' if '11-18' in path else '11-19'
    t_start = {}
    for line in open(path, errors='replace'):
        if 'vica_docking' not in line:
            continue
        m = STAMP.search(line)
        if not m:
            continue
        t = float(m.group(1))
        key = (round(t, 3), line.strip()[-60:])
        if key in seen:
            continue
        seen.add(key)
        hhmm = time.strftime('%H:%M:%S', time.localtime(t))
        if (s := START.search(line)):
            t_start[s.group(1)] = t
            if s.group(1) == 'approach' and 'retreat' in t_start and t - t_start['retreat'] < 60:
                pass
        elif (d := DOCK.search(line)):
            t0 = t_start.get('approach')
            rows.append(dict(day=day, run=run, time=hhmm, kind='dock', result='success', x_cm=d.group(1), y_cm=d.group(2),
                             th_deg=d.group(3), attempt=d.group(4), secs=f'{t - t0:.1f}' if t0 else '', reason=''))
        elif (u := UND.search(line)):
            t0 = t_start.get('undock')
            rows.append(dict(day=day, run=run, time=hhmm, kind='undock', result='success', x_cm=u.group(1), y_cm=u.group(2),
                             th_deg=u.group(3), attempt='', secs=f'{t - t0:.1f}' if t0 else '', reason=''))
        elif (f := FAIL.search(line)):
            rows.append(dict(day=day, run=run, time=hhmm, kind=f.group(1), result='failed', x_cm='', y_cm='', th_deg='',
                             attempt='', secs='', reason=f.group(2)))
        elif (r := REFINE.search(line)):
            rows.append(dict(day=day, run=run, time=hhmm, kind='refine', result=r.group(1), x_cm=r.group(2), y_cm='',
                             th_deg=r.group(3), attempt='', secs='', reason=''))
with open(f'{BASE}/report/dock_results.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
from collections import Counter
c = Counter((r['kind'], r['result']) for r in rows)
print(len(rows), dict(c))
print(Counter(r['reason'] for r in rows if r['result'] == 'failed'))
