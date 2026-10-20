#!/usr/bin/env python3
"""주행 중 "지금 이상하다" 순간을 찍는다."""

import os
import sys
import time
import datetime

name = sys.argv[1] if len(sys.argv) > 1 else "mark"
out_dir = os.path.expanduser("~/vica_data/marks")
os.makedirs(out_dir, exist_ok=True)
path = os.path.join(out_dir, f"{name}.csv")

new = not os.path.exists(path)
f = open(path, "a", buffering=1)
if new:
    f.write("epoch,시각,경과초,메모\n")

t0 = time.time()
print(f"표시 파일: {path}")
print("엔터=찍기   글+엔터=메모 붙여 찍기   q+엔터=끝\n")

n = 0
while True:
    try:
        s = input()
    except (EOFError, KeyboardInterrupt):
        break
    if s.strip().lower() in ("q", "quit", "exit"):
        break
    now = time.time()
    stamp = datetime.datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3]
    note = s.strip().replace(",", " ")
    f.write(f"{now:.3f},{stamp},{now - t0:.1f},{note}\n")
    n += 1
    print(f"  [{n}] {stamp}  (+{now - t0:.0f}초) {note}")

f.close()
print(f"\n표시 {n}개 저장: {path}")
