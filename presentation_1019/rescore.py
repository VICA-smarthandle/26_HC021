"""10-15 마이크 주변 대화 시험 녹음을 시험기 채점기(WakeScorer, 앞 1.5 s 붙임)로 다시 채점한다. 칩·마이크 안 씀."""
import glob, json, sys, wave
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path.home() / "wt-stt-qwen"))
from tools.mic_bystander_check import WakeScorer, summarize

DATA = Path.home() / "vica_data" / "mic_bystander"
OUT = Path.home() / "vica_data" / "presentation_1019"
rows = []
for f in sorted(DATA.glob("20261015-*/results.jsonl")):
    for line in open(f):
        r = json.loads(line); r["sess"] = f.parent.name; rows.append(r)
ws = WakeScorer()
for r in rows:
    with wave.open(str(DATA / r["sess"] / r["wav"])) as w:
        a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
    r["wake_a_orig"], r["stop_b_orig"] = r["wake_a"], r["stop_b"]
    r["wake_a"], r["stop_b"] = ws.score(a)
json.dump(rows, open(OUT / "rescored.json", "w"), ensure_ascii=False, indent=1)
s = summarize(rows, 0.012, 0.7, 0.5)
json.dump(s, open(OUT / "summary_rescored.json", "w"), ensure_ascii=False, indent=1)
for p in s["presets"]:
    print(p["label"], "| 옆사람 통과", p["bystander_pass_now"], "| 사용자 버림", p["user_drop_now"], "| 앞쪽 버림", p["far_drop_now"],
          "| 비카야", p["wake"], "| 이어말하기", p["wake_cont"], "| 멈춰", p["stop"], "| 2m 외침", p["shout2"], "| 여유", p["margin_db"])
