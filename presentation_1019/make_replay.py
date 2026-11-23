"""시험기 웹페이지(mic_bystander_page.html)를 1차 시험(10-15 19:15) 저장 결과로 다시 띄우는 정적 HTML 을 만든다."""
import json
from pathlib import Path

PAGE = Path.home() / "wt-stt-qwen" / "tools" / "mic_bystander_page.html"
SESS = Path.home() / "vica_data" / "mic_bystander" / "20261015-191514"
OUT = Path(__file__).resolve().parent / "replay_1015_s1.html"

recs = [json.loads(l) for l in open(SESS / "results.jsonl")]
import sys
sys.path.insert(0, str(Path.home() / "wt-stt-qwen"))
from tools.mic_bystander_check import summarize
summary = summarize(recs, 0.012, 0.7, 0.5)
gain2_chip = next(r["chip"] for r in recs if r["preset"] == "gain2")
presets = [("base", "지금 설정", "⑫ 와 같음(.env 값 — 증폭 한도 줄이 없으면 ×31.6)"),
           ("gain8", "증폭 최대 ×8", "먼 소리를 덜 키운다"),
           ("gain2", "증폭 최대 ×2", "먼 소리를 거의 안 키운다"),
           ("agcoff", "증폭 끔(고정 ×2)", "거리 차이가 그대로 남는다"),
           ("vad3", "말소리 감지 둔감 ×3", "칩이 약한 목소리를 말로 덜 본다"),
           ("gain2vad3", "증폭 ×2 + 감지 둔감 ×3", "둘 다")]
roles = [("user", "사용자(손잡이)"), ("near1", "옆사람 1 m"), ("near2", "옆사람 2 m"), ("near3", "옆사람 3 m")]
state = {"type": "state", "preset": "gain2", "role": recs[-1]["role"], "phrase": "",
         "presets": [{"key": k, "label": l, "note": n} for k, l, n in presets],
         "roles": [{"key": k, "label": l} for k, l in roles],
         "listen_min_rms": summary["listen_min_rms"], "chip": gain2_chip}
msgs = [state] + [{"type": "result", **r} for r in recs] + [state, {"type": "summary", **summary}]

html = PAGE.read_text(encoding="utf-8")
replay = "const REPLAY = " + json.dumps(msgs, ensure_ascii=False) + ";\n  REPLAY.forEach(onMessage);\n"
assert html.count("  connect();\n") == 1
html = html.replace("  connect();\n", replay)
html = html.replace("</head>", "<style>html{zoom:3}</style></head>", 1)
OUT.write_text(html, encoding="utf-8")
print(OUT, len(msgs), "메시지")
