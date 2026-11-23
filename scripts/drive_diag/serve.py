#!/usr/bin/env python3
"""주행 진단 뷰어 — x·y·θ 오차(① 위치추정 ② 오도메트리) 오프라인 화면.

    source /opt/ros/humble/setup.bash
    python3 scripts/drive_diag/serve.py            # http://127.0.0.1:8765
    # 노트북에서 보기:  ssh -L 8765:127.0.0.1:8765 <젯슨>  → 노트북 브라우저 http://127.0.0.1:8765

녹화된 bag 만 읽는다(주행 중인 ROS 에 붙지 않음, 노드 추가 없음). 분석 결과는 캐시에 남아 다음엔 바로 열린다.
기본 주소는 127.0.0.1 — 같은 와이파이의 다른 기기에 열지 않는다(--host 로 바꿀 수 있음).
"""
import argparse
import io
import json
import math
import os
import sys
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import locerr as le  # noqa: E402
import evaluate as ev  # noqa: E402

D = math.degrees
PNG_1PX = bytes.fromhex('89504e470d0a1a0a0000000d4948445200000001000000010806000000'
                        '1f15c4890000000d49444154789c6360000002000154a24f5d0000000049454e44ae426082')


# ── 화면용 데이터(HTTP 없이 시험) ────────────────────────

def _r(a, n=4):
    return [round(float(v), n) for v in np.asarray(a).ravel()]


def _decimate(t, *arrs, step=0.1):
    """시간 간격 step 마다 하나."""
    t = np.asarray(t)
    if len(t) == 0:
        return (t,) + tuple(np.asarray(a) for a in arrs)
    keep = np.r_[True, np.diff(np.floor((t - t[0]) / step)) > 0]
    return (t[keep],) + tuple(np.asarray(a)[keep] for a in arrs)


def build_result(z):
    t0 = float(z['odom_t'][0])
    out = dict(t0=t0, t_end=float(z['odom_t'][-1] - t0), sensor=_r(z['sensor'], 5),
               has_match='m_t' in z and len(z['m_t']) > 0, bag=str(z['bag']), map_yaml=str(z['map_yaml']))
    if out['has_match']:
        out['tf_shift'] = float(z['tf_shift'])
        out['map'] = json.loads(str(z['map_meta']))
        out['match'] = dict(
            t=_r(z['m_t'] - t0, 3), amcl_x=_r(z['m_amcl_x']), amcl_y=_r(z['m_amcl_y']), amcl_yaw=_r(D(1) * z['m_amcl_yaw'], 3),
            best_x=_r(z['m_best_x']), best_y=_r(z['m_best_y']), best_yaw=_r(D(1) * z['m_best_yaw'], 3),
            score_amcl=_r(z['m_score_amcl'], 2), score_best=_r(z['m_score_best'], 2),
            edge=[bool(e > 0.5) for e in z['m_edge']],
            fwd=_r(z['m_fwd'] * 100, 2), lat=_r(z['m_lat'] * 100, 2), dyaw=_r(D(1) * z['m_dyaw'], 3))
        pt, px, py = _decimate(z['path_t'], z['path'][:, 0], z['path'][:, 1])
        out['path'] = dict(t=_r(pt - t0, 2), x=_r(px), y=_r(py))
    vt, vx = _decimate(z['vx_t'], z['vx'])
    out['vx'] = dict(t=_r(vt - t0, 2), v=_r(vx, 3))
    if 'gz' in z:
        gt, gz = _decimate(z['imu_t'], z['gz'])
        out['gz'] = dict(t=_r(gt - t0, 2), w=_r(gz, 4))
    if 'cmd_w' in z:
        ct, cw = _decimate(z['cmd_t'], z['cmd_w'])
        out['cmd'] = dict(t=_r(ct - t0, 2), w=_r(cw, 4))
    out['lidar'] = dict(t=_r(z['lid_t'] - t0, 2), inl=_r(z['lid_inl'], 3))
    return out


def build_odomerr(z, t0_rel, t1_rel):
    T0 = float(z['odom_t'][0])
    a, b = T0 + t0_rel, T0 + t1_rel
    lt, lid = z['lid_t'], z['lid']
    out = dict(t0=t0_rel, t1=t1_rel)
    for key, tt, P in (('ekf', z['odom_t'], z['odom']), ('wheel', z.get('wheel_t'), z.get('wheel'))):
        if tt is None:
            continue
        e = le.odom_error(lt, lid, tt, P, a)
        s = e['t'] <= b
        out[key] = dict(fwd=_r(e['fwd'][s] * 100, 2), lat=_r(e['lat'][s] * 100, 2), dyaw=_r(D(1) * e['dyaw'][s], 3))
        out['t'] = _r(e['t'][s] - T0, 2)
    sel = (lt >= a) & (lt <= b)
    p = lid[sel]
    out['dist'] = float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])))) if len(p) > 1 else 0.0
    out['rot'] = float(D(1) * (np.unwrap(p[:, 2])[-1] - np.unwrap(p[:, 2])[0])) if len(p) > 1 else 0.0
    return out


def build_scan(z, t_rel):
    T0 = float(z['odom_t'][0])
    i = int(np.argmin(np.abs(z['scan_t'] - (T0 + t_rel))))
    j = int(np.argmin(np.abs(z['m_t'] - z['scan_t'][i])))
    r = z['scan_r'][i].astype(float)
    a = float(z['scan_amin'][i]) + float(z['scan_ainc'][i]) * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.25) & (r < 11.0)
    lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    sensor = tuple(z['sensor'])
    out = dict(t=float(z['scan_t'][i] - T0))
    for key, pose in (('amcl', (z['m_amcl_x'][j], z['m_amcl_y'][j], z['m_amcl_yaw'][j])),
                      ('best', (z['m_best_x'][j], z['m_best_y'][j], z['m_best_yaw'][j]))):
        L = le.compose(tuple(float(v) for v in pose), sensor)
        c, s = math.cos(L[2]), math.sin(L[2])
        out[key] = dict(x=_r(L[0] + c * lx - s * ly), y=_r(L[1] + s * lx + c * ly), pose=_r(pose, 4))
    return out


def list_bags(root):
    import yaml
    rows = []
    for d in Path(root).iterdir() if Path(root).is_dir() else []:
        meta = d / 'metadata.yaml'
        if not d.is_dir() or not meta.exists():
            continue
        try:
            info = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
            topics = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
            rows.append(dict(name=d.name, mtime=meta.stat().st_mtime, dur=info['duration']['nanoseconds'] * 1e-9,
                             scan=topics.get('/scan', 0), tf=topics.get('/tf', 0), odom=topics.get('/odom', 0),
                             wheel=topics.get('/wheel/odom', 0)))
        except Exception:  # noqa: BLE001 — 깨진 metadata 는 목록에서 뺀다
            continue
    rows.sort(key=lambda r: -r['mtime'])
    return [r for r in rows if r['scan'] and r['odom']][:300]


def list_maps(maps_dir):
    import vica_map_resolve as vmr
    cur = vmr.current_map_id(Path(maps_dir))
    rows = vmr.list_maps(Path(maps_dir))
    return [dict(id=r['map_id'], name=r['display_name'], current=r['map_id'] == cur) for r in rows]


# ── HTTP ─────────────────────────────────────────────────

class App:
    def __init__(self, bags, maps, cache):
        self.bags, self.maps, self.cache = Path(bags), Path(maps), Path(cache)
        self.jobs, self.loaded, self.lock = {}, {}, threading.Lock()

    def npz(self, key):
        p = (self.cache / key).resolve()
        if p.parent != self.cache.resolve() or not p.exists():
            raise FileNotFoundError(key)
        with self.lock:
            if key not in self.loaded:
                self.loaded[key] = dict(np.load(p, allow_pickle=False))
            return self.loaded[key]

    def segments(self, key):
        z = self.npz(key)
        with self.lock:
            if ('seg', key) not in self.loaded:
                self.loaded[('seg', key)] = ev.evaluate_bag(z)
            return self.loaded[('seg', key)]

    def start(self, bag, map_id, force=False):
        import analyze_loc
        bag_dir = (self.bags / bag).resolve()
        yml = (self.maps / f'{map_id}.yaml').resolve()
        if bag_dir.parent != self.bags.resolve() or not bag_dir.is_dir():
            raise FileNotFoundError(bag)
        if yml.parent != self.maps.resolve() or not yml.exists():
            raise FileNotFoundError(map_id)
        key = os.path.basename(analyze_loc.cache_path(str(bag_dir), str(yml), str(self.cache), 2.0))
        with self.lock:
            j = self.jobs.get(key)
            if j and j['status'] == 'running':
                return key
            if (self.cache / key).exists() and not force:
                self.jobs[key] = dict(status='done', log=['캐시에서 읽음'])
                return key
            self.jobs[key] = dict(status='running', log=[])
            self.loaded.pop(key, None)
            self.loaded.pop(('seg', key), None)

        def work():
            log = self.jobs[key]['log']
            try:
                os.makedirs(self.cache, exist_ok=True)
                analyze_loc.run(str(bag_dir), str(yml), str(self.cache), 2.0, log=log.append, force=True)
                self.jobs[key]['status'] = 'done'
            except BaseException as e:  # noqa: BLE001 — SystemExit 도 화면에 보여 준다
                log.append(f'실패: {e}')
                log.append(traceback.format_exc()[-800:])
                self.jobs[key]['status'] = 'error'
        threading.Thread(target=work, daemon=True).start()
        return key

    def mapimg(self, map_id):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.image as mpimg
        yml = (self.maps / f'{map_id}.yaml').resolve()
        if yml.parent != self.maps.resolve() or not yml.exists():
            raise FileNotFoundError(map_id)
        grid, _ = le.load_map(str(yml))
        img = np.where(grid.data >= 65, 40, np.where(grid.data == 0, 250, 205)).astype(np.uint8)[::-1]
        buf = io.BytesIO()
        mpimg.imsave(buf, img, cmap='gray', vmin=0, vmax=255, format='png')
        return buf.getvalue()


def make_handler(app):
    static = HERE / 'static'

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype='application/json; charset=utf-8'):
            if isinstance(body, (dict, list)):
                body = json.dumps(body, ensure_ascii=False, separators=(',', ':')).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                if u.path in ('/', '/index.html'):
                    return self._send(200, (static / 'index.html').read_bytes(), 'text/html; charset=utf-8')
                if u.path.startswith('/static/'):
                    f = (static / u.path[len('/static/'):]).resolve()
                    if f.parent != static.resolve() or not f.exists():
                        return self._send(404, {'error': 'not found'})
                    ctype = {'.js': 'text/javascript', '.css': 'text/css'}.get(f.suffix, 'application/octet-stream')
                    return self._send(200, f.read_bytes(), ctype + '; charset=utf-8')
                if u.path == '/api/bags':
                    return self._send(200, list_bags(app.bags))
                if u.path == '/api/maps':
                    return self._send(200, list_maps(app.maps))
                if u.path == '/api/job':
                    j = app.jobs.get(q.get('key', ''), dict(status='unknown', log=[]))
                    return self._send(200, dict(status=j['status'], log=j['log'][-12:]))
                if u.path == '/api/result':
                    return self._send(200, build_result(app.npz(q['key'])))
                if u.path == '/api/odomerr':
                    return self._send(200, build_odomerr(app.npz(q['key']), float(q['t0']), float(q['t1'])))
                if u.path == '/api/eval':
                    return self._send(200, ev.evaluate_range(app.npz(q['key']), float(q['t0']), float(q['t1'])))
                if u.path == '/api/segments':
                    return self._send(200, app.segments(q['key']))
                if u.path == '/api/scan':
                    return self._send(200, build_scan(app.npz(q['key']), float(q['t'])))
                if u.path == '/api/hold':  # 헤드리스 캡처 검증용 — load 이벤트를 늦춘다(최대 30 s)
                    import time
                    time.sleep(min(float(q.get('s', 5)), 30.0))
                    return self._send(200, PNG_1PX, 'image/png')
                if u.path == '/api/mapimg':
                    return self._send(200, app.mapimg(q['map']), 'image/png')
                return self._send(404, {'error': 'not found'})
            except (KeyError, FileNotFoundError, ValueError) as e:
                return self._send(400, {'error': f'{type(e).__name__}: {e}'})

        def do_POST(self):  # noqa: N802
            u = urlparse(self.path)
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                if u.path == '/api/analyze':
                    return self._send(200, dict(key=app.start(body['bag'], body['map'], bool(body.get('force')))))
                return self._send(404, {'error': 'not found'})
            except (KeyError, FileNotFoundError, ValueError) as e:
                return self._send(400, {'error': f'{type(e).__name__}: {e}'})

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default=os.path.expanduser('~/vica_data/bags'))
    ap.add_argument('--maps', default=str(HERE.parents[1] / 'vica_ros2_ws' / 'maps'))
    ap.add_argument('--cache', default=os.path.expanduser('~/vica_data/drive_diag_cache'))
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8765)
    a = ap.parse_args()
    app = App(a.bags, a.maps, a.cache)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(app))
    print(f'주행 진단 뷰어: http://{a.host}:{a.port}  (bag {a.bags} · 지도 {a.maps} · 캐시 {a.cache})  끝내기 Ctrl-C')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
