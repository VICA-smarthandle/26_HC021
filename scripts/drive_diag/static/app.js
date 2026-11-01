'use strict';
// 주행 진단 뷰어 — serve.py 의 /api/* 를 그린다. 외부 라이브러리 없음.

const $ = (id) => document.getElementById(id);
const css = (v) => getComputedStyle(document.body).getPropertyValue(v).trim();
const S = { key: null, R: null, sel: null, cur: null, odom: null, ev: null, segs: null, scan: null, img: null, view: null, charts: [] };
const LOW_SCORE = 65; // 지도 대조 최고 점수가 이보다 낮으면 지도와 실제가 달라 ① 을 믿기 어렵다

async function api(path, body) {
  const r = await fetch(path, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}

// ── 시작 ────────────────────────────────────────────────
async function init() {
  const [bags, maps] = await Promise.all([api('/api/bags'), api('/api/maps')]);
  $('bag').innerHTML = bags.map((b) =>
    `<option value="${b.name}">${b.name} · ${(b.dur / 60).toFixed(1)}분${b.tf ? '' : ' · TF 없음'}</option>`).join('');
  $('map').innerHTML = maps.map((m) =>
    `<option value="${m.id}"${m.current ? ' selected' : ''}>${m.current ? '★ ' : ''}${m.name}${m.name !== m.id ? ' (' + m.id + ')' : ''}</option>`).join('');
  $('go').onclick = () => analyze(false);
  $('redo').onclick = () => analyze(true);
  $('fit').onclick = () => { fitView(); drawMap(); };
  setupMap();
  window.addEventListener('resize', () => { if (S.R) renderAll(); });
  const q = new URLSearchParams(location.search);  // ?bag=…&map=… 이면 바로 분석
  if (q.get('bag') && q.get('map')) {
    $('bag').value = q.get('bag'); $('map').value = q.get('map');
    if ($('bag').value === q.get('bag') && $('map').value === q.get('map')) analyze(false);
  }
}

async function analyze(force) {
  $('go').disabled = $('redo').disabled = true;
  try {
    const { key } = await api('/api/analyze', { bag: $('bag').value, map: $('map').value, force });
    for (;;) {
      const j = await api('/api/job?key=' + encodeURIComponent(key));
      $('status').textContent = j.status === 'running' ? '계산 중… ' + (j.log.at(-1) || '') : (j.log.at(-1) || '');
      if (j.status === 'done') break;
      if (j.status === 'error') throw new Error(j.log.join('\n'));
      await new Promise((r) => setTimeout(r, 1000));
    }
    const q = new URLSearchParams(location.search);  // 같은 bag·지도면 sel·t 를 살린다
    if (q.get('bag') !== $('bag').value || q.get('map') !== $('map').value) { q.delete('sel'); q.delete('t'); }
    q.set('bag', $('bag').value); q.set('map', $('map').value);
    history.replaceState(null, '', '?' + q.toString());
    await load(key);
  } catch (e) {
    $('status').textContent = '실패: ' + e.message.split('\n')[0];
    console.error(e);
  } finally {
    $('go').disabled = $('redo').disabled = false;
  }
}

async function load(key) {
  S.key = key;
  S.R = await api('/api/result?key=' + encodeURIComponent(key));
  S.sel = [0, S.R.t_end];
  S.cur = null; S.scan = null; S.img = null;
  const q = new URLSearchParams(location.search);  // &sel=a,b&t=c → 그 구간·커서로 연다(공유용)
  const qs = (q.get('sel') || '').split(',').map(Number);
  if (qs.length === 2 && qs.every(isFinite) && qs[1] > qs[0]) S.sel = [Math.max(0, qs[0]), Math.min(S.R.t_end, qs[1])];
  const R = S.R;
  $('status').textContent = `${R.bag.split('/').pop()} · ${(R.t_end / 60).toFixed(1)}분 · 라이다 yaw ${(R.sensor[2] * 180 / Math.PI).toFixed(2)}°` +
    (R.has_match ? ` · 지도 대조 ${R.match.t.length}장` : ' · map→odom 없음(AMCL 미설정) — ① 없음');
  if (R.has_match) {
    const mapId = R.map_yaml.split('/').pop().replace(/\.yaml$/, '');
    S.img = new Image();
    S.img.onload = () => { fitView(); drawMap(); };
    S.img.src = '/api/mapimg?map=' + encodeURIComponent(mapId);
  }
  try { S.segs = await api('/api/segments?key=' + encodeURIComponent(key)); } catch (e) { S.segs = null; }
  renderThresholds();
  await selChanged();
  if (q.get('t') != null && isFinite(+q.get('t'))) await setCursor(+q.get('t'));
}

async function selChanged() {
  const [a, b] = S.sel;
  const k = encodeURIComponent(S.key);
  [S.odom, S.ev] = await Promise.all([
    api(`/api/odomerr?key=${k}&t0=${a}&t1=${b}`).catch(() => null),
    api(`/api/eval?key=${k}&t0=${a}&t1=${b}`).catch(() => null)]);
  renderAll();
}

// ── 그래프 ──────────────────────────────────────────────
function niceTicks(lo, hi, n) {
  const span = hi - lo || 1, raw = span / n, mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= n) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}
const NS = 'http://www.w3.org/2000/svg';
const el = (tag, attrs, parent) => {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
};

// cfg: {title, unit, xdom, series:[{name,color,t,y,dots,marks}], sym, low:[[a,b]], height, brush, ymin}
function chart(host, cfg) {
  host.className = 'chart';
  host.innerHTML = '';
  const W = host.clientWidth || 600, H = cfg.height || 104, L = 46, Rm = 10, T = 16, B = 18;
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, height: H }, host);
  const [a, b] = cfg.xdom;
  const X = (t) => L + (t - a) / (b - a || 1) * (W - L - Rm);
  const invX = (px) => a + (px - L) / (W - L - Rm) * (b - a);
  let lo = Infinity, hi = -Infinity;
  for (const s of cfg.series) for (let i = 0; i < s.t.length; i++) {
    if (s.t[i] < a || s.t[i] > b || s.y[i] == null) continue;
    lo = Math.min(lo, s.y[i]); hi = Math.max(hi, s.y[i]);
  }
  if (!isFinite(lo)) { lo = -1; hi = 1; }
  if (cfg.sym) { const m = Math.max(Math.abs(lo), Math.abs(hi), cfg.ymin || 1) * 1.1; lo = -m; hi = m; }
  else { const pad = (hi - lo || 1) * 0.08; lo -= pad; hi += pad; if (cfg.floor0) lo = Math.min(0, lo); }
  const Y = (v) => T + (hi - v) / (hi - lo) * (H - T - B);
  el('text', { x: 4, y: 11, class: 'ttl' }, svg).textContent = `${cfg.title}${cfg.unit ? ' (' + cfg.unit + ')' : ''}`;
  for (const [x0, x1] of cfg.low || []) {
    if (x1 < a || x0 > b) continue;
    el('rect', { x: X(Math.max(x0, a)), y: T, width: Math.max(1, X(Math.min(x1, b)) - X(Math.max(x0, a))), height: H - T - B, class: 'low' }, svg);
  }
  const g = el('g', { class: 'grid' }, svg), ax = el('g', { class: 'axis' }, svg);
  for (const v of niceTicks(lo, hi, 3)) {
    el('line', { x1: L, x2: W - Rm, y1: Y(v), y2: Y(v) }, g);
    el('text', { x: L - 4, y: Y(v) + 3, 'text-anchor': 'end' }, ax).textContent = v;
  }
  for (const v of niceTicks(a, b, 8)) el('text', { x: X(v), y: H - 4, 'text-anchor': 'middle' }, ax).textContent = v + 's';
  if (cfg.band) { const [b0, b1] = cfg.band; el('rect', { x: L, width: W - L - Rm, y: Y(Math.min(b1, hi)), height: Math.max(0, Y(Math.max(b0, lo)) - Y(Math.min(b1, hi))), class: 'band' }, svg); }
  if (lo < 0 && hi > 0) el('line', { x1: L, x2: W - Rm, y1: Y(0), y2: Y(0), class: 'zero' }, svg);
  if (cfg.selRect) { const [s0, s1] = cfg.selRect; el('rect', { x: X(s0), y: T, width: Math.max(1, X(s1) - X(s0)), height: H - T - B, class: 'sel' }, svg); }
  for (const s of cfg.series) {
    let d = '', pen = false;
    for (let i = 0; i < s.t.length; i++) {
      const t = s.t[i], v = s.y[i];
      if (t < a || t > b || v == null) { pen = false; continue; }
      d += `${pen ? 'L' : 'M'}${X(t).toFixed(1)},${Y(v).toFixed(1)}`; pen = true;
    }
    el('path', { d, class: 'line', stroke: s.color, 'stroke-width': s.dots ? 1 : 2, opacity: s.dots ? 0.6 : 1 }, svg);
    if (s.dots) for (let i = 0; i < s.t.length; i++) {
      const t = s.t[i], v = s.y[i];
      if (t < a || t > b || v == null) continue;
      if (s.marks && s.marks[i]) {
        const x = X(t), y = Y(v), r = 4, c = css('--critical');
        el('path', { d: `M${x - r},${y - r}L${x + r},${y + r}M${x - r},${y + r}L${x + r},${y - r}`, stroke: c, 'stroke-width': 2 }, svg);
      } else el('circle', { cx: X(t), cy: Y(v), r: 2.2, fill: s.color }, svg);
    }
  }
  const cur = el('line', { x1: -10, x2: -10, y1: T, y2: H - B, class: 'cur' }, svg);
  const hit = el('rect', { x: L, y: 0, width: W - L - Rm, height: H, fill: 'transparent' }, svg);
  const obj = { host, cfg, X, invX, cur, setCursor(t) { const x = (t == null || t < a || t > b) ? -10 : X(t); cur.setAttribute('x1', x); cur.setAttribute('x2', x); } };
  hit.addEventListener('mousemove', (ev) => hover(obj, ev));
  hit.addEventListener('mouseleave', () => { $('tip').hidden = true; S.charts.forEach((c) => c.setCursor(S.cur)); });
  if (cfg.brush) brush(obj, hit, svg, T, H - B);
  else hit.addEventListener('click', (ev) => setCursor(invX(svgX(svg, ev, W))));
  S.charts.push(obj);
  return obj;
}
const svgX = (svg, ev, W) => (ev.clientX - svg.getBoundingClientRect().left) * W / svg.getBoundingClientRect().width;

function nearest(t, ts) {
  let lo = 0, hi = ts.length - 1;
  if (hi < 0) return -1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (ts[m] < t) lo = m; else hi = m; }
  return Math.abs(ts[lo] - t) <= Math.abs(ts[hi] - t) ? lo : hi;
}
function hover(obj, ev) {
  const svg = obj.host.querySelector('svg'), W = svg.viewBox.baseVal.width;
  const t = obj.invX(svgX(svg, ev, W));
  S.charts.forEach((c) => c.setCursor(t));
  const rows = obj.cfg.series.map((s) => {
    const i = nearest(t, s.t);
    return i < 0 ? '' : `<div><i class="sw" style="background:${s.color}"></i> ${s.name}: <b>${s.y[i] == null ? '—' : (+s.y[i]).toFixed(2)}</b> ${obj.cfg.unit || ''}${s.marks && s.marks[i] ? ' (범위 밖)' : ''}</div>`;
  }).join('');
  const tip = $('tip');
  tip.innerHTML = `<div>${t.toFixed(1)} s</div>${rows}`;
  tip.hidden = false;
  tip.style.left = Math.min(ev.clientX + 14, innerWidth - 240) + 'px';
  tip.style.top = (ev.clientY + 12) + 'px';
}
function brush(obj, hit, svg, y0, y1) {
  let start = null, rect = null;
  const W = svg.viewBox.baseVal.width;
  hit.addEventListener('mousedown', (ev) => { start = obj.invX(svgX(svg, ev, W)); rect = el('rect', { y: y0, height: y1 - y0, class: 'sel' }, svg); });
  hit.addEventListener('mousemove', (ev) => {
    if (start == null) return;
    const t = obj.invX(svgX(svg, ev, W));
    rect.setAttribute('x', obj.X(Math.min(start, t))); rect.setAttribute('width', Math.abs(obj.X(t) - obj.X(start)));
  });
  hit.addEventListener('mouseup', (ev) => {
    if (start == null) return;
    const t = obj.invX(svgX(svg, ev, W)), a = Math.max(0, Math.min(start, t)), b = Math.min(S.R.t_end, Math.max(start, t));
    start = null;
    if (b - a > 0.5) { S.sel = [a, b]; shareUrl(); selChanged(); } else setCursor(t);
  });
  hit.addEventListener('dblclick', () => { S.sel = [0, S.R.t_end]; shareUrl(); selChanged(); });
}

function lowBands() {
  const M = S.R.match, out = [];
  if (!M) return out;
  let s = null;
  for (let i = 0; i < M.t.length; i++) {
    const low = M.score_best[i] < LOW_SCORE;
    if (low && s == null) s = M.t[i];
    if ((!low || i === M.t.length - 1) && s != null) { out.push([s - 0.25, M.t[i] + 0.25]); s = null; }
  }
  return out;
}

function renderAll() {
  const R = S.R, sel = S.sel, c1 = css('--series-1'), c2 = css('--series-2'), c3 = css('--series-3');
  S.charts = [];
  const all = [0, R.t_end], low = lowBands();
  chart($('ov'), { title: '|속도|', unit: 'm/s', xdom: all, height: 90, brush: true, selRect: sel, low, floor0: true,
    series: [{ name: '|vx|', color: c1, t: R.vx.t, y: R.vx.v.map(Math.abs) }] });
  if (R.has_match) {
    const M = R.match;
    const TH = S.segs && S.segs.thresholds;
    const bands = { lat: TH ? TH.loc_lat_p95_cm.good : 5, dyaw: TH ? TH.loc_yaw_p95_deg.good : 1.5 };
    for (const [id, key, ttl, unit, ymin] of [['m_lat', 'lat', '옆', 'cm', 2], ['m_fwd', 'fwd', '앞', 'cm', 2], ['m_dyaw', 'dyaw', '방향 θ', '°', 0.5]])
      chart($(id), { title: '① ' + ttl, unit, xdom: sel, sym: true, ymin, low, band: bands[key] ? [-bands[key], bands[key]] : null,
        series: [{ name: 'AMCL − 지도 대조', color: c1, t: M.t, y: M[key], dots: true, marks: M.edge }] });
    chart($('m_score'), { title: '지도 정합 점수(주황 띠 = 최고 점수 < ' + LOW_SCORE + ' %)', unit: '%', xdom: sel, low,
      series: [{ name: 'AMCL 자세', color: c1, t: M.t, y: M.score_amcl }, { name: '지도 대조 최고', color: c2, t: M.t, y: M.score_best }] });
  } else for (const id of ['m_lat', 'm_fwd', 'm_dyaw', 'm_score']) $(id).innerHTML = '<p class="meta">map→odom 이 없어 ① 을 계산하지 못했습니다(AMCL 초기 위치 미설정 bag).</p>';
  const O = S.odom;
  if (O && O.t && O.t.length > 1) {
    for (const [id, key, ttl, unit, ymin] of [['o_lat', 'lat', '옆', 'cm', 2], ['o_fwd', 'fwd', '앞', 'cm', 2], ['o_dyaw', 'dyaw', '방향 θ', '°', 0.5]]) {
      const series = [{ name: 'EKF', color: c1, t: O.t, y: O.ekf[key] }];
      if (O.wheel) series.push({ name: '바퀴만', color: c2, t: O.t, y: O.wheel[key] });
      chart($(id), { title: `② ${ttl} (${O.dist.toFixed(1)} m 기준)`, unit, xdom: sel, sym: true, ymin, series });
    }
  } else for (const id of ['o_lat', 'o_fwd', 'o_dyaw']) $(id).innerHTML = '<p class="meta">구간 안에 라이다 키프레임이 2개 이상 있어야 합니다.</p>';
  const rs = [];
  if (R.cmd) rs.push({ name: '명령 w', color: c1, t: R.cmd.t, y: R.cmd.w });
  if (R.gz) rs.push({ name: '자이로 wz', color: c3, t: R.gz.t, y: R.gz.w });
  chart($('rot'), { title: '회전 속도', unit: 'rad/s', xdom: sel, sym: true, ymin: 0.05, series: rs });
  S.charts.forEach((c) => c.setCursor(S.cur));
  renderStats();
  renderSegs();
  renderCursor();
  drawMap();
}

// ── 통계·커서 ───────────────────────────────────────────
const mean = (a) => a.reduce((s, v) => s + v, 0) / (a.length || 1);
const std = (a) => { const m = mean(a); return Math.sqrt(mean(a.map((v) => (v - m) ** 2))); };
const p95 = (a) => { const s = a.map(Math.abs).sort((x, y) => x - y); return s.length ? s[Math.min(s.length - 1, Math.floor(0.95 * s.length))] : NaN; };
const f = (v, d = 1) => (v == null || !isFinite(v) ? '—' : (v >= 0 ? '+' : '') + v.toFixed(d));

const GK = { good: '좋음', warn: '주의', bad: '나쁨' };
const chip = (g, big) => `<span class="chip ${g || 'none'}${big ? ' big' : ''}">${GK[g] || '—'}</span>`;
function gradeOf(key, v) {
  const T = S.segs && S.segs.thresholds && S.segs.thresholds[key];
  if (!T || v == null || !isFinite(v)) return null;
  if (key === 'score_pct') return v >= T.good ? 'good' : v >= T.warn ? 'warn' : 'bad';
  const a = Math.abs(v);
  return a <= T.good ? 'good' : a <= T.warn ? 'warn' : 'bad';
}

function renderStats() {  // 1. 한눈에 + 2. 솔루션
  const E = S.ev, [a, b] = S.sel;
  $('evalrange').textContent = `— 선택 구간 ${a.toFixed(1)}~${b.toFixed(1)} s (${(b - a).toFixed(1)} s)`;
  if (!E || !E.items.length) {
    $('eval').innerHTML = '<p class="meta">이 구간에서 평가할 항목이 없습니다.</p>'; $('evalgrade').innerHTML = '';
    $('sol').innerHTML = '—'; return;
  }
  $('evalgrade').innerHTML = '종합 ' + chip(E.grade, true);
  $('eval').classList.remove('meta');
  $('eval').innerHTML = '<div class="tbl"><table class="evtbl"><thead><tr><th>판정</th><th>항목</th><th>내용</th></tr></thead><tbody>' +
    E.items.map((i) => `<tr><td>${chip(i.grade)}</td><td>${i.label}</td><td>${i.text}</td></tr>`).join('') + '</tbody></table></div>';
  const L = E.solutions || [];
  $('sol').classList.remove('meta');
  $('sol').innerHTML = !L.length ? '<p class="ok">이 구간은 조치할 항목이 없습니다.</p>' :
    '<div class="sols">' + L.map((x, k) => `<div class="sol ${x.grade}"><div class="sh">${chip(x.grade)}<span class="st">${k + 1}. ${x.title}</span></div>` +
      `<div class="why">${x.why}</div><ol>${x.actions.map((t) => `<li>${t}</li>`).join('')}</ol></div>`).join('') + '</div>';
}

function renderSegs() {
  const G = S.segs;
  if (!G || !G.segments.length) { $('segs').innerHTML = '<p class="meta">구간을 나누지 못했습니다.</p>'; return; }
  const v = (c, k, f) => (c && c.values && c.values[k] != null && isFinite(c.values[k]) ? f(c.values[k]) : null);
  const td = (val, key, d) => { if (val == null) return '<td class="n meta">—</td>'; const g = key === 'odom_lat_cm_per_m' ? null : gradeOf(key, val);
    return `<td class="n ${g || ''}">${(val >= 0 && key !== 'score_pct' && key !== 'loc_lat_p95_cm' && key !== 'loc_yaw_p95_deg' && key !== 'wobble_deg' ? '+' : '') + val.toFixed(d)}</td>`; };
  const [a, b] = S.sel;
  let h = '<table><thead><tr><th>#</th><th>종류</th><th>시간 s</th><th>길이</th><th>판정</th><th>① |옆| 95% cm</th><th>① |θ| 95% °</th>' +
    '<th>지도 점수 %</th><th>② EKF 옆 cm/m(참고)</th><th>② EKF θ 표류 °/m</th><th>② 바퀴 θ 표류 °/m</th><th>흔들림 °</th></tr></thead><tbody>';
  G.segments.forEach((sg, i) => {
    const c = sg.cells, on = Math.abs(sg.t0 - a) < 0.05 && Math.abs(sg.t1 - b) < 0.05;
    h += `<tr data-i="${i}" class="${on ? 'on' : ''}"><td>${i + 1}</td><td>${sg.kind}</td><td class="n">${sg.t0.toFixed(1)}~${sg.t1.toFixed(1)}</td>` +
      `<td class="n">${(sg.t1 - sg.t0).toFixed(1)} s</td><td>${chip(sg.grade)}</td>` +
      td(v(c.loc, 'lat_p95', (x) => x), 'loc_lat_p95_cm', 1) + td(v(c.loc, 'yaw_p95', (x) => x), 'loc_yaw_p95_deg', 2) +
      td(v(c.score, 'score', (x) => x), 'score_pct', 0) +
      td(v(c.odom_ekf, 'lat_slope', (x) => x), 'odom_lat_cm_per_m', 1) + td(v(c.odom_ekf, 'yaw_slope', (x) => x), 'odom_yaw_deg_per_m', 2) +
      td(v(c.odom_wheel, 'yaw_slope', (x) => x), 'odom_yaw_deg_per_m', 2) + td(v(c.wobble, 'amp', (x) => x), 'wobble_deg', 1) + '</tr>';
  });
  h += '</tbody></table><p class="meta">② 는 라이다 이동 2 m 이상 구간만, 흔들림은 4 s 이상 직선 주행 구간만 계산. 지도 점수가 낮은(빨강) 구간의 ① 은 믿기 어려움.</p>';
  $('segs').classList.remove('meta');
  $('segs').innerHTML = h;
  $('segs').querySelectorAll('tbody tr').forEach((tr) => tr.addEventListener('click', () => {
    const sg = G.segments[+tr.dataset.i];
    S.sel = [sg.t0, sg.t1]; shareUrl(); fitView(); selChanged();
  }));
}

function renderThresholds() {
  const T = S.segs && S.segs.thresholds;
  if (!T) return;
  const name = { loc_lat_p95_cm: '① 위치 옆 |오차| 95 % (cm)', loc_yaw_p95_deg: '① 위치 방향 |오차| 95 % (°)', score_pct: '지도 정합 점수 평균 (%)',
    odom_lat_cm_per_m: '② 옆 표류 (cm/m)', odom_yaw_deg_per_m: '② 방향 표류 (°/m)', wobble_deg: '직선 흔들림 폭 (°)' };
  $('thr').classList.remove('meta');
  $('thr').innerHTML = '<table><tr><th>항목</th><th>좋음</th><th>주의</th><th>나쁨</th><th>근거</th></tr>' + Object.keys(T).map((k) => {
    const t = T[k], hb = k === 'score_pct';
    return `<tr><td>${name[k] || k}</td><td class="n">${hb ? '≥ ' : '≤ '}${t.good}</td><td class="n">${hb ? '≥ ' : '≤ '}${t.warn}</td>` +
      `<td class="n">${hb ? '< ' : '> '}${t.warn}</td><td style="white-space:normal">${t.basis}</td></tr>`; }).join('') +
    '</table>';
}

function shareUrl() {
  const q = new URLSearchParams(location.search);
  q.set('sel', S.sel.map((v) => v.toFixed(1)).join(','));
  if (S.cur != null) q.set('t', S.cur.toFixed(2)); else q.delete('t');
  q.delete('hold');
  history.replaceState(null, '', '?' + q.toString());
}
async function setCursor(t) {
  S.cur = t;
  shareUrl();
  S.charts.forEach((c) => c.setCursor(t));
  if (S.R.has_match) { try { S.scan = await api(`/api/scan?key=${encodeURIComponent(S.key)}&t=${t}`); } catch (e) { S.scan = null; } }
  renderCursor();
  drawMap();
}
function renderCursor() {
  if (S.cur == null) { $('cursor').innerHTML = '그래프를 눌러 시각을 고르세요'; return; }
  const R = S.R, t = S.cur;
  let h = `<div class="kv"><span>시각</span><span>${t.toFixed(2)} s</span>`;
  if (R.has_match && S.scan) {
    const M = R.match, i = nearest(S.scan.t, M.t);
    h += `<span>AMCL 자세</span><span>x ${M.amcl_x[i].toFixed(3)} · y ${M.amcl_y[i].toFixed(3)} · θ ${M.amcl_yaw[i].toFixed(2)}°</span>` +
      `<span>지도 대조 자세</span><span>x ${M.best_x[i].toFixed(3)} · y ${M.best_y[i].toFixed(3)} · θ ${M.best_yaw[i].toFixed(2)}°</span>` +
      `<span>① 오차</span><span>옆 ${f(M.lat[i])} cm · 앞 ${f(M.fwd[i])} cm · θ ${f(M.dyaw[i], 2)}°${M.edge[i] ? ' (범위 밖)' : ''}</span>` +
      `<span>점수</span><span>AMCL ${M.score_amcl[i].toFixed(1)} % · 최고 ${M.score_best[i].toFixed(1)} %</span>`;
  }
  const O = S.odom;
  if (O && O.t && O.t.length) {
    const j = nearest(t, O.t);
    if (j >= 0 && t >= O.t[0]) h += `<span>② EKF</span><span>옆 ${f(O.ekf.lat[j])} cm · 앞 ${f(O.ekf.fwd[j])} cm · θ ${f(O.ekf.dyaw[j], 2)}°</span>` +
      (O.wheel ? `<span>② 바퀴만</span><span>옆 ${f(O.wheel.lat[j])} cm · 앞 ${f(O.wheel.fwd[j])} cm · θ ${f(O.wheel.dyaw[j], 2)}°</span>` : '');
  }
  $('cursor').innerHTML = h + '</div>';
}

// ── 지도 ────────────────────────────────────────────────
function setupMap() {
  const c = $('mapc');
  let drag = null;
  c.addEventListener('wheel', (ev) => {
    if (!S.view) return;
    ev.preventDefault();
    const r = c.getBoundingClientRect(), [wx, wy] = toWorld(ev.clientX - r.left, ev.clientY - r.top);
    const k = ev.deltaY < 0 ? 1.2 : 1 / 1.2;
    S.view.s *= k;
    S.view.cx = wx - (wx - S.view.cx) / k; S.view.cy = wy - (wy - S.view.cy) / k;
    drawMap();
  }, { passive: false });
  c.addEventListener('mousedown', (ev) => { drag = [ev.clientX, ev.clientY]; c.style.cursor = 'grabbing'; });
  window.addEventListener('mouseup', () => { drag = null; c.style.cursor = 'grab'; });
  window.addEventListener('mousemove', (ev) => {
    if (!drag || !S.view) return;
    S.view.cx -= (ev.clientX - drag[0]) / S.view.s; S.view.cy += (ev.clientY - drag[1]) / S.view.s;
    drag = [ev.clientX, ev.clientY];
    drawMap();
  });
}
function canvasSize() {
  const c = $('mapc'), dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if (c.width !== Math.round(w * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
  return [w, h, dpr];
}
function toCanvas(x, y) { const [w, h] = canvasSize(); return [w / 2 + (x - S.view.cx) * S.view.s, h / 2 - (y - S.view.cy) * S.view.s]; }
function toWorld(px, py) { const [w, h] = canvasSize(); return [S.view.cx + (px - w / 2) / S.view.s, S.view.cy - (py - h / 2) / S.view.s]; }
function fitView() {
  const P = S.R && S.R.path;
  if (!P) return;
  const [a, b] = S.sel, xs = [], ys = [];
  for (let i = 0; i < P.t.length; i++) if (P.t[i] >= a && P.t[i] <= b) { xs.push(P.x[i]); ys.push(P.y[i]); }
  if (!xs.length) return;
  const [w, h] = canvasSize(), x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  S.view = { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, s: Math.min(w / Math.max(x1 - x0 + 2, 2), h / Math.max(y1 - y0 + 2, 2)) };
}
function arrow(ctx, x, y, yawDeg, color, len) {
  const [px, py] = toCanvas(x, y), a = -yawDeg * Math.PI / 180;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(px, py, 4, 0, 2 * Math.PI); ctx.fill();
  ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px + len * Math.cos(a), py + len * Math.sin(a)); ctx.stroke();
}
function drawMap() {
  const c = $('mapc'), ctx = c.getContext('2d'), [w, h, dpr] = canvasSize();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const R = S.R;
  if (!R || !R.has_match || !S.view) {
    ctx.fillStyle = css('--text-muted'); ctx.font = '13px sans-serif';
    ctx.fillText(R ? 'map→odom 이 없어 지도 위 자세를 그릴 수 없습니다' : 'bag 과 지도를 고르고 [분석]', 16, 28);
    return;
  }
  const m = R.map;
  if (S.img && S.img.complete) {
    const [x0, y0] = toCanvas(m.origin[0], m.origin[1] + m.height * m.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = 0.9;
    ctx.drawImage(S.img, x0, y0, m.width * m.resolution * S.view.s, m.height * m.resolution * S.view.s);
    ctx.globalAlpha = 1;
  }
  const P = R.path, [a, b] = S.sel, c1 = css('--series-1'), c2 = css('--series-2');
  const line = (from, to, width, alpha) => {
    ctx.beginPath(); let pen = false;
    for (let i = 0; i < P.t.length; i++) {
      if (P.t[i] < from || P.t[i] > to) { pen = false; continue; }
      const [px, py] = toCanvas(P.x[i], P.y[i]);
      if (pen) ctx.lineTo(px, py); else ctx.moveTo(px, py);
      pen = true;
    }
    ctx.strokeStyle = c1; ctx.globalAlpha = alpha; ctx.lineWidth = width; ctx.stroke(); ctx.globalAlpha = 1;
  };
  line(0, R.t_end, 1, 0.35);
  line(a, b, 2.5, 1);
  const M = R.match;
  ctx.fillStyle = c2;
  for (let i = 0; i < M.t.length; i++) {
    if (M.t[i] < a || M.t[i] > b || M.edge[i]) continue;
    const [px, py] = toCanvas(M.best_x[i], M.best_y[i]);
    ctx.beginPath(); ctx.arc(px, py, 2, 0, 2 * Math.PI); ctx.fill();
  }
  if (S.scan) {
    for (const [k, color, r] of [['amcl', css('--series-4'), 1.6], ['best', c2, 1.3]]) {
      ctx.fillStyle = color;
      const s = S.scan[k];
      for (let i = 0; i < s.x.length; i++) { const [px, py] = toCanvas(s.x[i], s.y[i]); ctx.fillRect(px - r / 2, py - r / 2, r, r); }
    }
    arrow(ctx, S.scan.amcl.pose[0], S.scan.amcl.pose[1], S.scan.amcl.pose[2] * 180 / Math.PI, c1, 22);
    arrow(ctx, S.scan.best.pose[0], S.scan.best.pose[1], S.scan.best.pose[2] * 180 / Math.PI, c2, 22);
  }
  // 축척 막대
  const bar = [0.5, 1, 2, 5, 10].find((v) => v * S.view.s > 60) || 10;
  ctx.fillStyle = css('--text-secondary'); ctx.fillRect(12, h - 16, bar * S.view.s, 3);
  ctx.font = '11px sans-serif'; ctx.fillText(bar + ' m', 12, h - 20);
}

// 화면 검증용: ?hold=초 → 그동안 load 이벤트를 늦춰 헤드리스 캡처가 다 그린 뒤 찍히게, 오류는 상태줄에
const HOLD = new URLSearchParams(location.search).get('hold');
if (HOLD) {
  window.addEventListener('error', (e) => { $('status').textContent = 'JS 오류: ' + e.message + ' @' + e.lineno; });
  window.addEventListener('unhandledrejection', (e) => { $('status').textContent = 'JS 오류: ' + e.reason; });
  const im = new Image(); im.src = '/api/hold?s=' + encodeURIComponent(HOLD); im.hidden = true; document.body.appendChild(im);
}
init().catch((e) => { $('status').textContent = '초기화 실패: ' + e.message; });
