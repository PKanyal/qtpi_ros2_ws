'use strict';
/* qtpi nav dashboard — canvas map viewer talking to nav_dashboard over a WebSocket. */
(() => {
  const $ = (id) => document.getElementById(id);
  const canvas = $('map');
  const ctx = canvas.getContext('2d');
  const view = $('view');

  // Footprint from robot_bringup/config/nav2_params.yaml, used until the
  // published footprint arrives.
  const DEFAULT_FOOTPRINT = [0.16, 0.145, 0.16, -0.145, -0.13, -0.145, -0.13, 0.145];

  const S = {
    hello: null, connected: false,
    grids: {},            // key -> {meta, canvas, ctx, bbox}
    state: null,
    paths: {},            // plan, lookahead -> flat [x0,y0,x1,y1,...] in global frame
    footprint: null,      // flat, in robot frame
    scan: null,           // Float32Array xy pairs in global frame
    trail: [],
  };
  const V = { cx: 0, cy: 0, scale: 25, follow: false, fitted: false };
  const layers = { map: true, global_costmap: true, local_costmap: true, plan: true, scan: true, trail: true };
  let mode = 'pan';
  let W = 0, H = 0, dpr = 1, needsDraw = false;
  let colors = {};

  // ---------- colour lookup tables (index = int8 cell value as uint8) ----------
  const rgba = (r, g, b, a) => ((a << 24) | (b << 16) | (g << 8) | r) >>> 0;
  const LUT_MAP = new Uint32Array(256);
  const LUT_COST = new Uint32Array(256);
  for (let i = 0; i < 256; i++) {
    const v = i > 127 ? i - 256 : i;
    if (v < 0) LUT_MAP[i] = 0;                         // unknown: show background
    else { const g = Math.round(236 - (Math.min(v, 100) / 100) * 202); LUT_MAP[i] = rgba(g, g, g + 3, 255); }
    if (v <= 0) LUT_COST[i] = 0;                       // free / unknown: transparent
    else if (v >= 100) LUT_COST[i] = rgba(255, 0, 255, 235);  // lethal
    else if (v === 99) LUT_COST[i] = rgba(0, 230, 255, 220);  // inscribed
    else {                                              // inflation: blue -> red
      const t = v / 98;
      LUT_COST[i] = rgba(Math.round(48 + 207 * t), 40, Math.round(255 * (1 - t)), Math.round(70 + 120 * t));
    }
  }

  // ---------- geometry ----------
  const compose = (a, b) => {
    const c = Math.cos(a[2]), s = Math.sin(a[2]);
    return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]];
  };
  const frameT = (frame) => {
    if (!frame || !S.hello || frame === S.hello.global_frame) return [0, 0, 0];
    const f = S.state && S.state.frames && S.state.frames[frame];
    return f || null;
  };
  const w2s = (x, y) => [W / 2 + (x - V.cx) * V.scale, H / 2 - (y - V.cy) * V.scale];
  const s2w = (sx, sy) => [V.cx + (sx - W / 2) / V.scale, V.cy - (sy - H / 2) / V.scale];
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ['canvas-bg', 'c-plan', 'c-trail', 'c-robot', 'c-goal', 'c-scan', 'c-pose', 'text', 'surface']) {
      colors[k] = cs.getPropertyValue('--' + k).trim();
    }
    dirty();
  }

  function resize() {
    const r = view.getBoundingClientRect();
    dpr = window.devicePixelRatio || 1;
    W = r.width; H = r.height;
    canvas.width = Math.max(1, Math.round(W * dpr));
    canvas.height = Math.max(1, Math.round(H * dpr));
    if (!V.fitted && S.grids.map) fitMap();
    dirty();
  }

  function dirty() {
    if (!needsDraw) { needsDraw = true; requestAnimationFrame(render); }
  }

  // ---------- rendering ----------
  function render() {
    needsDraw = false;
    const robot = S.state && S.state.robot;
    if (V.follow && robot) { V.cx = robot.x; V.cy = robot.y; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = colors['canvas-bg'];
    ctx.fillRect(0, 0, W, H);
    ctx.imageSmoothingEnabled = false;

    drawGrid('map', 1);
    drawGrid('global_costmap', 0.65);
    drawGrid('local_costmap', 0.85);
    if (layers.trail) drawLine(S.trail, colors['c-trail'], 2, 0.8);
    const nav = S.state && S.state.nav;
    const active = !!(nav && nav.active);
    if (layers.plan && S.paths.plan) drawLine(S.paths.plan, colors['c-plan'], 3, active ? 1 : 0.45);
    if (layers.plan && active && S.paths.lookahead) drawDot(S.paths.lookahead[0], S.paths.lookahead[1], 4, colors['c-plan']);
    if (layers.scan && S.scan) drawScan();
    if (nav && nav.goal) drawGoal(nav.goal, active);
    drawRobot();
    if (drag && (drag.type === 'goal' || drag.type === 'pose')) drawAim();
    updateScale();
  }

  function drawGrid(key, alpha) {
    const g = S.grids[key];
    if (!g || !layers[key]) return;
    const T = frameT(g.meta.frame);
    if (!T) return;
    const o = compose(T, [g.meta.ox, g.meta.oy, g.meta.oyaw]);
    const [sx, sy] = w2s(o[0], o[1]);
    const k = g.meta.res * V.scale;
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.translate(sx, sy);
    ctx.rotate(-o[2]);
    ctx.scale(k, -k);      // grid row 0 is the bottom of the map: flip y
    ctx.drawImage(g.canvas, 0, 0);
    ctx.restore();
  }

  function drawLine(pts, color, width, alpha) {
    // pts: flat [x,y,...] or array of [x,y]
    const flat = pts.length && Array.isArray(pts[0]) ? null : pts;
    const n = flat ? flat.length / 2 : pts.length;
    if (n < 2) return;
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const [sx, sy] = flat ? w2s(flat[2 * i], flat[2 * i + 1]) : w2s(pts[i][0], pts[i][1]);
      if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawDot(x, y, r, color) {
    const [sx, sy] = w2s(x, y);
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.fill();
  }

  function drawScan() {
    const p = S.scan;
    ctx.fillStyle = colors['c-scan'];
    const sz = clamp(V.scale * 0.04, 2, 4);
    for (let i = 0; i < p.length; i += 2) {
      const [sx, sy] = w2s(p[i], p[i + 1]);
      if (sx < -5 || sy < -5 || sx > W + 5 || sy > H + 5) continue;
      ctx.fillRect(sx - sz / 2, sy - sz / 2, sz, sz);
    }
  }

  function drawArrow(x, y, yaw, color, lenPx, width) {
    const [sx, sy] = w2s(x, y);
    const ex = sx + Math.cos(yaw) * lenPx, ey = sy - Math.sin(yaw) * lenPx;
    ctx.save();
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
    const h = Math.max(8, width * 3.2), a = -yaw;
    ctx.beginPath();
    ctx.moveTo(ex + Math.cos(a) * h * 0.4, ey + Math.sin(a) * h * 0.4);
    ctx.lineTo(ex + Math.cos(a + 2.5) * h, ey + Math.sin(a + 2.5) * h);
    ctx.lineTo(ex + Math.cos(a - 2.5) * h, ey + Math.sin(a - 2.5) * h);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  function drawGoal(g, active) {
    const [sx, sy] = w2s(g.x, g.y);
    const r = Math.max(7, 0.12 * V.scale);
    ctx.save();
    ctx.globalAlpha = active ? 1 : 0.45;
    ctx.strokeStyle = colors['c-goal']; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.stroke();
    drawArrow(g.x, g.y, g.yaw, colors['c-goal'], Math.max(26, 0.45 * V.scale), 3);
    ctx.restore();
  }

  function drawRobot() {
    const r = S.state && S.state.robot;
    if (!r) return;
    const fp = S.footprint && S.footprint.length >= 6 ? S.footprint : DEFAULT_FOOTPRINT;
    const c = Math.cos(r.yaw), s = Math.sin(r.yaw);
    ctx.save();
    ctx.beginPath();
    for (let i = 0; i < fp.length; i += 2) {
      const [sx, sy] = w2s(r.x + c * fp[i] - s * fp[i + 1], r.y + s * fp[i] + c * fp[i + 1]);
      if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
    }
    ctx.closePath();
    ctx.fillStyle = colors['c-robot']; ctx.globalAlpha = 0.35; ctx.fill();
    ctx.globalAlpha = 1; ctx.strokeStyle = colors['c-robot']; ctx.lineWidth = 2; ctx.stroke();
    const [sx, sy] = w2s(r.x, r.y);
    ctx.fillStyle = colors['c-robot'];
    ctx.strokeStyle = colors.surface; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(sx, sy, 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.restore();
    drawArrow(r.x, r.y, r.yaw, colors['c-robot'], Math.max(24, 0.35 * V.scale), 3);
  }

  function drawAim() {
    const color = drag.type === 'goal' ? colors['c-goal'] : colors['c-pose'];
    const yaw = aimYaw(drag);
    const [sx, sy] = w2s(drag.wx, drag.wy);
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.arc(sx, sy, Math.max(8, 0.12 * V.scale), 0, Math.PI * 2); ctx.stroke();
    ctx.restore();
    drawArrow(drag.wx, drag.wy, yaw, color, Math.max(36, 0.5 * V.scale), 3);
  }

  function updateScale() {
    const target = 110 / V.scale;  // metres in ~110 px
    const p = Math.pow(10, Math.floor(Math.log10(target)));
    const m = [1, 2, 5, 10].map((k) => k * p).filter((v) => v <= target).pop() || p;
    $('scaleLine').style.width = (m * V.scale) + 'px';
    $('scaleText').textContent = m >= 1 ? `${m} m` : `${Math.round(m * 100)} cm`;
  }

  // ---------- incoming data ----------
  async function inflate(u8) {
    const ds = new DecompressionStream('deflate');
    const buf = await new Response(new Blob([u8]).stream().pipeThrough(ds)).arrayBuffer();
    return new Uint8Array(buf);
  }

  async function onGrid(h, payload) {
    const raw = await inflate(payload);
    if (raw.length !== h.w * h.h) { console.warn('grid size mismatch', h); return; }
    let g = S.grids[h.key];
    if (h.full || !g || g.meta.width !== h.width || g.meta.height !== h.height) {
      if (!h.full && !g) return;
      if (!g || g.canvas.width !== h.width || g.canvas.height !== h.height) {
        const c = document.createElement('canvas');
        c.width = h.width; c.height = h.height;
        g = S.grids[h.key] = { canvas: c, ctx: c.getContext('2d'), meta: h, bbox: null };
      }
    }
    g.meta = h;
    const lut = h.key === 'map' ? LUT_MAP : LUT_COST;
    const img = g.ctx.createImageData(h.w, h.h);
    const px = new Uint32Array(img.data.buffer);
    for (let i = 0; i < raw.length; i++) px[i] = lut[raw[i]];
    g.ctx.putImageData(img, h.x, h.y);

    if (h.key === 'map' && h.full) {
      // bounding box of known cells, for "Fit map"
      let x0 = h.w, y0 = h.h, x1 = -1, y1 = -1;
      for (let y = 0; y < h.h; y++) {
        const row = y * h.w;
        for (let x = 0; x < h.w; x++) {
          if (raw[row + x] !== 255) {
            if (x < x0) x0 = x; if (x > x1) x1 = x;
            if (y < y0) y0 = y; if (y > y1) y1 = y;
          }
        }
      }
      g.bbox = x1 >= 0 ? [x0, y0, x1 + 1, y1 + 1] : null;
      if (!V.fitted) fitMap();
    }
    dirty();
  }

  function onState(m) {
    S.state = m;
    const r = m.robot;
    if (r) {
      const t = S.trail, last = t[t.length - 1];
      if (!last || Math.hypot(last[0] - r.x, last[1] - r.y) > 0.03) {
        t.push([r.x, r.y]);
        if (t.length > 4000) t.splice(0, t.length - 4000);
      }
    }
    updatePanel(m);
    dirty();
  }

  async function handle(data) {
    if (typeof data === 'string') {
      const m = JSON.parse(data);
      switch (m.type) {
        case 'hello': onHello(m); break;
        case 'state': onState(m); break;
        case 'path': S.paths[m.key] = m.pts; dirty(); break;
        case 'footprint': S.footprint = m.pts; dirty(); break;
        case 'event': toast(m.text, m.level); break;
      }
      return;
    }
    const dv = new DataView(data);
    const hl = dv.getUint32(0, true);
    const h = JSON.parse(new TextDecoder().decode(new Uint8Array(data, 4, hl)));
    if (h.type === 'grid') await onGrid(h, new Uint8Array(data, 4 + hl));
    else if (h.type === 'scan') { S.scan = new Float32Array(data, 4 + hl, h.n * 2); dirty(); }
  }

  // ---------- connection ----------
  let ws = null, chain = Promise.resolve();
  function connect() {
    const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
    ws = new WebSocket(url);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => setConnected(true);
    ws.onclose = () => { setConnected(false); setTimeout(connect, 1500); };
    ws.onerror = () => {};
    ws.onmessage = (ev) => {
      // serialise: grid decoding is async and must stay in order
      chain = chain.then(() => handle(ev.data)).catch((err) => console.error(err));
    };
  }
  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify(obj)); return true; }
    toast('Not connected to the robot', 'error');
    return false;
  }
  function setConnected(on) {
    S.connected = on;
    $('connDot').classList.toggle('on', on);
    $('connDot').title = on ? 'connected' : 'disconnected';
    $('overlay').hidden = on;
    $('overlay').textContent = on ? '' : 'Connection lost — reconnecting…';
  }
  function onHello(m) {
    S.hello = m;
    document.querySelectorAll('.ctl').forEach((b) => { b.disabled = !m.allow_control; });
    if (!m.allow_control) toast('View-only mode: control disabled on the robot', 'warn');
    buildSources();
  }

  // ---------- side panel ----------
  const fmt = (v, d, unit = '') => (v === null || v === undefined || Number.isNaN(v)) ? '—' : v.toFixed(d) + unit;
  function fmtTime(s) {
    if (s === null || s === undefined) return '—';
    const m = Math.floor(s / 60), r = Math.round(s % 60);
    return m ? `${m}m ${String(r).padStart(2, '0')}s` : `${r}s`;
  }
  function updatePanel(m) {
    const nav = m.nav || {};
    const badge = $('navBadge');
    const status = nav.status || 'idle';
    badge.textContent = status;
    badge.className = 'badge ' + status;
    $('nav2Ready').textContent = nav.ready ? 'Nav2 ready' : 'Nav2 not running';
    const fb = nav.active ? nav.feedback : null;
    $('fbDist').textContent = fb ? fmt(fb.distance, 2, ' m') : '—';
    $('fbEta').textContent = fb && fb.eta > 0 ? fmtTime(fb.eta) : '—';
    $('fbTime').textContent = fb ? fmtTime(fb.time) : '—';
    $('fbRec').textContent = fb ? String(fb.recoveries) : '—';
    $('goalTxt').textContent = nav.goal ? `${nav.goal.x.toFixed(2)}, ${nav.goal.y.toFixed(2)}` : '—';
    const r = m.robot;
    $('rPos').textContent = r ? `${r.x.toFixed(2)}, ${r.y.toFixed(2)}` : 'no localization';
    $('rYaw').textContent = r ? fmt(r.yaw * 180 / Math.PI, 0, '°') : '—';
    $('rVel').textContent = m.vel ? fmt(m.vel.v, 2, ' m/s') : '—';
    $('rRot').textContent = m.vel ? fmt(m.vel.w, 2, ' rad/s') : '—';
    updateSources(m.ages || {});
  }

  // fresh-within (s); null = latched topic, fine once received
  const SOURCES = [
    ['map', 'map', null], ['global_costmap', 'global_costmap', null], ['local_costmap', 'local_costmap', 5],
    ['scan', 'scan', 2], ['odom', 'odom', 2], ['plan', 'plan', null],
  ];
  function buildSources() {
    const ul = $('sources');
    ul.innerHTML = '';
    for (const [key, tkey] of SOURCES) {
      const li = document.createElement('li');
      li.innerHTML = '<span class="dot"></span><span class="topic mono"></span><span class="age mono"></span>';
      li.querySelector('.topic').textContent = (S.hello && S.hello.topics[tkey]) || key;
      li.dataset.key = key;
      ul.appendChild(li);
    }
  }
  function updateSources(ages) {
    for (const li of $('sources').children) {
      const [, , fresh] = SOURCES.find((s) => s[0] === li.dataset.key);
      const age = ages[li.dataset.key];
      const dot = li.querySelector('.dot');
      dot.className = 'dot' + (age === null || age === undefined ? '' : (fresh === null || age <= fresh ? ' ok' : ' stale'));
      li.querySelector('.age').textContent = age === null || age === undefined ? 'waiting' : (age < 60 ? age.toFixed(1) + 's' : Math.round(age / 60) + 'm');
    }
  }

  function toast(text, level = 'info') {
    const el = document.createElement('div');
    el.className = 'toast ' + level;
    el.textContent = text;
    $('toasts').appendChild(el);
    while ($('toasts').children.length > 5) $('toasts').firstChild.remove();
    setTimeout(() => { el.style.opacity = '0'; setTimeout(() => el.remove(), 400); }, 4000);
  }

  // ---------- view controls ----------
  function fitMap() {
    const g = S.grids.map;
    if (!g || W < 10) return;
    const T = frameT(g.meta.frame);
    if (!T) return;
    const b = g.bbox || [0, 0, g.meta.width, g.meta.height];
    const res = g.meta.res;
    const o = compose(T, [g.meta.ox, g.meta.oy, g.meta.oyaw]);
    const xs = [], ys = [];
    for (const [cx, cy] of [[b[0], b[1]], [b[2], b[1]], [b[0], b[3]], [b[2], b[3]]]) {
      const p = compose(o, [cx * res, cy * res, 0]);
      xs.push(p[0]); ys.push(p[1]);
    }
    const bw = Math.max(...xs) - Math.min(...xs), bh = Math.max(...ys) - Math.min(...ys);
    V.cx = (Math.max(...xs) + Math.min(...xs)) / 2;
    V.cy = (Math.max(...ys) + Math.min(...ys)) / 2;
    V.scale = clamp(0.92 * Math.min(W / Math.max(bw, 0.5), H / Math.max(bh, 0.5)), 0.5, 400);
    V.fitted = true;
    setFollow(false);
    dirty();
  }
  function setFollow(on) {
    V.follow = on;
    $('followBtn').setAttribute('aria-pressed', String(on));
    dirty();
  }
  function zoomAt(sx, sy, f) {
    if (V.follow) { sx = W / 2; sy = H / 2; }
    const [wx, wy] = s2w(sx, sy);
    V.scale = clamp(V.scale * f, 0.5, 400);
    const [nx, ny] = s2w(sx, sy);
    V.cx += wx - nx; V.cy += wy - ny;
    dirty();
  }
  const HINTS = {
    goal: 'Nav goal: press where the robot should go, drag toward the direction it should face, release. Esc to cancel.',
    pose: 'Set pose: press where the robot really is, drag toward the direction it faces, release. Esc to cancel.',
  };
  function setMode(m) {
    mode = m;
    document.querySelectorAll('.seg button').forEach((b) => b.classList.toggle('active', b.dataset.mode === m));
    canvas.classList.toggle('aim', m !== 'pan');
    $('hint').hidden = m === 'pan';
    $('hint').textContent = HINTS[m] || '';
    drag = null;
    dirty();
  }

  // ---------- pointer input ----------
  const pointers = new Map();
  let drag = null, pinch = null;

  function aimYaw(d) {
    const dx = d.ex - d.wx, dy = d.ey - d.wy;
    if (Math.hypot(dx, dy) * V.scale > 10) return Math.atan2(dy, dx);
    const r = S.state && S.state.robot;
    if (!r) return 0;
    if (d.type === 'pose') return r.yaw;                  // plain click: keep heading
    return Math.atan2(d.wy - r.y, d.wx - r.x);            // plain click: face away from robot
  }
  function pos(e) {
    const r = canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }

  canvas.addEventListener('pointerdown', (e) => {
    if (e.button !== 0 && e.pointerType === 'mouse') return;
    canvas.setPointerCapture(e.pointerId);
    const [x, y] = pos(e);
    pointers.set(e.pointerId, [x, y]);
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      pinch = { d: Math.hypot(a[0] - b[0], a[1] - b[1]), scale: V.scale, world: s2w(mid[0], mid[1]) };
      drag = null;
      return;
    }
    if (pointers.size > 2) return;
    if (mode === 'pan') {
      drag = { type: 'pan', sx: x, sy: y, cx: V.cx, cy: V.cy };
      canvas.classList.add('dragging');
    } else {
      const [wx, wy] = s2w(x, y);
      drag = { type: mode, wx, wy, ex: wx, ey: wy };
    }
    dirty();
  });

  canvas.addEventListener('pointermove', (e) => {
    const [x, y] = pos(e);
    const [wx, wy] = s2w(x, y);
    $('cursor').textContent = `x ${wx.toFixed(2)}  y ${wy.toFixed(2)}`;
    if (!pointers.has(e.pointerId)) return;
    pointers.set(e.pointerId, [x, y]);
    if (pinch && pointers.size >= 2) {
      const [a, b] = [...pointers.values()];
      const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      V.scale = clamp(pinch.scale * d / Math.max(pinch.d, 1), 0.5, 400);
      if (!V.follow) {
        V.cx = pinch.world[0] - (mid[0] - W / 2) / V.scale;
        V.cy = pinch.world[1] + (mid[1] - H / 2) / V.scale;
      }
      dirty();
      return;
    }
    if (!drag) return;
    if (drag.type === 'pan') {
      if (V.follow && Math.hypot(x - drag.sx, y - drag.sy) > 4) setFollow(false);
      if (!V.follow) {
        V.cx = drag.cx - (x - drag.sx) / V.scale;
        V.cy = drag.cy + (y - drag.sy) / V.scale;
      }
    } else {
      drag.ex = wx; drag.ey = wy;
    }
    dirty();
  });

  function endPointer(e, cancelled) {
    pointers.delete(e.pointerId);
    canvas.classList.remove('dragging');
    if (pinch) { if (pointers.size < 2) pinch = null; drag = null; dirty(); return; }
    if (drag && !cancelled && (drag.type === 'goal' || drag.type === 'pose')) {
      const yaw = wrap(aimYaw(drag));
      const ok = send({ cmd: drag.type === 'goal' ? 'goal' : 'initialpose', x: drag.wx, y: drag.wy, yaw });
      if (ok && drag.type === 'goal') S.trail = S.trail.slice(-1);
      setMode('pan');
    }
    drag = null;
    dirty();
  }
  canvas.addEventListener('pointerup', (e) => endPointer(e, false));
  canvas.addEventListener('pointercancel', (e) => endPointer(e, true));
  canvas.addEventListener('pointerleave', () => { $('cursor').textContent = ''; });
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const [x, y] = pos(e);
    zoomAt(x, y, Math.exp(-e.deltaY * (e.deltaMode === 1 ? 0.05 : 0.0015)));
  }, { passive: false });

  // ---------- buttons & keys ----------
  document.querySelectorAll('.seg button').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
  $('cancelBtn').addEventListener('click', () => send({ cmd: 'cancel' }));
  $('followBtn').addEventListener('click', () => {
    if (!V.follow && !(S.state && S.state.robot)) { toast('Robot position unknown (no map → base_link transform)', 'warn'); return; }
    setFollow(!V.follow);
  });
  $('fitBtn').addEventListener('click', fitMap);
  $('clearTrail').addEventListener('click', () => { S.trail = []; dirty(); });
  document.querySelectorAll('#layers input').forEach((cb) => cb.addEventListener('change', () => {
    layers[cb.dataset.layer] = cb.checked; dirty();
  }));
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.ctrlKey || e.metaKey || e.altKey) return;
    const k = e.key.toLowerCase();
    const control = S.hello && S.hello.allow_control;
    if (k === 'escape') setMode('pan');
    else if (k === 'g' && control) setMode('goal');
    else if (k === 'p' && control) setMode('pose');
    else if (k === 'f') $('followBtn').click();
    else if (k === '+' || k === '=') zoomAt(W / 2, H / 2, 1.25);
    else if (k === '-') zoomAt(W / 2, H / 2, 0.8);
  });

  new ResizeObserver(resize).observe(view);
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readColors);
  if (typeof DecompressionStream === 'undefined') {
    $('overlay').textContent = 'This browser is too old (no DecompressionStream). Use a current Chrome, Firefox or Safari.';
    return;
  }
  // URL options, e.g.  http://robot:8080/#follow&zoom=60   (zoom = pixels per metre)
  const opts = new URLSearchParams(location.hash.slice(1));
  if (opts.has('zoom')) V.scale = clamp(parseFloat(opts.get('zoom')) || 25, 0.5, 400);
  if (opts.has('follow')) { V.fitted = true; setFollow(true); }
  readColors();
  resize();
  buildSources();
  connect();
})();
