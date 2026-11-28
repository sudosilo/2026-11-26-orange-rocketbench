/*
  Rocketbench flight core v2. Pure simulation, no DOM, no rendering.

  var sim = RocketSim.create();
  sim.on('apogee', function (e) {});
  sim.setDesign(design);
  sim.launch({ tilt: 0, wind: 4, spin: 0, autoStab: true, autoStage: true, autoChute: true, mainAlt: 0, orbit: 0, breakMode: false });
  each frame: sim.step(realSeconds);
  sim.reset();

  Units are SI. The world is a round, non rotating planet in a 2D plane.
  Inertial frame: planet center at the origin, the pad at (0, RE). Body attitude phi is the
  inertial angle of the nose axis from +Y toward +X. Local tilt th = phi minus the position angle.
  Body stations s run from the nose tip (0) toward the tail.
*/
(function (root) {
  'use strict';

  var G0 = 9.80665, RE = 6371000, MU = 3.986004418e14, RAIR = 287.053, GAM = 1.4;
  var SIGMA_ALLOW = 90e6;   // allowable wall stress after buckling knockdown
  var WALL_RHO = 1600;      // composite wall density
  var RAIL_LEN = 6;
  var DT = 0.002;
  var NST = 48;             // stations per body for loads and heat
  var RCS_ISP = 70;
  var SB = 5.670e-8, EPS_RAD = 0.8, CP_WALL = 900, HEAT_MASS = 3;  // radiation and wall heat capacity

  // ---------- atmosphere: US Standard 1976 to 86 km, then a thermosphere fit ----------
  var LAY = [[0, -0.0065], [11000, 0], [20000, 0.001], [32000, 0.0028], [47000, 0], [51000, -0.0028], [71000, -0.002], [84852, 0]];
  var LT = [288.15], LP = [101325];
  (function () {
    for (var i = 1; i < LAY.length; i++) {
      var hb = LAY[i - 1][0], L = LAY[i - 1][1], h = LAY[i][0], Tb = LT[i - 1], Pb = LP[i - 1];
      if (L === 0) { LT.push(Tb); LP.push(Pb * Math.exp(-G0 * (h - hb) / (RAIR * Tb))); }
      else { var T = Tb + L * (h - hb); LT.push(T); LP.push(Pb * Math.pow(T / Tb, -G0 / (L * RAIR))); }
    }
  })();
  function layerAtm(h) {
    var k = LAY.length - 1;
    while (k > 0 && h < LAY[k][0]) k--;
    var hb = LAY[k][0], L = LAY[k][1], Tb = LT[k], Pb = LP[k], T, P;
    if (L === 0) { T = Tb; P = Pb * Math.exp(-G0 * (h - hb) / (RAIR * Tb)); }
    else { T = Tb + L * (h - hb); P = Pb * Math.pow(T / Tb, -G0 / (L * RAIR)); }
    return { T: T, p: P };
  }
  var A86 = layerAtm(86000), RHO86 = A86.p / (RAIR * A86.T);
  function atmosphere(alt) {
    var z = Math.max(0, alt);
    if (z > 86000) {
      var rho = RHO86 * Math.pow(1 + 0.075 * (z - 86000) / 6000, -1 / 0.075);
      var T = A86.T;
      return { T: T, p: rho * RAIR * T, rho: rho, a: Math.sqrt(GAM * RAIR * T) };
    }
    var h = RE * z / (RE + z);
    var r = layerAtm(h);
    return { T: r.T, p: r.p, rho: r.p / (RAIR * r.T), a: Math.sqrt(GAM * RAIR * r.T) };
  }

  // ---------- engines ----------
  // Fvac N, Isp vac s, Ae exit area m2 (thrust = Fvac - p * Ae), dia widest point, tier 1 hobby, 2 sounding, 3 orbital.
  var ENGINES = {
    H:     { key: 'H',     tier: 1, label: 'H hobby',        kind: 'solid',  Fvac: 230,    Isp: 190, Ae: 0.0006, prop: 0.13, dry: 0.25, len: 0.32, dia: 0.038, gimbal: 0, bell: 'sea' },
    J:     { key: 'J',     tier: 1, label: 'J hobby',        kind: 'solid',  Fvac: 700,    Isp: 195, Ae: 0.0012, prop: 0.55, dry: 0.6,  len: 0.5,  dia: 0.054, gimbal: 0, bell: 'sea' },
    K:     { key: 'K',     tier: 1, label: 'K hobby',        kind: 'solid',  Fvac: 1600,   Isp: 200, Ae: 0.0025, prop: 1.3,  dry: 1.4,  len: 0.6,  dia: 0.075, gimbal: 0, bell: 'sea' },
    L:     { key: 'L',     tier: 1, label: 'L hobby',        kind: 'solid',  Fvac: 2600,   Isp: 205, Ae: 0.004,  prop: 2.4,  dry: 2.3,  len: 0.75, dia: 0.075, gimbal: 0, bell: 'sea' },
    M:     { key: 'M',     tier: 1, label: 'M hobby',        kind: 'solid',  Fvac: 5200,   Isp: 210, Ae: 0.007,  prop: 4.6,  dry: 4,    len: 1.0,  dia: 0.098, gimbal: 0, bell: 'sea' },
    A:     { key: 'A',     tier: 2, label: 'A solid',        kind: 'solid',  Fvac: 5000,   Isp: 225, Ae: 0.006,  prop: 30,   dry: 10,   len: 1.0,  dia: 0.16,  gimbal: 0, bell: 'sea' },
    C:     { key: 'C',     tier: 2, label: 'C solid',        kind: 'solid',  Fvac: 14000,  Isp: 235, Ae: 0.016,  prop: 100,  dry: 30,   len: 1.6,  dia: 0.24,  gimbal: 0, bell: 'sea' },
    E:     { key: 'E',     tier: 2, label: 'E solid',        kind: 'solid',  Fvac: 40000,  Isp: 245, Ae: 0.035,  prop: 360,  dry: 100,  len: 2.4,  dia: 0.36,  gimbal: 0, bell: 'sea' },
    Perox: { key: 'Perox', tier: 2, label: 'Perox peroxide', kind: 'mono',   cryo: false, Fvac: 9000,    Isp: 190, Ae: 0.05, dry: 32,   len: 1.0, dia: 0.26, gimbal: 3, bell: 'vac', rho: 1390 },
    Meth:  { key: 'Meth',  tier: 2, label: 'Meth methalox',  kind: 'biprop', cryo: true,  Fvac: 60000,   Isp: 330, Ae: 0.09, dry: 120,  len: 1.5, dia: 0.36, gimbal: 5, bell: 'sea', rho: 830 },
    Vac:   { key: 'Vac',   tier: 2, label: 'Vac methalox',   kind: 'biprop', cryo: true,  Fvac: 36000,   Isp: 365, Ae: 0.36, dry: 95,   len: 2.0, dia: 0.68, gimbal: 4, bell: 'vac', rho: 830 },
    Hydra: { key: 'Hydra', tier: 3, label: 'Hydra storable', kind: 'biprop', cryo: false, Fvac: 90000,   Isp: 320, Ae: 0.4,  dry: 150,  len: 1.6, dia: 0.75, gimbal: 4, bell: 'vac', rho: 1150 },
    Titan: { key: 'Titan', tier: 3, label: 'Titan methalox', kind: 'biprop', cryo: true,  Fvac: 2000000, Isp: 345, Ae: 1.1,  dry: 1700, len: 3.6, dia: 1.3,  gimbal: 6, bell: 'sea', rho: 830 },
    Apex:  { key: 'Apex',  tier: 3, label: 'Apex vacuum',    kind: 'biprop', cryo: true,  Fvac: 650000,  Isp: 378, Ae: 4.0,  dry: 950,  len: 4.2, dia: 2.3,  gimbal: 5, bell: 'vac', rho: 830 }
  };
  var PACK = [0, 1, 2, 2.155, 2.414, 2.701, 3, 3, 3.305, 3.613];  // cluster footprint in engine diameters
  function engineThrust(p, pAmb) {
    var back = pAmb * p.Ae, sep = false, F = p.Fvac - back;
    if (back > 0.55 * p.Fvac) { sep = true; F = 0.45 * p.Fvac - 0.2 * back; }
    return { F: Math.max(0, F), sep: sep };
  }

  // ---------- helpers ----------
  function rng(seed) {
    var a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0; var t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function gauss(r) { var u = r() || 1e-9, v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(6.283185 * v); }
  function wrapPi(a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }

  function orbitOf(X, Y, VX, VY) {
    var r = Math.hypot(X, Y), v2 = VX * VX + VY * VY, eps = v2 / 2 - MU / r, h = X * VY - Y * VX;
    var vr = (X * VX + Y * VY) / r;
    var e = Math.sqrt(Math.max(0, 1 + 2 * eps * h * h / (MU * MU)));
    var o = { r: r, vr: vr, eps: eps, h: h, e: e };
    var rp = h * h / MU / (1 + e);
    o.pe = rp - RE;
    if (eps >= 0) { o.esc = true; o.ap = Infinity; return o; }
    var a = -MU / (2 * eps);
    o.a = a; o.ra = a * (1 + e); o.rp = rp; o.ap = o.ra - RE;
    o.T = 2 * Math.PI * Math.sqrt(a * a * a / MU);
    var cosE = e > 1e-7 ? clamp((1 - r / a) / e, -1, 1) : 1;
    var E = Math.acos(cosE); if (vr < 0) E = 2 * Math.PI - E;
    var M = E - e * Math.sin(E), n = Math.sqrt(MU / (a * a * a));
    o.tAp = (Math.PI - M) / n; if (o.tAp < 0) o.tAp += o.T;
    o.vAp = Math.sqrt(Math.max(0, MU * (2 / o.ra - 1 / a)));
    o.vCircAp = Math.sqrt(MU / o.ra);
    return o;
  }

  // ---------- parts ----------
  // design: { dia, wall (mm), parts: top to bottom:
  //   {type:'nose',len,payload} {type:'tank',len} {type:'engine',key,count} {type:'decoupler'}
  //   {type:'fins',span,count} {type:'rcs'} {type:'chute',dia,kind:'drogue'|'main'} }
  function buildParts(design, rand) {
    var D = design.dia, r = D / 2, t = design.wall / 1000;
    var aw = t * WALL_RHO * 2.0;
    var A = Math.PI * r * r;
    var parts = [], s = 0, section = 0, sections = [{ id: 0, propType: null, engines: [], tanks: [] }];
    design.parts.forEach(function (src, idx) {
      var p = { type: src.type, src: src, idx: idx, sTop: s, len: 0, dry: 0, prop: 0, cap: 0, strength: 1, section: section, tlim: 700 };
      switch (src.type) {
        case 'nose':
          p.len = src.len;
          p.payload = Math.max(0, +src.payload || 0);
          p.dry = aw * Math.PI * r * Math.sqrt(src.len * src.len + r * r) + 0.2 + 8 * D * D + p.payload;
          p.tlim = 1000;
          break;
        case 'tank':
          p.len = src.len;
          p.dry = aw * (Math.PI * D * src.len + 2 * A * 1.5);
          break;
        case 'engine':
          var e = ENGINES[src.key] || ENGINES.C, n = clamp((src.count | 0) || 1, 1, 9);
          p.eng = e; p.n = n; p.len = e.len;
          p.Fvac = e.Fvac * n; p.Ae = e.Ae * n; p.mdot = e.Fvac * n / (e.Isp * G0);
          p.foot = e.dia * PACK[n];
          p.dry = e.dry * n + aw * Math.PI * D * e.len * 0.65;
          if (e.kind === 'solid') { p.prop = p.cap = e.prop * n; }
          p.strength = 3; p.tlim = 1400;
          p.mis = (rand() - 0.5) * 0.005;
          p.gim = 0; p.gimCmd = 0; p.firing = false; p.burnedOut = false; p.used = false; p.igniteT = 0; p.sideOU = 0;
          break;
        case 'decoupler':
          section++;
          sections.push({ id: section, propType: null, engines: [], tanks: [] });
          p.section = section;
          p.len = clamp(D * 0.5, 0.08, 1.2);
          p.dry = aw * Math.PI * D * p.len * 1.5 + 6 * A + 0.1;
          p.strength = 0.75;
          break;
        case 'fins':
          var span = src.span, cr = 0.03 + 0.9 * span + 0.1 * D, ct = 0.4 * cr, fc = src.count | 0;
          p.span = span; p.cr = cr; p.ct = ct; p.count = fc;
          p.area = 0.5 * (cr + ct) * span;
          p.dry = fc * p.area * Math.max(0.002, 3 * t) * WALL_RHO;
          p.sLE = Math.max(0, s - cr);
          p.strength = 1.2; p.tlim = 900;
          if (parts.length) p.section = parts[parts.length - 1].section;
          break;
        case 'rcs':
          p.len = clamp(D * 0.4, 0.12, 1.0);
          p.dry = aw * Math.PI * D * p.len + 2 + 30 * A;
          p.prop = p.cap = 2 + 12 * A;
          p.strength = 1.2; p.cmd = 0;
          break;
        case 'chute':
          p.kind = src.kind === 'drogue' ? 'drogue' : 'main';
          p.chuteDia = src.dia;
          p.len = clamp(D * 1.2, 0.1, 0.6) + src.dia * 0.01;
          p.dry = aw * Math.PI * D * p.len + 0.25 * src.dia * src.dia + 0.05;
          p.rating = (p.kind === 'drogue' ? 6000 : 2200) * src.dia * src.dia;
          p.cd = p.kind === 'drogue' ? 0.6 : 1.3;
          p.machMax = p.kind === 'drogue' ? 2.5 : 0.7;
          p.open = false; p.frac = 0; p.t = 0; p.health = 1; p.ripped = false; p.cut = false; p.load = 0;
          break;
      }
      s += p.len;
      parts.push(p);
    });
    parts.forEach(function (p) {
      var sec = sections[p.section];
      if (p.type === 'engine') sec.engines.push(p);
      if (p.type === 'tank') sec.tanks.push(p);
    });
    sections.forEach(function (sec) {
      var liq = sec.engines.filter(function (p) { return p.eng.kind !== 'solid'; });
      sec.propType = liq.length ? liq[0].eng.key : null;
      sec.mixed = liq.some(function (p) { return p.eng.rho !== liq[0].eng.rho; });
      sec.tanks.forEach(function (p) {
        p.usable = Math.max(0, p.len - 0.25);
        p.lossUllage = p.len > 0 ? 1 - p.usable / p.len : 0;
        p.lossBoil = 0;
        if (!sec.propType) { p.cap = 0; p.prop = 0; return; }
        var e = ENGINES[sec.propType];
        var cap = e.rho * A * p.usable;
        if (e.cryo) { var sv = 4 / D + 2 / p.len; p.lossBoil = Math.min(0.4, 0.008 * sv); }
        p.cap = cap;
        p.prop = cap * (1 - p.lossBoil);
        p.fluid = e.kind;
        p.dry += 0.06 * cap;
      });
    });
    return { parts: parts, sections: sections, D: D, A: A, wall: t, aw: aw };
  }

  function partCenter(p) { return p.type === 'fins' ? p.sLE + p.cr * 0.5 : p.sTop + p.len * 0.5; }
  function propCenter(p) {
    if (p.type === 'tank') { var f = p.cap > 0 ? p.prop / p.cap : 0; return p.sTop + p.len - p.usable * f * 0.5 - 0.12; }
    return p.sTop + p.len * 0.5;
  }

  // ---------- bodies ----------
  var bodyId = 0;
  function newBody(parts, geo) {
    var b = { id: ++bodyId, parts: parts, D: geo.D, r: geo.D / 2, A: geo.A, wall: geo.wall, aw: geo.aw,
      X: 0, Y: RE, VX: 0, VY: 0, phi: 0, om: 0, spin: 0, roll: 0,
      x: 0, y: 0, th: 0, lam: 0,
      onRail: false, railDist: 0, railV: 0, alive: true, isMain: false, broken: false,
      outcome: null, maxY: 0, rising: false, pastApogee: false, apoT: 0,
      traj: [], trajT: -1, lastF: null, loads: new Float32Array(NST), heat: new Float32Array(NST), T: new Float32Array(NST),
      loadMax: 0, heatMax: 0, Tmax: 290, mp: null, tele: {}, chuteOpen: false };
    for (var j = 0; j < NST; j++) b.T[j] = 290;
    prepBody(b);
    return b;
  }
  function prepBody(b) {
    var off = b.parts.length ? b.parts[0].sTop : 0;
    b.parts.forEach(function (p) { p.sTop -= off; if (p.type === 'fins') p.sLE -= off; });
    var L = 0;
    b.parts.forEach(function (p) { L = Math.max(L, p.sTop + p.len); });
    b.L = Math.max(0.1, L);
    var top = b.parts[0];
    var lift = [];
    if (top && top.type === 'nose') {
      b.Ln = top.len; b.sinD2 = Math.pow(Math.sin(Math.atan(b.r / top.len)), 2);
      lift.push({ cna: 2, s: 0.666 * top.len });
      b.Rn = Math.max(0.006, 0.3 * b.r / Math.max(1, top.len / b.r));
    } else { b.Ln = 0; b.sinD2 = 1; lift.push({ cna: 2, s: 0.02 }); b.Rn = b.r; }
    var finDrag = 0;
    b.parts.forEach(function (p) {
      if (p.type !== 'fins' || p.count < 1) return;
      var s = p.span, cr = p.cr, ct = p.ct, xt = cr - ct, R = b.r, D = b.D;
      var lm = Math.sqrt(s * s + Math.pow(xt + ct / 2 - cr / 2, 2));
      var n = p.count;
      var cna = (1 + R / (s + R)) * (4 * n * Math.pow(s / D, 2)) / (1 + Math.sqrt(1 + Math.pow(2 * lm / (cr + ct), 2)));
      if (n > 4) cna *= 0.92;
      var xf = xt * (cr + 2 * ct) / (3 * (cr + ct)) + (1 / 6) * (cr + ct - cr * ct / (cr + ct));
      p.cna = cna;
      lift.push({ cna: cna, s: p.sLE + xf });
      finDrag += n * p.area * 0.02;
    });
    b.lift = lift;
    b.cnaTot = lift.reduce(function (a, l) { return a + l.cna; }, 0);
    b.sCp = lift.reduce(function (a, l) { return a + l.cna * l.s; }, 0) / b.cnaTot;
    b.cdFric = 0.0035 * (Math.PI * b.D * b.L) / b.A;
    b.cdFins = finDrag / b.A;
    b.hasNose = !!(top && top.type === 'nose');
    var ds = b.L / NST;
    b.ds = ds;
    b.stPart = new Int16Array(NST);
    b.stDry = new Float32Array(NST);
    b.stFin = new Uint8Array(NST);
    b.stTlim = new Float32Array(NST);
    b.hFwd = new Float32Array(NST);
    b.hBwd = new Float32Array(NST);
    for (var j = 0; j < NST; j++) {
      var sj = (j + 0.5) * ds, k = 0;
      for (var i = 0; i < b.parts.length; i++) { var p = b.parts[i]; if (p.len > 0 && sj >= p.sTop && sj < p.sTop + p.len) { k = i; break; } if (p.len > 0 && sj >= p.sTop) k = i; }
      b.stPart[j] = k;
      b.stTlim[j] = b.parts[k] ? b.parts[k].tlim : 700;
    }
    b.parts.forEach(function (p) {
      var a0 = p.type === 'fins' ? p.sLE : p.sTop, l = p.type === 'fins' ? p.cr : p.len;
      spread(b.stDry, a0, l, p.dry, ds);
      if (p.type === 'fins') for (var jj = 0; jj < NST; jj++) { var sx = (jj + 0.5) * ds; if (sx >= p.sLE && sx <= p.sLE + p.cr) { b.stFin[jj] = 1; b.stTlim[jj] = Math.min(b.stTlim[jj], 900); } }
    });
    // heating shape along the body, nose first and tail first
    for (j = 0; j < NST; j++) {
      var s0 = (j + 0.5) * ds;
      b.hFwd[j] = heatShape(s0, b.Ln, b.D) + (b.stFin[j] ? 0.12 : 0);
      b.hBwd[j] = heatShape(b.L - s0, 0, b.D) + (b.stFin[j] ? 0.12 : 0);
    }
    b.chutes = b.parts.filter(function (p) { return p.type === 'chute'; });
    b.rcsParts = b.parts.filter(function (p) { return p.type === 'rcs'; });
  }
  function heatShape(s, Ln, D) {
    if (Ln > 0 && s < Ln) return 0.1 + 0.9 * Math.exp(-s / (0.2 * Ln));
    if (Ln <= 0 && s < D * 0.5) return 1;
    var sb = Math.max(0, s - Math.max(Ln, D * 0.5));
    return Math.max(0.03, 0.08 * Math.pow(1 + sb / D, -0.3));
  }
  function spread(arr, a0, l, mass, ds) {
    if (mass <= 0) return;
    if (l <= 1e-6) { arr[clamp(Math.floor(a0 / ds), 0, NST - 1)] += mass; return; }
    var a1 = a0 + l;
    for (var j = clamp(Math.floor(a0 / ds), 0, NST - 1); j < NST; j++) {
      var s0 = j * ds, s1 = s0 + ds;
      if (s0 >= a1) break;
      var ov = Math.min(s1, a1) - Math.max(s0, a0);
      if (ov > 0) arr[j] += mass * ov / l;
    }
  }

  function massProps(b) {
    var m = 0, ms = 0;
    for (var i = 0; i < b.parts.length; i++) {
      var p = b.parts[i];
      m += p.dry; ms += p.dry * partCenter(p);
      if (p.prop > 0) { m += p.prop; ms += p.prop * propCenter(p); }
    }
    var scg = ms / m, I = 0;
    for (i = 0; i < b.parts.length; i++) {
      p = b.parts[i];
      var c = partCenter(p), l = p.type === 'fins' ? p.cr : p.len;
      I += p.dry * ((c - scg) * (c - scg) + l * l / 12);
      if (p.prop > 0) { var pc = propCenter(p); I += p.prop * ((pc - scg) * (pc - scg) + l * l / 24); }
    }
    I += m * b.r * b.r / 4;
    return { m: m, scg: scg, Iy: Math.max(I, 1e-4), Ix: Math.max(m * b.r * b.r * 0.6, 1e-5) };
  }

  function liquidPool(b, section) {
    var tot = 0;
    for (var i = 0; i < b.parts.length; i++) { var p = b.parts[i]; if (p.type === 'tank' && p.section === section) tot += p.prop; }
    return tot;
  }
  function drainLiquid(b, section, dm) {
    var tot = liquidPool(b, section);
    if (tot <= 0) return 0;
    var take = Math.min(dm, tot), f = take / tot;
    for (var i = 0; i < b.parts.length; i++) { var p = b.parts[i]; if (p.type === 'tank' && p.section === section) p.prop -= p.prop * f; }
    return take / dm;
  }
  function engineHasProp(b, p, secs) {
    if (p.eng.kind === 'solid') return p.prop > 0 && !p.used;
    var sec = secs[p.section];
    if (!sec || sec.propType === null || p.eng.rho !== ENGINES[sec.propType].rho) return false;
    return liquidPool(b, p.section) > 0.01;
  }

  function windProfile(y, w0) {
    if (w0 <= 0) return 0;
    var w = w0 * Math.pow(Math.max(y, 2) / 10, 0.14);
    if (y > 1000) w = w0 * 1.9;
    w += 1.5 * w0 * Math.exp(-Math.pow((y - 11000) / 3500, 2));
    if (y > 20000) w *= Math.exp(-(y - 20000) / 8000);
    return w;
  }

  // ---------- design analysis (static) ----------
  function analyze(design) {
    var g = buildParts(design, rng(1));
    var parts = g.parts, secs = g.sections;
    var warnings = [], blockers = [];
    var b = newBody(parts.slice(), g);
    var mp = massProps(b);
    var tier = 1;
    if (!parts.length) blockers.push('Add parts to build a rocket.');
    parts.forEach(function (p) {
      if (p.type === 'engine') {
        tier = Math.max(tier, p.eng.tier);
        if (p.foot > g.D + 1e-6) blockers.push('Engine ' + p.eng.key + (p.n > 1 ? ' x' + p.n : '') + ' needs ' + p.foot.toFixed(2) + ' m and does not fit a ' + g.D.toFixed(2) + ' m body.');
      }
    });
    if (g.D > 0.16) tier = Math.max(tier, 2);
    if (g.D > 1.2) tier = 3;
    if (parts.length && parts[0].type !== 'nose') warnings.push('No nose cone on top: blunt front, heavy drag and strong heating.');
    var chutes = parts.filter(function (p) { return p.type === 'chute'; });
    if (!chutes.length) warnings.push('No parachute: the rocket will not survive landing.');
    if (parts.length && parts[parts.length - 1].type === 'decoupler') warnings.push('Decoupler at the very bottom does nothing.');
    var stages = [];
    var maxSec = secs.length - 1;
    for (var k = maxSec; k >= 0; k--) {
      var sec = secs[k];
      var above = parts.filter(function (p) { return p.section <= k; });
      var m0 = 0, propS = 0;
      above.forEach(function (p) { m0 += p.dry + p.prop; });
      var eng = sec.engines;
      var Fsl = 0, Fv = 0, md = 0;
      eng.forEach(function (p) {
        var ok = p.eng.kind === 'solid' || (sec.propType && p.eng.rho === ENGINES[sec.propType].rho);
        if (!ok) return;
        Fv += p.Fvac; Fsl += engineThrust(p, 101325).F; md += p.mdot;
        propS += p.eng.kind === 'solid' ? p.prop : 0;
      });
      sec.tanks.forEach(function (p) { propS += p.prop; });
      var ispv = md > 0 ? Fv / (md * G0) : 0, ispsl = md > 0 ? Fsl / (md * G0) : 0;
      var m1 = m0 - propS;
      stages.push({ n: maxSec - k + 1, section: k, mass: m0, prop: propS, dvVac: ispv * G0 * Math.log(m0 / Math.max(m1, 1e-6)),
        dvSL: ispsl * G0 * Math.log(m0 / Math.max(m1, 1e-6)), twrSL: Fsl / (m0 * G0), twrVac: Fv / (m0 * G0), burn: md > 0 ? propS / md : 0,
        engines: eng.map(function (p) { return p.eng.key + (p.n > 1 ? ' x' + p.n : ''); }) });
      var liq = eng.filter(function (p) { return p.eng.kind !== 'solid'; });
      var sn = maxSec - k + 1;
      if (sec.tanks.length && !liq.length) warnings.push('Stage ' + sn + ' has tanks but no liquid engine, they fly empty.');
      if (liq.length && !sec.tanks.length) blockers.push('Engine ' + liq[0].eng.key + ' in stage ' + sn + ' has no tank to feed it.');
      if (sec.mixed) warnings.push('Stage ' + sn + ' mixes propellants. Tanks load for ' + sec.propType + ' only.');
      if (!eng.length && k === maxSec) blockers.push('The bottom stage has no engine.');
      if (eng.length > 1) warnings.push('Stage ' + sn + ' has engines stacked inline. Use one engine part with a cluster count instead.');
      sec.tanks.forEach(function (p) {
        if (p.cap > 0 && (p.lossBoil + p.lossUllage) > 0.12) warnings.push('Tank of ' + p.len.toFixed(1) + ' m in stage ' + sn + ' loses ' + Math.round((p.lossBoil + p.lossUllage) * 100) + '% to ' + (p.lossBoil > 0 ? 'boil off and ' : '') + 'ullage. Longer tanks waste less.');
      });
      eng.forEach(function (p) {
        if (k === maxSec && p.eng.bell === 'vac' && p.eng.kind !== 'mono') warnings.push('Vacuum bell on the first stage: at sea level the flow separates and thrust collapses.');
      });
    }
    var s1 = stages[0];
    if (s1 && s1.twrSL < 1) blockers.push('Liftoff thrust to weight is ' + s1.twrSL.toFixed(2) + '. It cannot leave the pad.');
    else if (s1 && s1.twrSL < 1.3) warnings.push('Liftoff thrust to weight ' + s1.twrSL.toFixed(2) + ': slow off the rail, wind will push it around.');
    var margin0 = (b.sCp - mp.scg) / g.D;
    var saved = parts.map(function (p) { return p.prop; });
    parts.forEach(function (p) { if (p.section === maxSec && (p.type === 'tank' || p.type === 'engine')) p.prop = 0; });
    var margin1 = (b.sCp - massProps(b).scg) / g.D;
    parts.forEach(function (p, i) { p.prop = saved[i]; });
    var gimbaled = parts.some(function (p) { return p.type === 'engine' && p.eng.gimbal > 0 && p.section === maxSec; });
    if (margin0 < 0) warnings.push('Unstable: center of pressure is ahead of center of mass by ' + (-margin0).toFixed(2) + ' calibers.' + (gimbaled ? ' Gimbal can hold it under power only.' : ' Add fins lower down or move mass up.'));
    else if (margin0 < 1) warnings.push('Stability margin ' + margin0.toFixed(2) + ' calibers is thin. One to two is the usual target.');
    else if (margin0 > 4) warnings.push('Margin ' + margin0.toFixed(1) + ' calibers is very high: it will weathercock hard into the wind.');
    stages.forEach(function (st) {
      if (st.n === 1) { st.margin = margin0; st.marginEmpty = margin1; return; }
      var sub = buildParts(design, rng(1)).parts.filter(function (p) { return p.section <= st.section; });
      if (!sub.length) return;
      var sb = newBody(sub, g), m1p = massProps(sb);
      st.margin = (sb.sCp - m1p.scg) / g.D;
      var gim = sub.some(function (p) { return p.type === 'engine' && p.section === st.section && p.eng.gimbal > 0; });
      var rcs = sub.some(function (p) { return p.type === 'rcs'; });
      if (st.margin < 0 && !gim && !rcs) warnings.push('Stage ' + st.n + ' is unstable on its own (' + st.margin.toFixed(2) + ' calibers) and has no gimbal or RCS.');
    });
    var dvTot = stages.reduce(function (a, s) { return a + s.dvVac; }, 0);
    // terminal descent speed under the largest main chute, with the top stage dry
    var top = parts.filter(function (p) { return p.section === 0; });
    var mTop = top.reduce(function (a, p) { return a + p.dry + (p.type === 'rcs' ? p.prop : 0); }, 0);
    var mains = chutes.filter(function (p) { return p.section === 0 && p.kind === 'main'; });
    var landV = null;
    if (mains.length) {
      var Ac = mains.reduce(function (a, p) { return a + p.cd * Math.PI * p.chuteDia * p.chuteDia / 4; }, 0);
      landV = Math.sqrt(2 * mTop * G0 / (1.225 * Ac));
      if (landV > 9) warnings.push('Main chute lands the top stage at about ' + landV.toFixed(1) + ' m/s. Under 9 counts as a landing. Use a bigger canopy.');
    }
    return { mass: mp.m, length: b.L, scg: mp.scg, scp: b.sCp, margin0: margin0, margin1: margin1, stages: stages, dvTot: dvTot,
      tier: tier, landV: landV, warnings: warnings, blockers: blockers, ok: blockers.length === 0, D: g.D };
  }

  // ---------- simulation instance ----------
  function create() {
    var handlers = {};
    var sim = {
      state: 'idle', t: 0, warp: 1, bodies: [], design: null, opts: null, geo: null,
      input: { nose: 0, tail: 0 }, summary: null, gust: 0, mode: 'auto', guide: null
    };
    var rand = rng(7), gustOU = 0, gustT = 0, gustDur = 0, gustAmp = 0, gustNext = 5;
    var stageTimer = -1, maxQ = 0, maxQDone = false, liftoff = false, ended = false;

    function on(name, fn) { (handlers[name] = handlers[name] || []).push(fn); return sim; }
    function off(name, fn) { var h = handlers[name]; if (h) handlers[name] = h.filter(function (f) { return f !== fn; }); return sim; }
    function emit(name, data) { var h = handlers[name]; if (h) h.slice().forEach(function (f) { try { f(data || {}, sim); } catch (e) { if (root.console) console.error(e); } }); }

    function setDesign(design) { sim.design = JSON.parse(JSON.stringify(design)); reset(); }

    function reset() {
      sim.state = 'idle'; sim.t = 0; sim.summary = null; ended = false; liftoff = false;
      stageTimer = -1; maxQ = 0; maxQDone = false; sim.input.nose = 0; sim.input.tail = 0; sim.mode = 'auto'; sim.guide = null;
      sim.geo = null; sim.bodies = [];
      if (sim.design && sim.design.parts.length) {
        var g = buildParts(sim.design, rng(3));
        sim.geo = g;
        var b = newBody(g.parts, g);
        b.isMain = true;
        placeOnPad(b, 0);
        updateDerived(b);
        sim.bodies = [b];
      }
      emit('reset');
    }
    function placeOnPad(b, tilt) {
      var mp = massProps(b);
      b.mp = mp;
      b.phi = tilt;
      var tailToCg = b.L - mp.scg;
      b.X = Math.sin(tilt) * tailToCg; b.Y = RE + Math.cos(tilt) * tailToCg + 0.05;
      b.VX = b.VY = 0; b.om = 0; b.onRail = true; b.railDist = 0; b.railV = 0;
    }
    function updateDerived(b) {
      b.lam = Math.atan2(b.X, b.Y);
      b.y = Math.hypot(b.X, b.Y) - RE;
      b.x = RE * b.lam;
      b.th = wrapPi(b.phi - b.lam);
    }

    function launch(opts) {
      opts = opts || {};
      sim.opts = {
        tilt: (opts.tilt || 0) * Math.PI / 180, wind: opts.wind || 0, spin: (opts.spin || 0) * 2 * Math.PI,
        autoStab: opts.autoStab !== false, autoStage: opts.autoStage !== false, autoChute: opts.autoChute !== false,
        mainAlt: +opts.mainAlt || 0, orbit: (+opts.orbit || 0) * 1000, gLimit: +opts.gLimit || 4,
        breakMode: !!opts.breakMode, seed: opts.seed || ((Math.random() * 1e9) | 0)
      };
      if (!sim.design || !sim.design.parts.length) return false;
      var seed = sim.opts.seed;
      rand = rng(seed);
      var g = buildParts(sim.design, rand);
      sim.geo = g;
      var b = newBody(g.parts, g);
      b.isMain = true;
      placeOnPad(b, sim.opts.tilt);
      updateDerived(b);
      sim.bodies = [b];
      sim.t = 0; ended = false; liftoff = false; stageTimer = -1; maxQ = 0; maxQDone = false;
      sim.mode = sim.opts.autoStab ? 'auto' : 'off';
      sim.guide = sim.opts.orbit > 0 ? { phase: 'ascent', target: sim.opts.orbit, kick: +opts.kick || 0, turnAlt: (+opts.turnAlt || 0) * 1000 } : null;
      if (sim.guide) { var a0 = analyze(sim.design), tw = a0.stages[0].twrSL; if (!sim.guide.kick) sim.guide.kick = 1 + 4 * Math.max(0, tw - 1.1); if (!sim.guide.turnAlt) sim.guide.turnAlt = clamp(35000 + 40000 * (tw - 1.2), 30000, 90000); }
      gustOU = 0; gustT = 0; gustNext = 3 + rand() * 6;
      sim.summary = { apogee: 0, maxV: 0, maxMach: 0, maxQ: 0, peakG: 0, peakLoad: 0, peakHeat: 0, peakTemp: 290, railExitV: 0, lostControl: false,
        result: 'flying', landV: 0, landX: 0, time: 0, stages: 0, chuteRipped: false, chutesUsed: 0, seed: seed,
        payload: (b.parts[0] && b.parts[0].payload) || 0, wall: sim.design.wall, tilt: opts.tilt || 0, wind: sim.opts.wind, breakMode: sim.opts.breakMode,
        orbit: false, bestPe: -RE, deorbited: false, tier: analyze(sim.design).tier };
      sim.state = 'flying';
      ignite(b, 0);
      emit('launch', { seed: seed });
      return true;
    }

    function lowestSection(b) { var m = -1; b.parts.forEach(function (p) { if (p.section > m) m = p.section; }); return m; }
    function ignite(b, delay, liquidOnly) {
      var sec = lowestSection(b), any = false;
      b.parts.forEach(function (p) {
        if (p.type !== 'engine' || p.section !== sec || p.firing || p.burnedOut) return;
        if (liquidOnly && p.eng.kind === 'solid') return;
        if (!engineHasProp(b, p, sim.geo.sections)) return;
        p.firing = true; p.igniteT = delay; p.used = true; any = true;
      });
      if (any) emit('ignition', { body: b.id, section: sec, delay: delay });
      return any;
    }
    function shutdown(b) {
      var any = false;
      b.parts.forEach(function (p) { if (p.type === 'engine' && p.firing && p.eng.kind !== 'solid') { p.firing = false; any = true; } });
      if (any) emit('shutdown', { body: b.id });
      return any;
    }
    function liquidFiring(b) { return b.parts.some(function (p) { return p.type === 'engine' && p.firing && p.eng.kind !== 'solid'; }); }

    function splitBody(b, k, dvSep) {
      var mp = massProps(b);
      var ux = Math.sin(b.phi), uy = Math.cos(b.phi);
      var tipX = b.X + mp.scg * ux, tipY = b.Y + mp.scg * uy;
      var sK = b.parts[k].sTop;
      var up = newBody(b.parts.slice(0, k), sim.geo), lo = newBody(b.parts.slice(k), sim.geo);
      [up, lo].forEach(function (nb, i) {
        var tX = i === 0 ? tipX : tipX - sK * ux, tY = i === 0 ? tipY : tipY - sK * uy;
        var m2 = massProps(nb); nb.mp = m2;
        nb.X = tX - m2.scg * ux; nb.Y = tY - m2.scg * uy;
        nb.VX = b.VX; nb.VY = b.VY; nb.phi = b.phi; nb.om = b.om; nb.spin = b.spin; nb.roll = b.roll;
        nb.maxY = b.maxY; nb.rising = b.rising; nb.pastApogee = b.pastApogee; nb.apoT = b.apoT;
        nb.traj = i === 0 ? b.traj : b.traj.slice(-2);
        nb.tele = b.tele;
        // carry skin temperatures over by position
        for (var j = 0; j < NST; j++) {
          var s = (i === 0 ? 0 : sK) + (j + 0.5) * nb.ds;
          nb.T[j] = b.T[clamp(Math.floor(s / b.ds), 0, NST - 1)];
        }
        updateDerived(nb);
      });
      var mU = up.mp.m, mL = lo.mp.m, mT = mU + mL;
      up.VX += dvSep * ux * mL / mT; up.VY += dvSep * uy * mL / mT;
      lo.VX -= dvSep * ux * mU / mT; lo.VY -= dvSep * uy * mU / mT;
      up.isMain = b.isMain; up.broken = b.broken; lo.broken = b.broken;
      up.chuteOpen = up.chutes.some(function (p) { return p.open; });
      lo.chuteOpen = lo.chutes.some(function (p) { return p.open; });
      var i0 = sim.bodies.indexOf(b);
      sim.bodies.splice(i0, 1, up, lo);
      return { up: up, lo: lo };
    }

    function stage() {
      var main = mainBody();
      if (!main || !main.alive || main.onRail) return false;
      var k = -1;
      main.parts.forEach(function (p, i) { if (p.type === 'decoupler') k = i; });
      if (k <= 0) return false;
      main.parts.forEach(function (p) { if (p.type === 'engine' && p.section >= main.parts[k].section) p.firing = false; });
      var r = splitBody(main, k, 2.0);
      r.lo.om += (rand() - 0.5) * 0.2;
      sim.summary.stages++;
      emit('stage', { upper: r.up.id, spent: r.lo.id, alt: r.up.y, t: sim.t });
      if (!sim.guide || sim.guide.phase === 'ascent') ignite(r.up, 0.8);
      return true;
    }

    function openChute(b, p) {
      if (!p || p.open || p.ripped || p.cut || b.onRail) return false;
      p.open = true; p.frac = 0; p.t = 0;
      b.chuteOpen = true;
      if (b.isMain && sim.summary) sim.summary.chutesUsed++;
      if (p.kind === 'main') b.chutes.forEach(function (q) { if (q.kind === 'drogue' && q.open) { q.open = false; q.cut = true; emit('drogueCut', { body: b.id }); } });
      emit('chute', { body: b.id, alt: b.y, main: b.isMain, kind: p.kind });
      return true;
    }
    function nextChute(b) {
      var d = b.chutes.filter(function (p) { return p.kind === 'drogue' && !p.open && !p.ripped && !p.cut; })[0];
      var anyDrogueOpen = b.chutes.some(function (p) { return p.kind === 'drogue' && p.open; });
      if (d && !anyDrogueOpen) return d;
      return b.chutes.filter(function (p) { return p.kind === 'main' && !p.open && !p.ripped; })[0] || null;
    }
    function deployChute() {
      var b = mainBody();
      if (!b || !b.alive) return false;
      return openChute(b, nextChute(b));
    }

    function mainBody() { for (var i = 0; i < sim.bodies.length; i++) if (sim.bodies[i].isMain) return sim.bodies[i]; return null; }

    // ----- attitude modes: auto, prograde, retrograde, hold, off -----
    function setMode(m) { sim.mode = m; var b = mainBody(); if (b) b.holdPhi = b.phi; emit('mode', { mode: m }); }
    function toggleEngine() {
      var b = mainBody(); if (!b || !b.alive || b.onRail) return false;
      if (sim.guide) sim.guide.phase = 'manual';
      if (liquidFiring(b)) return shutdown(b);
      return ignite(b, 0.3, true);
    }

    function targetPhi(b, T, thrusting) {
      var lam = b.lam;
      var vu = b.VX * Math.sin(lam) + b.VY * Math.cos(lam), ve = b.VX * Math.cos(lam) - b.VY * Math.sin(lam);
      var gamI = Math.atan2(ve, vu);
      var vabs = Math.hypot(b.VX, b.VY);
      var mode = sim.mode;
      if (mode === 'off') return null;
      if (mode === 'hold') return b.holdPhi != null ? b.holdPhi : b.phi;
      if (mode === 'pro') return vabs > 5 ? lam + gamI : b.phi;
      if (mode === 'retro') return vabs > 5 ? lam + gamI + Math.PI : b.phi;
      // auto
      if (b.onRail) return b.phi;
      var G = sim.guide;
      if (G && b.isMain) {
        var alt = b.y, th;
        if (G.phase === 'ascent' || G.phase === 'coast') {
          // vertical climb, a small kick sized by thrust to weight, a gravity turn through the thick air,
          // then a vertical speed steering law in thin air that lands the apoapsis on the target
          var kick = clamp(G.kick, 0.5, 12) * Math.PI / 180;
          if (vabs < 50 || alt < 150) th = sim.opts.tilt;
          else if (vabs < 200) th = Math.max(gamI, sim.opts.tilt + kick * Math.min(1, (vabs - 50) / 80));
          else if (T.q > 3000) th = gamI;
          else {
            var rr0 = RE + alt, gl = MU / (rr0 * rr0), geff = Math.max(0.5, gl - ve * ve / rr0);
            var dh = G.target - alt;
            var vrDes = dh > 0 ? Math.min(G.target > 400000 ? 700 : 1e9, 0.9 * Math.sqrt(2 * geff * dh)) : -Math.min(120, 0.02 * -dh);
            var Fm = G.F && b.mp ? G.F / b.mp.m : 20;
            var areq = gl - ve * ve / rr0 + 0.06 * (vrDes - vu);
            var up = Math.asin(clamp(areq / Math.max(Fm, 0.1), -0.5, 1));
            var lim = 0.06 + 0.29 * clamp((800 - T.q) / 700, 0, 1);
            th = clamp(Math.PI / 2 - up, gamI - lim, gamI + lim);
            if (T.q < 50) th = Math.PI / 2 - up;
          }
          if (G.phase === 'coast') th = gamI;
          return lam + th;
        }
        if (G.phase === 'circ') {
          // hold altitude near the target while thrust builds horizontal speed
          var rc = RE + alt, gc = MU / (rc * rc);
          var vrD = clamp(0.02 * (G.target - alt), -120, 120);
          var Fc = G.F && b.mp ? G.F / b.mp.m : 20;
          var ar = gc - ve * ve / rc + 0.08 * (vrD - vu);
          var upc = Math.asin(clamp(ar / Math.max(Fc, 0.1), -0.6, 1));
          return lam + Math.PI / 2 - upc;
        }
        if (G.phase === 'done' || G.phase === 'manual') return vabs > 5 ? lam + gamI : b.phi;
      }
      var target = thrusting || T.V < 20 ? sim.opts.tilt : T.gam;
      return lam + target;
    }

    function control(b, mp, T, thrusting) {
      var nose = b.isMain ? sim.input.nose : 0, tail = b.isMain ? sim.input.tail : 0;
      var auto = false, Mcmd = 0;
      if (b.isMain && !b.broken && !b.chuteOpen) {
        var tp = targetPhi(b, T, thrusting);
        if (tp != null) {
          auto = true;
          b.phiT = tp;
          var e = clamp(wrapPi(b.phi - tp), -0.12, 0.12);
          var kp = T.q < 100 ? 2 : 6, kdg = T.q < 100 ? 2.5 : 4;
          Mcmd = -mp.Iy * (kp * e + kdg * b.om) - (b.mAero || 0);
        } else b.phiT = null;
      }
      if (sim.guide && b.isMain) sim.guide.F = b.parts.reduce(function (acc, p) { return acc + (p.type === 'engine' && p.firing ? (p.lastF || p.Fvac) : 0); }, 0);
      var Mg = 0;
      b.parts.forEach(function (p) {
        if (p.type !== 'engine') return;
        if (!p.firing || p.igniteT > 0 || p.eng.gimbal <= 0) { p.gimCmd = 0; return; }
        var gmax = p.eng.gimbal * Math.PI / 180;
        var d = mp.scg - (p.sTop + p.len);
        var F = p.lastF || p.Fvac * 0.8;
        var want = 0;
        if (auto && Math.abs(d) > 0.05 && F > 0) want = Math.asin(clamp((Mcmd - Mg) / (d * F), -1, 1));
        want += tail * gmax;
        p.gimCmd = clamp(want, -gmax, gmax);
        Mg += d * F * Math.sin(p.gimCmd);
      });
      var Mrem = Mcmd - Mg;
      var gimbalOn = b.parts.some(function (p) { return p.type === 'engine' && p.firing && p.igniteT <= 0 && p.eng.gimbal > 0; });
      var Frcs = 150 + 600 * b.A;
      b.rcsParts.forEach(function (p) {
        p.cmd = 0;
        if (p.prop <= 0 || b.broken) return;
        var d = mp.scg - partCenter(p);
        if (d > 0 && nose) p.cmd = nose;
        else if (d < 0 && tail) p.cmd = tail;
        else if (auto && !gimbalOn && Math.abs(d) > 0.05 && Math.abs(Mrem) > 0.25 * Math.abs(d) * Frcs) {
          p.cmd = (Mrem * d > 0) ? 1 : -1;
          Mrem -= p.cmd * d * Frcs;
        }
      });
      b.rcsF = Frcs;
      b.canSteer = b.rcsParts.some(function (p) { return p.prop > 0; }) || b.parts.some(function (p) { return p.type === 'engine' && p.firing && p.eng.gimbal > 0; });
    }

    function cdOf(b, M, thrusting) {
      var g = M < 0.8 ? 0 : (M < 1.1 ? (M - 0.8) / 0.3 : Math.sqrt(1.1 / M));
      var c = b.cdFric * (M < 1 ? 1 : Math.pow(M, -0.25));
      c += b.sinD2 * (0.8 + 1.6 * g) + 0.05 * g;
      c += thrusting ? 0.04 : (M < 1 ? 0.12 + 0.13 * M * M : 0.25 / M);
      c += b.cdFins * (1 + 0.6 * g);
      return c;
    }

    // ----- one body, one substep -----
    function stepBody(b, dt, coarse) {
      var lam = Math.atan2(b.X, b.Y), rr = Math.hypot(b.X, b.Y), alt = rr - RE;
      var upx = b.X / rr, upy = b.Y / rr, ex = upy, ey = -upx;
      var atm = atmosphere(alt);
      var mp = massProps(b); b.mp = mp;
      var m = mp.m, scg = mp.scg;
      var ux = Math.sin(b.phi), uy = Math.cos(b.phi), nx = uy, ny = -ux;
      var wmag = windProfile(alt, sim.opts.wind) + sim.gust * Math.exp(-alt / 12000);
      var vax = b.VX - wmag * ex, vay = b.VY - wmag * ey;
      var V = Math.hypot(vax, vay);
      var q = 0.5 * atm.rho * V * V, M = V / atm.a;
      var gam = Math.atan2(vax * ex + vay * ey, vax * upx + vay * upy);
      var Fx = 0, Fy = 0, Mst = 0, Mdist = 0, Mdamp = 0, mThr = 0;
      var pf = b.pf || (b.pf = []); pf.length = 0;
      var thrusting = false, Ftot = 0, sepAny = false, mdTot = 0;

      for (var i = 0; i < b.parts.length; i++) {
        var p = b.parts[i];
        if (p.type !== 'engine' || !p.firing) continue;
        if (p.igniteT > 0) { p.igniteT -= dt; continue; }
        var thr = p.eng.kind === 'solid' ? 1 : (p.throttle || 1);
        var th = engineThrust(p, atm.p), F = th.F * thr, md = p.mdot * thr * dt, frac = 1;
        if (p.eng.kind === 'solid') {
          if (p.prop < 0.05 * p.cap) frac = Math.max(0, p.prop / (0.05 * p.cap));
          p.prop = Math.max(0, p.prop - md * Math.max(frac, 0.2));
          if (p.prop <= 1e-6) { p.prop = 0; p.firing = false; p.burnedOut = true; emit('burnout', { body: b.id, engine: p.eng.key, t: sim.t, alt: alt }); }
        } else {
          frac = drainLiquid(b, p.section, md);
          if (frac < 1) { p.firing = false; p.burnedOut = true; emit('burnout', { body: b.id, engine: p.eng.key, t: sim.t, alt: alt }); }
        }
        F *= frac;
        p.lastF = F; p.sep = th.sep; p.thrustFrac = F / p.Fvac;
        if (F <= 0) continue;
        thrusting = true; Ftot += F; mdTot += p.mdot * thr * frac; if (th.sep) sepAny = true;
        p.gim += clamp((p.gimCmd || 0) - p.gim, -0.35 * dt, 0.35 * dt);
        var fa = F * Math.cos(p.gim), fn = F * Math.sin(p.gim);
        Fx += fa * ux + fn * nx; Fy += fa * uy + fn * ny;
        var sE = p.sTop + p.len;
        Mst += (scg - sE) * fn; mThr += (scg - sE) * fn;
        var dist = F * p.mis;
        if (th.sep) { p.sideOU += (-p.sideOU / 0.15) * dt + Math.sqrt(2 / 0.15 * dt) * gauss(rand); dist += p.Fvac * 0.05 * p.sideOU; }
        Mdist += (scg - sE) * dist;
        pf.push(p.sTop, fa, fn + dist);
      }
      b.thrusting = thrusting;

      var T0 = { V: V, q: q, gam: gam };
      if (!coarse) control(b, mp, T0, thrusting);
      else {
        b.parts.forEach(function (pp) { if (pp.type === 'rcs') pp.cmd = 0; });
        b.phiT = (b.isMain && sim.mode !== 'off' && !b.broken && !b.chuteOpen) ? targetPhi(b, T0, thrusting) : null;
        b.canSteer = b.rcsParts.some(function (pp) { return pp.prop > 0; });
      }

      var alpha = 0, ca = 1, sa = 0;
      if (V > 0.3 && atm.rho > 1e-12) {
        var vhx = vax / V, vhy = vay / V;
        ca = ux * vhx + uy * vhy;
        sa = vhx * uy - vhy * ux;
        alpha = Math.atan2(sa, ca);
        var cd = cdOf(b, M, thrusting);
        var Dg = q * b.A * cd;
        Fx -= Dg * vhx; Fy -= Dg * vhy;
        pf.push(0, -Dg * ca, 0);
        var dn = -Dg * (vhx * nx + vhy * ny);
        pf.push(b.L * 0.5, 0, dn);
        Mst += (scg - b.L * 0.5) * dn;
        var vu = vax * ux + vay * uy;
        var cfx = -(vax - vu * ux), cfy = -(vay - vu * uy);
        var cfl = Math.hypot(cfx, cfy);
        if (cfl > 1e-9) {
          var dir = (cfx * nx + cfy * ny) / cfl;
          var asa = Math.abs(sa), aca = Math.abs(ca);
          for (var li = 0; li < b.lift.length; li++) {
            var l = b.lift[li];
            var Nn = q * b.A * l.cna * asa * aca * dir;
            Fx += Nn * nx; Fy += Nn * ny;
            pf.push(l.s, 0, Nn);
            Mst += (scg - l.s) * Nn;
          }
          var Nc = q * 1.2 * asa * asa * b.D * b.L * dir;
          Fx += Nc * nx; Fy += Nc * ny; pf.push(b.L * 0.5, 0, Nc);
          Mst += (scg - b.L * 0.5) * Nc;
        }
        b.mAero = Mst - mThr;
        var kd = 0;
        for (li = 0; li < b.lift.length; li++) { var dd = b.lift[li].s - scg; kd += q * b.A * b.lift[li].cna * dd * dd / V; }
        Mdamp -= kd * b.om;
      } else b.mAero = 0;
      var tumbleK = 0.5 * atm.rho * 1.2 * b.D * (Math.pow(scg, 4) + Math.pow(b.L - scg, 4)) / 4;
      Mdamp -= tumbleK * b.om * Math.abs(b.om);

      if (b.spin > 0.5) {
        if (b.sCp < scg && q > 1) {
          var Ma = q * b.A * b.cnaTot * (scg - b.sCp);
          var Sg = Math.pow(mp.Ix * b.spin, 2) / (4 * mp.Iy * Ma);
          Mst /= (1 + Sg);
        }
        Mdist /= (1 + Math.pow(b.spin / 3, 2));
        b.spin *= (1 - 0.03 * dt * (0.2 + atm.rho));
      }
      b.roll += b.spin * dt;

      var Frcs = b.rcsF || 0;
      b.rcsParts.forEach(function (rp) {
        if (!rp.cmd || rp.prop <= 0) return;
        var f = rp.cmd * Frcs;
        Fx += f * nx; Fy += f * ny;
        Mst += (scg - partCenter(rp)) * f;
        pf.push(partCenter(rp), 0, f);
        rp.prop = Math.max(0, rp.prop - Frcs / (RCS_ISP * G0) * dt);
      });

      // parachutes: inflate, load, accumulate damage
      var anyOpen = false;
      for (i = 0; i < b.chutes.length; i++) {
        var c = b.chutes[i];
        if (!c.open) { c.load = 0; continue; }
        anyOpen = true;
        c.t += dt;
        var tf = 1.2 + 0.1 * c.chuteDia;
        c.frac = Math.min(1, Math.pow(c.t / tf, 2));
        if (V < 0.3) continue;
        var Ac = Math.PI * c.chuteDia * c.chuteDia / 4 * c.frac;
        var Fc = q * c.cd * Ac;
        c.load = Fc / c.rating;
        if (c.load > 0.65) c.health -= (c.load - 0.65) * 2.5 * dt;
        if (M > c.machMax && c.frac > 0.05) c.health -= (M - c.machMax) * 1.5 * dt;
        if (c.load > 1.6 || c.health <= 0) {
          c.open = false; c.ripped = true;
          if (b.isMain && sim.summary) sim.summary.chuteRipped = true;
          emit('chuteRip', { body: b.id, force: Fc, alt: alt, v: V, main: b.isMain, kind: c.kind });
          continue;
        }
        var cvx = -vax / V, cvy = -vay / V;
        Fx += Fc * cvx; Fy += Fc * cvy;
        var sC = partCenter(c);
        var cn = Fc * (cvx * nx + cvy * ny), cax = Fc * (cvx * ux + cvy * uy);
        Mst += (scg - sC) * cn;
        pf.push(sC, cax, cn);
        Mdamp -= 0.5 * atm.rho * V * c.cd * Ac * Math.pow(Math.max(0, b.L - scg + 4), 2) * b.om * 0.2;
      }
      b.chuteOpen = anyOpen;

      // aerodynamic heating (Sutton-Graves stagnation, scaled along the body) and radiation
      var qS = 0;
      if (V > 30 && atm.rho > 1e-14) {
        var Rn = ca >= 0 ? b.Rn : b.r;
        qS = 1.7415e-4 * Math.sqrt(atm.rho / Rn) * V * V * V;
      }
      var capA = b.aw * CP_WALL * HEAT_MASS, Ta = atm.T, hm = 0, shape = ca >= 0 ? b.hFwd : b.hBwd, aoaF = 1 + 1.5 * Math.abs(sa);
      var Taw = Ta * (1 + 0.18 * M * M);   // adiabatic wall (recovery) temperature, the hottest the skin can get from friction
      for (var j = 0; j < NST; j++) {
        var Tj = b.T[j];
        var qin = Taw > 300 ? qS * shape[j] * aoaF * (Taw - Tj) / (Taw - 300) : 0;
        var qout = EPS_RAD * SB * (Tj * Tj * Tj * Tj - Ta * Ta * Ta * Ta) + 8 * (Tj - Ta) * Math.min(1, atm.rho / 1.225 + 0.02);
        Tj += (qin - qout) / capA * dt;
        if (Tj < 150) Tj = 150;
        b.T[j] = Tj;
        var hr = (Tj - 290) / (b.stTlim[j] - 290);
        b.heat[j] = hr > 0 ? hr : 0;
        if (b.heat[j] > hm) { hm = b.heat[j]; b.heatWorstS = (j + 0.5) * b.ds; }
      }
      b.heatMax = hm; b.qStag = qS; b.Taw = Taw;

      var Fng = { x: Fx, y: Fy };
      var g = MU / (rr * rr);
      Fx -= m * g * upx; Fy -= m * g * upy;
      var Mth = Mst + Mdist + Mdamp;

      if (b.onRail) {
        var aAlong = (Fx * ux + Fy * uy) / m;
        if (b.railV <= 0 && aAlong <= 0) { b.railV = 0; }
        else {
          if (!liftoff) { liftoff = true; emit('liftoff', { t: sim.t }); }
          b.railV = Math.max(0, b.railV + aAlong * dt);
          b.railDist += b.railV * dt;
          b.X += b.railV * ux * dt; b.Y += b.railV * uy * dt;
          b.VX = b.railV * ux; b.VY = b.railV * uy;
        }
        b.om = 0;
        if (b.railDist >= RAIL_LEN) {
          b.onRail = false;
          b.spin = sim.opts.spin;
          sim.summary.railExitV = b.railV;
          emit('railExit', { v: b.railV, t: sim.t });
        }
      } else {
        b.VX += Fx / m * dt; b.VY += Fy / m * dt;
        b.X += b.VX * dt; b.Y += b.VY * dt;
        if (coarse) {
          if (b.isMain && b.phiT != null && b.canSteer && !b.chuteOpen) { b.phi = b.phiT; b.om = 0; }
          else b.phi = wrapPi(b.phi + b.om * dt);
        } else {
          b.om += Mth / mp.Iy * dt;
          b.om = clamp(b.om, -40, 40);
          b.phi = wrapPi(b.phi + b.om * dt);
        }
      }
      updateDerived(b);

      if (b.isMain && sim.opts.autoStab) {
        var gNow = Math.hypot(Fng.x, Fng.y) / m / G0, gl = sim.opts.gLimit;
        b.parts.forEach(function (pp) {
          if (pp.type !== 'engine' || pp.eng.kind === 'solid' || !pp.firing) return;
          var tr = pp.throttle || 1;
          if (gNow > gl) tr *= Math.max(0.9, gl / gNow); else if (gNow < gl * 0.97) tr = Math.min(1, tr * 1.01);
          pp.throttle = Math.max(0.4, tr);
        });
      }
      b.lastF = { ax: (Fng.x * ux + Fng.y * uy) / m, an: (Fng.x * nx + Fng.y * ny) / m, thdd: b.onRail || coarse ? 0 : Mth / mp.Iy };
      var vabs = Math.hypot(b.VX, b.VY);
      b.tele = { alt: b.y, V: V, vabs: vabs, vr: b.VX * upx + b.VY * upy, vh: b.VX * ex + b.VY * ey, M: M, q: q, rho: atm.rho, p: atm.p, Tair: atm.T,
        alpha: alpha, gam: gam, thrust: Ftot, isp: mdTot > 0 ? Ftot / (mdTot * G0) : 0, sep: sepAny, m: m, scg: scg,
        margin: (b.sCp - scg) / b.D, gNG: Math.hypot(Fng.x, Fng.y) / m / G0, wind: wmag, qStag: qS, vax: vax, vay: vay };
      return b;
    }

    // ----- structural loads (beam statics with d'Alembert inertia), strength derated by temperature -----
    function computeLoads(b) {
      var lf = b.lastF, mp = b.mp;
      if (!lf || !mp) return 0;
      var ds = b.ds, n = NST;
      var ms = b.msBuf || (b.msBuf = new Float32Array(n));
      var fa = b.faBuf || (b.faBuf = new Float32Array(n));
      var fn = b.fnBuf || (b.fnBuf = new Float32Array(n));
      ms.set(b.stDry); fa.fill(0); fn.fill(0);
      b.parts.forEach(function (p) {
        if (p.prop <= 0) return;
        if (p.type === 'tank') { var f = p.cap > 0 ? p.prop / p.cap : 0, h = p.usable * f; spread(ms, p.sTop + p.len - 0.12 - h, Math.max(h, 0.01), p.prop, ds); }
        else spread(ms, p.sTop, p.len, p.prop, ds);
      });
      var pf = b.pf || [];
      for (var i = 0; i < pf.length; i += 3) {
        var j = clamp(Math.floor(pf[i] / ds), 0, n - 1);
        fa[j] += pf[i + 1]; fn[j] += pf[i + 2];
      }
      var t = b.wall, r = b.r;
      var Pcap = SIGMA_ALLOW * 2 * Math.PI * r * t, Mcap = SIGMA_ALLOW * Math.PI * r * r * t;
      var P = 0, Vs = 0, Mb = 0, sPrev = 0, worst = 0, worstJ = 0, om2 = b.om * b.om;
      for (j = 0; j < n; j++) {
        var sj = (j + 0.5) * ds, d = mp.scg - sj;
        Mb += Vs * (sj - sPrev); sPrev = sj;
        P += fa[j] - ms[j] * (lf.ax - om2 * d);
        Vs += fn[j] - ms[j] * (lf.an + lf.thdd * d);
        var sCut = (j + 1) * ds;
        var Mcut = Mb + Vs * (sCut - sj);
        var C = -P;
        var jj = Math.min(j + 1, n - 1);
        var part = b.parts[b.stPart[jj]] || b.parts[0];
        var derate = clamp(1 - (b.T[jj] - 400) / 600, 0.15, 1);
        var k = part.strength * (part.type === 'tank' && part.prop > 0 ? 1.6 : 1) * derate;
        var ratio = (C > 0 ? C / Pcap : -C / (2 * Pcap)) / k + Math.abs(Mcut) / (Mcap * k);
        if (j === n - 1) ratio *= 0.2;
        b.loads[j] = ratio;
        if (ratio > worst) { worst = ratio; worstJ = j; }
      }
      b.loadMax = worst; b.loadWorstS = (worstJ + 1) * ds;
      return worst;
    }

    function breakup(b, why, sAt) {
      var s = sAt, best = -1, bd = 1e9;
      for (var k = 1; k < b.parts.length; k++) {
        var p = b.parts[k];
        if (p.type === 'fins' || p.len <= 0) continue;
        var d = Math.abs(p.sTop - s);
        if (d < bd) { bd = d; best = k; }
      }
      b.parts.forEach(function (p) { if (p.type === 'engine') p.firing = false; });
      b.broken = true;
      emit('breakup', { body: b.id, alt: b.y, load: why === 'heat' ? b.heatMax : b.loadMax, why: why, s: s, t: sim.t, main: b.isMain });
      if (best > 0) {
        var r = splitBody(b, best, 0.5);
        r.up.om += (rand() - 0.5) * 3; r.lo.om += (rand() - 0.5) * 3;
        r.up.broken = r.lo.broken = true;
      }
      if (sim.summary && b.isMain) { sim.summary.result = why === 'heat' ? 'burned up' : 'broke up'; if (!sim.summary.breakT) sim.summary.breakT = sim.t; }
    }

    function land(b) {
      var v = Math.hypot(b.VX, b.VY);
      b.alive = false; b.landV = v;
      var rr = Math.hypot(b.X, b.Y); b.X *= RE / rr; b.Y *= RE / rr; updateDerived(b);
      b.parts.forEach(function (p) { if (p.type === 'engine') p.firing = false; if (p.type === 'rcs') p.cmd = 0; });
      b.outcome = b.broken ? 'debris' : (v < 9 ? 'landed' : (v < 20 ? 'hard landing' : 'crashed'));
      emit(b.outcome === 'landed' ? 'landed' : 'impact', { body: b.id, v: v, x: b.x, main: b.isMain, outcome: b.outcome });
      if (b.isMain && sim.summary) {
        sim.summary.landV = v; sim.summary.landX = b.x;
        if (sim.summary.result === 'flying' || sim.summary.result === 'in orbit') sim.summary.result = b.outcome;
      }
    }

    function updateGust(dt) {
      var w0 = sim.opts.wind;
      var sig = 0.25 * w0 + (w0 > 0 ? 0.8 : 0);
      gustOU += (-gustOU / 2) * dt + sig * Math.sqrt(dt) * gauss(rand);
      gustNext -= dt;
      var disc = 0;
      if (gustNext <= 0 && gustDur <= 0 && w0 > 0) {
        gustDur = 1.5 + rand() * 1.5; gustT = 0; gustAmp = (rand() < 0.5 ? -1 : 1) * (0.6 * w0 + 2) * (0.5 + rand() * 0.5);
        emit('gust', { amp: gustAmp });
      }
      if (gustDur > 0) {
        gustT += dt;
        disc = gustAmp * 0.5 * (1 - Math.cos(2 * Math.PI * gustT / gustDur));
        if (gustT >= gustDur) { gustDur = 0; gustNext = 4 + rand() * 7; }
      }
      sim.gust = gustOU + disc;
    }

    function guidance(main) {
      var G = sim.guide;
      if (!G || sim.mode !== 'auto' || !main.alive || main.broken) return;
      var o = orbitOf(main.X, main.Y, main.VX, main.VY);
      if (!liftoff || main.onRail) return;
      var sec = lowestSection(main);
      var left = main.parts.some(function (p) { return p.type === 'engine' && p.section === sec && !p.burnedOut && engineHasProp(main, p, sim.geo.sections); });
      var anyFiring = main.parts.some(function (p) { return p.type === 'engine' && p.firing; });
      var err = main.phiT != null ? Math.abs(wrapPi(main.phi - main.phiT)) : 0;
      if (G.phase === 'ascent') {
        if (!anyFiring && left && stageTimer < 0) ignite(main, 0.3, true);
        // high targets: cut off once the apoapsis is high enough, coast up, circularize there
        if (G.target > 400000 && o.ap >= G.target && main.y < G.target - 60000 && o.pe < G.target - 20000 && main.y > 80000) {
          if (shutdown(main)) emit('meco', { ap: o.ap });
          G.phase = 'coast';
        }
      } else if (G.phase === 'coast') {
        var F = 0, m = main.mp ? main.mp.m : 1;
        main.parts.forEach(function (p) { if (p.type === 'engine' && p.eng.kind !== 'solid' && !p.burnedOut && p.section === sec) F += p.Fvac; });
        var dv = Math.max(0, o.vCircAp - o.vAp), burn = F > 0 ? dv * m / F : 0;
        if (o.tAp < burn * 0.5 + 5 || o.vr < 0) { G.phase = 'circ'; emit('circularize', { dv: dv, burn: burn }); if (F === 0) stageIfSpent(main); }
      } else if (G.phase === 'circ') {
        if (anyFiring && err > 0.35) shutdown(main);
        if (!anyFiring && left && err < 0.12 && stageTimer < 0) ignite(main, 0.3, true);
      }
      if ((G.phase === 'ascent' || G.phase === 'circ') && (o.pe >= G.target - 20000 || (o.pe > 140000 && o.ap > G.target * 1.5) || o.esc)) {
        shutdown(main); G.phase = 'done';
        emit('orbit', { ap: o.ap, pe: o.pe });
      }
    }
    function stageIfSpent(main) {
      var hasDec = main.parts.some(function (p) { return p.type === 'decoupler'; });
      if (hasDec) stage();
    }

    function chooseDt() {
      var h = 0.5;
      for (var i = 0; i < sim.bodies.length; i++) {
        var b = sim.bodies[i];
        if (!b.alive) continue;
        var c;
        if (b.onRail || b.thrusting || b.chutes.some(function (p) { return p.open && p.frac < 1; })) c = DT;
        else if (b.y > 140000) c = 0.5;
        else if (b.y > 90000) c = 0.02;
        else if (b.tele && b.tele.q < 5000) c = 0.005;
        else c = DT;
        if (b.rcsParts.some(function (p) { return p.cmd; }) || sim.input.nose || sim.input.tail) c = Math.min(c, DT);
        if (c < h) h = c;
      }
      return h;
    }

    function substep(dt) {
      updateGust(dt);
      var coarse = dt > 0.01;
      for (var i = 0; i < sim.bodies.length; i++) {
        var b = sim.bodies[i];
        if (!b.alive) continue;
        stepBody(b, dt, coarse);
        if (b.tele.vr > 0.5 && !b.onRail) b.rising = true;
        if (b.y > b.maxY) b.maxY = b.y;
        if (b.rising && !b.pastApogee && b.tele.vr <= 0) {
          b.pastApogee = true;
          emit('apogee', { body: b.id, alt: b.maxY, main: b.isMain, t: sim.t, x: b.x });
        }
        if (b.pastApogee) b.apoT += dt;
        if (sim.opts.autoChute && b.pastApogee && !b.broken && b.y < 100000) {
          b.chutes.forEach(function (c) {
            if (c.open || c.ripped || c.cut) return;
            var Mn = b.tele.M || 0;
            var fullLoad = (b.tele.q || 0) * c.cd * Math.PI * c.chuteDia * c.chuteDia / 4 / c.rating;
            if (c.kind === 'drogue' && b.apoT > 0.6 && Mn < c.machMax * 0.9 && (fullLoad < 1.2 || b.y < 1500)) openChute(b, c);
            if (c.kind === 'main') {
              var low = sim.opts.mainAlt > 0;
              var ok = Mn < c.machMax * 0.8;
              if (ok && ((!low && b.apoT > 0.6) || (low && b.y < sim.opts.mainAlt) || (low && b.maxY < sim.opts.mainAlt && b.apoT > 0.6))) openChute(b, c);
            }
          });
        }
        if (!b.onRail && b.y <= 0 && b.tele.vr < 0) land(b);
      }
      var main = mainBody();
      if (main && main.alive) {
        var T = main.tele, s = sim.summary;
        if (main.maxY > s.apogee) s.apogee = main.maxY;
        if (T.V > s.maxV && !main.onRail) s.maxV = T.V;
        if (T.M > s.maxMach && T.rho > 1e-6) s.maxMach = T.M;
        if (T.q > s.maxQ) s.maxQ = T.q;
        if (T.gNG > s.peakG && main.thrusting) s.peakG = T.gNG;
        if (main.heatMax > s.peakHeat) s.peakHeat = main.heatMax;
        if (!maxQDone && liftoff && T.q > 500 && T.q < maxQ * 0.97) { maxQDone = true; emit('maxq', { q: maxQ, alt: main.y, t: sim.t }); }
        if (T.q > maxQ) maxQ = T.q;
        if (Math.abs(T.alpha) > 0.8 && T.q > 800 && !main.chuteOpen && !main.pastApogee) {
          if (!s.lostControl) emit('tumble', { alt: main.y, t: sim.t });
          s.lostControl = true;
        }
        var o = orbitOf(main.X, main.Y, main.VX, main.VY);
        main.orbit = o;
        if (o.pe > 140000 && !o.esc) {
          if (!s.orbit) { s.orbit = true; emit('inOrbit', { ap: o.ap, pe: o.pe }); }
          if (o.pe > s.bestPe) s.bestPe = o.pe;
        }
        if (s.orbit && o.pe < 60000 && !s.deorbited) { s.deorbited = true; emit('deorbit', { pe: o.pe }); }
        guidance(main);
        var sec = lowestSection(main), hasDec = main.parts.some(function (p) { return p.type === 'decoupler'; });
        var firing = main.parts.some(function (p) { return p.type === 'engine' && p.firing; });
        var engs = main.parts.filter(function (p) { return p.type === 'engine' && p.section === sec; });
        var spentAll = engs.every(function (p) { return p.burnedOut || !engineHasProp(main, p, sim.geo.sections); });
        if (sim.opts.autoStage && hasDec && liftoff && !main.onRail && !firing && spentAll && stageTimer < 0) stageTimer = 0.5;
      }
      if (stageTimer >= 0) { stageTimer -= dt; if (stageTimer < 0) { stage(); stageTimer = -1; } }
    }

    function step(realDt) {
      if (sim.state !== 'flying') return;
      var simDt = Math.min(0.1, realDt) * sim.warp;
      var done = 0, guard = 0;
      while (done < simDt - 1e-9 && guard++ < 2500) {
        var h = Math.min(chooseDt(), simDt - done);
        substep(h);
        sim.t += h; done += h;
        if (ended) break;
        if ((guard & 7) === 0 || h > 0.01) checkLoads();
      }
      checkLoads();
      sim.bodies.forEach(function (b) {
        if (!b.alive) return;
        if (sim.t - b.trajT >= 0.5 || b.trajT < 0) { b.traj.push(b.X, b.Y); b.trajT = sim.t; if (b.traj.length > 16000) b.traj.splice(0, 2); }
      });
      checkEnd();
    }
    function checkLoads() {
      for (var i = 0; i < sim.bodies.length; i++) {
        var b = sim.bodies[i];
        if (!b.alive || (b.onRail && !liftoff)) continue;
        var w = computeLoads(b);
        if (b.isMain && sim.summary && w > sim.summary.peakLoad) sim.summary.peakLoad = w;
        if (b.isMain && sim.summary) sim.summary.peakTemp = Math.max(sim.summary.peakTemp, maxOf(b.T));
        if (sim.opts.breakMode && !b.broken) {
          if (b.heatMax > 1) { breakup(b, 'heat', b.heatWorstS); break; }
          if (w > 1) { breakup(b, 'load', b.loadWorstS); break; }
        }
      }
    }
    function maxOf(a) { var m = 0; for (var i = 0; i < a.length; i++) if (a[i] > m) m = a[i]; return m; }
    function endFlight(reason) {
      if (ended || !sim.summary) return;
      ended = true;
      var s = sim.summary, main = mainBody();
      if (s.result === 'flying') {
        if (main && main.alive && main.orbit && main.orbit.pe > 140000) s.result = 'in orbit';
        else s.result = reason || 'ended';
      }
      s.time = sim.t;
      sim.state = 'done';
      emit('end', s);
    }
    function checkEnd() {
      if (ended) return;
      var main = mainBody(), s = sim.summary;
      var padFail = !liftoff && sim.t > 2 && !sim.bodies[0].parts.some(function (p) { return p.type === 'engine' && p.firing; });
      var mainDown = main && !main.alive;
      var brokenDone = (s.result === 'broke up' || s.result === 'burned up') && (sim.bodies.every(function (b) { return !b.alive; }) || sim.t - (s.breakT || 0) > 600);
      if (padFail) { s.result = 'stuck on pad'; endFlight(); return; }
      if (mainDown || brokenDone) { endFlight(); return; }
      if (sim.t > 6 * 3600) endFlight('timeout');
    }

    function setWarp(w) { sim.warp = clamp(w, 0.25, 2000); }

    sim.on = on; sim.off = off; sim.setDesign = setDesign; sim.reset = reset; sim.launch = launch;
    sim.step = step; sim.stage = stage; sim.deployChute = deployChute; sim.setWarp = setWarp;
    sim.setMode = setMode; sim.toggleEngine = toggleEngine; sim.endFlight = function () { endFlight('ended'); };
    sim.mainBody = mainBody;
    sim.analyze = function () { return sim.design ? analyze(sim.design) : null; };
    return sim;
  }

  root.RocketSim = { create: create, analyze: analyze, ENGINES: ENGINES, PACK: PACK, atmosphere: atmosphere, orbitOf: orbitOf,
    NST: NST, RE: RE, MU: MU, engineThrust: engineThrust };
  if (typeof module !== 'undefined' && module.exports) module.exports = root.RocketSim;
})(typeof window !== 'undefined' ? window : globalThis);
