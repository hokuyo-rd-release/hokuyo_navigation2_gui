// 通過モード設定 (pass_through_config.html)
//
// obstacle_monitor の通過モードのパラメータを、ロボットのモデルの上に不感帯・検出帯を描きながら編集する。
//  - 座標は ROS と同じ (x = 前, y = 左, z = 上)。原点はロボット (base_link) 直下の床面。
//  - 経路はロボットの真ん中を前方へまっすぐ通るものとして描く。
//  - 帯の端の ● をドラッグするか、右のパネルのスライダーで値を変える。
//  - 帯の形は obstacle_monitor.cpp と同じ計算で決める:
//      不感帯   : 中心線から 左 path_clearance × 左割合 / 右 path_clearance × 右割合、
//                 プランの後方 1 m 〜 前方 pass_through_lookahead
//      検出帯   : 前方 stop_distance × 左右 path_clearance のうち、不感帯を除いた部分
//      costmap  : 不感帯の外側 pass_through_costmap_margin までは local_costmap に入れない
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

const PATH_BACK = 1.0;      // obstacle_monitor の pathAround(…, 1.0, lookahead) と同じ
const PERSON_RADIUS = 0.2;  // 試しに置く人 (直径 0.4 m)
const PERSON_HEIGHT = 1.7;

// パネルに並べる項目
const FIELDS = {
    deadband: [
        { key: 'pass_through_deadband_left_ratio', name: '左側の割合', step: 0.01,
          derived: v => `= ${(v * state.values.path_clearance).toFixed(2)} m` },
        { key: 'pass_through_deadband_right_ratio', name: '右側の割合', step: 0.01,
          derived: v => `= ${(v * state.values.path_clearance).toFixed(2)} m` },
        { key: 'pass_through_lookahead', name: '前方の長さ [m]', step: 0.1, sliderMax: 12 },
    ],
    detect: [
        { key: 'path_clearance', name: '左右の幅 [m]', step: 0.01, sliderMax: 1.0, common: true },
        { key: 'stop_distance', name: '前方の長さ [m]', step: 0.1, sliderMax: 6.0, common: true },
        { key: 'pass_through_min_blocking_cells', name: '停止するセル数 (通過モード中)', step: 1, sliderMax: 60 },
    ],
    other: [
        { key: 'pass_through_costmap_margin', name: 'costmap に入れない幅 [m]', step: 0.01, sliderMax: 1.0 },
        { key: 'pass_through_timeout', name: '指令途絶で終了するまで [s]', step: 0.1, sliderMax: 5.0 },
    ],
};

const state = {
    values: {},      // 編集中の値
    saved: {},       // ファイルに保存されている値
    live: null,      // 実行中の obstacle_monitor の値 (動いていなければ null)
    spec: {},
    reference: {},
    sensorOrigin: null,
    person: new THREE.Vector2(2.0, 0.25),
};

// ---------------------------------------------------------------- 色 (tokens.css)
const css = getComputedStyle(document.documentElement);
const tokenColor = name => new THREE.Color(css.getPropertyValue(name).trim() || '#ffffff');
const COLORS = {
    deadband: tokenColor('--pt-deadband'),
    detect: tokenColor('--pt-detect'),
    detectStop: tokenColor('--pt-detect-stop'),
    margin: tokenColor('--pt-margin'),
    path: tokenColor('--pt-path'),
    footprint: tokenColor('--pt-footprint'),
    robot: tokenColor('--pt-robot'),
    sensor: tokenColor('--pt-sensor'),
    person: tokenColor('--pt-person'),
    cellIgnored: tokenColor('--pt-cell-ignored'),
    cellDetect: tokenColor('--pt-cell-detect'),
    cellCostmap: tokenColor('--pt-cell-costmap'),
    gridMajor: tokenColor('--pt-grid-major'),
    gridMinor: tokenColor('--pt-grid-minor'),
    background: tokenColor('--viewer-bg'),
};

// ---------------------------------------------------------------- 3D シーン
const container = document.getElementById('pt-viewer');
const scene = new THREE.Scene();
scene.background = COLORS.background;
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 200);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);
const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
container.appendChild(labelRenderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.15;

scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(-3, -4, 6);
scene.add(sun);

// 床のグリッド (1 m と 0.1 m = obstacle_monitor のセル)。GridHelper は XZ 面なので XY 面へ回す
function addGrid(size, divisions, color, opacity, z) {
    const g = new THREE.GridHelper(size, divisions, color, color);
    g.rotation.x = Math.PI / 2;
    g.position.z = z;
    g.material.transparent = true;
    g.material.opacity = opacity;
    g.material.depthWrite = false;
    scene.add(g);
    return g;
}
addGrid(40, 40, COLORS.gridMajor, 0.8, 0.0);
const fineGrid = addGrid(8, 80, COLORS.gridMinor, 0.6, 0.0005);
fineGrid.position.x = 2.0;

// 進行方向の矢印
const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.3, 0, 0.01), 0.6,
                                    0xff4444, 0.15, 0.1);
scene.add(arrow);

const zoneGroup = new THREE.Group();     // 帯 (値が変わるたびに作り直す)
const handleGroup = new THREE.Group();   // ドラッグできる ●
const robotGroup = new THREE.Group();    // ロボットのモデル・外形・センサ
const personGroup = new THREE.Group();   // 試しに置く人とセル
scene.add(zoneGroup, handleGroup, robotGroup, personGroup);

function setView(name) {
    const sd = state.values.stop_distance || 2.0;
    if (name === 'top') {
        camera.position.set(sd * 0.6, -0.001, 7.5);
        controls.target.set(sd * 0.6, 0, 0);
    } else if (name === 'behind') {
        camera.position.set(-2.5, 0, 1.8);
        controls.target.set(2.0, 0, 0.3);
    } else {
        camera.position.set(-2.2, -3.4, 3.2);
        controls.target.set(1.4, 0, 0.2);
    }
    controls.update();
}

function resize() {
    const w = container.clientWidth;
    const h = container.clientHeight;
    renderer.setSize(w, h);
    labelRenderer.setSize(w, h);
    camera.aspect = w / Math.max(1, h);
    camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(container);

// ---------------------------------------------------------------- 描画部品
function disposeGroup(group) {
    for (const obj of [...group.children]) {
        group.remove(obj);
        obj.traverse(o => {
            if (o.geometry) o.geometry.dispose();
            if (o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => m.dispose());
            if (o.isCSS2DObject) o.element.remove();
        });
    }
}

// 床に置く長方形 (x0〜x1, y0〜y1) と、その輪郭
function addRect(group, x0, x1, y0, y1, z, color, opacity) {
    if (x1 - x0 <= 1e-6 || y1 - y0 <= 1e-6) return;
    const geo = new THREE.PlaneGeometry(x1 - x0, y1 - y0);
    const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false,
                                              side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set((x0 + x1) / 2, (y0 + y1) / 2, z);
    group.add(mesh);
    const edge = new THREE.LineSegments(new THREE.EdgesGeometry(geo),
                                        new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.9 }));
    edge.position.copy(mesh.position);
    group.add(edge);
}

function addLine(group, points, color, dashed = false) {
    const geo = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(...p)));
    const mat = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 0.08, gapSize: 0.05 })
                       : new THREE.LineBasicMaterial({ color });
    const line = new THREE.Line(geo, mat);
    if (dashed) line.computeLineDistances();
    group.add(line);
}

function addLabel(group, text, x, y, z) {
    const el = document.createElement('div');
    el.className = 'pt-label';
    el.textContent = text;
    const label = new CSS2DObject(el);
    label.position.set(x, y, z);
    group.add(label);
}

// 今の値から決まる帯の寸法
function geometry() {
    const v = state.values;
    const pc = v.path_clearance;
    const dl = pc * v.pass_through_deadband_left_ratio;
    const dr = pc * v.pass_through_deadband_right_ratio;
    return {
        pc, dl, dr,
        m: v.pass_through_costmap_margin,
        sd: v.stop_distance,
        la: v.pass_through_lookahead,
        // 検出帯 (横方向 lo〜hi、左が正)。不感帯に覆われた側は幅 0
        detect: [[Math.min(dl, pc), pc], [-pc, -Math.min(dr, pc)]],
    };
}

// ---------------------------------------------------------------- 帯とハンドル
const handles = [];   // { mesh, apply(x, y) }

function addHandle(x, y, color, apply, label) {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.07, 20, 14),
                                new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35 }));
    mesh.position.set(x, y, 0.07);
    handleGroup.add(mesh);
    handles.push({ mesh, apply });
    if (label) addLabel(handleGroup, label, x, y, 0.28);
}

function rebuildZones() {
    disposeGroup(zoneGroup);
    disposeGroup(handleGroup);
    handles.length = 0;
    const g = geometry();
    const stop = personVerdict().stop;
    const detectColor = stop ? COLORS.detectStop : COLORS.detect;
    const xb = -PATH_BACK;

    // 不感帯 (後方 1 m 〜 前方 lookahead)
    addRect(zoneGroup, xb, g.la, -g.dr, g.dl, 0.004, COLORS.deadband, 0.28);
    // costmap に入れない範囲 (不感帯の外側 margin)
    addRect(zoneGroup, xb, g.la, g.dl, g.dl + g.m, 0.003, COLORS.margin, 0.18);
    addRect(zoneGroup, xb, g.la, -g.dr - g.m, -g.dr, 0.003, COLORS.margin, 0.18);
    // 検出帯 (前方 stop_distance)
    for (const [lo, hi] of g.detect) {
        addRect(zoneGroup, 0, g.sd, lo, hi, 0.006, detectColor, 0.35);
    }
    // path_clearance の範囲 (点線)
    for (const y of [g.pc, -g.pc]) {
        addLine(zoneGroup, [[0, y, 0.008], [g.sd, y, 0.008]], detectColor, true);
    }
    // 経路の中心線
    addLine(zoneGroup, [[xb - 0.5, 0, 0.01], [Math.max(g.la, g.sd) + 1.0, 0, 0.01]], COLORS.path, true);

    // ドラッグできる ● (帯の端)。x 位置は重ならないようにずらす
    const xDead = Math.min(g.la, Math.max(2.6, g.sd + 0.4));
    const xMargin = Math.min(g.la, xDead + 0.8);
    const xClear = Math.max(0.3, g.sd * 0.5);
    const v = state.values;
    addHandle(xDead, g.dl, COLORS.deadband, (x, y) => setValue('pass_through_deadband_left_ratio', y / v.path_clearance),
              `不感帯 左 ${g.dl.toFixed(2)} m`);
    addHandle(xDead, -g.dr, COLORS.deadband, (x, y) => setValue('pass_through_deadband_right_ratio', -y / v.path_clearance),
              `不感帯 右 ${g.dr.toFixed(2)} m`);
    addHandle(xMargin, g.dl + g.m, COLORS.margin, (x, y) => setValue('pass_through_costmap_margin', y - geometry().dl),
              `costmap 除外 左 ${(g.dl + g.m).toFixed(2)} m`);
    addHandle(xMargin, -g.dr - g.m, COLORS.margin, (x, y) => setValue('pass_through_costmap_margin', -y - geometry().dr),
              `右 ${(g.dr + g.m).toFixed(2)} m`);
    addHandle(xClear, -g.pc, COLORS.detect, (x, y) => setValue('path_clearance', Math.abs(y)),
              `検出 左右 ${g.pc.toFixed(2)} m`);
    addHandle(g.sd, 0, COLORS.detect, (x) => setValue('stop_distance', x), `検出 前方 ${g.sd.toFixed(1)} m`);
    addHandle(g.la, -g.dr - 0.001, COLORS.deadband, (x) => setValue('pass_through_lookahead', x),
              `不感帯 前方 ${g.la.toFixed(1)} m`);
}

// ---------------------------------------------------------------- 試しに置く人
// 人の円にかかる 0.1 m セルを、obstacle_monitor と同じ考え方で分類する
function personCells() {
    const cs = state.reference.cell_size || 0.1;
    const { x: px, y: py } = state.person;
    const g = geometry();
    const cells = [];
    for (let ix = Math.floor((px - PERSON_RADIUS) / cs); ix <= Math.floor((px + PERSON_RADIUS) / cs); ix++) {
        for (let iy = Math.floor((py - PERSON_RADIUS) / cs); iy <= Math.floor((py + PERSON_RADIUS) / cs); iy++) {
            const cx = (ix + 0.5) * cs;
            const cy = (iy + 0.5) * cs;
            if (Math.hypot(cx - px, cy - py) > PERSON_RADIUS) continue;
            const inPathRange = cx >= -PATH_BACK && cx <= g.la;
            const inDead = inPathRange && cy <= g.dl && cy >= -g.dr;
            const inDetect = !inDead && cx >= 0 && cx <= g.sd && Math.abs(cy) <= g.pc;
            const inMargin = inPathRange && cy <= g.dl + g.m && cy >= -g.dr - g.m;
            cells.push({ cx, cy, kind: inDead ? 'ignored' : inDetect ? 'detect' : inMargin ? 'hidden' : 'costmap' });
        }
    }
    return cells;
}

function personEnabled() {
    return document.getElementById('pt-person-on').checked;
}

function personVerdict() {
    if (!personEnabled() || state.values.path_clearance === undefined) return { stop: false, cells: [] };
    const cells = personCells();
    const detected = cells.filter(c => c.kind === 'detect').length;
    return { stop: detected >= state.values.pass_through_min_blocking_cells, detected, cells };
}

function rebuildPerson() {
    disposeGroup(personGroup);
    const verdictEl = document.getElementById('pt-verdict');
    const resultEl = document.getElementById('pt-person-result');
    if (!personEnabled()) {
        verdictEl.hidden = true;
        resultEl.textContent = '';
        return;
    }
    const { x, y } = state.person;
    const body = new THREE.Mesh(new THREE.CylinderGeometry(PERSON_RADIUS, PERSON_RADIUS, PERSON_HEIGHT, 24),
                                new THREE.MeshStandardMaterial({ color: COLORS.person, transparent: true, opacity: 0.55 }));
    body.rotation.x = Math.PI / 2;  // 円柱を z 方向に立てる
    body.position.set(x, y, PERSON_HEIGHT / 2);
    body.userData.isPerson = true;
    personGroup.add(body);

    const verdict = personVerdict();
    const cs = state.reference.cell_size || 0.1;
    const colorOf = { ignored: COLORS.cellIgnored, detect: COLORS.cellDetect, hidden: COLORS.margin,
                      costmap: COLORS.cellCostmap };
    for (const c of verdict.cells) {
        const cube = new THREE.Mesh(new THREE.BoxGeometry(cs * 0.92, cs * 0.92, 0.05),
                                    new THREE.MeshBasicMaterial({ color: colorOf[c.kind] }));
        cube.position.set(c.cx, c.cy, 0.03);
        personGroup.add(cube);
    }
    const count = kind => verdict.cells.filter(c => c.kind === kind).length;
    const need = state.values.pass_through_min_blocking_cells;
    verdictEl.hidden = false;
    verdictEl.className = 'pt-verdict ' + (verdict.stop ? 'is-stop' : 'is-pass');
    verdictEl.textContent = verdict.stop
        ? `通過モード中: 停止する (検出 ${verdict.detected} ≥ ${need} セル)`
        : `通過モード中: 止まらない (検出 ${verdict.detected} < ${need} セル)`;
    resultEl.innerHTML = '';
    for (const [kind, text] of [['detect', '検出帯で検出'], ['ignored', '不感帯で無視'],
                                ['hidden', 'costmap に入れない (検出帯の外)'], ['costmap', 'local_costmap に入る']]) {
        const n = count(kind);
        if (!n) continue;
        const row = document.createElement('div');
        const sw = document.createElement('span');
        sw.className = `pt-swatch pt-swatch--cell-${kind}`;
        row.append(sw, ` ${text}: ${n} セル`);
        resultEl.appendChild(row);
    }
}

// ---------------------------------------------------------------- ロボット
const robotMaterial = new THREE.MeshStandardMaterial({ color: COLORS.robot, metalness: 0.2, roughness: 0.7 });

function showLoading(text) {
    const el = document.getElementById('pt-loading');
    el.hidden = !text;
    el.textContent = text || '';
}

function loadRobotMesh(name) {
    for (const obj of [...robotGroup.children]) {
        if (obj.userData.isModel) robotGroup.remove(obj);
    }
    if (!name) return;
    const url = `/robot_mesh/${encodeURIComponent(name)}`;
    const ext = name.split('.').pop().toLowerCase();
    const onProgress = e => {
        if (e.lengthComputable) showLoading(`モデルを読み込んでいます… ${Math.round(e.loaded / e.total * 100)}%`);
    };
    const onError = err => {
        showLoading('');
        setMessage(`モデルを読み込めませんでした: ${name} (${err?.message || err})`, true);
    };
    // メッシュは URDF と同じく base_link 直下の床面が原点 (wizurg.xacro の mesh_z) なので、そのまま置く
    const add = obj => {
        obj.userData.isModel = true;
        robotGroup.add(obj);
        showLoading('');
    };
    showLoading('モデルを読み込んでいます…');
    if (ext === 'stl') {
        new STLLoader().load(url, geo => {
            geo.computeVertexNormals();
            add(new THREE.Mesh(geo, robotMaterial));
        }, onProgress, onError);
    } else if (ext === 'obj') {
        const base = name.replace(/\.obj$/i, '');
        new MTLLoader().setPath('/robot_mesh/').load(`${base}.mtl`, materials => {
            materials.preload();
            new OBJLoader().setMaterials(materials).load(url, add, onProgress, onError);
        }, undefined, () => new OBJLoader().load(url, obj => {
            obj.traverse(o => { if (o.isMesh) o.material = robotMaterial; });
            add(obj);
        }, onProgress, onError));
    } else if (ext === 'dae') {
        new ColladaLoader().load(url, collada => add(collada.scene), onProgress, onError);
    }
}

// 車体の外形 (footprint) とセンサ
function buildRobotOutline() {
    for (const obj of [...robotGroup.children]) {
        if (!obj.userData.isModel) robotGroup.remove(obj);
    }
    const r = state.reference;
    const minX = r.footprint_min_x ?? -0.535;
    const maxX = r.footprint_max_x ?? 0.195;
    const hw = r.footprint_half_width ?? 0.29;
    const outline = new THREE.Group();
    addLine(outline, [[minX, -hw, 0.012], [maxX, -hw, 0.012], [maxX, hw, 0.012], [minX, hw, 0.012], [minX, -hw, 0.012]],
            COLORS.footprint);
    robotGroup.add(outline);
    if (state.sensorOrigin) {
        // URDF の位置は base_link 基準。base_link は床から -ground_z の高さ
        const [sx, sy, sz] = state.sensorOrigin;
        const baseH = -(r.ground_z ?? -0.068);
        const sensor = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.07, 20),
                                      new THREE.MeshStandardMaterial({ color: COLORS.sensor }));
        sensor.rotation.x = Math.PI / 2;
        sensor.position.set(sx, sy, sz + baseH);
        robotGroup.add(sensor);
        addLabel(robotGroup, 'YVT', sx, sy, sz + baseH + 0.12);
    }
}

// ---------------------------------------------------------------- パネル
function clampToSpec(key, v) {
    const s = state.spec[key];
    if (!s) return v;
    v = Math.min(s.max, Math.max(s.min, v));
    return s.type === 'int' ? Math.round(v) : Math.round(v * 1000) / 1000;
}

function setValue(key, v, { fromInput = false } = {}) {
    if (!Number.isFinite(v)) return;
    const field = Object.values(FIELDS).flat().find(f => f.key === key);
    const step = field?.step || 0.01;
    if (!fromInput) v = Math.round(v / step) * step;  // ドラッグはスライダーの刻みに合わせる
    state.values[key] = clampToSpec(key, v);
    refresh();
}

function buildFields() {
    for (const [group, fields] of Object.entries(FIELDS)) {
        const host = document.getElementById(`pt-fields-${group}`);
        host.innerHTML = '';
        for (const f of fields) {
            const spec = state.spec[f.key];
            const wrap = document.createElement('div');
            wrap.className = 'pt-field';
            wrap.dataset.key = f.key;
            wrap.innerHTML = `
                <div class="pt-field__head">
                    <span class="pt-field__name"></span><span class="pt-field__key"></span>
                </div>
                <div class="pt-field__row">
                    <input type="range"><input type="number"><span class="pt-field__derived"></span>
                </div>
                <p class="pt-field__desc"></p>`;
            wrap.querySelector('.pt-field__name').textContent = f.name;
            wrap.querySelector('.pt-field__key').textContent = f.key;
            wrap.querySelector('.pt-field__desc').textContent = spec?.description || '';
            const range = wrap.querySelector('input[type="range"]');
            const num = wrap.querySelector('input[type="number"]');
            for (const input of [range, num]) {
                input.min = spec?.min ?? 0;
                input.max = spec?.max ?? 1;
                input.step = f.step;
            }
            if (f.sliderMax !== undefined) range.max = Math.min(spec?.max ?? f.sliderMax, f.sliderMax);
            range.addEventListener('input', () => setValue(f.key, parseFloat(range.value), { fromInput: true }));
            num.addEventListener('change', () => setValue(f.key, parseFloat(num.value), { fromInput: true }));
            host.appendChild(wrap);
        }
    }
}

function refreshFields() {
    for (const f of Object.values(FIELDS).flat()) {
        const wrap = document.querySelector(`.pt-field[data-key="${f.key}"]`);
        if (!wrap) continue;
        const v = state.values[f.key];
        const range = wrap.querySelector('input[type="range"]');
        const num = wrap.querySelector('input[type="number"]');
        if (parseFloat(range.max) < v) range.max = v;  // 範囲を超える値もスライダーで表せるように
        range.value = v;
        if (document.activeElement !== num) num.value = v;
        wrap.querySelector('.pt-field__derived').textContent = f.derived ? f.derived(v) : '';
        wrap.classList.toggle('is-changed', v !== state.saved[f.key]);
    }
    // 検出帯に入りうる最大のセル数
    const g = geometry();
    const cs = state.reference.cell_size || 0.1;
    const width = g.detect.reduce((s, [lo, hi]) => s + Math.max(0, hi - lo), 0);
    const capacity = Math.floor(g.sd / cs) * Math.floor(width / cs + 1e-9);
    const need = state.values.pass_through_min_blocking_cells;
    const cap = document.getElementById('pt-capacity');
    if (width <= 1e-6) {
        cap.textContent = '検出帯がありません。通過モード中は何があっても止まりません (強制通過)。';
        cap.className = 'pt-note is-warning';
    } else if (need > capacity) {
        cap.textContent = `検出帯に入るセルは最大 ${capacity} 個です。しきい値 ${need} では通過モード中に止まりません。`;
        cap.className = 'pt-note is-warning';
    } else {
        cap.textContent = `検出帯の幅: 左 ${Math.max(0, g.detect[0][1] - g.detect[0][0]).toFixed(2)} m / ` +
            `右 ${Math.max(0, g.detect[1][1] - g.detect[1][0]).toFixed(2)} m (最大 ${capacity} セル)。` +
            `通常走行のしきい値は ${state.reference.min_blocking_cells ?? '-'} セルです。`;
        cap.className = 'pt-note';
    }
}

function refreshLive() {
    const el = document.getElementById('pt-live');
    el.className = 'pt-live';
    if (!state.live) {
        el.textContent = '実行中の obstacle_monitor はありません。保存した値は次にナビゲーションを起動したときに使われます。';
        return;
    }
    const diffs = Object.keys(state.saved).filter(k => state.live[k] !== undefined && state.live[k] !== null &&
                                                       Number(state.live[k]) !== Number(state.saved[k]));
    if (diffs.length) {
        el.classList.add('is-diff');
        el.textContent = 'obstacle_monitor 実行中。ファイルと違う値で動いています: ' +
            diffs.map(k => `${k} = ${state.live[k]}`).join(', ');
    } else {
        el.classList.add('is-running');
        el.textContent = 'obstacle_monitor 実行中 (ファイルと同じ値で動いています)。';
    }
}

function refresh() {
    refreshFields();
    rebuildZones();
    rebuildPerson();
}

function setMessage(text, isError = false) {
    const el = document.getElementById('pt-message');
    el.textContent = text;
    el.className = 'pt-message ' + (isError ? 'is-error' : 'is-success');
}

// ---------------------------------------------------------------- ドラッグ
const raycaster = new THREE.Raycaster();
const floor = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
let dragging = null;  // { apply } or 'person'

function pointerNdc(e) {
    const rect = renderer.domElement.getBoundingClientRect();
    return new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1,
                             -((e.clientY - rect.top) / rect.height) * 2 + 1);
}

function pick(e) {
    raycaster.setFromCamera(pointerNdc(e), camera);
    const hit = raycaster.intersectObjects(handles.map(h => h.mesh), false)[0];
    if (hit) return handles.find(h => h.mesh === hit.object);
    if (personEnabled()) {
        const p = raycaster.intersectObjects(personGroup.children, false).find(i => i.object.userData.isPerson);
        if (p) return 'person';
    }
    return null;
}

function floorPoint(e) {
    raycaster.setFromCamera(pointerNdc(e), camera);
    const p = new THREE.Vector3();
    return raycaster.ray.intersectPlane(floor, p) ? p : null;
}

renderer.domElement.addEventListener('pointerdown', e => {
    if (e.button !== 0) return;
    dragging = pick(e);
    if (dragging) {
        controls.enabled = false;
        container.classList.add('is-dragging');
        renderer.domElement.setPointerCapture(e.pointerId);
    }
});
renderer.domElement.addEventListener('pointermove', e => {
    if (!dragging) {
        container.classList.toggle('is-hover-handle', !!pick(e));
        return;
    }
    const p = floorPoint(e);
    if (!p) return;
    if (dragging === 'person') {
        state.person.set(Math.round(p.x * 100) / 100, Math.round(p.y * 100) / 100);
        rebuildZones();
        rebuildPerson();
    } else {
        dragging.apply(p.x, p.y);
    }
});
const endDrag = () => {
    if (!dragging) return;
    dragging = null;
    controls.enabled = true;
    container.classList.remove('is-dragging');
};
renderer.domElement.addEventListener('pointerup', endDrag);
renderer.domElement.addEventListener('pointercancel', endDrag);

// ---------------------------------------------------------------- 読み込み・保存
async function loadConfig() {
    const res = await fetch('/api/pass_through_config');
    const data = await res.json();
    if (data.status !== 'success') throw new Error(data.message);
    state.spec = data.spec;
    state.reference = data.reference;
    state.sensorOrigin = data.sensor_origin;
    state.values = {};
    for (const [k, s] of Object.entries(data.spec)) {
        const v = data.values[k];
        state.values[k] = (v === null || v === undefined) ? s.min : Number(v);
    }
    state.saved = { ...state.values };
    document.getElementById('pt-yaml-path').textContent = data.yaml_path;
}

async function loadLive() {
    try {
        const res = await fetch('/api/pass_through_config/live');
        const data = await res.json();
        state.live = data.running ? data.values : null;
    } catch {
        state.live = null;
    }
    refreshLive();
}

async function save(applyLive) {
    const buttons = document.querySelectorAll('.pt-actions button');
    buttons.forEach(b => { b.disabled = true; });
    setMessage(applyLive ? '保存して、実行中の obstacle_monitor に送っています…' : '保存しています…');
    try {
        const res = await fetch('/api/pass_through_config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ values: state.values, apply_live: applyLive }),
        });
        const data = await res.json();
        if (data.status !== 'success') {
            setMessage(data.message, true);
            return;
        }
        state.saved = { ...state.values };
        let text = data.message;
        if (data.live) {
            text += data.live.ok ? '\n走行中の obstacle_monitor にも反映しました。'
                                 : `\n走行中への反映に失敗しました: ${data.live.output || '(出力なし)'}`;
        }
        setMessage(text, data.live && !data.live.ok);
        refresh();
        loadLive();
    } catch (err) {
        setMessage(`保存できませんでした: ${err}`, true);
    } finally {
        buttons.forEach(b => { b.disabled = false; });
    }
}

const PRESETS = {
    all: { pass_through_deadband_left_ratio: 1.0, pass_through_deadband_right_ratio: 1.0 },
    left: { pass_through_deadband_left_ratio: 1.0, pass_through_deadband_right_ratio: 0.0 },
    right: { pass_through_deadband_left_ratio: 0.0, pass_through_deadband_right_ratio: 1.0 },
    none: { pass_through_deadband_left_ratio: 0.0, pass_through_deadband_right_ratio: 0.0 },
};

document.querySelectorAll('[data-preset]').forEach(b => b.addEventListener('click', () => {
    Object.assign(state.values, PRESETS[b.dataset.preset]);
    refresh();
}));
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => setView(b.dataset.view)));
document.getElementById('pt-mesh').addEventListener('change', e => {
    const name = e.target.value;
    const opt = e.target.selectedOptions[0];
    if (/\.obj$/i.test(name) && !confirm(`${opt.textContent} は大きいため、読み込みに時間がかかります。読み込みますか？`)) {
        return;
    }
    loadRobotMesh(name);
});
document.getElementById('pt-person-on').addEventListener('change', refresh);
document.getElementById('pt-save').addEventListener('click', () => save(false));
document.getElementById('pt-save-apply').addEventListener('click', () => save(true));
document.getElementById('pt-reload').addEventListener('click', async () => {
    await loadConfig();
    refresh();
    setMessage('ファイルから読み直しました。');
    loadLive();
});
window.addEventListener('beforeunload', e => {
    const changed = Object.keys(state.saved).some(k => state.values[k] !== state.saved[k]);
    if (changed) e.preventDefault();
});

// ---------------------------------------------------------------- 起動
(async () => {
    resize();
    try {
        await loadConfig();
    } catch (err) {
        setMessage(`設定を読み込めませんでした: ${err.message || err}`, true);
        return;
    }
    buildFields();
    buildRobotOutline();
    setView('oblique');
    refresh();
    loadRobotMesh(document.getElementById('pt-mesh').value);
    loadLive();
})();

renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
    labelRenderer.render(scene, camera);
});
