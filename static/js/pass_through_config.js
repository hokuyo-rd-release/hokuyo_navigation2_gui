// 障害物検出設定 (pass_through_config.html)
//
// obstacle_monitor の検出範囲 (停止判定の範囲・不感帯・costmap 除外) を、ウェイポイントの属性ごとに
// ロボットのモデルの上に帯を描きながら編集する。パラメータは zone.<属性>.<項目> (obstacle_monitor.yaml)。
// ウェイポイント個別の上書きは Map Viewer のウェイポイント編集で行う。
//  - 座標は ROS と同じ (x = 前, y = 左, z = 上)。原点はロボット (base_link) 直下の床面。
//  - 経路はロボットの真ん中を前方へまっすぐ通るものとして描く。
//  - 帯の端の ● をドラッグするか、右のパネルのスライダーで値を変える。
//  - 帯の形は obstacle_monitor.cpp と同じ計算で決める:
//      不感帯   : 中心線から 左 path_clearance_left × deadband_left_ratio / 右 path_clearance_right × deadband_right_ratio、
//                 プランの後方 lookbehind 〜 前方 lookahead
//      検出帯   : 後方 stop_distance_back 〜 前方 stop_distance、左 path_clearance_left・
//                 右 path_clearance_right のうち、不感帯を除いた部分
//      costmap  : 不感帯の外側 costmap_margin までは local_costmap に入れない (前後の長さは不感帯と同じ)
//  - YVT で見える範囲 (視野・検出空間・高さ別の床の検出マップ) を yvt_coverage.js で重ねて描く。
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { YvtCoverage } from './yvt_coverage.js';

const OBSTACLE_SIZE = 0.4;  // 試しに置く障害物の一辺 [m] (高さは床の検出マップと共通の「物の高さ」)

// パネルに並べる項目
// パネルに並べる項目。key は属性ごとの項目名 (zone.<属性>.<key>)。global: true は属性によらない
const FIELDS = {
    deadband: [
        { key: 'deadband_left_ratio', name: '左側の割合', step: 0.01,
          derived: v => `= ${(v * z('path_clearance_left')).toFixed(2)} m` },
        { key: 'deadband_right_ratio', name: '右側の割合', step: 0.01,
          derived: v => `= ${(v * z('path_clearance_right')).toFixed(2)} m` },
        { key: 'lookahead', name: '前方の長さ [m]', step: 0.1, sliderMax: 12 },
        { key: 'lookbehind', name: '後方の長さ [m]', step: 0.05, sliderMax: 3 },
    ],
    detect: [
        { key: 'path_clearance_left', name: '左の幅 [m]', step: 0.01, sliderMax: 1.5 },
        { key: 'path_clearance_right', name: '右の幅 [m]', step: 0.01, sliderMax: 1.5 },
        { key: 'stop_distance', name: '前方の長さ [m]', step: 0.1, sliderMax: 6.0 },
        { key: 'stop_distance_back', name: '後方の長さ [m]', step: 0.05, sliderMax: 2.0 },
        { key: 'min_blocking_cells', name: '停止するセル数', step: 1, sliderMax: 60 },
    ],
    other: [
        { key: 'costmap_margin', name: 'costmap に入れない幅 [m]', step: 0.01, sliderMax: 1.0 },
        { key: 'zone_timeout', name: '指令途絶で normal に戻すまで [s] (全属性共通)', step: 0.1, sliderMax: 5.0,
          global: true },
    ],
};

// ウェイポイントの属性 (表示名と、Map Viewer と同じ色のトークン)
const ZONE_TYPE_INFO = {
    normal: { label: 'normal', token: '--wp-normal' },
    slow: { label: 'slow', token: '--wp-slow' },
    stop: { label: 'stop', token: '--wp-stop' },
    wait: { label: 'wait', token: '--wp-wait' },
    pass_through: { label: 'pass_through', token: '--wp-pass-through' },
};

function isGlobalField(field) {
    return !!Object.values(FIELDS).flat().find(x => x.key === field)?.global;
}
// 選んでいる属性の項目のパラメータ名
function zk(field) {
    return isGlobalField(field) ? field : `zone.${state.type}.${field}`;
}
// 表示・編集している範囲の項目の値。ウェイポイントを選んでいるときは、その属性の値に個別設定を重ねる
function z(field) {
    if (inWaypointMode() && !isGlobalField(field)) {
        const ov = waypointOverrides(state.wp.index);
        if (ov[field] !== undefined) return ov[field];
    }
    return state.values[zk(field)];
}

// ---------------------------------------------------------------- ウェイポイント個別の設定
// state.wp.list[i].overrides はファイルの値、state.wp.edits[i] は保存していない変更 (null は「消す」)
function inWaypointMode() {
    return state.wp.index !== null && state.wp.list[state.wp.index] !== undefined;
}
function waypointOverrides(index) {
    const ov = { ...(state.wp.list[index]?.overrides || {}) };
    for (const [f, v] of Object.entries(state.wp.edits[index] || {})) {
        if (v === null) delete ov[f]; else ov[f] = v;
    }
    return ov;
}
function waypointEditsPending() {
    return Object.values(state.wp.edits).some(e => Object.keys(e).length > 0);
}
function setWaypointOverride(field, v) {
    const i = state.wp.index;
    if (!state.wp.edits[i]) state.wp.edits[i] = {};
    state.wp.edits[i][field] = v;
}

const state = {
    type: 'normal',  // 編集中の属性 (ウェイポイントを選んでいるときはその属性)
    wp: { file: '', list: [], index: null, edits: {} },  // ウェイポイント個別の設定
    types: [],
    values: {},      // 編集中の値
    saved: {},       // ファイルに保存されている値
    live: null,      // 実行中の obstacle_monitor の値 (動いていなければ null)
    spec: {},
    reference: {},
    sensorOrigin: null,
    sensorRpy: [0, 0, 0],
    obstacle: new THREE.Vector2(2.0, 0.25),  // 試しに置く障害物の中心
    obstacleHeight: 0.4,  // 床の検出マップと共通の「物の高さ」(pt-cov-height)
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
    obstacleSeen: tokenColor('--pt-obstacle-seen'),
    obstacleUnseen: tokenColor('--pt-obstacle-unseen'),
    cellIgnored: tokenColor('--pt-cell-ignored'),
    cellDetect: tokenColor('--pt-cell-detect'),
    cellCostmap: tokenColor('--pt-cell-costmap'),
    cellUnseen: tokenColor('--pt-cell-unseen'),
    fov: tokenColor('--pt-fov'),
    covOk: tokenColor('--pt-cov-ok'),
    covLow: tokenColor('--pt-cov-low'),
    covRear: tokenColor('--pt-cov-rear'),
    covOut: tokenColor('--pt-cov-out'),
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
const obstacleGroup = new THREE.Group(); // 試しに置く障害物とセル
const coverageGroup = new THREE.Group(); // YVT で見える範囲
scene.add(zoneGroup, handleGroup, robotGroup, obstacleGroup, coverageGroup);
const coverage = new YvtCoverage(coverageGroup, {
    fov: COLORS.fov, ok: COLORS.covOk, low: COLORS.covLow, rear: COLORS.covRear, out: COLORS.covOut,
});

function setView(name) {
    const sd = z('stop_distance') || 2.0;
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
    const pl = z('path_clearance_left');
    const pr = z('path_clearance_right');
    const dl = pl * z('deadband_left_ratio');
    const dr = pr * z('deadband_right_ratio');
    return {
        pl, pr, dl, dr,
        m: z('costmap_margin'),
        sd: z('stop_distance'),
        sb: z('stop_distance_back'),
        la: z('lookahead'),
        lb: z('lookbehind'),
        // 検出帯 (横方向 lo〜hi、左が正)。不感帯に覆われた側は幅 0
        detect: [[Math.min(dl, pl), pl], [-pr, -Math.min(dr, pr)]],
    };
}

// ---------------------------------------------------------------- 帯とハンドル
const handles = [];   // { mesh, apply(x, y) }

function addHandle(x, y, color, apply, label) {
    // 車体のモデルや障害物の中にあっても見えてつかめるよう、常に手前に描く
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.07, 20, 14),
                                new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35,
                                                                 depthTest: false }));
    mesh.renderOrder = 10;
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
    const stop = obstacleVerdict().stop;
    const detectColor = stop ? COLORS.detectStop : COLORS.detect;
    const xb = -g.lb;

    // 不感帯 (後方 lookbehind 〜 前方 lookahead)
    addRect(zoneGroup, xb, g.la, -g.dr, g.dl, 0.004, COLORS.deadband, 0.28);
    // costmap に入れない範囲 (不感帯の外側 margin)
    addRect(zoneGroup, xb, g.la, g.dl, g.dl + g.m, 0.003, COLORS.margin, 0.18);
    addRect(zoneGroup, xb, g.la, -g.dr - g.m, -g.dr, 0.003, COLORS.margin, 0.18);
    // 検出帯 (後方 stop_distance_back 〜 前方 stop_distance)
    for (const [lo, hi] of g.detect) {
        addRect(zoneGroup, -g.sb, g.sd, lo, hi, 0.006, detectColor, 0.35);
    }
    // 停止判定の範囲の外枠 (点線)
    addLine(zoneGroup, [[-g.sb, g.pl, 0.008], [g.sd, g.pl, 0.008], [g.sd, -g.pr, 0.008],
                        [-g.sb, -g.pr, 0.008], [-g.sb, g.pl, 0.008]], detectColor, true);
    // 経路の中心線
    addLine(zoneGroup, [[Math.min(xb, -g.sb) - 0.5, 0, 0.01], [Math.max(g.la, g.sd) + 1.0, 0, 0.01]],
            COLORS.path, true);

    // ドラッグできる ●。それぞれが動かす辺の真ん中に置く
    //   左右の幅を動かす ● : その辺の前後方向の中央 (axis 'x' = 辺に沿って x 方向)
    //   前後の長さを動かす ● : その辺の左右方向の中央 (axis 'y')
    // 帯が重なって ● 同士が同じ位置に来るときだけ、辺に沿って少しずらす
    const midDead = (-g.lb + g.la) / 2;   // 不感帯・costmap 除外の前後の中央
    const midDetect = (-g.sb + g.sd) / 2; // 検出帯の前後の中央
    const midDeadY = (g.dl - g.dr) / 2;   // 不感帯 (と costmap 除外) の左右の中央
    const midDetectY = (g.pl - g.pr) / 2; // 検出帯の左右の中央
    const placed = [];
    const place = (x, y, axis, color, apply, label) => {
        for (let k = 0; k < 8 && placed.some(p => Math.hypot(p[0] - x, p[1] - y) < 0.2); k++) {
            if (axis === 'x') x += 0.3; else y -= 0.25;
        }
        placed.push([x, y]);
        addHandle(x, y, color, apply, label);
    };
    place(midDetect, g.pl, 'x', COLORS.detect, (x, y) => setValue('path_clearance_left', Math.max(0, y)),
          `検出 左 ${g.pl.toFixed(2)} m`);
    place(midDetect, -g.pr, 'x', COLORS.detect, (x, y) => setValue('path_clearance_right', Math.max(0, -y)),
          `検出 右 ${g.pr.toFixed(2)} m`);
    place(g.sd, midDetectY, 'y', COLORS.detect, (x) => setValue('stop_distance', x), `検出 前方 ${g.sd.toFixed(1)} m`);
    // 車体の中に入っても、● は常に手前に描かれるので見える
    place(-g.sb, midDetectY, 'y', COLORS.detect, (x) => setValue('stop_distance_back', Math.max(0, -x)),
          `検出 後方 ${g.sb.toFixed(2)} m`);
    place(midDead, g.dl, 'x', COLORS.deadband,
          (x, y) => setValue('deadband_left_ratio', y / z('path_clearance_left')),
          `不感帯 左 ${g.dl.toFixed(2)} m`);
    place(midDead, -g.dr, 'x', COLORS.deadband,
          (x, y) => setValue('deadband_right_ratio', -y / z('path_clearance_right')),
          `不感帯 右 ${g.dr.toFixed(2)} m`);
    place(midDead, g.dl + g.m, 'x', COLORS.margin, (x, y) => setValue('costmap_margin', y - geometry().dl),
          `costmap 除外 左 ${(g.dl + g.m).toFixed(2)} m`);
    place(midDead, -g.dr - g.m, 'x', COLORS.margin, (x, y) => setValue('costmap_margin', -y - geometry().dr),
          `costmap 除外 右 ${(g.dr + g.m).toFixed(2)} m`);
    // 前後の長さは不感帯と costmap 除外で共通なので、両方を合わせた辺の真ん中に置く
    place(g.la, midDeadY, 'y', COLORS.deadband, (x) => setValue('lookahead', x),
          `不感帯 前方 ${g.la.toFixed(1)} m`);
    place(-g.lb, midDeadY, 'y', COLORS.deadband, (x) => setValue('lookbehind', Math.max(0, -x)),
          `不感帯 後方 ${g.lb.toFixed(2)} m`);
}

// ---------------------------------------------------------------- 試しに置く障害物
// 障害物 (OBSTACLE_SIZE 角・高さ state.obstacleHeight) にかかる 0.1 m セルを、obstacle_monitor と同じ考え方で分類する。
// YVT から見えないセル (低すぎる・視野の外など。床の検出マップと同じ判定) は点が出ないため、
// 停止判定にも local_costmap にも入らない ('unseen')。
function obstacleCells() {
    const cs = state.reference.cell_size || 0.1;
    const { x: px, y: py } = state.obstacle;
    const half = OBSTACLE_SIZE / 2;
    const g = geometry();
    const cells = [];
    for (let ix = Math.floor((px - half) / cs); ix <= Math.floor((px + half) / cs); ix++) {
        for (let iy = Math.floor((py - half) / cs); iy <= Math.floor((py + half) / cs); iy++) {
            const cx = (ix + 0.5) * cs;
            const cy = (iy + 0.5) * cs;
            if (Math.abs(cx - px) > half || Math.abs(cy - py) > half) continue;
            const seen = !coverage.params || coverage.floorReason(cx, cy, state.obstacleHeight) === 'ok';
            const inPathRange = cx >= -g.lb && cx <= g.la;
            const inDead = inPathRange && cy <= g.dl && cy >= -g.dr;
            const inDetect = !inDead && cx >= -g.sb && cx <= g.sd && cy <= g.pl && cy >= -g.pr;
            const inMargin = inPathRange && cy <= g.dl + g.m && cy >= -g.dr - g.m;
            const kind = !seen ? 'unseen' : inDead ? 'ignored' : inDetect ? 'detect' : inMargin ? 'hidden' : 'costmap';
            cells.push({ cx, cy, kind, inDetect });
        }
    }
    return cells;
}

function obstacleEnabled() {
    return document.getElementById('pt-obstacle-on').checked;
}

function obstacleVerdict() {
    if (!obstacleEnabled() || z('path_clearance_left') === undefined) return { stop: false, cells: [] };
    const cells = obstacleCells();
    const detected = cells.filter(c => c.kind === 'detect').length;
    // 検出帯にかかっているのに YVT から見えないセル
    const hiddenInZone = cells.filter(c => c.kind === 'unseen' && c.inDetect).length;
    return { stop: detected >= z('min_blocking_cells'), detected, hiddenInZone, cells };
}

function rebuildObstacle() {
    disposeGroup(obstacleGroup);
    const verdictEl = document.getElementById('pt-verdict');
    const resultEl = document.getElementById('pt-obstacle-result');
    if (!obstacleEnabled()) {
        verdictEl.hidden = true;
        resultEl.textContent = '';
        return;
    }
    const { x, y } = state.obstacle;
    const h = state.obstacleHeight;
    const verdict = obstacleVerdict();
    // YVT から 1 セルでも見えていれば検出中 (点が出て obstacle_monitor に入る)
    const seen = verdict.cells.some(c => c.kind !== 'unseen');
    const bodyColor = seen ? COLORS.obstacleSeen : COLORS.obstacleUnseen;
    const box = new THREE.BoxGeometry(OBSTACLE_SIZE, OBSTACLE_SIZE, h);
    const body = new THREE.Mesh(box, new THREE.MeshStandardMaterial({ color: bodyColor, transparent: true,
                                                                      opacity: seen ? 0.6 : 0.4 }));
    body.position.set(x, y, h / 2);
    body.userData.isObstacle = true;
    obstacleGroup.add(body);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box), new THREE.LineBasicMaterial({ color: bodyColor }));
    edges.position.copy(body.position);
    obstacleGroup.add(edges);
    // 帯の ● のラベル (床から 0.28 m) と重ならないよう少し高めに置く
    addLabel(obstacleGroup, `高さ ${h.toFixed(2)} m ・ ${seen ? '検出中' : '検出できない'}`, x, y, h + 0.4);

    const cs = state.reference.cell_size || 0.1;
    const colorOf = { ignored: COLORS.cellIgnored, detect: COLORS.cellDetect, hidden: COLORS.margin,
                      costmap: COLORS.cellCostmap, unseen: COLORS.cellUnseen };
    for (const c of verdict.cells) {
        const cube = new THREE.Mesh(new THREE.BoxGeometry(cs * 0.92, cs * 0.92, 0.05),
                                    new THREE.MeshBasicMaterial({ color: colorOf[c.kind] }));
        cube.position.set(c.cx, c.cy, 0.03);
        obstacleGroup.add(cube);
    }
    const count = kind => verdict.cells.filter(c => c.kind === kind).length;
    const need = z('min_blocking_cells');
    verdictEl.hidden = false;
    verdictEl.className = 'pt-verdict ' + (verdict.stop ? 'is-stop' : 'is-pass');
    let text = verdict.stop
        ? `${zoneLabel()} の区間: 停止する (検出 ${verdict.detected} ≥ ${need} セル)`
        : `${zoneLabel()} の区間: 止まらない (検出 ${verdict.detected} < ${need} セル)`;
    if (!verdict.stop && verdict.hiddenInZone > 0) {
        text += `\n検出帯にかかっているが YVT から見えない: ${verdict.hiddenInZone} セル`;
    }
    verdictEl.textContent = text;
    resultEl.innerHTML = '';
    for (const [kind, text] of [['detect', '検出帯で検出'], ['ignored', '不感帯で無視'],
                                ['hidden', 'costmap に入れない (検出帯の外)'], ['costmap', 'local_costmap に入る'],
                                ['unseen', 'YVT から見えない (どこにも入らない)']]) {
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
    if (inWaypointMode() && !isGlobalField(key)) {
        setWaypointOverride(key, clampToSpec(zk(key), v));  // ウェイポイントの個別設定として記録する
    } else {
        state.values[zk(key)] = clampToSpec(zk(key), v);
    }
    refresh();
}

function buildFields() {
    for (const [group, fields] of Object.entries(FIELDS)) {
        const host = document.getElementById(`pt-fields-${group}`);
        host.innerHTML = '';
        for (const f of fields) {
            const spec = state.spec[zk(f.key)];
            const wrap = document.createElement('div');
            wrap.className = 'pt-field';
            wrap.dataset.key = f.key;
            wrap.innerHTML = `
                <div class="pt-field__head">
                    <span class="pt-field__name"></span><span class="pt-field__key"></span>
                </div>
                <div class="pt-field__row">
                    <input type="range"><input type="number"><span class="pt-field__derived"></span>
                    <button type="button" class="pt-field__reset" hidden>既定に戻す</button>
                </div>
                <p class="pt-field__desc"></p>`;
            wrap.querySelector('.pt-field__name').textContent = f.name;
            wrap.querySelector('.pt-field__key').textContent = zk(f.key);
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
            // ウェイポイントの個別設定を消して、属性の値に戻す
            wrap.querySelector('.pt-field__reset').addEventListener('click', () => {
                setWaypointOverride(f.key, null);
                refresh();
            });
            host.appendChild(wrap);
        }
    }
}

function refreshFields() {
    for (const f of Object.values(FIELDS).flat()) {
        const wrap = document.querySelector(`.pt-field[data-key="${f.key}"]`);
        if (!wrap) continue;
        const v = z(f.key);
        const range = wrap.querySelector('input[type="range"]');
        const num = wrap.querySelector('input[type="number"]');
        if (parseFloat(range.max) < v) range.max = v;  // 範囲を超える値もスライダーで表せるように
        range.value = v;
        if (document.activeElement !== num) num.value = v;
        wrap.querySelector('.pt-field__derived').textContent = f.derived ? f.derived(v) : '';
        const wpField = inWaypointMode() && !f.global;
        const overridden = wpField && waypointOverrides(state.wp.index)[f.key] !== undefined;
        const pendingWp = wpField && state.wp.edits[state.wp.index]?.[f.key] !== undefined;
        wrap.querySelector('.pt-field__key').textContent = wpField
            ? (overridden ? `zone_${f.key} (#${state.wp.index} の個別設定)` : `${zk(f.key)} (属性の値)`)
            : zk(f.key);
        wrap.querySelector('.pt-field__reset').hidden = !overridden;
        wrap.classList.toggle('is-override', overridden);
        wrap.classList.toggle('is-changed', wpField ? pendingWp : v !== state.saved[zk(f.key)]);
    }
    refreshTypeTabs();
    refreshWaypointPanel();
    // 検出帯に入りうる最大のセル数
    const g = geometry();
    const cs = state.reference.cell_size || 0.1;
    const width = g.detect.reduce((s, [lo, hi]) => s + Math.max(0, hi - lo), 0);
    const capacity = Math.floor((g.sd + g.sb) / cs + 1e-9) * Math.floor(width / cs + 1e-9);
    const need = z('min_blocking_cells');
    const cap = document.getElementById('pt-capacity');
    if (width <= 1e-6) {
        cap.textContent = `検出帯がありません。${state.type} の区間では何があっても止まりません (強制通過)。`;
        cap.className = 'pt-note is-warning';
    } else if (need > capacity) {
        cap.textContent = `検出帯に入るセルは最大 ${capacity} 個です。しきい値 ${need} では ${state.type} の区間で止まりません。`;
        cap.className = 'pt-note is-warning';
    } else {
        cap.textContent = `検出帯の幅: 左 ${Math.max(0, g.detect[0][1] - g.detect[0][0]).toFixed(2)} m / ` +
            `右 ${Math.max(0, g.detect[1][1] - g.detect[1][0]).toFixed(2)} m (最大 ${capacity} セル)。`;
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
    rebuildObstacle();
}

function setMessage(text, isError = false) {
    const el = document.getElementById('pt-message');
    el.textContent = text;
    el.className = 'pt-message ' + (isError ? 'is-error' : 'is-success');
}

// ---------------------------------------------------------------- ドラッグ
const raycaster = new THREE.Raycaster();
const floor = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
let dragging = null;  // { apply } or 'obstacle'

function pointerNdc(e) {
    const rect = renderer.domElement.getBoundingClientRect();
    return new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1,
                             -((e.clientY - rect.top) / rect.height) * 2 + 1);
}

function pick(e) {
    raycaster.setFromCamera(pointerNdc(e), camera);
    const hit = raycaster.intersectObjects(handles.map(h => h.mesh), false)[0];
    if (hit) return handles.find(h => h.mesh === hit.object);
    if (obstacleEnabled()) {
        const p = raycaster.intersectObjects(obstacleGroup.children, false).find(i => i.object.userData.isObstacle);
        if (p) return 'obstacle';
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
    if (dragging === 'obstacle') {
        state.obstacle.set(Math.round(p.x * 100) / 100, Math.round(p.y * 100) / 100);
        rebuildZones();
        rebuildObstacle();
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
    state.sensorRpy = data.sensor_rpy || [0, 0, 0];
    state.values = {};
    for (const [k, s] of Object.entries(data.spec)) {
        const v = data.values[k];
        state.values[k] = (v === null || v === undefined) ? s.min : Number(v);
    }
    state.saved = { ...state.values };
    state.types = data.zone_types || Object.keys(ZONE_TYPE_INFO);
    if (!state.types.includes(state.type)) state.type = state.types[0];
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

// 不感帯のよく使う設定 (編集中の属性に当てる)
const PRESETS = {
    all: { deadband_left_ratio: 1.0, deadband_right_ratio: 1.0 },
    left: { deadband_left_ratio: 1.0, deadband_right_ratio: 0.0 },
    right: { deadband_left_ratio: 0.0, deadband_right_ratio: 1.0 },
    none: { deadband_left_ratio: 0.0, deadband_right_ratio: 0.0 },
};

document.querySelectorAll('[data-preset]').forEach(b => b.addEventListener('click', () => {
    for (const [field, v] of Object.entries(PRESETS[b.dataset.preset])) {
        if (inWaypointMode()) setWaypointOverride(field, v); else state.values[zk(field)] = v;
    }
    refresh();
}));

// ---------------------------------------------------------------- 属性のタブ
function buildTypeTabs() {
    const host = document.getElementById('pt-type-tabs');
    host.innerHTML = '';
    for (const type of state.types) {
        const info = ZONE_TYPE_INFO[type] || { label: type, token: '--wp-default' };
        const b = document.createElement('button');
        b.type = 'button';
        b.className = 'pt-type-tab';
        b.dataset.type = type;
        b.style.setProperty('--pt-type-color', `var(${info.token})`);
        b.textContent = info.label;
        b.addEventListener('click', () => {
            // 属性を選んだら、ウェイポイントの選択をやめて属性の設定を編集する
            state.type = type;
            state.wp.index = null;
            document.getElementById('pt-wp-index').value = '';
            buildFields();
            refresh();
        });
        host.appendChild(b);
    }
}

function refreshTypeTabs() {
    for (const b of document.querySelectorAll('.pt-type-tab')) {
        const t = b.dataset.type;
        b.classList.toggle('is-active', t === state.type);
        // 保存していない変更がある属性に印を付ける
        const changed = Object.keys(state.saved).some(k => k.startsWith(`zone.${t}.`) && state.values[k] !== state.saved[k]);
        b.classList.toggle('is-changed', changed);
    }
    const title = document.getElementById('pt-type-current');
    if (title) title.textContent = zoneLabel();
    const target = document.getElementById('pt-edit-target');
    if (target) {
        target.textContent = inWaypointMode()
            ? `編集中: ウェイポイント #${state.wp.index} の個別設定 (属性 ${state.type} の値に重ねる)`
            : `編集中: 属性 ${state.type} の設定 (obstacle_monitor.yaml)`;
        target.classList.toggle('is-waypoint', inWaypointMode());
    }
}

function zoneLabel() {
    return inWaypointMode() ? `#${state.wp.index} (${state.type})` : state.type;
}

// ---------------------------------------------------------------- ウェイポイントの選択・保存
const wpFileSelect = document.getElementById('pt-wp-file');
const wpIndexSelect = document.getElementById('pt-wp-index');

function waypointOptionText(w) {
    const n = Object.keys(waypointOverrides(w.index)).length;
    return `#${w.index} ${w.type}` + (n ? ` ・個別 ${n} 件` : '') +
        (state.wp.edits[w.index] && Object.keys(state.wp.edits[w.index]).length ? ' ●' : '');
}

function refreshWaypointPanel() {
    for (const opt of wpIndexSelect.options) {
        if (opt.value === '') continue;
        const w = state.wp.list[Number(opt.value)];
        if (w) opt.textContent = waypointOptionText(w);
    }
    const status = document.getElementById('pt-wp-status');
    const pending = waypointEditsPending();
    document.getElementById('pt-wp-save').disabled = !pending;
    document.getElementById('pt-wp-clear').disabled = !inWaypointMode();
    if (!state.wp.file) {
        status.textContent = 'ウェイポイントファイルを選ぶと、ウェイポイントごとに検出範囲を上書きできます。';
    } else if (inWaypointMode()) {
        const n = Object.keys(waypointOverrides(state.wp.index)).length;
        status.textContent = `#${state.wp.index} から次のウェイポイントまでの区間に使う範囲です。` +
            `個別設定 ${n} 件 (オレンジ枠の項目)。それ以外は属性 ${state.type} の値です。` +
            (pending ? ' 保存していない変更があります。' : '');
    } else {
        status.textContent = `${state.wp.list.length} 個のウェイポイント。番号を選ぶと、その区間の範囲を表示・編集します。` +
            (pending ? ' 保存していない変更があります。' : '');
    }
}

async function loadWaypointFiles() {
    try {
        const res = await fetch('/api/obstacle_zone/waypoint_files');
        const data = await res.json();
        wpFileSelect.innerHTML = '<option value="">(選んでください)</option>';
        for (const name of data.files || []) {
            const o = document.createElement('option');
            o.value = name;
            o.textContent = `${name}.json`;
            wpFileSelect.appendChild(o);
        }
    } catch (err) {
        setMessage(`ウェイポイントファイルの一覧を読めませんでした: ${err}`, true);
    }
}

async function loadWaypoints(name) {
    state.wp = { file: name, list: [], index: null, edits: {} };
    wpIndexSelect.innerHTML = '<option value="">(属性の設定を編集)</option>';
    if (name) {
        const res = await fetch(`/api/obstacle_zone/waypoints/${encodeURIComponent(name)}`);
        const data = await res.json();
        if (data.status !== 'success') {
            setMessage(data.message, true);
            state.wp.file = '';
        } else {
            state.wp.list = data.waypoints;
            for (const w of state.wp.list) {
                const o = document.createElement('option');
                o.value = String(w.index);
                o.textContent = waypointOptionText(w);
                wpIndexSelect.appendChild(o);
            }
        }
    }
    wpIndexSelect.disabled = !state.wp.file;
    buildFields();
    refresh();
}

wpFileSelect.addEventListener('change', async () => {
    if (waypointEditsPending() &&
        !confirm('保存していないウェイポイントの個別設定があります。破棄して別のファイルを開きますか？')) {
        wpFileSelect.value = state.wp.file;
        return;
    }
    await loadWaypoints(wpFileSelect.value);
});

wpIndexSelect.addEventListener('change', () => {
    if (wpIndexSelect.value === '') {
        state.wp.index = null;
    } else {
        state.wp.index = Number(wpIndexSelect.value);
        const t = state.wp.list[state.wp.index].type;
        state.type = state.types.includes(t) ? t : 'normal';  // 範囲はこのウェイポイントの属性の値に重ねる
    }
    buildFields();
    refresh();
});

document.getElementById('pt-wp-clear').addEventListener('click', () => {
    if (!inWaypointMode()) return;
    for (const f of Object.keys(waypointOverrides(state.wp.index))) setWaypointOverride(f, null);
    refresh();
});

document.getElementById('pt-wp-save').addEventListener('click', async () => {
    const button = document.getElementById('pt-wp-save');
    button.disabled = true;
    try {
        const res = await fetch(`/api/obstacle_zone/waypoints/${encodeURIComponent(state.wp.file)}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ edits: state.wp.edits }),
        });
        const data = await res.json();
        if (data.status !== 'success') {
            setMessage(data.message, true);
            return;
        }
        const keep = state.wp.index;
        await loadWaypoints(state.wp.file);
        if (keep !== null && state.wp.list[keep]) {
            wpIndexSelect.value = String(keep);
            wpIndexSelect.dispatchEvent(new Event('change'));
        }
        setMessage(data.message + '\n次にナビゲーションを起動したとき (waypoint_manager がファイルを読み直したとき) から使われます。');
    } catch (err) {
        setMessage(`保存できませんでした: ${err}`, true);
    } finally {
        refreshWaypointPanel();
    }
});
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => setView(b.dataset.view)));
document.getElementById('pt-mesh').addEventListener('change', e => {
    const name = e.target.value;
    const opt = e.target.selectedOptions[0];
    if (/\.obj$/i.test(name) && !confirm(`${opt.textContent} は大きいため、読み込みに時間がかかります。読み込みますか？`)) {
        return;
    }
    loadRobotMesh(name);
});
document.getElementById('pt-obstacle-on').addEventListener('change', refresh);
document.getElementById('pt-save').addEventListener('click', () => save(false));
document.getElementById('pt-save-apply').addEventListener('click', () => save(true));
document.getElementById('pt-reload').addEventListener('click', async () => {
    await loadConfig();
    refresh();
    coverage.setParams(state.reference, state.sensorOrigin, state.sensorRpy);
    coverage.rebuildAll();
    refreshCoverageNote();
    setMessage('ファイルから読み直しました。');
    loadLive();
});
window.addEventListener('beforeunload', e => {
    const changed = Object.keys(state.saved).some(k => state.values[k] !== state.saved[k]);
    if (changed || waypointEditsPending()) e.preventDefault();
});

// ---------------------------------------------------------------- YVT で見える範囲
const covHeight = document.getElementById('pt-cov-height');
const covHeightOut = document.getElementById('pt-cov-height-value');
const covPass = document.getElementById('pt-cov-pass');

function refreshCoverageNote() {
    // 横 1.0 m を追い抜く物が見え始める前後位置 (中心)。表示中の高さで計算する
    const h = parseFloat(covHeight.value);
    const lines = [];
    for (const y of [0.5, 1.0, 1.5]) {
        const x = coverage.firstVisibleX(y, h);
        lines.push(x === null ? `横 ${y.toFixed(1)} m: 見えない (前後 −6〜9 m の範囲で)`
                              : `横 ${y.toFixed(1)} m: 中心が x = ${x >= 0 ? '+' : ''}${x.toFixed(2)} m に来てから`);
    }
    covPass.textContent = `高さ ${h.toFixed(2)} m・長さ 0.6 m の物が追い抜くとき、見え始めるのは\n` + lines.join('\n');
}

function setupCoverage() {
    coverage.setParams(state.reference, state.sensorOrigin, state.sensorRpy);
    for (const [id, name] of [['pt-cov-fov', 'fov'], ['pt-cov-volume', 'volume'], ['pt-cov-floor', 'floor']]) {
        const el = document.getElementById(id);
        coverage.setVisible(name, el.checked);
        el.addEventListener('change', () => coverage.setVisible(name, el.checked));
    }
    const mode = document.getElementById('pt-cov-volume-mode');
    coverage.setVolumeMode(mode.value);
    mode.addEventListener('change', () => {
        coverage.setVolumeMode(mode.value);
        // 点群を選び直したら表示もオンにする
        const vol = document.getElementById('pt-cov-volume');
        if (!vol.checked) {
            vol.checked = true;
            coverage.setVisible('volume', true);
        }
    });
    // 「物の高さ」は床の検出マップと試しに置く障害物で共通
    const applyHeight = () => {
        const h = parseFloat(covHeight.value);
        covHeightOut.textContent = `${h.toFixed(2)} m`;
        coverage.setObjectHeight(h);
        state.obstacleHeight = h;
        refreshCoverageNote();
        if (z('path_clearance_left') !== undefined) refresh();
    };
    covHeight.addEventListener('input', applyHeight);
    applyHeight();
    if (!state.sensorOrigin) {
        covPass.textContent = 'URDF (wizurg.xacro) に yvt の位置が見つからないため、YVT の範囲は描けません。';
    }
}

// ---------------------------------------------------------------- 起動
(async () => {
    resize();
    try {
        await loadConfig();
    } catch (err) {
        setMessage(`設定を読み込めませんでした: ${err.message || err}`, true);
        return;
    }
    buildTypeTabs();
    buildFields();
    buildRobotOutline();
    setupCoverage();
    setView('oblique');
    refresh();
    loadRobotMesh(document.getElementById('pt-mesh').value);
    loadLive();
    loadWaypointFiles();
})();

renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
    labelRenderer.render(scene, camera);
});
