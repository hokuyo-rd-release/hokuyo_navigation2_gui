// YVT で見える範囲の 3D 表示 (pass_through_config.js から使う)
//
// 座標は障害物検出設定の 3D 表示と同じ (x = 前, y = 左, z = 上, 原点はロボット直下の床面)。
// 表示するもの:
//   fov    : YVT の視野 (水平 sensor_horizontal_fov_deg・垂直 sensor_min_elevation_deg〜+35°) の境界面。
//            床に当たるところで切り、水平距離 max_range までを描く。
//   volume : 点群。volumeMode で 2 通り:
//            'counted' : obstacle_monitor が障害物点として数える空間だけ。
//                        視野内 かつ センサからの水平距離 min_range〜max_range かつ 床から min_obstacle_height〜
//                        max_obstacle_height かつ 自車体 (footprint + self_filter_margin) の外。
//                        obstacle_monitor.cpp の extractObstacles と同じ判定。
//            'fov'     : 視野全体 (床より上・水平距離 max_range まで)。数える部分を緑、
//                        数えない部分 (高さの範囲外・センサの近く・自車体) を視野の色で塗り分ける。
//   floor  : 高さ objectHeight の物が床のどこにあれば検出されるか (物のどこか 1 点でも検出空間に入るか)。
//            見えない場所は理由 (下方の死角 / 後方の死角 / 範囲外) で色分けする。
import * as THREE from 'three';

const D2R = Math.PI / 180;
// YVT-35LX の垂直視野の上限 (仕様)。obstacle_monitor のパラメータには無い
export const SENSOR_MAX_ELEVATION_DEG = 35.0;

const FLOOR_MAP = { x0: -6, x1: 9, y0: -8, y1: 8, res: 0.1 };
const VOLUME_STEP = 0.14;   // 検出空間の点の間隔 (水平) [m]
const VOLUME_STEP_Z = 0.1;  // 同 (高さ) [m]
// 視野全体は高さ 6 m 程度まであり点が多くなるため粗くする
const FOV_STEP = 0.2;
const FOV_STEP_Z = 0.15;

export class YvtCoverage {
    // colors: { fov, ok, low, rear, out } (THREE.Color)
    constructor(parent, colors) {
        this.colors = colors;
        this.groups = { fov: new THREE.Group(), volume: new THREE.Group(), floor: new THREE.Group() };
        for (const g of Object.values(this.groups)) parent.add(g);
        this.params = null;
        this.objectHeight = 0.4;
        this.volumeMode = 'counted';
        this.dirty = { fov: true, volume: true, floor: true };
    }

    // ref: pass_through_config の reference。origin / rpy: URDF の yvt (base_link 基準)
    setParams(ref, origin, rpy) {
        if (!origin) { this.params = null; return; }
        const baseH = -(ref.ground_z ?? -0.068);   // base_link の床からの高さ
        const rot = new THREE.Matrix4().makeRotationFromEuler(
            new THREE.Euler(rpy?.[0] || 0, rpy?.[1] || 0, rpy?.[2] || 0, 'ZYX'));  // URDF の rpy
        const margin = ref.self_filter_margin ?? 0.05;
        this.params = {
            sx: origin[0], sy: origin[1], sz: origin[2] + baseH,
            rot, inv: rot.clone().transpose(),
            minR: ref.min_range ?? 0.3, maxR: ref.max_range ?? 8.0,
            minH: ref.min_obstacle_height ?? 0.10, maxH: ref.max_obstacle_height ?? 1.5,
            azHalf: 0.5 * (ref.sensor_horizontal_fov_deg ?? 210) * D2R,
            elMin: (ref.sensor_min_elevation_deg ?? -5) * D2R, elMax: SENSOR_MAX_ELEVATION_DEG * D2R,
            self: {
                x0: (ref.footprint_min_x ?? -0.535) - margin, x1: (ref.footprint_max_x ?? 0.195) + margin,
                hw: (ref.footprint_half_width ?? 0.29) + margin,
            },
            body: {
                x0: ref.footprint_min_x ?? -0.535, x1: ref.footprint_max_x ?? 0.195,
                hw: ref.footprint_half_width ?? 0.29,
            },
        };
        this.dirty = { fov: true, volume: true, floor: true };
    }

    setVisible(name, on) {
        this.groups[name].visible = on;
        if (on && this.dirty[name]) this.rebuild(name);
    }

    setVolumeMode(mode) {
        this.volumeMode = mode;
        this.dirty.volume = true;
        if (this.groups.volume.visible) this.rebuild('volume');
    }

    setObjectHeight(h) {
        this.objectHeight = h;
        this.dirty.floor = true;
        if (this.groups.floor.visible) this.rebuild('floor');
    }

    rebuildAll() {
        for (const name of Object.keys(this.groups)) {
            this.dirty[name] = true;
            if (this.groups[name].visible) this.rebuild(name);
        }
    }

    rebuild(name) {
        const g = this.groups[name];
        disposeGroup(g);
        this.dirty[name] = false;
        if (!this.params) return;
        if (name === 'fov') this.buildFov(g);
        else if (name === 'volume') this.buildVolume(g);
        else if (name === 'floor') this.buildFloor(g);
    }

    // 床面基準の点 (x, y, z) を分類する: 'ok' | 'low' | 'high' | 'rear' | 'range'
    classify(x, y, z) {
        const p = this.params;
        if (z < p.minH || z > p.maxH) return 'range';
        if (x >= p.self.x0 && x <= p.self.x1 && Math.abs(y) <= p.self.hw) return 'range';
        const v = new THREE.Vector3(x - p.sx, y - p.sy, z - p.sz).applyMatrix4(p.inv);  // センサ座標
        const r = Math.hypot(v.x, v.y);
        if (r < p.minR || r > p.maxR) return 'range';
        if (Math.abs(Math.atan2(v.y, v.x)) > p.azHalf) return 'rear';
        const el = Math.atan2(v.z, r);
        if (el < p.elMin) return 'low';
        if (el > p.elMax) return 'high';
        return 'ok';
    }

    // センサ座標の向き (az, el) のレイの終点。床に当たるところ・水平距離 maxR で止める
    rayEnd(az, el, scale = 1) {
        const p = this.params;
        const d = new THREE.Vector3(Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el))
            .applyMatrix4(p.rot);
        let t = p.maxR / Math.max(0.05, Math.cos(el));
        if (d.z < -1e-6) t = Math.min(t, -p.sz / d.z);
        t *= scale;
        return new THREE.Vector3(p.sx + d.x * t, p.sy + d.y * t, p.sz + d.z * t);
    }

    buildFov(g) {
        const p = this.params;
        const c = this.colors.fov;
        const lerp = (a, b, t) => a + (b - a) * t;
        const [a0, a1, e0, e1] = [-p.azHalf, p.azHalf, p.elMin, p.elMax];
        // 下端 (-5°)・上端 (+35°) の円錐面、外周、左右端の扇
        g.add(surface(84, 24, (u, v) => this.rayEnd(lerp(a0, a1, u), e0, v), c, 0.12));
        g.add(surface(84, 24, (u, v) => this.rayEnd(lerp(a0, a1, u), e1, v), c, 0.06));
        g.add(surface(84, 20, (u, v) => this.rayEnd(lerp(a0, a1, u), lerp(e0, e1, v)), c, 0.05));
        for (const az of [a0, a1]) {
            g.add(surface(20, 20, (u, v) => this.rayEnd(az, lerp(e0, e1, u), v), c, 0.09));
        }
        // 端のレイと、下端 (-5°) が床に当たる線
        const origin = new THREE.Vector3(p.sx, p.sy, p.sz);
        const lineMat = new THREE.LineBasicMaterial({ color: c, transparent: true, opacity: 0.9 });
        for (const az of [a0, 0, a1]) {
            for (const el of [e0, e1]) {
                g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([origin, this.rayEnd(az, el)]), lineMat));
            }
        }
        const pts = [];
        for (let i = 0; i <= 84; i++) pts.push(this.rayEnd(lerp(a0, a1, i / 84), e0).setZ(0.004));
        g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: c })));
    }

    // 床より上で、センサの視野内・水平距離 max_range 以内か (高さ・自車体・min_range の除外はしない)
    inFov(x, y, z) {
        const p = this.params;
        if (z < 0) return false;
        const v = new THREE.Vector3(x - p.sx, y - p.sy, z - p.sz).applyMatrix4(p.inv);
        const r = Math.hypot(v.x, v.y);
        if (r > p.maxR || Math.hypot(r, v.z) < 0.05) return false;
        if (Math.abs(Math.atan2(v.y, v.x)) > p.azHalf) return false;
        const el = Math.atan2(v.z, r);
        return el >= p.elMin && el <= p.elMax;
    }

    buildVolume(g) {
        const p = this.params;
        const pos = [];
        const cols = [];
        const R = p.maxR + 0.5;
        // 格子のままだと縞模様 (モアレ) が出るため、描く位置だけ少しずらす (判定は格子点で行う)。
        // 毎回同じ見た目になるよう、乱数は種を固定する
        let seed = 12345;
        const rnd = () => {
            seed = (seed * 1664525 + 1013904223) >>> 0;
            return seed / 4294967296 - 0.5;
        };
        const push = (x, y, z, sxy, sz) => pos.push(x + rnd() * sxy, y + rnd() * sxy, z + rnd() * sz);
        if (this.volumeMode === 'fov') {
            // 視野全体。上端は水平距離 max_range で仰角の上限に達する高さ
            const top = p.sz + p.maxR * Math.tan(p.elMax) + 0.2;
            const used = this.colors.ok;
            const unused = this.colors.fov;
            for (let x = p.sx - R; x <= p.sx + R; x += FOV_STEP) {
                for (let y = p.sy - R; y <= p.sy + R; y += FOV_STEP) {
                    for (let z = FOV_STEP_Z / 2; z <= top; z += FOV_STEP_Z) {
                        if (!this.inFov(x, y, z)) continue;
                        push(x, y, z, FOV_STEP, FOV_STEP_Z);
                        const c = this.classify(x, y, z) === 'ok' ? used : unused;
                        cols.push(c.r, c.g, c.b);
                    }
                }
            }
        } else {
            const lo = this.colors.ok;
            const hi = this.colors.fov;
            for (let x = p.sx - R; x <= p.sx + R; x += VOLUME_STEP) {
                for (let y = p.sy - R; y <= p.sy + R; y += VOLUME_STEP) {
                    for (let z = p.minH; z <= p.maxH + 1e-6; z += VOLUME_STEP_Z) {
                        if (this.classify(x, y, z) !== 'ok') continue;
                        push(x, y, z, VOLUME_STEP, VOLUME_STEP_Z);
                        const c = lo.clone().lerp(hi, (z - p.minH) / Math.max(1e-6, p.maxH - p.minH));
                        cols.push(c.r, c.g, c.b);
                    }
                }
            }
        }
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
        geo.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
        g.add(new THREE.Points(geo, new THREE.PointsMaterial({
            size: 0.035, vertexColors: true, transparent: true, opacity: 0.5, depthWrite: false,
        })));
    }

    // 高さ h の物 (床から h まで) が (x, y) にあるときの見え方
    floorReason(x, y, h) {
        const p = this.params;
        if (x >= p.body.x0 && x <= p.body.x1 && Math.abs(y) <= p.body.hw) return 'body';
        const top = Math.min(h, p.maxH);
        if (top < p.minH) return 'low';
        let low = false;
        let rear = false;
        for (let z = p.minH; z <= top + 1e-6; z += 0.025) {
            const r = this.classify(x, y, z);
            if (r === 'ok') return 'ok';
            if (r === 'low') low = true;
            if (r === 'rear') rear = true;
        }
        return rear ? 'rear' : (low ? 'low' : 'range');
    }

    buildFloor(g) {
        const f = FLOOR_MAP;
        const nx = Math.round((f.x1 - f.x0) / f.res);
        const ny = Math.round((f.y1 - f.y0) / f.res);
        const canvas = document.createElement('canvas');
        canvas.width = nx;
        canvas.height = ny;
        const ctx = canvas.getContext('2d');
        const img = ctx.createImageData(nx, ny);
        const pal = { ok: this.colors.ok, low: this.colors.low, rear: this.colors.rear,
                      range: this.colors.out, body: this.colors.out };
        // 帯 (不感帯・検出帯) が読めるよう薄めにする
        const alpha = { ok: 90, low: 110, rear: 80, range: 30, body: 0 };
        for (let j = 0; j < ny; j++) {
            for (let i = 0; i < nx; i++) {
                const x = f.x0 + (i + 0.5) * f.res;
                const y = f.y1 - (j + 0.5) * f.res;  // 画像の上 = 左 (y の大きい側)
                const k = this.floorReason(x, y, this.objectHeight);
                const c = pal[k];
                const o = (j * nx + i) * 4;
                img.data[o] = c.r * 255;
                img.data[o + 1] = c.g * 255;
                img.data[o + 2] = c.b * 255;
                img.data[o + 3] = alpha[k];
            }
        }
        ctx.putImageData(img, 0, 0);
        const tex = new THREE.CanvasTexture(canvas);
        tex.magFilter = THREE.NearestFilter;
        const plane = new THREE.Mesh(new THREE.PlaneGeometry(f.x1 - f.x0, f.y1 - f.y0),
            new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
        plane.position.set((f.x0 + f.x1) / 2, (f.y0 + f.y1) / 2, 0.001);  // 帯 (z 0.003〜) より下
        g.add(plane);
    }

    // 高さ h・横 y を通る長さ len・幅 wid の物が、前後位置いくつから見え始めるか (無ければ null)
    firstVisibleX(y, h, len = 0.6, wid = 0.45) {
        if (!this.params) return null;
        for (let x = -6; x <= 9; x += 0.05) {
            for (let yy = y - wid / 2; yy <= y + wid / 2 + 1e-6; yy += 0.05) {
                for (const xx of [x - len / 2, x + len / 2]) {
                    if (this.floorReason(xx, yy, h) === 'ok') return x;
                }
            }
        }
        return null;
    }
}

function surface(nu, nv, f, color, opacity) {
    const pos = [];
    const idx = [];
    for (let i = 0; i <= nu; i++) {
        for (let j = 0; j <= nv; j++) {
            const v = f(i / nu, j / nv);
            pos.push(v.x, v.y, v.z);
        }
    }
    for (let i = 0; i < nu; i++) {
        for (let j = 0; j < nv; j++) {
            const a = i * (nv + 1) + j;
            const b = a + nv + 1;
            idx.push(a, b, a + 1, b, b + 1, a + 1);
        }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setIndex(idx);
    return new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
        color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false,
    }));
}

function disposeGroup(group) {
    for (const obj of [...group.children]) {
        group.remove(obj);
        obj.traverse(o => {
            if (o.geometry) o.geometry.dispose();
            if (o.material) {
                if (o.material.map) o.material.map.dispose();
                o.material.dispose();
            }
        });
    }
}
