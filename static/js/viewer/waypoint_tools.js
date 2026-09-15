// =============================================================================
// waypoint_tools.js — ウェイポイントをまとめて扱うためのドラッグ操作
// -----------------------------------------------------------------------------
// ScreenRectSelector: 画面上をドラッグして四角く囲む (囲んだウェイポイントをまとめて削除する用)
// FreehandStroke:     地図の上をドラッグして線を引く (線に沿ってウェイポイントを並べる用)
// resamplePolyline:   折れ線を一定の間隔の点に並べ直す
//
// どちらも有効な間だけ、キャンバス上の左ドラッグをカメラ操作やウェイポイントの移動より先に受け取る
// (window のキャプチャ段階で聞き、伝播を止める)。
// =============================================================================
import * as THREE from 'three';

/** ドラッグが終わった直後の click を 1 回だけ止める (ウェイポイントの選択などが動かないように) */
class DragToolBase {
    constructor(domElement) {
        this.domElement = domElement;
        this.active = false;
        this._dragging = false;
        this._suppressClick = false;
        this._onPointerDown = this._onPointerDown.bind(this);
        this._onPointerMove = this._onPointerMove.bind(this);
        this._onPointerUp = this._onPointerUp.bind(this);
        this._onClick = this._onClick.bind(this);
        window.addEventListener('pointerdown', this._onPointerDown, true);
        window.addEventListener('pointermove', this._onPointerMove, true);
        window.addEventListener('pointerup', this._onPointerUp, true);
        window.addEventListener('click', this._onClick, true);
    }

    setActive(active) {
        this.active = active;
        if (!active && this._dragging) this._cancelDrag();
        this._dragging = false;
        this.domElement.style.cursor = active ? 'crosshair' : '';
    }

    dispose() {
        this.setActive(false);
        window.removeEventListener('pointerdown', this._onPointerDown, true);
        window.removeEventListener('pointermove', this._onPointerMove, true);
        window.removeEventListener('pointerup', this._onPointerUp, true);
        window.removeEventListener('click', this._onClick, true);
    }

    _onPointerDown(event) {
        if (!this.active || event.button !== 0 || event.target !== this.domElement) return;
        if (!this._startDrag(event)) return;
        event.stopPropagation();
        event.preventDefault();
        this._dragging = true;
    }

    _onPointerMove(event) {
        if (!this.active || !this._dragging) return;
        event.stopPropagation();
        this._moveDrag(event);
    }

    _onPointerUp(event) {
        if (!this.active || !this._dragging) return;
        event.stopPropagation();
        this._dragging = false;
        this._suppressClick = event.target === this.domElement;
        this.setActive(false);
        this._endDrag(event);
    }

    _onClick(event) {
        if (!this._suppressClick) return;
        this._suppressClick = false;
        event.stopPropagation();
    }

    _startDrag() { return true; }
    _moveDrag() {}
    _endDrag() {}
    _cancelDrag() {}
}

/** 画面上の四角い範囲を選ぶ。onSelect には client 座標の {left, top, right, bottom} を渡す。 */
export class ScreenRectSelector extends DragToolBase {
    constructor({ domElement, onSelect = () => {} }) {
        super(domElement);
        this.onSelect = onSelect;
        this._start = null;
        this.box = document.createElement('div');
        this.box.className = 'screen-rect-selector';
        this.box.hidden = true;
        document.body.appendChild(this.box);
    }

    dispose() {
        super.dispose();
        this.box.remove();
    }

    _rect(event) {
        return {
            left: Math.min(this._start.x, event.clientX),
            top: Math.min(this._start.y, event.clientY),
            right: Math.max(this._start.x, event.clientX),
            bottom: Math.max(this._start.y, event.clientY),
        };
    }

    _startDrag(event) {
        this._start = { x: event.clientX, y: event.clientY };
        this._draw(this._rect(event));
        this.box.hidden = false;
        return true;
    }

    _moveDrag(event) {
        this._draw(this._rect(event));
    }

    _endDrag(event) {
        const rect = this._rect(event);
        this.box.hidden = true;
        this._start = null;
        this.onSelect(rect);
    }

    _cancelDrag() {
        this.box.hidden = true;
        this._start = null;
    }

    _draw(rect) {
        Object.assign(this.box.style, {
            left: `${rect.left}px`,
            top: `${rect.top}px`,
            width: `${rect.right - rect.left}px`,
            height: `${rect.bottom - rect.top}px`,
        });
    }
}

/**
 * 地図の上に線を引く。pickPoint(event) は地図座標の {x, y} (取れなければ null) を返す。
 * onFinish には引いた線の点 ([{x, y}, ...]) を渡す。
 */
export class FreehandStroke extends DragToolBase {
    constructor({ domElement, scene, pickPoint, onFinish = () => {}, color = 0xff00aa, z = 0.05 }) {
        super(domElement);
        this.scene = scene;
        this.pickPoint = pickPoint;
        this.onFinish = onFinish;
        this.z = z;
        this.points = [];
        this.line = new THREE.Line(new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true }));
        this.line.renderOrder = 1000;
        this.line.frustumCulled = false;
        this.line.visible = false;
        scene.add(this.line);
    }

    dispose() {
        super.dispose();
        this.scene.remove(this.line);
        this.line.geometry.dispose();
        this.line.material.dispose();
    }

    _add(event) {
        const point = this.pickPoint(event);
        if (!point) return;
        const last = this.points[this.points.length - 1];
        // 細かすぎる点は捨てる (マウスの微小な動きで点が増えないように)
        if (last && Math.hypot(point.x - last.x, point.y - last.y) < 0.05) return;
        this.points.push({ x: point.x, y: point.y });
        this.line.geometry.dispose();
        this.line.geometry = new THREE.BufferGeometry().setFromPoints(
            this.points.map(p => new THREE.Vector3(p.x, p.y, this.z)));
    }

    _startDrag(event) {
        this.points = [];
        this._add(event);
        if (this.points.length === 0) return false;   // 地図の外から引き始めた
        this.line.visible = true;
        return true;
    }

    _moveDrag(event) {
        this._add(event);
    }

    _endDrag(event) {
        this._add(event);
        const points = this.points;
        this._cancelDrag();
        this.onFinish(points);
    }

    _cancelDrag() {
        this.points = [];
        this.line.visible = false;
    }
}

/**
 * 折れ線 ([{x, y}, ...]) を、始点から interval ごとの点に並べ直す。
 * 終点までの残りが interval の半分以上あれば、終点も加える。
 */
export function resamplePolyline(points, interval) {
    if (points.length === 0 || !(interval > 0)) return [];
    const result = [{ x: points[0].x, y: points[0].y }];
    let carried = 0;   // 前の点から、いまの区間の始まりまでに進んだ距離
    for (let i = 1; i < points.length; i++) {
        const a = points[i - 1];
        const b = points[i];
        const length = Math.hypot(b.x - a.x, b.y - a.y);
        if (length === 0) continue;
        let along = interval - carried;
        while (along <= length) {
            const t = along / length;
            result.push({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
            along += interval;
        }
        carried = length - (along - interval);
    }
    const last = points[points.length - 1];
    const tail = result[result.length - 1];
    if (Math.hypot(last.x - tail.x, last.y - tail.y) >= interval * 0.5) {
        result.push({ x: last.x, y: last.y });
    }
    return result;
}
