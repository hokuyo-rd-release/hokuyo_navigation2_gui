// =============================================================================
// region_selector.js — 3Dビューアで地面の四角い範囲をドラッグで選ぶ
// -----------------------------------------------------------------------------
// 選択中は、キャンバス上の左ドラッグをカメラ操作ではなく範囲選択として扱う。
// 範囲は地図座標の軸にそろった四角形 (THREE.Box2) で、高さ planeZ の平面上で決める。
// =============================================================================
import * as THREE from 'three';

const OUTLINE_COLOR = 0xffd000;
// 地面 (背景の写真) と重ならないよう、少し浮かせて描く
const OUTLINE_LIFT = 0.15;

export class RegionSelector {
    /**
     * @param {Object} options
     * @param {THREE.Scene} options.scene
     * @param {THREE.Camera} options.camera
     * @param {HTMLElement} options.domElement レンダラーのキャンバス
     * @param {() => number} [options.getPlaneZ] 範囲を決める平面の高さ
     * @param {(box: THREE.Box2|null, dragging: boolean) => void} [options.onChange]
     * @param {number} [options.color] 枠の色
     */
    constructor({ scene, camera, domElement, getPlaneZ = () => 0, onChange = () => {}, color = OUTLINE_COLOR }) {
        this.scene = scene;
        this.camera = camera;
        this.domElement = domElement;
        this.getPlaneZ = getPlaneZ;
        this.onChange = onChange;

        this.active = false;
        this.region = null;
        this._dragStart = null;
        this._suppressClick = false;
        this._raycaster = new THREE.Raycaster();

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(15), 3));
        this.outline = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, depthTest: false }));
        this.outline.name = 'region-selector-outline';
        this.outline.renderOrder = 1000;
        this.outline.visible = false;
        this.outline.frustumCulled = false;
        scene.add(this.outline);

        this._onPointerDown = this._onPointerDown.bind(this);
        this._onPointerMove = this._onPointerMove.bind(this);
        this._onPointerUp = this._onPointerUp.bind(this);
        this._onClick = this._onClick.bind(this);
        // OrbitControls やウェイポイント操作より先に受け取るため、window のキャプチャ段階で聞く
        window.addEventListener('pointerdown', this._onPointerDown, true);
        window.addEventListener('pointermove', this._onPointerMove, true);
        window.addEventListener('pointerup', this._onPointerUp, true);
        window.addEventListener('click', this._onClick, true);
    }

    /** ドラッグで範囲を選ぶ状態にする / やめる */
    setActive(active) {
        this.active = active;
        this._dragStart = null;
        this.domElement.style.cursor = active ? 'crosshair' : '';
    }

    setRegion(box) {
        this.region = box ? box.clone() : null;
        this._draw();
        this.onChange(this.region, false);
    }

    dispose() {
        window.removeEventListener('pointerdown', this._onPointerDown, true);
        window.removeEventListener('pointermove', this._onPointerMove, true);
        window.removeEventListener('pointerup', this._onPointerUp, true);
        window.removeEventListener('click', this._onClick, true);
        this.scene.remove(this.outline);
        this.outline.geometry.dispose();
        this.outline.material.dispose();
    }

    _groundPoint(event) {
        const rect = this.domElement.getBoundingClientRect();
        const ndc = new THREE.Vector2(
            ((event.clientX - rect.left) / rect.width) * 2 - 1,
            -((event.clientY - rect.top) / rect.height) * 2 + 1);
        this._raycaster.setFromCamera(ndc, this.camera);
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -this.getPlaneZ());
        const hit = new THREE.Vector3();
        return this._raycaster.ray.intersectPlane(plane, hit) ? new THREE.Vector2(hit.x, hit.y) : null;
    }

    _onPointerDown(event) {
        if (!this.active || event.button !== 0 || event.target !== this.domElement) return;
        const point = this._groundPoint(event);
        if (!point) return;
        event.stopPropagation();
        event.preventDefault();
        this._dragStart = point;
        this.region = new THREE.Box2(point.clone(), point.clone());
        this._draw();
        this.onChange(this.region, true);
    }

    _onPointerMove(event) {
        if (!this.active || !this._dragStart) return;
        event.stopPropagation();
        const point = this._groundPoint(event);
        if (!point) return;
        this.region = new THREE.Box2().setFromPoints([this._dragStart, point]);
        this._draw();
        this.onChange(this.region, true);
    }

    _onPointerUp(event) {
        if (!this.active || !this._dragStart) return;
        event.stopPropagation();
        this._dragStart = null;
        // 離したときの click で、ウェイポイントの選択などが動かないようにする
        this._suppressClick = true;
        this.setActive(false);
        this.onChange(this.region, false);
    }

    _onClick(event) {
        if (!this._suppressClick) return;
        this._suppressClick = false;
        event.stopPropagation();
    }

    _draw() {
        if (!this.region) {
            this.outline.visible = false;
            return;
        }
        const z = this.getPlaneZ() + OUTLINE_LIFT;
        const { min, max } = this.region;
        const position = this.outline.geometry.attributes.position;
        [[min.x, min.y], [max.x, min.y], [max.x, max.y], [min.x, max.y], [min.x, min.y]]
            .forEach(([x, y], i) => position.setXYZ(i, x, y, z));
        position.needsUpdate = true;
        this.outline.geometry.computeBoundingSphere();
        this.outline.visible = true;
    }
}
