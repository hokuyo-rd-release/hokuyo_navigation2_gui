// =============================================================================
// google_background.js — 3Dビューアの背景に Google の航空写真 / 3D を表示する
// -----------------------------------------------------------------------------
//  - '2d': Map Tiles API の衛星写真タイルを、地図座標に合わせた平面として並べる。
//          見えている範囲と解像度 (ズーム) は、カメラの拡大縮小・移動に合わせて選び直す。
//  - '3d': Photorealistic 3D Tiles (Google Earth と同じ立体データ) を重ねる
// どちらも地図座標 (map フレーム) に変換して置くので、点群・PGM・ウェイポイントは
// そのままの座標で重なる。座標の対応は geo_frame.js の MapGeoFrame が決める。
//
// Google Maps Platform の API キー (Map Tiles API を有効化したもの) が必要。
// 利用規約により、表示中は Google の表記と出典 (copyright) を画面に出す。
// =============================================================================
import * as THREE from 'three';
import { tileToLatLon, latLonToTile } from './geo_frame.js';

const TILE_API = 'https://tile.googleapis.com';
const MAX_2D_ZOOM = 20;
const MIN_2D_ZOOM = 3;
// 1つの視点で読み込む枚数の上限。注視点のまわり (細かい写真) と、その外側の見えている範囲
// (粗い写真) に分けて読み、それぞれ上限を超えるときはズームを下げて収める。
const MAX_2D_NEAR_TILES = 80;
const MAX_2D_FAR_TILES = 48;
// 細かい写真を並べる範囲 (注視点までの距離の倍数)
const NEAR_RANGE_FACTOR = 1.5;
// 地平線の方まで読み込まないよう、カメラから地面上でこの距離より先は含めない (注視点までの距離の倍数)
const FAR_RANGE_FACTOR = 6;
// 画面の1画素あたり、写真の何画素を目安にするか (1 = 等倍)。
const TILE_PIXELS_PER_SCREEN_PIXEL = 1.0;
// 読み込んだタイルをメモリに残しておく枚数。拡大縮小を戻したときに読み直さないため (ページを閉じると消える)。
const MAX_2D_CACHED_TILES = 600;
// 視点が止まってからタイルを選び直すまでの時間 [ms]。動かしている最中に何度も読み込まないため。
const VIEW_SETTLE_MS = 400;
// Web メルカトルで、赤道におけるズーム 0 の1画素の大きさ [m]
const WEB_MERCATOR_M_PER_PX_Z0 = 156543.03392;
// 背景を表示している間のカメラの描画距離と、ズームアウトできる最大距離
const CAMERA_FAR_BACKGROUND = 50000;
const ORBIT_MAX_DISTANCE_BACKGROUND = 5000;

export const DEFAULT_HEIGHT_OFFSET = { '2d': -0.05, '3d': 0.0 };

export class GoogleBackground {
    /**
     * @param {Object} options
     * @param {THREE.Scene} options.scene
     * @param {THREE.PerspectiveCamera} options.camera
     * @param {THREE.WebGLRenderer} options.renderer
     * @param {Object} [options.controls] OrbitControls (背景の表示中だけ最大距離を広げる)
     * @param {string} options.dracoDecoderPath three の examples/jsm/libs/draco/gltf/ の URL
     * @param {HTMLElement} options.attributionElement 出典を表示する要素
     * @param {(message: string, level: 'info'|'warning'|'error') => void} [options.onStatus]
     */
    constructor({ scene, camera, renderer, controls = null, dracoDecoderPath,
                  attributionElement, onStatus = () => {} }) {
        this.scene = scene;
        this.camera = camera;
        this.renderer = renderer;
        this.controls = controls;
        this.dracoDecoderPath = dracoDecoderPath;
        this.attributionElement = attributionElement;
        this.onStatus = onStatus;

        this.apiKey = '';
        this.mode = 'none';
        this.frame = null;
        this.heightOffset = { ...DEFAULT_HEIGHT_OFFSET };
        this.opacity2D = 1.0;

        this.group2D = new THREE.Group();
        this.group2D.name = 'google-background-2d';
        this._session2D = null;
        this._sessionPromise2D = null;
        // 読み込んだ (読み込み中の) タイル。キーは "z/x/y"
        this._tiles2D = new Map();
        // いまの視点で表示したいタイルのキー
        this._wanted2D = new Set();
        this._view2D = null;
        this._view2DSeq = 0;
        this._zoomCap2D = MAX_2D_ZOOM;
        this._sessionRetriedAt = 0;
        this._attribution2D = '';
        this._last2DStatus = '';
        this._lastCameraMatrix = new THREE.Matrix4();
        this._lastProjectionMatrix = new THREE.Matrix4();
        this._viewDirty = false;
        this._viewChangedAt = 0;

        this.tiles = null;
        // 3D Tiles を背景として描き、点群・2D 地図・ウェイポイントを常に手前に見せるか (render 参照)
        this.mapOnTop3D = true;
        // 高さ合わせの間だけ使う、真上から見下ろすカメラ (setProbe 参照)
        this._probeCamera = null;
        this._clearColor = new THREE.Color();
        this._build3DSeq = 0;
        this._savedCamera = null;
        this._attributionTimer = 0;
    }

    get hasApiKey() {
        return this.apiKey.length > 0;
    }

    setApiKey(key) {
        this.apiKey = (key || '').trim();
        this._session2D = null;
    }

    /** 地図の座標系を差し替える。表示中なら新しい位置で置き直す。 */
    setFrame(frame) {
        this.frame = frame;
        // タイルの頂点は地図座標で作っているので、座標系が変わったら作り直す
        this._clear2D();
        this._zoomCap2D = MAX_2D_ZOOM;
        if (!frame) {
            this._disable3D();
            this._updateAttribution();
            return;
        }
        if (this.mode === '2d') this._start2D();
        if (this.mode === '3d') this._apply3DMatrix();
    }

    getHeightOffset(mode = this.mode) {
        return this.heightOffset[mode] ?? 0;
    }

    setHeightOffset(value, mode = this.mode) {
        if (!(mode in this.heightOffset) || !Number.isFinite(value)) return;
        this.heightOffset[mode] = value;
        if (mode === '2d') this.group2D.position.z = value;
        if (mode === '3d') this._apply3DMatrix();
    }

    setOpacity2D(value) {
        this.opacity2D = value;
        this._tiles2D.forEach(tile => {
            if (tile.mesh) tile.mesh.material.opacity = value;
        });
    }

    /** 表示の種類を切り替える。'none' | '2d' | '3d' */
    async setMode(mode) {
        if (mode === this.mode) return true;
        if (mode !== 'none' && !this.hasApiKey) {
            this.onStatus('Google Maps の API キーが設定されていません。', 'error');
            return false;
        }
        if (mode !== 'none' && !this.frame) {
            this.onStatus('このマップの原点 (緯度経度) が決まっていないため表示できません。', 'error');
            return false;
        }
        this.mode = mode;
        if (mode !== '2d') this._clear2D();
        if (mode !== '3d') this._disable3D();
        if (mode === 'none') {
            this._restoreCamera();
            this.onStatus('', 'info');
        } else {
            this._expandCamera();
        }
        if (mode === '2d') this._start2D();
        if (mode === '3d') await this._enable3D();
        this._updateAttribution();
        return true;
    }

    /** 毎フレーム、描画の直前に呼ぶ。 */
    update() {
        if (this.mode === '2d' && this.frame) {
            this._track2DView();
        }
        if (this.tiles) {
            this.camera.updateMatrixWorld();
            this.tiles.setResolutionFromRenderer(this.camera, this.renderer);
            this.tiles.update();
            const now = performance.now();
            if (now - this._attributionTimer > 1000) {
                this._attributionTimer = now;
                this._updateAttribution();
            }
        }
    }

    /**
     * シーンを描く (animate から renderer.render の代わりに呼ぶ)。
     * 3D Tiles を表示していて mapOnTop3D のときは、3D Tiles だけを先に描いてから深度を消し、
     * 残り (点群・2D 地図・ウェイポイントなど) を重ねる。建物や地面の起伏に地図やウェイポイントが
     * 隠れず、上から見ても編集できる。地図どうしの前後関係 (点群と PGM など) はこれまでどおり。
     */
    render(renderer, scene, camera) {
        const group = this.tiles ? this.tiles.group : null;
        if (this.mode !== '3d' || !this.mapOnTop3D || !group || group.parent !== scene) {
            renderer.render(scene, camera);
            return;
        }
        const background = scene.background;
        const autoClear = renderer.autoClear;
        renderer.getClearColor(this._clearColor);
        const clearAlpha = renderer.getClearAlpha();
        try {
            // 1. 背景色で塗り、3D Tiles だけを描く
            if (background && background.isColor) renderer.setClearColor(background, 1);
            renderer.clear();
            renderer.autoClear = false;
            renderer.render(group, camera);
            // 2. 深度を消して、3D Tiles 以外を重ねる。
            //    背景が色だと three.js は毎回塗りつぶすので、その間だけ背景を外す。
            renderer.clearDepth();
            group.visible = false;
            scene.background = null;
            renderer.render(scene, camera);
        } finally {
            group.visible = true;
            scene.background = background;
            renderer.autoClear = autoClear;
            renderer.setClearColor(this._clearColor, clearAlpha);
        }
    }

    /** 3D Tiles の読み込みが一段落していて、表示中のタイルがあるか。 */
    is3DIdle() {
        if (!this.tiles) return false;
        const { downloading, parsing } = this.tiles.stats;
        return downloading === 0 && parsing === 0 && this.tiles.visibleTiles.size > 0;
    }

    /**
     * 3D 表示中、地図座標 (x, y) の真上から見た 3D Tiles の表面の高さ (地図座標の z)。
     * maxError: この詳細度 (タイルの geometricError [m]) より粗いタイルは使わない。
     *   読み込み始めは地球全体を覆う粗いタイル (誤差数十〜数百 km) しか無く、その表面は
     *   実際の地面から数 km ずれているため、高さ合わせに使うと大きく外れる。
     */
    surfaceHeightAt(x, y, { maxError = Infinity } = {}) {
        if (!this.tiles) return null;
        const raycaster = new THREE.Raycaster(
            new THREE.Vector3(x, y, 5000), new THREE.Vector3(0, 0, -1), 0, 20000);
        const hits = raycaster.intersectObject(this.tiles.group, true);
        if (hits.length === 0) return null;
        if (maxError === Infinity) return hits[0].point.z;
        const errors = new Map();
        for (const tile of this.tiles.visibleTiles) {
            if (tile.cached && tile.cached.scene) errors.set(tile.cached.scene, tile.geometricError);
        }
        for (const hit of hits) {
            let object = hit.object;
            while (object && !errors.has(object)) object = object.parent;
            if (object && errors.get(object) <= maxError) return hit.point.z;
        }
        return null;
    }

    /**
     * 地図座標 (x, y) のまわりのタイルを、表示のカメラと関係なく細かく読み込ませる。
     * 原点の高度が 3D Tiles の高さと大きくずれていると、カメラが地表の下に入って真下のタイルが
     * 視野に入らず、粗いタイルのまま読み込みが止まる。高い所から狭い画角で真下を見るカメラを足して、
     * 高さ合わせに使える細かいタイル (誤差数 m) を読み込む。合わせ終わったら clearProbe で外す。
     */
    setProbe(x, y) {
        if (!this.tiles) return;
        if (!this._probeCamera) {
            // 高さ 3000 m から画角 4° で見下ろすと、真下の約 200 m 四方が写る
            this._probeCamera = new THREE.PerspectiveCamera(4, 1, 100, 10000);
            this._probeCamera.up.set(0, 1, 0);
            this.tiles.setCamera(this._probeCamera);
            this.tiles.setResolution(this._probeCamera, 256, 256);
        }
        const camera = this._probeCamera;
        camera.position.set(x, y, 3000);
        camera.lookAt(x, y, 0);
        camera.updateMatrixWorld(true);
        camera.updateProjectionMatrix();
    }

    clearProbe() {
        if (this._probeCamera && this.tiles) this.tiles.deleteCamera(this._probeCamera);
        this._probeCamera = null;
    }

    /**
     * (x, y) の真上に読み込み済みのタイルが無いとき、まわり (半径 25m〜maxRadius) で最も近い表面の高さを探す。
     * 高さが大きくずれてカメラが表面の下にあると、真上のタイルが視野に入らず読み込まれないため、
     * まず近くのタイルでおおまかに合わせるのに使う。見つからなければ null。
     */
    surfaceHeightNear(x, y, maxRadius = 2000, { maxError = Infinity } = {}) {
        const direct = this.surfaceHeightAt(x, y, { maxError });
        if (direct !== null) return { z: direct, distance: 0 };
        for (let radius = 25; radius <= maxRadius; radius *= 2) {
            for (let i = 0; i < 12; i++) {
                const angle = i / 12 * Math.PI * 2;
                const z = this.surfaceHeightAt(x + radius * Math.cos(angle), y + radius * Math.sin(angle), { maxError });
                if (z !== null) return { z, distance: radius };
            }
        }
        return null;
    }

    dispose() {
        this._clear2D();
        this._disable3D();
        this._restoreCamera();
    }

    // -------------------------------------------------------------------------
    // カメラ (背景を遠くまで見られるように、表示中だけ描画距離とズームアウトの上限を広げる)
    // -------------------------------------------------------------------------

    _expandCamera() {
        if (this._savedCamera) return;
        this._savedCamera = {
            far: this.camera.far,
            maxDistance: this.controls ? this.controls.maxDistance : null,
        };
        this.camera.far = Math.max(this.camera.far, CAMERA_FAR_BACKGROUND);
        this.camera.updateProjectionMatrix();
        if (this.controls) {
            this.controls.maxDistance = Math.max(this.controls.maxDistance, ORBIT_MAX_DISTANCE_BACKGROUND);
        }
    }

    _restoreCamera() {
        if (!this._savedCamera) return;
        this.camera.far = this._savedCamera.far;
        this.camera.updateProjectionMatrix();
        if (this.controls && this._savedCamera.maxDistance !== null) {
            this.controls.maxDistance = this._savedCamera.maxDistance;
            // 広げた距離まで引いていた場合は、元の上限まで戻す
            this.controls.update();
        }
        this._savedCamera = null;
    }

    // -------------------------------------------------------------------------
    // 2D 衛星写真 (Map Tiles API)
    // -------------------------------------------------------------------------

    async _ensure2DSession() {
        if (this._session2D && Date.now() < this._session2D.expiresAt - 60 * 1000) {
            return this._session2D.token;
        }
        // 視点の更新が続けて来ても、セッションは1回だけ作る
        if (!this._sessionPromise2D) {
            this._sessionPromise2D = this._create2DSession().finally(() => {
                this._sessionPromise2D = null;
            });
        }
        return this._sessionPromise2D;
    }

    async _create2DSession() {
        const res = await fetch(`${TILE_API}/v1/createSession?key=${encodeURIComponent(this.apiKey)}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mapType: 'satellite', language: 'ja-JP', region: 'JP' }),
        });
        if (!res.ok) {
            throw new Error(`衛星写真のセッションを作成できません (HTTP ${res.status}): ${await googleErrorMessage(res)}`);
        }
        const json = await res.json();
        const expirySec = Number(json.expiry);
        this._session2D = {
            token: json.session,
            expiresAt: Number.isFinite(expirySec) ? expirySec * 1000 : Date.now() + 3600 * 1000,
        };
        return this._session2D.token;
    }

    /** 2D の表示を始める。最初の視点ですぐに読み込む。 */
    _start2D() {
        this.group2D.position.z = this.heightOffset['2d'];
        if (!this.group2D.parent) this.scene.add(this.group2D);
        this._viewDirty = true;
        this._viewChangedAt = -Infinity;
    }

    _clear2D() {
        this._view2DSeq++;
        this._tiles2D.forEach(tile => this._dispose2DTile(tile));
        this._tiles2D.clear();
        this._wanted2D.clear();
        this._view2D = null;
        this._last2DStatus = '';
        if (this.group2D.parent) this.scene.remove(this.group2D);
        this._attribution2D = '';
    }

    _dispose2DTile(tile) {
        tile.disposed = true;
        if (!tile.mesh) return;
        this.group2D.remove(tile.mesh);
        tile.mesh.geometry.dispose();
        tile.mesh.material.map.dispose();
        tile.mesh.material.dispose();
        tile.mesh = null;
    }

    /** カメラが動いたら、止まってから少し待って表示するタイルを選び直す。 */
    _track2DView() {
        this.camera.updateMatrixWorld();
        // OrbitControls は止まっていても計算の丸めで行列がごくわずかに変わるので、小さな差は動いたとみなさない
        const moved = !matricesNearlyEqual(this.camera.matrixWorld, this._lastCameraMatrix)
            || !matricesNearlyEqual(this.camera.projectionMatrix, this._lastProjectionMatrix);
        const now = performance.now();
        if (moved) {
            this._lastCameraMatrix.copy(this.camera.matrixWorld);
            this._lastProjectionMatrix.copy(this.camera.projectionMatrix);
            this._viewDirty = true;
            // 表示を始めた直後 (-Infinity) はすぐ読み込み、動かしている最中は待つ
            if (this._viewChangedAt !== -Infinity) this._viewChangedAt = now;
        }
        if (this._viewDirty && now - this._viewChangedAt >= VIEW_SETTLE_MS) {
            this._viewDirty = false;
            this._viewChangedAt = 0;
            this._update2DView();
        }
    }

    /**
     * 画面に写っている地面 (高さ補正の平面) の範囲を地図座標で求める。
     * near: 注視点のまわりの細かい写真を並べる範囲 / far: 見えている範囲全体 (地平線側は打ち切る)
     */
    _visibleGround() {
        const camera = this.camera;
        const planeZ = this.heightOffset['2d'];
        const eye = new THREE.Vector3().setFromMatrixPosition(camera.matrixWorld);

        const hitGround = (ndcX, ndcY) => {
            const direction = new THREE.Vector3(ndcX, ndcY, 0.5).unproject(camera).sub(eye).normalize();
            if (Math.abs(direction.z) < 1e-9) return { point: null, direction };
            const t = (planeZ - eye.z) / direction.z;
            return { point: t > 0 ? eye.clone().addScaledVector(direction, t) : null, direction };
        };

        // 注視点: 画面中心の視線が地面に当たる点。当たらなければ OrbitControls の注視点
        const centerHit = hitGround(0, 0).point;
        const center = centerHit
            || (this.controls ? this.controls.target.clone().setZ(planeZ) : new THREE.Vector3(eye.x, eye.y, planeZ));
        const distance = Math.max(eye.distanceTo(center), 1);

        const nearRange = distance * NEAR_RANGE_FACTOR;
        const farRange = distance * FAR_RANGE_FACTOR;
        const near = new THREE.Box2();
        const far = new THREE.Box2();
        const eyeXY = new THREE.Vector2(eye.x, eye.y);
        const centerXY = new THREE.Vector2(center.x, center.y);
        near.expandByPoint(centerXY);
        far.expandByPoint(centerXY);

        const STEPS = 6;
        for (let i = 0; i <= STEPS; i++) {
            for (let j = 0; j <= STEPS; j++) {
                const { point, direction } = hitGround(i / STEPS * 2 - 1, j / STEPS * 2 - 1);
                let xy;
                if (point) {
                    xy = new THREE.Vector2(point.x, point.y);
                } else {
                    // 地平線より上を向いている視線は、その方向の遠く (farRange 先) までとする
                    const horizontal = new THREE.Vector2(direction.x, direction.y);
                    if (horizontal.lengthSq() < 1e-12) continue;
                    xy = eyeXY.clone().addScaledVector(horizontal.normalize(), farRange);
                }
                // カメラから遠すぎる所は打ち切る
                const fromEye = xy.clone().sub(eyeXY);
                if (fromEye.length() > farRange) xy = eyeXY.clone().addScaledVector(fromEye.normalize(), farRange);
                far.expandByPoint(xy);
                // 注視点から nearRange 以内に収めたものを細かい範囲にする
                const fromCenter = xy.clone().sub(centerXY);
                if (fromCenter.length() > nearRange) {
                    xy = centerXY.clone().addScaledVector(fromCenter.normalize(), nearRange);
                }
                near.expandByPoint(xy);
            }
        }
        return { near, far, distance };
    }

    /** 地図座標の範囲 -> 緯度経度の範囲 */
    _boxToBounds(box) {
        const corners = [
            [box.min.x, box.min.y], [box.min.x, box.max.y],
            [box.max.x, box.min.y], [box.max.x, box.max.y],
        ].map(([x, y]) => this.frame.mapToLatLon(x, y));
        return {
            north: Math.max(...corners.map(c => c.lat)),
            south: Math.min(...corners.map(c => c.lat)),
            east: Math.max(...corners.map(c => c.lon)),
            west: Math.min(...corners.map(c => c.lon)),
        };
    }

    /** いまの視点に必要なタイルを決めて、足りない分を読み込む。 */
    async _update2DView() {
        if (!this.frame || this.mode !== '2d') return;
        const seq = ++this._view2DSeq;
        const { near, far, distance } = this._visibleGround();

        // 注視点での画面1画素の大きさから、写真のズームを決める
        const canvasHeight = this.renderer.domElement.clientHeight || 600;
        const metersPerScreenPixel = 2 * distance * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)) / canvasHeight;
        const metersPerTilePixelZ0 = WEB_MERCATOR_M_PER_PX_Z0 * Math.cos(THREE.MathUtils.degToRad(this.frame.lat));
        let nearZoom = Math.ceil(Math.log2(metersPerTilePixelZ0 * TILE_PIXELS_PER_SCREEN_PIXEL / metersPerScreenPixel));
        nearZoom = THREE.MathUtils.clamp(nearZoom, MIN_2D_ZOOM, this._zoomCap2D);
        const nearBounds = this._boxToBounds(near);
        let nearRange = tileRange(nearBounds, nearZoom);
        while (nearRange.count > MAX_2D_NEAR_TILES && nearZoom > MIN_2D_ZOOM) {
            nearZoom--;
            nearRange = tileRange(nearBounds, nearZoom);
        }

        // 見えている範囲全体は、細かい範囲に収まらないときだけ粗い写真で埋める
        let farRange = null;
        let farZoom = null;
        const farBounds = this._boxToBounds(far);
        const nearCoversFar = farBounds.north <= nearBounds.north && farBounds.south >= nearBounds.south
            && farBounds.east <= nearBounds.east && farBounds.west >= nearBounds.west;
        if (!nearCoversFar && nearZoom > MIN_2D_ZOOM) {
            farZoom = nearZoom - 1;
            farRange = tileRange(farBounds, farZoom);
            while (farRange.count > MAX_2D_FAR_TILES && farZoom > MIN_2D_ZOOM) {
                farZoom--;
                farRange = tileRange(farBounds, farZoom);
            }
        }

        const wanted = new Set();
        const addRange = (range, zoom) => {
            for (let ty = range.minY; ty <= range.maxY; ty++) {
                for (let tx = range.minX; tx <= range.maxX; tx++) wanted.add(`${zoom}/${tx}/${ty}`);
            }
        };
        addRange(nearRange, nearZoom);
        if (farRange) addRange(farRange, farZoom);
        this._wanted2D = wanted;
        this._view2D = { seq, nearZoom, farZoom, bounds: farRange ? farBounds : nearBounds, count: wanted.size };

        let session;
        try {
            session = await this._ensure2DSession();
        } catch (error) {
            if (seq === this._view2DSeq) this.onStatus(error.message, 'error');
            return;
        }
        if (seq !== this._view2DSeq) return;

        const now = performance.now();
        wanted.forEach((key) => {
            let tile = this._tiles2D.get(key);
            if (!tile) {
                const [zoom, tx, ty] = key.split('/').map(Number);
                tile = { key, zoom, tx, ty, status: 'loading', mesh: null, lastUsed: now, disposed: false };
                this._tiles2D.set(key, tile);
                this._load2DTile(tile, session);
            }
            tile.lastUsed = now;
        });
        this._refresh2DTiles();
        this._fetch2DAttribution(this._view2D.bounds, nearZoom, session, seq);
    }

    async _load2DTile(tile, session) {
        const url = `${TILE_API}/v1/2dtiles/${tile.zoom}/${tile.tx}/${tile.ty}`
            + `?session=${encodeURIComponent(session)}&key=${encodeURIComponent(this.apiKey)}`;
        let status = 'ok';
        let image = null;
        try {
            const res = await fetch(url);
            if (res.status === 404) status = 'notfound';
            else if (res.status === 401 || res.status === 403) status = 'denied';
            else if (!res.ok) status = 'error';
            else {
                const objectUrl = URL.createObjectURL(await res.blob());
                try {
                    image = new Image();
                    image.src = objectUrl;
                    await image.decode();
                } finally {
                    URL.revokeObjectURL(objectUrl);
                }
            }
        } catch (error) {
            status = 'error';
        }
        if (tile.disposed) return;
        tile.status = status;
        if (status === 'ok') {
            const texture = new THREE.Texture(image);
            texture.colorSpace = THREE.SRGBColorSpace;
            texture.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
            texture.needsUpdate = true;
            const material = new THREE.MeshBasicMaterial({
                map: texture,
                transparent: true,
                opacity: this.opacity2D,
                side: THREE.DoubleSide,
                // 粗い写真と細かい写真を同じ平面に重ねるので、深度は書かず描く順番で重ねる
                depthWrite: false,
            });
            const mesh = new THREE.Mesh(this._tileGeometry(tile.tx, tile.ty, tile.zoom), material);
            mesh.name = `google-2d-tile-${tile.key}`;
            // 背景なので点群や PGM より先に描き、ズームが大きい (細かい) 写真ほど上に描く
            mesh.renderOrder = -100 + tile.zoom;
            mesh.visible = false;
            tile.mesh = mesh;
            this.group2D.add(mesh);
        }
        this._refresh2DTiles();
    }

    /** 表示するタイルを切り替え、エラーの扱いと古いタイルの片付けをする。 */
    _refresh2DTiles() {
        const wanted = this._wanted2D;
        const wantedTiles = [...wanted].map(key => this._tiles2D.get(key)).filter(Boolean);
        const settled = wantedTiles.every(tile => tile.status !== 'loading');

        this._tiles2D.forEach((tile) => {
            if (!tile.mesh) return;
            // 新しい視点のタイルが揃うまでは、前の視点のタイルも残してすき間を埋める
            tile.mesh.visible = wanted.has(tile.key) || !settled;
        });
        if (!settled || !this._view2D) return;

        const view = this._view2D;
        const count = (status, zoom = null) => wantedTiles
            .filter(tile => tile.status === status && (zoom === null || tile.zoom === zoom)).length;

        if (count('denied') > 0 && Date.now() - this._sessionRetriedAt > 30 * 1000) {
            // セッションの期限切れの可能性があるので、作り直して読み直す (連続では行わない)
            this._sessionRetriedAt = Date.now();
            this._session2D = null;
            wantedTiles.filter(tile => tile.status === 'denied').forEach((tile) => {
                this._dispose2DTile(tile);
                this._tiles2D.delete(tile.key);
            });
            this._update2DView();
            return;
        }
        const nearTiles = wantedTiles.filter(tile => tile.zoom === view.nearZoom);
        if (count('notfound', view.nearZoom) > nearTiles.length / 2 && view.nearZoom > MIN_2D_ZOOM) {
            // この場所ではこのズームの写真が無いので、上限を下げて選び直す
            this._zoomCap2D = view.nearZoom - 1;
            this._update2DView();
            return;
        }

        this._evict2DTiles();

        const failed = wantedTiles.length - count('ok');
        const zoomText = view.farZoom !== null ? `ズーム ${view.farZoom}〜${view.nearZoom}` : `ズーム ${view.nearZoom}`;
        let message;
        let level = 'info';
        if (count('ok') === 0) {
            message = `衛星写真を読み込めませんでした (${failed} 枚失敗)。API キーの権限 (Map Tiles API) とネットワークを確認してください。`;
            level = 'error';
        } else if (failed > 0) {
            message = `衛星写真を表示しました (${zoomText})。${failed} 枚は読み込めませんでした。`;
            level = 'warning';
        } else {
            message = `衛星写真を表示しました (${zoomText}、${wantedTiles.length} 枚)。`;
        }
        // 同じ内容なら出し直さない (高さ合わせなど、ほかの案内を上書きしないため)
        if (message !== this._last2DStatus) {
            this._last2DStatus = message;
            this.onStatus(message, level);
        }
    }

    /** メモリに残すタイルが多すぎたら、表示に使っていないものから古い順に捨てる。 */
    _evict2DTiles() {
        if (this._tiles2D.size <= MAX_2D_CACHED_TILES) return;
        const unused = [...this._tiles2D.values()]
            .filter(tile => !this._wanted2D.has(tile.key) && tile.status !== 'loading')
            .sort((a, b) => a.lastUsed - b.lastUsed);
        for (const tile of unused) {
            if (this._tiles2D.size <= MAX_2D_CACHED_TILES) break;
            this._dispose2DTile(tile);
            this._tiles2D.delete(tile.key);
        }
    }

    /** タイル1枚ぶんの格子。頂点は地図座標、UV はタイル画像の位置。 */
    _tileGeometry(tx, ty, zoom) {
        // UTM とメルカトルの歪みを吸収するため格子に分ける。広いタイル (小さいズーム) ほど細かく分ける。
        const segments = zoom >= 17 ? 4 : zoom >= 13 ? 8 : 16;
        const positions = [];
        const uvs = [];
        const indices = [];
        for (let j = 0; j <= segments; j++) {
            const v = j / segments;
            for (let i = 0; i <= segments; i++) {
                const u = i / segments;
                // 画像の上端 (v=1) がタイル番号 ty の北端
                const ll = tileToLatLon(tx + u, ty + 1 - v, zoom);
                const p = this.frame.latLonToMap(ll.lat, ll.lon);
                positions.push(p.x, p.y, 0);
                uvs.push(u, v);
            }
        }
        for (let j = 0; j < segments; j++) {
            for (let i = 0; i < segments; i++) {
                const a = j * (segments + 1) + i;
                const b = a + 1;
                const c = a + segments + 1;
                const d = c + 1;
                indices.push(a, b, d, a, d, c);
            }
        }
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
        geometry.setIndex(indices);
        return geometry;
    }

    async _fetch2DAttribution(bounds, zoom, session, seq) {
        const params = new URLSearchParams({
            session, key: this.apiKey, zoom: String(zoom),
            north: String(bounds.north), south: String(bounds.south),
            east: String(bounds.east), west: String(bounds.west),
        });
        try {
            const res = await fetch(`${TILE_API}/tile/v1/viewport?${params}`);
            if (!res.ok) return;
            const json = await res.json();
            if (seq !== this._view2DSeq) return;
            this._attribution2D = json.copyright || '';
            this._updateAttribution();
        } catch (error) {
            console.warn('衛星写真の出典を取得できませんでした:', error);
        }
    }

    // -------------------------------------------------------------------------
    // 3D (Photorealistic 3D Tiles)
    // -------------------------------------------------------------------------

    async _enable3D() {
        const seq = ++this._build3DSeq;
        this.onStatus('3D Tiles を準備中…', 'info');
        let reportedError = false;
        let reportedLoaded = false;
        let tiles;
        try {
            tiles = await createGoogleTilesRenderer({
                apiKey: this.apiKey,
                dracoDecoderPath: this.dracoDecoderPath,
                onError: (message) => {
                    if (reportedError || seq !== this._build3DSeq) return;
                    reportedError = true;
                    this.onStatus(`3D Tiles を読み込めません: ${message}。API キーの権限 (Map Tiles API) とネットワークを確認してください。`, 'error');
                },
                onLoaded: () => {
                    // Google の 3D Tiles は入れ子の tileset を順に読むので、最初の1回だけ知らせる
                    if (reportedLoaded || reportedError || seq !== this._build3DSeq) return;
                    reportedLoaded = true;
                    this.onStatus('3D Tiles を表示しています。点群とずれる場合は「高さ補正」を調整してください。', 'info');
                },
            });
        } catch (error) {
            console.error('3D Tiles のライブラリを読み込めません:', error);
            this.onStatus(`3D Tiles のライブラリを読み込めません: ${error.message}`, 'error');
            return;
        }
        if (seq !== this._build3DSeq || this.mode !== '3d') {
            tiles.dispose();
            return;
        }
        tiles.setCamera(this.camera);
        tiles.setResolutionFromRenderer(this.camera, this.renderer);

        tiles.group.name = 'google-background-3d';
        tiles.group.matrixAutoUpdate = false;
        this.tiles = tiles;
        this._apply3DMatrix();
        this.scene.add(tiles.group);
    }

    _apply3DMatrix() {
        if (!this.tiles || !this.frame) return;
        const matrix = new THREE.Matrix4()
            .makeTranslation(0, 0, this.heightOffset['3d'])
            .multiply(this.frame.ecefToMapMatrix());
        this.tiles.group.matrix.copy(matrix);
        this.tiles.group.updateMatrixWorld(true);
    }

    _disable3D() {
        this._build3DSeq++;
        this._probeCamera = null;
        if (!this.tiles) return;
        this.scene.remove(this.tiles.group);
        this.tiles.dispose();
        this.tiles = null;
    }

    // -------------------------------------------------------------------------
    // 出典表示
    // -------------------------------------------------------------------------

    _updateAttribution() {
        const el = this.attributionElement;
        if (!el) return;
        let copyright = '';
        if (this.mode === '2d') {
            copyright = this._attribution2D;
        } else if (this.mode === '3d' && this.tiles) {
            copyright = this.tiles.getAttributions()
                .filter(a => a.type === 'string' && a.value)
                .map(a => a.value)
                .join(' ');
        }
        if (this.mode === 'none') {
            el.hidden = true;
            return;
        }
        el.hidden = false;
        el.replaceChildren();
        const brand = document.createElement('strong');
        brand.textContent = 'Google';
        el.appendChild(brand);
        // 出典に "Google" だけの項目があると表記が重複するので除く
        const others = copyright.split(/[;,]/).map(s => s.trim()).filter(s => s && s !== 'Google');
        if (others.length > 0) {
            el.appendChild(document.createTextNode(` ${others.join(', ')}`));
        }
    }
}

/**
 * Google Photorealistic 3D Tiles を読む TilesRenderer を作る (まだカメラは設定しない)。
 * ビューアの three.js は r160 のため、r160 で動く 3d-tiles-renderer 0.3.46 を同梱している
 * (0.4 以降は three r166 以上が必要)。3D を使うときだけ読み込むので、ライブラリが無くても他の機能は動く。
 *
 * onError(message): 通信に失敗したとき (このライブラリ版には読み込み失敗のイベントが無いので、通信の結果を見る)
 * onLoaded(): tileset を読み込むたび (入れ子の tileset ごとに何度も呼ばれる)
 * unlitMaterials: 照明を使わない材質に置き換える (画面に表示するとき)
 */
export async function createGoogleTilesRenderer({ apiKey, dracoDecoderPath, onError = () => {},
                                                 onLoaded = () => {}, unlitMaterials = true }) {
    const [{ TilesRenderer }, { GoogleCloudAuthPlugin, GLTFExtensionsPlugin }, { DRACOLoader }] = await Promise.all([
        import('3d-tiles-renderer'),
        import('3d-tiles-renderer/plugins'),
        import('three/addons/loaders/DRACOLoader.js'),
    ]);

    const tiles = new TilesRenderer();
    // セッションの自動更新は、最初の読み込みに成功してから有効にする。
    // 最初から有効だと、キーが無効なときに Google のエラー内容ではなくライブラリ内部の例外になってしまう。
    const auth = new GoogleCloudAuthPlugin({ apiToken: apiKey, autoRefreshToken: false });
    const authFetch = auth.fetchData.bind(auth);
    auth.fetchData = async (uri, options) => {
        let res;
        try {
            res = await authFetch(uri, options);
        } catch (error) {
            onError(error.message);
            throw error;
        }
        if (!res.ok) {
            const message = `HTTP ${res.status} ${await googleErrorMessage(res.clone())}`;
            onError(message.replace(/[.。]\s*$/, ''));
        }
        return res;
    };
    tiles.registerPlugin(auth);
    const dracoLoader = new DRACOLoader().setDecoderPath(dracoDecoderPath);
    tiles.registerPlugin(new GLTFExtensionsPlugin({ dracoLoader }));

    if (unlitMaterials) {
        // 写真から作られたモデルは陰影が画像に焼き込まれている。ビューアにはライトが無いので、
        // 照明を使う材質のままだと黒くなる。照明を使わない材質に置き換える。
        tiles.addEventListener('load-model', ({ scene }) => {
            scene.traverse((object) => {
                if (!object.isMesh || object.material.isMeshBasicMaterial) return;
                const original = object.material;
                object.material = new THREE.MeshBasicMaterial({
                    map: original.map || null,
                    color: original.map ? 0xffffff : original.color,
                    side: original.side,
                    transparent: original.transparent,
                    opacity: original.opacity,
                });
                object.userData.googleBackgroundMaterial = true;
            });
        });
        tiles.addEventListener('dispose-model', ({ scene }) => {
            // 置き換えた材質はライブラリの後始末の対象外なので、ここで破棄する
            scene.traverse((object) => {
                if (object.isMesh && object.userData.googleBackgroundMaterial) object.material.dispose();
            });
        });
    }

    tiles.addEventListener('load-tile-set', () => {
        auth.autoRefreshToken = true;
        onLoaded();
    });
    return tiles;
}

function matricesNearlyEqual(a, b, tolerance = 1e-6) {
    for (let i = 0; i < 16; i++) {
        const x = a.elements[i];
        const y = b.elements[i];
        if (Math.abs(x - y) > tolerance * Math.max(1, Math.abs(x), Math.abs(y))) return false;
    }
    return true;
}

function tileRange(bounds, zoom) {
    const nw = latLonToTile(bounds.north, bounds.west, zoom);
    const se = latLonToTile(bounds.south, bounds.east, zoom);
    const last = Math.pow(2, zoom) - 1;
    const clamp = v => Math.min(Math.max(Math.floor(v), 0), last);
    const minX = clamp(nw.x), maxX = clamp(se.x);
    const minY = clamp(nw.y), maxY = clamp(se.y);
    return { minX, maxX, minY, maxY, count: (maxX - minX + 1) * (maxY - minY + 1) };
}

async function googleErrorMessage(res) {
    try {
        const json = await res.json();
        return (json.error && json.error.message) || res.statusText;
    } catch (error) {
        return res.statusText;
    }
}
