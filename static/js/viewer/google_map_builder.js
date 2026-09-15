// =============================================================================
// google_map_builder.js — 選んだ範囲の Google 3D Tiles のメッシュから点群を作る
// -----------------------------------------------------------------------------
// 画面の表示とは別に TilesRenderer を作り、範囲の真上に並べた仮想カメラで
// 細かい詳細度まで読み込んでから、メッシュの表面に一定の間隔で点を置く。
//
// 点群の座標は「範囲の中心の緯度経度 + その地面の高度」を原点にした地図座標
// (fix2xyz と同じ UTM の差) で返す。この原点を init_lat_lon_alt.txt に保存すれば、
// GNSS モードの自律走行でそのまま重なる。
//
// ※ Google Maps Platform の利用規約は、コンテンツの取り出し・保存・それをもとにしたデータの作成を
//    禁じている可能性が高い。利用者の判断で使う機能 (README 参照)。
// =============================================================================
import * as THREE from 'three';
import { MapGeoFrame } from './geo_frame.js';
import { createGoogleTilesRenderer } from './google_background.js';

// 詳細度の目安 (画面上の誤差 [px])。小さいほど細かいメッシュまで読む (表示用の既定は 40)。
// 最初に地表の高さをつかむときは粗くてよいので、読み込む量を抑える。
const OVERVIEW_ERROR_TARGET = 40;
const EXTRACT_ERROR_TARGET = 2;
// 範囲の真上に並べる仮想カメラ。高さ CAMERA_HEIGHT から真下を見て、CAMERA_STEP ごとに並べる。
const CAMERA_HEIGHT = 120;
const CAMERA_STEP = 100;
const CAMERA_RESOLUTION = 1024;
// 読み込みが止まったとみなすまでの確認回数と間隔、全体の待ち時間の上限
const IDLE_CHECKS = 15;
const POLL_INTERVAL_MS = 200;
const LOAD_TIMEOUT_MS = 240 * 1000;
// 地面の高さを決めるときに見る、中心からの半径 [m]
const GROUND_SEARCH_RADIUS = 15;

export const MAX_EXTRACTED_POINTS = 8_000_000;

const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

/** 再現性のある乱数 (同じ範囲から作り直したとき、同じ点群になるように) */
function mulberry32(seed) {
    let a = seed >>> 0;
    return () => {
        a = (a + 0x6D2B79F5) >>> 0;
        let t = a;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

/**
 * @param {Object} options
 * @param {string} options.apiKey
 * @param {string} options.dracoDecoderPath
 * @param {number} options.lat 範囲の中心の緯度
 * @param {number} options.lon 範囲の中心の経度
 * @param {THREE.Box2} options.bounds 範囲 (中心を原点にした地図座標 [m])
 * @param {number} options.spacing 点の間隔 [m]
 * @param {(message: string) => void} [options.onProgress]
 * @param {() => boolean} [options.isCancelled]
 * @returns {Promise<{points: Float32Array, colors: Uint32Array, count: number, alt: number, frame: MapGeoFrame, timedOut: boolean}>}
 *   colors は各点の色 (0x00RRGGBB、sRGB)。メッシュの写真 (テクスチャ) から取る。
 */
export async function extractGoogleMeshPoints({ apiKey, dracoDecoderPath, lat, lon, bounds, spacing,
                                                onProgress = () => {}, isCancelled = () => false }) {
    // まず高度 0 を原点にして読み、z が楕円体高になるようにする。地面の高さが分かったら差し引く。
    const frame0 = new MapGeoFrame({ lat, lon, alt: 0 });
    let lastError = null;
    const tiles = await createGoogleTilesRenderer({
        apiKey, dracoDecoderPath, unlitMaterials: false,
        onError: (message) => { lastError = message; },
    });
    const cameras = [];
    try {
        tiles.errorTarget = OVERVIEW_ERROR_TARGET;
        // 範囲全体の細かいタイルを捨てずに持っておけるよう、キャッシュの容量を広げる (既定 0.4GB)
        tiles.lruCache.minBytesSize = 1.0 * 1024 ** 3;
        tiles.lruCache.maxBytesSize = 1.5 * 1024 ** 3;
        tiles.group.matrixAutoUpdate = false;
        tiles.group.matrix.copy(frame0.ecefToMapMatrix());
        tiles.group.updateMatrixWorld(true);

        const center = bounds.getCenter(new THREE.Vector2());
        const size = bounds.getSize(new THREE.Vector2());

        // 1. 高い所から粗く読み、範囲の中心の地表の高さをつかむ
        const overview = makeDownCamera(center.x, center.y, 3000, 1000, Math.max(size.x, size.y) + 200);
        tiles.setCamera(overview);
        tiles.setResolution(overview, 256, 256);
        cameras.push(overview);
        onProgress('範囲のまわりのタイルを読み込んでいます…');
        await waitForTiles(tiles, { minVisible: 1, onProgress, isCancelled, getError: () => lastError, timeoutMs: 60 * 1000 });
        const surfaceZ = raycastDown(tiles.group, center.x, center.y) ?? 0;

        // 2. 範囲の真上に仮想カメラを並べ、細かいタイルまで読む
        tiles.deleteCamera(overview);
        tiles.errorTarget = EXTRACT_ERROR_TARGET;
        cameras.length = 0;
        const nx = Math.max(1, Math.ceil(size.x / CAMERA_STEP));
        const ny = Math.max(1, Math.ceil(size.y / CAMERA_STEP));
        for (let i = 0; i < nx; i++) {
            for (let j = 0; j < ny; j++) {
                const x = bounds.min.x + (i + 0.5) * size.x / nx;
                const y = bounds.min.y + (j + 0.5) * size.y / ny;
                const camera = makeDownCamera(x, y, surfaceZ + CAMERA_HEIGHT, CAMERA_HEIGHT + 500,
                                              Math.max(size.x / nx, size.y / ny) * 1.2);
                tiles.setCamera(camera);
                tiles.setResolution(camera, CAMERA_RESOLUTION, CAMERA_RESOLUTION);
                cameras.push(camera);
            }
        }
        onProgress(`範囲の細かいタイルを読み込んでいます… (カメラ ${cameras.length} 台)`);
        const timedOut = await waitForTiles(tiles, {
            minVisible: 1, onProgress, isCancelled, getError: () => lastError, timeoutMs: LOAD_TIMEOUT_MS,
        });

        // 3. メッシュの表面に点を置く
        onProgress('メッシュから点群を作っています…');
        await sleep(0);
        const sampled = sampleMeshSurfaces(tiles.group, bounds, spacing);
        if (sampled.count === 0) {
            throw new Error(lastError
                ? `3D Tiles を読み込めませんでした: ${lastError}`
                : 'この範囲にメッシュがありませんでした。範囲や場所を確認してください。');
        }

        // 4. 中心付近の地面の高さを原点の高度にし、点群の z をその差にする
        const k = frame0.utmScaleFactor();
        const groundZ = lowerPercentileNear(sampled.points, sampled.count, center.x, center.y,
                                            GROUND_SEARCH_RADIUS, 0.05) ?? surfaceZ;
        for (let i = 2; i < sampled.count * 3; i += 3) sampled.points[i] -= groundZ;
        const alt = groundZ / k;
        return {
            points: sampled.points.subarray(0, sampled.count * 3),
            colors: sampled.colors.subarray(0, sampled.count),
            textured: sampled.textured,
            count: sampled.count,
            alt,
            frame: new MapGeoFrame({ lat, lon, alt }),
            timedOut,
        };
    } finally {
        cameras.forEach(camera => tiles.deleteCamera(camera));
        tiles.dispose();
    }
}

/** (x, y) の真上 height から真下を見るカメラ。coverage は真下に写したい幅 [m]。 */
function makeDownCamera(x, y, height, far, coverage) {
    const fov = 2 * THREE.MathUtils.radToDeg(Math.atan(coverage / 2 / Math.max(height, 1)));
    const camera = new THREE.PerspectiveCamera(THREE.MathUtils.clamp(fov, 10, 120), 1, 1, far + height);
    camera.up.set(0, 1, 0);
    camera.position.set(x, y, height);
    camera.lookAt(x, y, height - 1);
    camera.updateMatrixWorld(true);
    camera.updateProjectionMatrix();
    return camera;
}

/** タイルの読み込みが一段落するまで待つ。時間切れなら true を返す (途中までの結果で続ける)。 */
async function waitForTiles(tiles, { minVisible, onProgress, isCancelled, getError, timeoutMs }) {
    const startedAt = performance.now();
    let idle = 0;
    for (;;) {
        if (isCancelled()) throw new Error('中止しました。');
        tiles.update();
        const { downloading, parsing } = tiles.stats;
        const visible = tiles.visibleTiles.size;
        if (getError() && visible === 0 && performance.now() - startedAt > 3000) {
            throw new Error(`3D Tiles を読み込めません: ${getError()}`);
        }
        idle = (downloading === 0 && parsing === 0 && visible >= minVisible) ? idle + 1 : 0;
        if (idle >= IDLE_CHECKS) return false;
        const elapsed = performance.now() - startedAt;
        if (elapsed > timeoutMs) return true;
        onProgress(`タイルを読み込んでいます… (表示 ${visible} / ダウンロード中 ${downloading} / 解析中 ${parsing}、`
            + `${Math.round(elapsed / 1000)} 秒)`);
        await sleep(POLL_INTERVAL_MS);
    }
}

function raycastDown(object, x, y) {
    const raycaster = new THREE.Raycaster(new THREE.Vector3(x, y, 10000), new THREE.Vector3(0, 0, -1), 0, 20000);
    const hits = raycaster.intersectObject(object, true);
    return hits.length > 0 ? hits[0].point.z : null;
}

/**
 * テクスチャの画素を読めるようにする。読めない (圧縮テクスチャなど) ときは null。
 * glTF のテクスチャは flipY = false なので、uv の v=0 が画像の上端。
 */
function readTexturePixels(texture) {
    const image = texture && texture.image;
    if (!image || !image.width || !image.height) return null;
    try {
        const canvas = typeof OffscreenCanvas !== 'undefined'
            ? new OffscreenCanvas(image.width, image.height)
            : Object.assign(document.createElement('canvas'), { width: image.width, height: image.height });
        const context = canvas.getContext('2d', { willReadFrequently: true });
        context.drawImage(image, 0, 0);
        return { data: context.getImageData(0, 0, image.width, image.height).data, width: image.width, height: image.height };
    } catch (error) {
        return null;
    }
}

/** 材質の色 (sRGB 0〜255)。テクスチャに掛ける係数として使う。 */
function materialBaseColor(material) {
    const color = new THREE.Color(1, 1, 1);
    if (material && material.color) color.copy(material.color).convertLinearToSRGB();
    return [color.r * 255, color.g * 255, color.b * 255];
}

/** 表示中のタイルのメッシュ表面に、面積に比例した数の点をランダムに置く (範囲内だけ)。色はテクスチャから取る。 */
function sampleMeshSurfaces(group, bounds, spacing) {
    const random = mulberry32(20260914);
    const perArea = 1 / (spacing * spacing);
    let points = new Float32Array(1 << 22);
    let colors = new Uint32Array(points.length / 3);
    let count = 0;
    let textured = 0;
    const push = (x, y, z, rgb) => {
        if (count * 3 + 3 > points.length) {
            if (points.length >= MAX_EXTRACTED_POINTS * 3) {
                throw new Error(`点が多すぎます (${MAX_EXTRACTED_POINTS} 点を超えました)。点の間隔を広げるか、範囲を狭めてください。`);
            }
            const grown = new Float32Array(Math.min(points.length * 2, MAX_EXTRACTED_POINTS * 3));
            grown.set(points);
            points = grown;
            const grownColors = new Uint32Array(grown.length / 3);
            grownColors.set(colors);
            colors = grownColors;
        }
        points[count * 3] = x;
        points[count * 3 + 1] = y;
        points[count * 3 + 2] = z;
        colors[count] = rgb;
        count++;
    };

    const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
    const ab = new THREE.Vector3(), ac = new THREE.Vector3();
    const uvA = new THREE.Vector2(), uvB = new THREE.Vector2(), uvC = new THREE.Vector2(), uv = new THREE.Vector2();
    const box = new THREE.Box3();
    group.updateMatrixWorld(true);
    group.traverse((object) => {
        if (!object.isMesh || !object.geometry || !object.geometry.attributes.position) return;
        const geometry = object.geometry;
        if (!geometry.boundingBox) geometry.computeBoundingBox();
        box.copy(geometry.boundingBox).applyMatrix4(object.matrixWorld);
        if (box.max.x < bounds.min.x || box.min.x > bounds.max.x
            || box.max.y < bounds.min.y || box.min.y > bounds.max.y) return;

        const material = Array.isArray(object.material) ? object.material[0] : object.material;
        const base = materialBaseColor(material);
        const baseRgb = (Math.round(base[0]) << 16) | (Math.round(base[1]) << 8) | Math.round(base[2]);
        const map = material && material.map;
        const uvAttribute = geometry.attributes.uv;
        // テクスチャの画素はメッシュごとに読んで捨てる (全タイル分を持つとメモリが足りない)
        const pixels = map && uvAttribute ? readTexturePixels(map) : null;
        if (pixels && map.matrixAutoUpdate) map.updateMatrix();
        const flipY = map ? map.flipY : false;

        const position = geometry.attributes.position;
        const index = geometry.index;
        const triangles = index ? index.count / 3 : position.count / 3;
        const matrix = object.matrixWorld;
        for (let t = 0; t < triangles; t++) {
            const ia = index ? index.getX(t * 3) : t * 3;
            const ib = index ? index.getX(t * 3 + 1) : t * 3 + 1;
            const ic = index ? index.getX(t * 3 + 2) : t * 3 + 2;
            a.fromBufferAttribute(position, ia).applyMatrix4(matrix);
            b.fromBufferAttribute(position, ib).applyMatrix4(matrix);
            c.fromBufferAttribute(position, ic).applyMatrix4(matrix);
            if (Math.max(a.x, b.x, c.x) < bounds.min.x || Math.min(a.x, b.x, c.x) > bounds.max.x
                || Math.max(a.y, b.y, c.y) < bounds.min.y || Math.min(a.y, b.y, c.y) > bounds.max.y) continue;

            const area = 0.5 * ab.subVectors(b, a).cross(ac.subVectors(c, a)).length();
            const expected = area * perArea;
            let samples = Math.floor(expected);
            if (random() < expected - samples) samples++;
            if (samples === 0) continue;
            if (pixels) {
                uvA.fromBufferAttribute(uvAttribute, ia);
                uvB.fromBufferAttribute(uvAttribute, ib);
                uvC.fromBufferAttribute(uvAttribute, ic);
            }
            for (let s = 0; s < samples; s++) {
                // 三角形の中に一様に置く
                const r1 = Math.sqrt(random());
                const r2 = random();
                const wa = 1 - r1, wb = r1 * (1 - r2), wc = r1 * r2;
                const x = wa * a.x + wb * b.x + wc * c.x;
                const y = wa * a.y + wb * b.y + wc * c.y;
                if (x < bounds.min.x || x > bounds.max.x || y < bounds.min.y || y > bounds.max.y) continue;
                let rgb = baseRgb;
                if (pixels) {
                    uv.set(wa * uvA.x + wb * uvB.x + wc * uvC.x, wa * uvA.y + wb * uvB.y + wc * uvC.y);
                    uv.applyMatrix3(map.matrix);
                    const u = uv.x - Math.floor(uv.x);
                    const v = uv.y - Math.floor(uv.y);
                    const px = Math.min(pixels.width - 1, Math.floor(u * pixels.width));
                    const py = Math.min(pixels.height - 1, Math.floor((flipY ? 1 - v : v) * pixels.height));
                    const o = (py * pixels.width + px) * 4;
                    const r = Math.min(255, Math.round(pixels.data[o] * base[0] / 255));
                    const g = Math.min(255, Math.round(pixels.data[o + 1] * base[1] / 255));
                    const bl = Math.min(255, Math.round(pixels.data[o + 2] * base[2] / 255));
                    rgb = (r << 16) | (g << 8) | bl;
                    textured++;
                }
                push(x, y, wa * a.z + wb * b.z + wc * c.z, rgb);
            }
        }
    });
    return { points, colors, count, textured };
}

/** 点群と色を、サーバーに送る形 (1 点 16 バイト: x, y, z の float32 と 0x00RRGGBB の uint32) にまとめる */
export function packPointsWithColors(points, colors) {
    const count = Math.floor(points.length / 3);
    const buffer = new ArrayBuffer(count * 16);
    const xyz = new Float32Array(buffer);
    const rgb = new Uint32Array(buffer);
    for (let i = 0; i < count; i++) {
        xyz[i * 4] = points[i * 3];
        xyz[i * 4 + 1] = points[i * 3 + 1];
        xyz[i * 4 + 2] = points[i * 3 + 2];
        rgb[i * 4 + 3] = colors ? colors[i] : 0xffffff;
    }
    return buffer;
}

/** (x, y) から radius 以内の点の z の下位 percentile。点が少なければ null。 */
function lowerPercentileNear(points, count, x, y, radius, percentile) {
    const r2 = radius * radius;
    const heights = [];
    for (let i = 0; i < count; i++) {
        const dx = points[i * 3] - x;
        const dy = points[i * 3 + 1] - y;
        if (dx * dx + dy * dy < r2) heights.push(points[i * 3 + 2]);
    }
    if (heights.length < 20) return null;
    heights.sort((p, q) => p - q);
    return heights[Math.floor(heights.length * percentile)];
}
