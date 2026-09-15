// =============================================================================
// geo_frame.js — 地図座標 (map フレーム) と緯度経度・ECEF の相互変換
// -----------------------------------------------------------------------------
// hokuyo_navigation2 の地図座標は fix2xyz と同じ定義で作られている。
//   xyz = R(origin_quat) * (UTM(緯度経度) - UTM(原点)),  z = 高度 - 原点高度
// origin_quat は単位クォータニオンなので、地図の X 軸は「UTM のグリッド東」になる
// (真東ではない)。単純な東西南北 (ENU) 近似で重ねると、子午線収差の分だけ
// 回転してずれるため、ここでは UTM をそのまま計算する。
//
// 参照: fix2xyz_packages_ros2/fix2xyz/src/fix_xyz_transform.cpp
//       fix2xyz_packages_ros2/fix2xyz/src/fix_utm_transform.cpp
// =============================================================================
import * as THREE from 'three';

// WGS84 楕円体
const WGS84_A = 6378137.0;
const WGS84_F = 1 / 298.257223563;
const WGS84_E2 = WGS84_F * (2 - WGS84_F);
const WGS84_E = Math.sqrt(WGS84_E2);

// UTM (横メルカトル) の定数
const UTM_K0 = 0.9996;
const UTM_FALSE_EASTING = 500000.0;

// Krüger 級数 (6次) の係数。PROJ の etmerc と mm 未満で一致する精度。
const N = WGS84_F / (2 - WGS84_F);
const N2 = N * N, N3 = N2 * N, N4 = N3 * N, N5 = N4 * N, N6 = N5 * N;
const RECTIFYING_A = WGS84_A / (1 + N) * (1 + N2 / 4 + N4 / 64 + N6 / 256);
const ALPHA = [
    N / 2 - 2 * N2 / 3 + 5 * N3 / 16 + 41 * N4 / 180 - 127 * N5 / 288 + 7891 * N6 / 37800,
    13 * N2 / 48 - 3 * N3 / 5 + 557 * N4 / 1440 + 281 * N5 / 630 - 1983433 * N6 / 1935360,
    61 * N3 / 240 - 103 * N4 / 140 + 15061 * N5 / 26880 + 167603 * N6 / 181440,
    49561 * N4 / 161280 - 179 * N5 / 168 + 6601661 * N6 / 7257600,
    34729 * N5 / 80640 - 3418889 * N6 / 1995840,
    212378941 * N6 / 319334400,
];
const BETA = [
    N / 2 - 2 * N2 / 3 + 37 * N3 / 96 - N4 / 360 - 81 * N5 / 512 + 96199 * N6 / 604800,
    N2 / 48 + N3 / 15 - 437 * N4 / 1440 + 46 * N5 / 105 - 1118711 * N6 / 3870720,
    17 * N3 / 480 - 37 * N4 / 840 - 209 * N5 / 4480 + 5569 * N6 / 90720,
    4397 * N4 / 161280 - 11 * N5 / 504 - 830251 * N6 / 7257600,
    4583 * N5 / 161280 - 108847 * N6 / 3991680,
    20648693 * N6 / 638668800,
];

const DEG = Math.PI / 180;

/** fix_utm_transform.cpp の judge_utm_zone と同じ式でゾーンを決める。
 *  標準の floor((lon+180)/6)+1 とは経度の端数によって1ゾーンずれるが、
 *  地図はこの式で作られているので合わせる。 */
export function fix2xyzUtmZone(longitude) {
    return Math.trunc(Math.trunc(longitude + 180.0 + 5) / 6);
}

function centralMeridianDeg(zone) {
    return zone * 6 - 183;
}

/** 緯度経度 [deg] -> UTM (EPSG:326xx、北半球の定義) [m] */
export function latLonToUtm(latDeg, lonDeg, zone) {
    const phi = latDeg * DEG;
    const dLambda = (lonDeg - centralMeridianDeg(zone)) * DEG;
    const sinPhi = Math.sin(phi);
    const t = Math.sinh(Math.atanh(sinPhi) - WGS84_E * Math.atanh(WGS84_E * sinPhi));
    const xiP = Math.atan2(t, Math.cos(dLambda));
    const etaP = Math.atanh(Math.sin(dLambda) / Math.sqrt(1 + t * t));
    let xi = xiP;
    let eta = etaP;
    for (let j = 1; j <= 6; j++) {
        xi += ALPHA[j - 1] * Math.sin(2 * j * xiP) * Math.cosh(2 * j * etaP);
        eta += ALPHA[j - 1] * Math.cos(2 * j * xiP) * Math.sinh(2 * j * etaP);
    }
    return {
        x: UTM_FALSE_EASTING + UTM_K0 * RECTIFYING_A * eta,
        y: UTM_K0 * RECTIFYING_A * xi,
    };
}

/** UTM [m] -> 緯度経度 [deg] */
export function utmToLatLon(x, y, zone) {
    const xi = y / (UTM_K0 * RECTIFYING_A);
    const eta = (x - UTM_FALSE_EASTING) / (UTM_K0 * RECTIFYING_A);
    let xiP = xi;
    let etaP = eta;
    for (let j = 1; j <= 6; j++) {
        xiP -= BETA[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
        etaP -= BETA[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
    }
    const tauP = Math.sin(xiP) / Math.sqrt(Math.sinh(etaP) ** 2 + Math.cos(xiP) ** 2);
    const dLambda = Math.atan2(Math.sinh(etaP), Math.cos(xiP));

    // 等角緯度 tau' から緯度 tau を Newton 法で求める (Karney 2011)
    let tau = tauP;
    for (let i = 0; i < 10; i++) {
        const sqrt1tau2 = Math.sqrt(1 + tau * tau);
        const sigma = Math.sinh(WGS84_E * Math.atanh(WGS84_E * tau / sqrt1tau2));
        const tauI = tau * Math.sqrt(1 + sigma * sigma) - sigma * sqrt1tau2;
        const dTau = (tauP - tauI) / Math.sqrt(1 + tauI * tauI)
            * (1 + (1 - WGS84_E2) * tau * tau) / ((1 - WGS84_E2) * sqrt1tau2);
        tau += dTau;
        if (Math.abs(dTau) < 1e-14) break;
    }
    return {
        lat: Math.atan(tau) / DEG,
        lon: centralMeridianDeg(zone) + dLambda / DEG,
    };
}

/** 緯度経度高度 -> ECEF [m] */
export function latLonAltToEcef(latDeg, lonDeg, alt) {
    const phi = latDeg * DEG;
    const lambda = lonDeg * DEG;
    const sinPhi = Math.sin(phi);
    const primeVertical = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinPhi * sinPhi);
    return new THREE.Vector3(
        (primeVertical + alt) * Math.cos(phi) * Math.cos(lambda),
        (primeVertical + alt) * Math.cos(phi) * Math.sin(lambda),
        (primeVertical * (1 - WGS84_E2) + alt) * sinPhi,
    );
}

/**
 * 1つの地図の座標系。原点の緯度経度高度 (init_lat_lon_alt.txt) から作る。
 * yawDeg は GNSS を使わずに作った地図を手で合わせるための方位補正で、
 * 地図座標を反時計回りに回すと UTM の向きになる角度。GNSS 地図では 0。
 */
export class MapGeoFrame {
    constructor({ lat, lon, alt = 0, yawDeg = 0 }) {
        this.lat = lat;
        this.lon = lon;
        this.alt = alt;
        this.yawDeg = yawDeg;
        this.zone = fix2xyzUtmZone(lon);
        this.originUtm = latLonToUtm(lat, lon, this.zone);
        this._cos = Math.cos(yawDeg * DEG);
        this._sin = Math.sin(yawDeg * DEG);
    }

    /** 緯度経度高度 -> 地図座標 */
    latLonToMap(latDeg, lonDeg, alt = this.alt) {
        const utm = latLonToUtm(latDeg, lonDeg, this.zone);
        const dx = utm.x - this.originUtm.x;
        const dy = utm.y - this.originUtm.y;
        // UTM 相対 = Rz(yaw) * 地図座標 なので、逆回転で地図座標に戻す
        return {
            x: this._cos * dx + this._sin * dy,
            y: -this._sin * dx + this._cos * dy,
            z: alt - this.alt,
        };
    }

    /** 地図座標 -> 緯度経度高度 */
    mapToLatLon(x, y, z = 0) {
        const dx = this._cos * x - this._sin * y;
        const dy = this._sin * x + this._cos * y;
        const ll = utmToLatLon(this.originUtm.x + dx, this.originUtm.y + dy, this.zone);
        return { lat: ll.lat, lon: ll.lon, alt: this.alt + z };
    }

    /** 原点で東へ 1m / 北へ 1m 動いたときの地図座標の変化 (UTM の縮尺と子午線収差を含む) */
    _localJacobian() {
        const phi = this.lat * DEG;
        const sinPhi = Math.sin(phi), cosPhi = Math.cos(phi);
        const w = Math.sqrt(1 - WGS84_E2 * sinPhi * sinPhi);
        const primeVertical = WGS84_A / w;
        const meridional = WGS84_A * (1 - WGS84_E2) / (w * w * w);
        const step = 1.0;
        const dLon = step / ((primeVertical + this.alt) * cosPhi) / DEG;
        const dLat = step / (meridional + this.alt) / DEG;
        const pe1 = this.latLonToMap(this.lat, this.lon + dLon);
        const pe0 = this.latLonToMap(this.lat, this.lon - dLon);
        const pn1 = this.latLonToMap(this.lat + dLat, this.lon);
        const pn0 = this.latLonToMap(this.lat - dLat, this.lon);
        return {
            jEx: (pe1.x - pe0.x) / (2 * step), jEy: (pe1.y - pe0.y) / (2 * step),
            jNx: (pn1.x - pn0.x) / (2 * step), jNy: (pn1.y - pn0.y) / (2 * step),
        };
    }

    /** 原点での UTM の縮尺係数 (地面上の 1m が地図座標で何 m になるか。0.9996 前後) */
    utmScaleFactor() {
        const { jEx, jEy, jNx, jNy } = this._localJacobian();
        return Math.sqrt(Math.abs(jEx * jNy - jEy * jNx));
    }

    /**
     * ECEF -> 地図座標 の変換行列。3D Tiles (ECEF で配信される) を地図に重ねるのに使う。
     * 原点まわりで UTM を1次近似している。pyproj との比較で、原点から 300m 以内は 2cm 程度、
     * 1.4km 先でも 16cm 程度 (主に地球の丸みによる高さ方向) で、3D Tiles 自体の精度より十分小さい。
     */
    ecefToMapMatrix() {
        const phi = this.lat * DEG;
        const lambda = this.lon * DEG;
        const sinPhi = Math.sin(phi), cosPhi = Math.cos(phi);
        const sinLam = Math.sin(lambda), cosLam = Math.cos(lambda);

        // 原点での東・北・上の単位ベクトル (ECEF)
        const east = new THREE.Vector3(-sinLam, cosLam, 0);
        const north = new THREE.Vector3(-sinPhi * cosLam, -sinPhi * sinLam, cosPhi);
        const up = new THREE.Vector3(cosPhi * cosLam, cosPhi * sinLam, sinPhi);

        const { jEx, jEy, jNx, jNy } = this._localJacobian();

        // 地図 = J * ENU,  ENU = [east; north; up] * (ECEF - 原点ECEF)
        // UTM は等角なので J は「縮尺 k × 回転」。高さにも同じ k を掛けて等倍の拡大にする。
        // (xy と z で縮尺が違うと 3D Tiles の詳細度計算が警告を出す。k≈0.9996 なので高さ 50m で 2cm の差)
        const k = Math.sqrt(Math.abs(jEx * jNy - jEy * jNx));
        const rowX = east.clone().multiplyScalar(jEx).add(north.clone().multiplyScalar(jNx));
        const rowY = east.clone().multiplyScalar(jEy).add(north.clone().multiplyScalar(jNy));
        const rowZ = up.clone().multiplyScalar(k);
        const originEcef = latLonAltToEcef(this.lat, this.lon, this.alt);

        const m = new THREE.Matrix4();
        m.set(
            rowX.x, rowX.y, rowX.z, -rowX.dot(originEcef),
            rowY.x, rowY.y, rowY.z, -rowY.dot(originEcef),
            rowZ.x, rowZ.y, rowZ.z, -rowZ.dot(originEcef),
            0, 0, 0, 1,
        );
        return m;
    }
}

/** Web メルカトルのタイル番号 -> 緯度経度 (タイルの左上角。小数で途中の位置も可) */
export function tileToLatLon(tx, ty, zoom) {
    const n = Math.pow(2, zoom);
    return {
        lat: Math.atan(Math.sinh(Math.PI * (1 - 2 * ty / n))) / DEG,
        lon: tx / n * 360 - 180,
    };
}

/** 緯度経度 -> Web メルカトルのタイル番号 (小数) */
export function latLonToTile(latDeg, lonDeg, zoom) {
    const n = Math.pow(2, zoom);
    const phi = latDeg * DEG;
    return {
        x: (lonDeg + 180) / 360 * n,
        y: (1 - Math.log(Math.tan(phi) + 1 / Math.cos(phi)) / Math.PI) / 2 * n,
    };
}
