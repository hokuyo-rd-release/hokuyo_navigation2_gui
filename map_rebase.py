"""地図の原点を初期位置 (localization の初期位置) に合わせて、地図一式の座標をずらすモジュール。

初期位置 (x0, y0) が新しい原点 (0, 0) になるよう、次のファイルの x, y を (-x0, -y0) だけ平行移動する。
向き (UTM に沿った軸) と高さ (z) は変えない。

  * map/<名前>.yaml                  origin
  * map/<名前>.pcd                   点群 (x y 以外の項目・ascii / binary の形式はそのまま)
  * waypoints/<名前>.json            各ウェイポイントの位置 (4 要素目の緯度経度高度は場所が変わらないのでそのまま)
  * data/<名前>/init_lat_lon_alt.txt 新しい原点の緯度経度 (fix2xyz と同じ UTM で計算。高度はそのまま)
  * data/<名前>/init_pose.txt        (0, 0, z) と向き
  * data/<名前>/source_points.pcd, source_points.pending.pcd, map_params.json, source_points.pending.json
                                     地図のパラメータ調整の元の点群と設定

ファイルはすべて一時ファイルに書いてから最後にまとめて置き換えるので、途中で失敗しても元のファイルは壊さない。
"""

import json
import math
import os
import tempfile

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# UTM (static/js/viewer/geo_frame.js と同じ計算。fix2xyz と同じゾーンの決め方)
# ---------------------------------------------------------------------------
WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
WGS84_E = math.sqrt(WGS84_E2)
UTM_K0 = 0.9996
UTM_FALSE_EASTING = 500000.0
_N = WGS84_F / (2 - WGS84_F)
_N2, _N3, _N4, _N5, _N6 = _N ** 2, _N ** 3, _N ** 4, _N ** 5, _N ** 6
RECTIFYING_A = WGS84_A / (1 + _N) * (1 + _N2 / 4 + _N4 / 64 + _N6 / 256)
_ALPHA = [
    _N / 2 - 2 * _N2 / 3 + 5 * _N3 / 16 + 41 * _N4 / 180 - 127 * _N5 / 288 + 7891 * _N6 / 37800,
    13 * _N2 / 48 - 3 * _N3 / 5 + 557 * _N4 / 1440 + 281 * _N5 / 630 - 1983433 * _N6 / 1935360,
    61 * _N3 / 240 - 103 * _N4 / 140 + 15061 * _N5 / 26880 + 167603 * _N6 / 181440,
    49561 * _N4 / 161280 - 179 * _N5 / 168 + 6601661 * _N6 / 7257600,
    34729 * _N5 / 80640 - 3418889 * _N6 / 1995840,
    212378941 * _N6 / 319334400,
]
_BETA = [
    _N / 2 - 2 * _N2 / 3 + 37 * _N3 / 96 - _N4 / 360 - 81 * _N5 / 512 + 96199 * _N6 / 604800,
    _N2 / 48 + _N3 / 15 - 437 * _N4 / 1440 + 46 * _N5 / 105 - 1118711 * _N6 / 3870720,
    17 * _N3 / 480 - 37 * _N4 / 840 - 209 * _N5 / 4480 + 5569 * _N6 / 90720,
    4397 * _N4 / 161280 - 11 * _N5 / 504 - 830251 * _N6 / 7257600,
    4583 * _N5 / 161280 - 108847 * _N6 / 3991680,
    20648693 * _N6 / 638668800,
]


def fix2xyz_utm_zone(lon):
    """fix_utm_transform.cpp の judge_utm_zone と同じ式"""
    return int(int(lon + 180.0 + 5) / 6)


def _central_meridian(zone):
    return zone * 6 - 183


def latlon_to_utm(lat, lon, zone):
    phi = math.radians(lat)
    d_lambda = math.radians(lon - _central_meridian(zone))
    sin_phi = math.sin(phi)
    t = math.sinh(math.atanh(sin_phi) - WGS84_E * math.atanh(WGS84_E * sin_phi))
    xi_p = math.atan2(t, math.cos(d_lambda))
    eta_p = math.atanh(math.sin(d_lambda) / math.sqrt(1 + t * t))
    xi, eta = xi_p, eta_p
    for j in range(1, 7):
        xi += _ALPHA[j - 1] * math.sin(2 * j * xi_p) * math.cosh(2 * j * eta_p)
        eta += _ALPHA[j - 1] * math.cos(2 * j * xi_p) * math.sinh(2 * j * eta_p)
    return UTM_FALSE_EASTING + UTM_K0 * RECTIFYING_A * eta, UTM_K0 * RECTIFYING_A * xi


def utm_to_latlon(x, y, zone):
    xi = y / (UTM_K0 * RECTIFYING_A)
    eta = (x - UTM_FALSE_EASTING) / (UTM_K0 * RECTIFYING_A)
    xi_p, eta_p = xi, eta
    for j in range(1, 7):
        xi_p -= _BETA[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
        eta_p -= _BETA[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
    tau_p = math.sin(xi_p) / math.sqrt(math.sinh(eta_p) ** 2 + math.cos(xi_p) ** 2)
    d_lambda = math.atan2(math.sinh(eta_p), math.cos(xi_p))
    tau = tau_p
    for _ in range(10):
        s = math.sqrt(1 + tau * tau)
        sigma = math.sinh(WGS84_E * math.atanh(WGS84_E * tau / s))
        tau_i = tau * math.sqrt(1 + sigma * sigma) - sigma * s
        d_tau = (tau_p - tau_i) / math.sqrt(1 + tau_i * tau_i) \
            * (1 + (1 - WGS84_E2) * tau * tau) / ((1 - WGS84_E2) * s)
        tau += d_tau
        if abs(d_tau) < 1e-14:
            break
    return math.degrees(math.atan(tau)), _central_meridian(zone) + math.degrees(d_lambda)


def shifted_origin_latlon(lat, lon, dx, dy):
    """地図座標 (dx, dy) の地点の緯度経度 (原点 lat, lon の UTM ゾーンで計算)"""
    zone = fix2xyz_utm_zone(lon)
    e0, n0 = latlon_to_utm(lat, lon, zone)
    return utm_to_latlon(e0 + dx, n0 + dy, zone)


# ---------------------------------------------------------------------------
# ファイルの書き換え
# ---------------------------------------------------------------------------
class RebaseError(ValueError):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def _temp_path(path):
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.rebase_', suffix=os.path.splitext(path)[1])
    os.close(fd)
    return tmp


_PCD_TYPES = {('F', 4): '<f4', ('F', 8): '<f8', ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4',
              ('I', 8): '<i8', ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8'}


def shift_pcd(src, dst, dx, dy):
    """PCD の x, y だけを平行移動して dst に書く。ヘッダーと x y 以外の項目はそのまま残す。"""
    with open(src, 'rb') as f:
        header_lines = []
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise RebaseError(f'{os.path.basename(src)}: PCD のヘッダーが途中で終わっています。')
            header_lines.append(line)
            text = line.decode('ascii', errors='replace').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.split()
            if key.upper() == 'DATA':
                break
        body = f.read()
    fields = header.get('FIELDS', [])
    if 'x' not in fields or 'y' not in fields:
        raise RebaseError(f'{os.path.basename(src)}: PCD に x, y の項目がありません。')
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
    data_format = header['DATA'][0].lower() if header.get('DATA') else ''
    head = b''.join(header_lines)

    if data_format == 'binary':
        sizes = [int(v) for v in header.get('SIZE', [])]
        types = [v.upper() for v in header.get('TYPE', [])]
        dtype_fields = []
        for i, (name, size, kind, count) in enumerate(zip(fields, sizes, types, counts)):
            base = _PCD_TYPES.get((kind, size))
            if base is None:
                raise RebaseError(f'{os.path.basename(src)}: PCD の型 {kind}{size} には対応していません。')
            dtype_fields.append((f'f{i}', base, (count,)) if count > 1 else (f'f{i}', base))
        dtype = np.dtype(dtype_fields)
        n = len(body) // dtype.itemsize
        record = np.frombuffer(body[:n * dtype.itemsize], dtype=dtype).copy()
        for name, delta in (('x', dx), ('y', dy)):
            column = f'f{fields.index(name)}'
            if record.dtype[column].kind != 'f':
                raise RebaseError(f'{os.path.basename(src)}: PCD の {name} が小数ではありません。')
            record[column] = record[column] + delta
        with open(dst, 'wb') as f:
            f.write(head)
            f.write(record.tobytes())
            f.write(body[n * dtype.itemsize:])
    elif data_format == 'ascii':
        total = sum(counts)
        column_of = {}
        column = 0
        for name, count in zip(fields, counts):
            column_of[name] = column
            column += count
        lines = body.decode('ascii', errors='replace').splitlines()
        out = []
        cx, cy = column_of['x'], column_of['y']
        for line in lines:
            tokens = line.split()
            if len(tokens) < total:
                if line.strip():
                    out.append(line)
                continue
            for index, delta in ((cx, dx), (cy, dy)):
                value = float(tokens[index])
                if math.isfinite(value):
                    tokens[index] = repr(round(value + delta, 6))
            out.append(' '.join(tokens))
        with open(dst, 'wb') as f:
            f.write(head)
            f.write(('\n'.join(out) + '\n').encode('ascii'))
    else:
        raise RebaseError(f'{os.path.basename(src)}: PCD の DATA {data_format} には対応していません (ascii か binary のみ)。')


def _shift_bounds(bounds, dx, dy):
    if not isinstance(bounds, dict):
        return bounds
    return {**bounds, 'min_x': bounds['min_x'] + dx, 'max_x': bounds['max_x'] + dx,
            'min_y': bounds['min_y'] + dy, 'max_y': bounds['max_y'] + dy}


def plan_rebase(map_dir, data_dir, wp_dir, map_name):
    """書き換え対象のうち、実際にあるファイル (パッケージからの相対表示用の短い名前と絶対パス)。"""
    data = os.path.join(data_dir, map_name)
    candidates = [
        ('yaml', os.path.join(map_dir, f'{map_name}.yaml')),
        ('pcd', os.path.join(map_dir, f'{map_name}.pcd')),
        ('waypoints', os.path.join(wp_dir, f'{map_name}.json')),
        ('latlon', os.path.join(data, 'init_lat_lon_alt.txt')),
        ('source_pcd', os.path.join(data, 'source_points.pcd')),
        ('pending_pcd', os.path.join(data, 'source_points.pending.pcd')),
        ('params', os.path.join(data, 'map_params.json')),
        ('pending_info', os.path.join(data, 'source_points.pending.json')),
    ]
    return [(kind, path) for kind, path in candidates if os.path.isfile(path)]


def _read_origin(path, default_latlon):
    """init_lat_lon_alt.txt を読む。実際の場所を表す値なら [lat, lon, alt]、そうでなければ None。"""
    with open(path, 'r', encoding='utf-8') as f:
        lines = [line.strip() for line in f if line.strip()]
    values = [float(v) for v in lines[-1].split(',')] if lines else []
    valid = len(values) >= 2 and all(math.isfinite(v) for v in values[:3]) \
        and -90 <= values[0] <= 90 and -180 <= values[1] <= 180
    is_default = default_latlon is not None and len(values) >= 3 \
        and all(abs(a - b) < 1e-9 for a, b in zip(values[:3], default_latlon))
    return values if valid and not is_default else None


def validate_rebase(data_dir, map_name, x0, y0, default_latlon=None):
    """原点を (x0, y0) に動かしてよいか確かめる。だめなら RebaseError。

    fix2xyz は、原点と GNSS の点を「それぞれの経度から決めた UTM ゾーン」で UTM にしてから引き算する。
    新しい原点のゾーンが元の原点と変わると、元のゾーンにある GNSS の点が数 m 以上ずれた位置に変換されるので、動かさない。
    """
    if not (math.isfinite(x0) and math.isfinite(y0)):
        raise RebaseError('初期位置の値が不正です。')
    path = os.path.join(data_dir, map_name, 'init_lat_lon_alt.txt')
    if not os.path.isfile(path):
        return None
    values = _read_origin(path, default_latlon)
    if values is None:
        return None
    lat, lon = shifted_origin_latlon(values[0], values[1], x0, y0)
    zone_old = fix2xyz_utm_zone(values[1])
    zone_new = fix2xyz_utm_zone(lon)
    if zone_new != zone_old:
        boundary = zone_old * 6 - 185 if zone_new < zone_old else zone_old * 6 - 179
        raise RebaseError(
            f'初期位置が UTM ゾーンの境目 (経度 {boundary}°) の向こう側 (新しい原点の経度 {lon:.6f}°、ゾーン {zone_old} → {zone_new}) にあるため、'
            '原点を合わせられません。fix2xyz は点ごとにゾーンを決めるので、原点のゾーンが変わると GNSS の位置が地図とずれます。'
            '初期位置だけを保存する場合は「地図の原点を初期位置に合わせる」を外してください。')
    return {'lat': lat, 'lon': lon, 'alt': values[2] if len(values) > 2 else 0.0}


def rebase_map(map_dir, data_dir, wp_dir, map_name, x0, y0, pose, default_latlon=None):
    """初期位置 (x0, y0) を新しい原点にする。pose は保存する初期位置 [x, y, z, qx, qy, qz, qw] (x, y は 0 にする)。"""
    validate_rebase(data_dir, map_name, x0, y0, default_latlon)
    dx, dy = -x0, -y0
    plan = plan_rebase(map_dir, data_dir, wp_dir, map_name)
    kinds = {kind for kind, _ in plan}
    if not kinds & {'yaml', 'pcd', 'waypoints'}:
        raise RebaseError(f'マップ「{map_name}」の地図・ウェイポイントのファイルがありません。', 404)

    temps = []   # (一時ファイル, 置き換え先)
    notes = []
    new_origin = None
    try:
        for kind, path in plan:
            tmp = _temp_path(path)
            temps.append((tmp, path))
            if kind == 'yaml':
                with open(path, 'r', encoding='utf-8') as f:
                    info = yaml.safe_load(f) or {}
                origin = list(info.get('origin') or [0.0, 0.0, 0.0])
                origin[0] = round(float(origin[0]) + dx, 6)
                origin[1] = round(float(origin[1]) + dy, 6)
                info['origin'] = origin
                with open(tmp, 'w', encoding='utf-8') as f:
                    yaml.dump(info, f, default_flow_style=True)
            elif kind in ('pcd', 'source_pcd', 'pending_pcd'):
                shift_pcd(path, tmp, dx, dy)
            elif kind == 'waypoints':
                with open(path, 'r', encoding='utf-8') as f:
                    waypoints = json.load(f)
                for wp in waypoints:
                    if isinstance(wp, list) and wp and isinstance(wp[0], list) and len(wp[0]) >= 2:
                        wp[0][0] = float(wp[0][0]) + dx
                        wp[0][1] = float(wp[0][1]) + dy
                with open(tmp, 'w', encoding='utf-8') as f:
                    json.dump(waypoints, f, indent=4)
            elif kind == 'latlon':
                values = _read_origin(path, default_latlon)
                if values is None:
                    # 実際の場所を表していない原点は、ずらしても意味が無いのでそのまま残す
                    os.remove(tmp)
                    temps.pop()
                    notes.append('init_lat_lon_alt.txt は実際の場所を表していない (GNSS を使わずに作った地図の既定値など) ため変えていません。')
                    continue
                lat, lon = shifted_origin_latlon(values[0], values[1], x0, y0)
                alt = values[2] if len(values) > 2 else 0.0
                new_origin = {'lat': lat, 'lon': lon, 'alt': alt}
                with open(tmp, 'w', encoding='utf-8') as f:
                    f.write(f'{lat:.10f},{lon:.10f},{alt:.10f}\n')
            elif kind in ('params', 'pending_info'):
                with open(path, 'r', encoding='utf-8') as f:
                    params = json.load(f)
                if kind == 'params':
                    source = params.get('source') or {}
                    if source.get('region'):
                        source['region'] = _shift_bounds(source['region'], dx, dy)
                    pcd = params.get('pcd') or {}
                    if pcd.get('crop'):
                        pcd['crop'] = _shift_bounds(pcd['crop'], dx, dy)
                else:
                    if params.get('region'):
                        params['region'] = _shift_bounds(params['region'], dx, dy)
                with open(tmp, 'w', encoding='utf-8') as f:
                    json.dump(params, f, ensure_ascii=False, indent=2)
                    f.write('\n')

        # map_params.json の原点は、書き換えた init_lat_lon_alt.txt に合わせる
        if new_origin is not None:
            for tmp, path in temps:
                if path.endswith('map_params.json'):
                    with open(tmp, 'r', encoding='utf-8') as f:
                        params = json.load(f)
                    if params.get('origin'):
                        params['origin'] = new_origin
                    with open(tmp, 'w', encoding='utf-8') as f:
                        json.dump(params, f, ensure_ascii=False, indent=2)
                        f.write('\n')

        pose_path = os.path.join(data_dir, map_name, 'init_pose.txt')
        os.makedirs(os.path.dirname(pose_path), exist_ok=True)
        pose_tmp = _temp_path(pose_path)
        temps.append((pose_tmp, pose_path))
        new_pose = [0.0, 0.0] + [float(v) for v in pose[2:7]]
        with open(pose_tmp, 'w', encoding='utf-8') as f:
            f.write(','.join(f'{v:.6f}' for v in new_pose) + '\n')

        # すべて書けてから置き換える
        for tmp, path in temps:
            os.replace(tmp, path)
        temps_done = [path for _, path in temps]
        temps = []
    finally:
        for tmp, _ in temps:
            if os.path.exists(tmp):
                os.remove(tmp)

    return {'files': temps_done, 'dx': dx, 'dy': dy, 'origin': new_origin, 'pose': new_pose, 'notes': notes}
