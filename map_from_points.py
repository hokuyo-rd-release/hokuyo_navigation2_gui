"""範囲を指定して作った点群 (Map Viewer で Google 3D Tiles から取り出したもの) から、
自律走行で使える地図ファイル一式を書き出すモジュール。

書き出すもの (いずれも hokuyo_navigation2 の既存の地図と同じ置き場所・形式):
  * map/<名前>.pcd              点群 (binary PCD, x y z)
  * map/<名前>.pgm, .yaml       2D 占有格子地図
  * data/<名前>/init_lat_lon_alt.txt  地図座標の原点 (範囲の中心) の緯度,経度,高度
  * data/<名前>/init_pose.txt         自己位置推定の初期位置 x,y,z,qx,qy,qz,qw
  * data/<名前>/map_params.json       作ったときの設定 (後からパラメータを変えて作り直すため)

作った後は、元の点群と設定から PCD / PGM を作り直せる (derive_points, build_occupancy_pgm)。
元の点群は data/<名前>/source_points.pcd があればそれ、無ければ map/<名前>.pcd。
PCD を初めて書き換えるときに、元の map/<名前>.pcd を source_points.pcd に残す。

点群の座標は、原点の緯度経度を fix2xyz と同じく UTM に変換した差 (X=グリッド東, Y=グリッド北,
Z=高度差) で渡される。この座標系のまま PGM の origin を決めるので、GNSS モードの自律走行
(fix2xyz が同じ原点で /fix を地図座標に直す) でそのまま重なる。

PGM の値は pcd2pgm_converter.py と同じ規約 (0=障害物, 254=通行可, 205=未知) で、
上下を反転して保存する。ただし写真から作ったメッシュには地面も含まれるため、
既存の変換のように高さの範囲で一律に切るのではなく、場所ごとの地面からの高さで障害物を判定する。
"""

import datetime
import json
import math
import os
import shutil
import tempfile

import numpy as np
import yaml
from PIL import Image

try:
    from scipy import ndimage
except ImportError:  # scipy が無くても動くように (地面の推定が粗くなる)
    ndimage = None

PGM_OCCUPIED = 0
PGM_FREE = 254

# 地面の高さを推定するときの格子の大きさ [m]
GROUND_CELL_SIZE = 1.0
# 1 辺の画素数の上限 (大きすぎる範囲でメモリを使い切らないため)
MAX_PGM_SIDE_PIXELS = 12000

PARAMS_FILE_NAME = 'map_params.json'
SOURCE_POINTS_FILE_NAME = 'source_points.pcd'
# Google から取り直して、まだ保存していない点群
PENDING_SOURCE_FILE_NAME = 'source_points.pending.pcd'
PENDING_SOURCE_INFO_NAME = 'source_points.pending.json'

DEFAULT_PGM_PARAMS = {
    'resolution': 0.05,
    'obstacle_min': 0.5,
    'obstacle_max': 2.0,
    'ground_window': 10.0,
    'inflate': 0.1,
}
# 障害物とみなす高さの範囲 (obstacle_min / obstacle_max) の基準
#   'ground': 場所ごとの地面 (まわり ground_window の最低点) からの高さ。坂や段差があっても地面を障害物にしない
#   'pcd_z':  PCD の z 座標そのもの (pcd2pgm と同じ)。範囲をはっきり決めたいとき
HEIGHT_REFERENCES = ('ground', 'pcd_z')
DEFAULT_HEIGHT_REFERENCE = 'ground'


# 点の送り方: 'xyz' は 1 点 12 バイト (float32 x,y,z)、'xyzrgb' は 1 点 16 バイト (float32 x,y,z と uint32 0x00RRGGBB)
POINT_RECORD_BYTES = {'xyz': 12, 'xyzrgb': 16}
_XYZRGB_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])


def points_from_bytes(data, point_format='xyz'):
    """ブラウザから送られた点群を (points (N, 3) float32, colors (N,) uint32 か None) にする。"""
    if point_format == 'xyzrgb':
        record = np.frombuffer(data, dtype=_XYZRGB_DTYPE)
        points = np.stack([record['x'], record['y'], record['z']], axis=1)
        return np.ascontiguousarray(points), np.ascontiguousarray(record['rgb'])
    return np.frombuffer(data, dtype='<f4').reshape(-1, 3), None


def pcd_binary_bytes(points, colors=None):
    """(N, 3) の点群を binary PCD のバイト列にする。

    colors (0x00RRGGBB の uint32) があれば、PCL の PointXYZRGB と同じ rgb 項目 (float32 に詰めた色) を付ける。
    """
    points = np.ascontiguousarray(points, dtype='<f4')
    count = points.shape[0]
    if colors is not None:
        record = np.empty(count, dtype=_XYZRGB_DTYPE)
        record['x'], record['y'], record['z'] = points[:, 0], points[:, 1], points[:, 2]
        record['rgb'] = colors
        fields, sizes, types, counts, body = 'x y z rgb', '4 4 4 4', 'F F F F', '1 1 1 1', record.tobytes()
    else:
        fields, sizes, types, counts, body = 'x y z', '4 4 4', 'F F F', '1 1 1', points.tobytes()
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        f'FIELDS {fields}\n'
        f'SIZE {sizes}\n'
        f'TYPE {types}\n'
        f'COUNT {counts}\n'
        f'WIDTH {count}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {count}\n'
        'DATA binary\n'
    )
    return header.encode('ascii') + body


def _replace_file(path, write):
    """同じフォルダの一時ファイルに書いてから置き換える (途中で失敗しても元のファイルを壊さない)。"""
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=directory, prefix='.tmp_', suffix=os.path.splitext(path)[1])
    try:
        with os.fdopen(fd, 'wb') as f:
            write(f)
        os.replace(tmp_path, path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise


def write_pcd_binary(path, points, colors=None):
    """(N, 3) の float32 点群 (と色) を binary PCD で保存する。"""
    data = pcd_binary_bytes(points, colors)
    _replace_file(path, lambda f: f.write(data))


_PCD_TYPES = {('F', 4): '<f4', ('F', 8): '<f8', ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4',
              ('I', 8): '<i8', ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8'}


def read_pcd_xyz(path):
    """PCD (ascii / binary) から x, y, z だけを (N, 3) float32 で読む。NaN の点は除く。

    戻り値: (points, fields) fields は元のファイルの FIELDS (x y z 以外の項目があるか確認する用)
    """
    points, _, fields = read_pcd(path)
    return points, fields


def _color_field(fields):
    return next((name for name in ('rgb', 'rgba') if name in fields), None)


def read_pcd(path):
    """PCD (ascii / binary) から x, y, z と色 (rgb / rgba があれば) を読む。NaN の点は除く。

    戻り値: (points (N, 3) float32, colors (N,) uint32 の 0x00RRGGBB か None, fields)
    """
    header = {}
    with open(path, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError('PCD のヘッダーが途中で終わっています。')
            text = line.decode('ascii', errors='replace').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.split()
            if key.upper() == 'DATA':
                break
        body = f.read()

    fields = header.get('FIELDS', [])
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('PCD に x, y, z の項目がありません。')
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
    if 'POINTS' in header:
        n = int(header['POINTS'][0])
    else:
        n = int(header.get('WIDTH', ['0'])[0]) * int(header.get('HEIGHT', ['1'])[0])
    data_format = header['DATA'][0].lower() if header.get('DATA') else ''

    if data_format == 'ascii':
        columns = []
        column = 0
        for name, count in zip(fields, counts):
            columns.append(column)
            column += count
        values = np.fromstring(body.decode('ascii', errors='replace'), sep=' ')
        rows = min(n, values.size // column) if n > 0 else values.size // column
        values = values[:rows * column].reshape(rows, column)
        points = values[:, [columns[fields.index(c)] for c in ('x', 'y', 'z')]].astype(np.float32)
        colors = None
        color_name = _color_field(fields)
        if color_name:
            raw = values[:, columns[fields.index(color_name)]]
            types = [v.upper() for v in header.get('TYPE', [])]
            if len(types) == len(fields) and types[fields.index(color_name)] == 'F':
                # PCL は色を float32 のビット列に詰めて書く
                colors = raw.astype('<f4').view('<u4')
            else:
                colors = raw.astype(np.uint64).astype('<u4')
    elif data_format == 'binary':
        sizes = [int(v) for v in header.get('SIZE', [])]
        types = [v.upper() for v in header.get('TYPE', [])]
        if len(sizes) != len(fields) or len(types) != len(fields):
            raise ValueError('PCD の SIZE / TYPE が FIELDS と合いません。')
        dtype_fields = []
        for i, (name, size, kind, count) in enumerate(zip(fields, sizes, types, counts)):
            base = _PCD_TYPES.get((kind, size))
            if base is None:
                raise ValueError(f'PCD の型 {kind}{size} には対応していません。')
            dtype_fields.append((f'f{i}_{name}', base, (count,)) if count > 1 else (f'f{i}_{name}', base))
        dtype = np.dtype(dtype_fields)
        n = min(n, len(body) // dtype.itemsize)
        record = np.frombuffer(body, dtype=dtype, count=n)
        points = np.stack([record[f'f{fields.index(c)}_{c}'].astype(np.float32)
                           for c in ('x', 'y', 'z')], axis=1)
        colors = None
        color_name = _color_field(fields)
        if color_name:
            i = fields.index(color_name)
            if sizes[i] == 4 and counts[i] == 1:
                # F4 (float に詰めた色) も U4 も、4 バイトをそのまま uint32 として読む
                colors = np.frombuffer(np.ascontiguousarray(record[f'f{i}_{color_name}']).tobytes(), dtype='<u4')
    else:
        raise ValueError(f'PCD の DATA {data_format} には対応していません (ascii か binary のみ)。')

    finite = np.isfinite(points).all(axis=1)
    if not finite.all():
        points = points[finite]
        if colors is not None:
            colors = colors[finite]
    if colors is not None:
        colors = np.ascontiguousarray(colors & 0x00FFFFFF, dtype='<u4')
    return np.ascontiguousarray(points), colors, fields


def voxel_downsample_indices(points, voxel):
    """一辺 voxel の立方体ごとに 1 点だけ残すときの、残す点の番号 (元の並び順)。"""
    keys = np.floor(points / voxel).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    linear = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, first = np.unique(linear, return_index=True)
    return np.sort(first)


def voxel_downsample(points, voxel):
    """一辺 voxel の立方体ごとに 1 点だけ残す (元の点をそのまま使うので、面から外れた点を作らない)。"""
    if points.shape[0] == 0 or not voxel or voxel <= 0:
        return points
    return points[voxel_downsample_indices(points, voxel)]


def _bounds_tuple(bounds):
    """{min_x, min_y, max_x, max_y} か (min_x, min_y, max_x, max_y) を tuple にする。無ければ None。"""
    if bounds is None:
        return None
    if isinstance(bounds, dict):
        bounds = (bounds['min_x'], bounds['min_y'], bounds['max_x'], bounds['max_y'])
    bounds = tuple(float(v) for v in bounds)
    if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds) \
            or bounds[2] <= bounds[0] or bounds[3] <= bounds[1]:
        raise ValueError('範囲の値が不正です。')
    return bounds


def _bounds_dict(bounds):
    if bounds is None:
        return None
    return {'min_x': bounds[0], 'min_y': bounds[1], 'max_x': bounds[2], 'max_y': bounds[3]}


def normalize_pcd_params(params, source_spacing=None):
    """PCD の作り直しの設定を検査して整える。

    spacing: 点の間隔 [m] (0 か元の間隔以下なら間引かない)
    crop: 残す範囲 {min_x, min_y, max_x, max_y} (None なら全体)
    z_min, z_max: 残す高さ [m] (None なら制限なし)
    """
    params = params or {}

    def optional_float(key):
        value = params.get(key)
        if value is None or value == '':
            return None
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f'{key} の値が不正です。')
        return value

    spacing = optional_float('spacing') or 0.0
    if spacing < 0:
        raise ValueError('点の間隔は 0 以上にしてください。')
    result = {
        'spacing': spacing if spacing > 0 else (source_spacing or 0.0),
        'crop': _bounds_dict(_bounds_tuple(params.get('crop'))),
        'z_min': optional_float('z_min'),
        'z_max': optional_float('z_max'),
    }
    if result['z_min'] is not None and result['z_max'] is not None and result['z_max'] <= result['z_min']:
        raise ValueError('残す高さは 上限 > 下限 にしてください。')
    return result


def normalize_pgm_params(params):
    params = {**DEFAULT_PGM_PARAMS, **(params or {})}
    result = {key: float(params[key]) for key in DEFAULT_PGM_PARAMS}
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError('PGM の設定に数値でない値があります。')
    reference = params.get('height_reference') or DEFAULT_HEIGHT_REFERENCE
    if reference not in HEIGHT_REFERENCES:
        raise ValueError(f'高さの基準 {reference} には対応していません (ground か pcd_z)。')
    result['height_reference'] = reference
    if result['resolution'] < 0.01:
        raise ValueError('PGM の解像度は 0.01 m 以上にしてください。')
    if result['obstacle_max'] <= result['obstacle_min']:
        raise ValueError('障害物とみなす高さは 上限 > 下限 にしてください。')
    if result['ground_window'] < GROUND_CELL_SIZE:
        raise ValueError(f'地面の高さを探す範囲は {GROUND_CELL_SIZE} m 以上にしてください。')
    if result['inflate'] < 0:
        raise ValueError('障害物を太らせる幅は 0 以上にしてください。')
    return result


def pcd_params_change_points(pcd_params, source_spacing):
    """この設定で元の点群から点が減る (PCD の中身が変わる) か。"""
    spacing = pcd_params.get('spacing') or 0.0
    thinned = spacing > 0 and (not source_spacing or spacing > source_spacing * 1.001)
    return bool(thinned or pcd_params.get('crop') or pcd_params.get('z_min') is not None
                or pcd_params.get('z_max') is not None)


def derive_points(source_points, pcd_params, source_spacing=None, colors=None):
    """元の点群に PCD の設定 (切り取り・高さの制限・間引き) を当てる。戻り値: (points, colors)"""
    points = source_points
    crop = _bounds_tuple(pcd_params.get('crop'))
    mask = np.ones(points.shape[0], dtype=bool)
    if crop is not None:
        mask &= (points[:, 0] >= crop[0]) & (points[:, 0] <= crop[2]) \
            & (points[:, 1] >= crop[1]) & (points[:, 1] <= crop[3])
    if pcd_params.get('z_min') is not None:
        mask &= points[:, 2] >= pcd_params['z_min']
    if pcd_params.get('z_max') is not None:
        mask &= points[:, 2] <= pcd_params['z_max']
    if not mask.all():
        points = points[mask]
        if colors is not None:
            colors = colors[mask]
    spacing = pcd_params.get('spacing') or 0.0
    if spacing > 0 and (not source_spacing or spacing > source_spacing * 1.001) and points.shape[0] > 0:
        keep = voxel_downsample_indices(points, spacing)
        points = points[keep]
        if colors is not None:
            colors = colors[keep]
    return np.ascontiguousarray(points), (None if colors is None else np.ascontiguousarray(colors))


def pgm_bounds_for(points, pcd_params, source_region):
    """PGM にする範囲: 切り取り範囲 > 元の点群の範囲 (作成時の範囲) > 点群の広がり"""
    crop = _bounds_tuple(pcd_params.get('crop'))
    if crop is not None:
        return crop
    region = _bounds_tuple(source_region)
    if region is not None:
        return region
    if points.shape[0] == 0:
        raise ValueError('点群に点がありません。')
    lo = points[:, :2].min(axis=0)
    hi = points[:, :2].max(axis=0)
    return (float(lo[0]), float(lo[1]), float(max(hi[0], lo[0] + 0.01)), float(max(hi[1], lo[1] + 0.01)))


def _ground_heights(points, min_x, min_y, width_m, height_m, window_m):
    """GROUND_CELL_SIZE 四方ごとの地面の高さ (2D 配列) を返す。

    各格子の最低点を、まわり window_m の範囲で最小にしたものを地面とする。
    屋根しか写っていない格子 (建物の上) も、近くに地面があればその高さになる。
    """
    cols = max(1, int(math.ceil(width_m / GROUND_CELL_SIZE)))
    rows = max(1, int(math.ceil(height_m / GROUND_CELL_SIZE)))
    gx = np.clip(((points[:, 0] - min_x) / GROUND_CELL_SIZE).astype(np.int64), 0, cols - 1)
    gy = np.clip(((points[:, 1] - min_y) / GROUND_CELL_SIZE).astype(np.int64), 0, rows - 1)

    lowest = np.full(rows * cols, np.inf, dtype=np.float64)
    np.minimum.at(lowest, gy * cols + gx, points[:, 2])
    lowest = lowest.reshape(rows, cols)

    window_cells = max(1, int(round(window_m / GROUND_CELL_SIZE)))
    if ndimage is not None:
        ground = ndimage.minimum_filter(lowest, size=window_cells, mode='nearest')
    else:
        ground = lowest.copy()
    # 点が1つも無い場所は、全体の地面の中央値で埋める
    finite = np.isfinite(ground)
    fill = float(np.median(ground[finite])) if finite.any() else 0.0
    ground[~finite] = fill
    return ground, gx, gy


def build_occupancy_pgm(points, bounds, resolution, obstacle_min, obstacle_max,
                        ground_window, inflate, height_reference=DEFAULT_HEIGHT_REFERENCE):
    """点群から PGM (上下反転済みの uint8 配列) と YAML の内容を作る。

    bounds: (min_x, min_y, max_x, max_y) 地図にする範囲 [m]
    obstacle_min, obstacle_max: この高さの間に点があれば障害物 [m]。基準は height_reference
        'ground' なら場所ごとの地面からの高さ、'pcd_z' なら PCD の z 座標
    ground_window: 地面の高さを探す範囲 [m] ('ground' のときだけ使う)
    inflate: 障害物を太らせる幅 [m]。点と点のすき間で壁が途切れないよう、点の間隔の半分くらいにする。
    """
    min_x, min_y, max_x, max_y = bounds
    # 原点を解像度の整数倍にそろえると、同じ範囲から作り直しても画素がずれない
    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    width = int(math.ceil((max_x - origin_x) / resolution))
    height = int(math.ceil((max_y - origin_y) / resolution))
    if width <= 0 or height <= 0:
        raise ValueError('地図にする範囲が空です。')
    if width > MAX_PGM_SIDE_PIXELS or height > MAX_PGM_SIDE_PIXELS:
        raise ValueError(f'PGM が大きすぎます ({width}x{height} 画素)。範囲を狭めるか解像度を粗くしてください。')

    inside = ((points[:, 0] >= origin_x) & (points[:, 0] < origin_x + width * resolution)
              & (points[:, 1] >= origin_y) & (points[:, 1] < origin_y + height * resolution))
    pts = points[inside]

    occupied = np.zeros((height, width), dtype=bool)
    if pts.shape[0] > 0:
        if height_reference == 'pcd_z':
            is_obstacle = (pts[:, 2] >= obstacle_min) & (pts[:, 2] <= obstacle_max)
        else:
            ground, gx, gy = _ground_heights(pts, origin_x, origin_y, width * resolution,
                                             height * resolution, ground_window)
            above_ground = pts[:, 2] - ground[gy, gx]
            is_obstacle = (above_ground >= obstacle_min) & (above_ground <= obstacle_max)
        obs = pts[is_obstacle]
        ix = ((obs[:, 0] - origin_x) / resolution).astype(np.int64)
        iy = ((obs[:, 1] - origin_y) / resolution).astype(np.int64)
        occupied[np.clip(iy, 0, height - 1), np.clip(ix, 0, width - 1)] = True

        radius_px = int(math.ceil(inflate / resolution - 1e-9))
        if radius_px > 0 and ndimage is not None:
            yy, xx = np.mgrid[-radius_px:radius_px + 1, -radius_px:radius_px + 1]
            disk = (xx * xx + yy * yy) <= radius_px * radius_px
            occupied = ndimage.binary_dilation(occupied, structure=disk)

    # 既存の pcd2pgm_converter.py と同じく、点の無い所は通行可にする
    pgm = np.full((height, width), PGM_FREE, dtype=np.uint8)
    pgm[occupied] = PGM_OCCUPIED
    # PGM は左上が原点なので上下を反転する (pcd2pgm_converter.py と同じ)
    pgm = np.flipud(pgm)

    map_info = {
        'resolution': float(resolution),
        'origin': [round(origin_x, 6), round(origin_y, 6), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    stats = {
        'width': width,
        'height': height,
        'occupied_cells': int(occupied.sum()),
        'points_in_bounds': int(pts.shape[0]),
    }
    return pgm, map_info, stats


def yaw_to_quaternion(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def _now_iso():
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


def write_pgm_and_yaml(map_dir, map_name, pgm, map_info):
    """PGM (上下反転済みの uint8 配列) と YAML を書き出す。"""
    pgm_path = os.path.join(map_dir, f'{map_name}.pgm')
    yaml_path = os.path.join(map_dir, f'{map_name}.yaml')
    _replace_file(pgm_path, lambda f: Image.fromarray(pgm, mode='L').save(f, format='PPM'))
    map_info = {'image': f'{map_name}.pgm', **{k: v for k, v in map_info.items() if k != 'image'}}
    # pcd2pgm_converter.py と同じ書式 ([x, y, z] のリスト形式)
    text = yaml.dump(map_info, default_flow_style=True)
    _replace_file(yaml_path, lambda f: f.write(text.encode('utf-8')))
    return pgm_path, yaml_path, map_info


def params_path(data_dir, map_name):
    return os.path.join(data_dir, map_name, PARAMS_FILE_NAME)


def load_params(data_dir, map_name):
    """保存してある設定 (map_params.json) を読む。無い・読めないときは None。"""
    try:
        with open(params_path(data_dir, map_name), 'r', encoding='utf-8') as f:
            params = json.load(f)
        return params if isinstance(params, dict) else None
    except (OSError, ValueError):
        return None


def write_params(data_dir, map_name, params):
    text = json.dumps(params, ensure_ascii=False, indent=2) + '\n'
    _replace_file(params_path(data_dir, map_name), lambda f: f.write(text.encode('utf-8')))


def save_map_files(map_dir, data_dir, map_name, points, bounds, origin_lla, pgm_params, init_pose,
                   source=None, colors=None):
    """地図ファイル一式を書き出し、書いたファイルと PGM の情報を返す。

    origin_lla: (lat, lon, alt) 地図座標の原点
    pgm_params: resolution, obstacle_min, obstacle_max, ground_window, inflate
    init_pose: (x, y, yaw) 初期位置
    source: 元の点群の説明 {type, spacing} (map_params.json に残す)
    colors: 各点の色 (0x00RRGGBB)。あればカラー点群の PCD にする
    """
    pgm_params = normalize_pgm_params(pgm_params)
    pgm, map_info, stats = build_occupancy_pgm(points, bounds, **pgm_params)

    os.makedirs(map_dir, exist_ok=True)
    pcd_path = os.path.join(map_dir, f'{map_name}.pcd')
    write_pcd_binary(pcd_path, points, colors)
    pgm_path, yaml_path, map_info = write_pgm_and_yaml(map_dir, map_name, pgm, map_info)

    map_data_dir = os.path.join(data_dir, map_name)
    os.makedirs(map_data_dir, exist_ok=True)
    # 同じ名前で作り直したときは、前の地図の元の点群を使わないよう消す
    for stale in (SOURCE_POINTS_FILE_NAME, PENDING_SOURCE_FILE_NAME, PENDING_SOURCE_INFO_NAME):
        stale_path = os.path.join(map_data_dir, stale)
        if os.path.exists(stale_path):
            os.remove(stale_path)

    lat, lon, alt = origin_lla
    latlon_path = os.path.join(map_data_dir, 'init_lat_lon_alt.txt')
    with open(latlon_path, 'w', encoding='utf-8') as f:
        f.write(f'{lat:.10f},{lon:.10f},{alt:.10f}\n')

    x, y, yaw = init_pose
    pose_path = os.path.join(map_data_dir, 'init_pose.txt')
    with open(pose_path, 'w', encoding='utf-8') as f:
        f.write(','.join(f'{v:.6f}' for v in [x, y, 0.0, *yaw_to_quaternion(yaw)]) + '\n')

    source = source or {}
    spacing = float(source.get('spacing') or 0.0)
    params = {
        'version': 1,
        'created_at': _now_iso(),
        'updated_at': _now_iso(),
        'origin': {'lat': lat, 'lon': lon, 'alt': alt},
        # 元の点群 (作ったときの点群)。source_points.pcd が無い間は map/<名前>.pcd がそれ。
        'source': {
            'type': source.get('type', 'points'),
            'spacing': spacing,
            'has_color': colors is not None,
            'region': _bounds_dict(_bounds_tuple(bounds)),
            'fetched_at': _now_iso(),
        },
        'pcd': {'spacing': spacing, 'crop': None, 'z_min': None, 'z_max': None},
        # 作るときは場所ごとの地面からの高さで障害物を決める。基準は書かずに残し、後から調整するときは
        # Map Viewer が PCD の z 座標で指定する既定にする (書いていなければ計算は地面からの高さのまま)
        'pgm': {k: v for k, v in pgm_params.items() if k != 'height_reference'},
    }
    write_params(data_dir, map_name, params)

    return {
        'files': [pcd_path, pgm_path, yaml_path, latlon_path, pose_path, params_path(data_dir, map_name)],
        'pgm': stats,
        'yaml': map_info,
    }


# =============================================================================
# 作った地図をパラメータを変えて作り直す
# =============================================================================

class MapEditError(ValueError):
    """利用者に見せるメッセージ付きのエラー。status は HTTP ステータス。"""

    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def _read_yaml_resolution(map_dir, map_name):
    try:
        with open(os.path.join(map_dir, f'{map_name}.yaml'), 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f) or {}
        resolution = float(info.get('resolution'))
        return resolution if resolution > 0 else None
    except (OSError, yaml.YAMLError, TypeError, ValueError):
        return None


def describe_map_params(map_dir, data_dir, map_name):
    """地図の今の設定と元の点群の情報をまとめる (設定が無い地図は既定値)。"""
    pcd_path = os.path.join(map_dir, f'{map_name}.pcd')
    if not os.path.isfile(pcd_path):
        raise MapEditError(f'map/{map_name}.pcd がありません。', 404)
    saved = load_params(data_dir, map_name)
    map_data_dir = os.path.join(data_dir, map_name)
    source_file = os.path.join(map_data_dir, SOURCE_POINTS_FILE_NAME)
    pending_info = _read_pending_info(data_dir, map_name)

    if saved:
        source = dict(saved.get('source') or {})
        pcd = dict(saved.get('pcd') or {})
        pgm = normalize_pgm_params(saved.get('pgm'))
        # 高さの基準を保存していない (この設定ができる前に作った) 地図は、ブラウザ側で既定を決める
        if not (saved.get('pgm') or {}).get('height_reference'):
            pgm['height_reference'] = None
    else:
        resolution = _read_yaml_resolution(map_dir, map_name)
        source = {'type': 'pcd', 'spacing': 0.0, 'region': None}
        pcd = {'spacing': 0.0, 'crop': None, 'z_min': None, 'z_max': None}
        pgm = normalize_pgm_params({'resolution': resolution} if resolution else None)
        pgm['height_reference'] = None
    source['file'] = (f'data/{map_name}/{SOURCE_POINTS_FILE_NAME}' if os.path.isfile(source_file)
                      else f'map/{map_name}.pcd')
    return {
        'map_name': map_name,
        'has_saved_params': saved is not None,
        'source': source,
        'pending_source': pending_info,
        'pcd': pcd,
        'pgm': pgm,
        'origin': (saved or {}).get('origin'),
        'updated_at': (saved or {}).get('updated_at'),
    }


def _read_pending_info(data_dir, map_name):
    info_path = os.path.join(data_dir, map_name, PENDING_SOURCE_INFO_NAME)
    if not os.path.isfile(os.path.join(data_dir, map_name, PENDING_SOURCE_FILE_NAME)):
        return None
    try:
        with open(info_path, 'r', encoding='utf-8') as f:
            info = json.load(f)
        return info if isinstance(info, dict) else None
    except (OSError, ValueError):
        return None


# 同じ点群を何度も読み直さないためのキャッシュ (プレビューを繰り返すため)
_source_cache = {}
_derived_cache = {}


def _file_key(path):
    st = os.stat(path)
    return (path, st.st_mtime_ns, st.st_size)


def _load_points_cached(path):
    key = _file_key(path)
    if _source_cache.get('key') != key:
        points, colors, fields = read_pcd(path)
        _source_cache.clear()
        _source_cache.update({'key': key, 'points': points, 'colors': colors, 'fields': fields})
    return _source_cache['points'], _source_cache['colors'], _source_cache['fields']


def _source_for(map_dir, data_dir, map_name, use_pending):
    """元の点群のパスと説明 (type, spacing, region) を返す。"""
    described = describe_map_params(map_dir, data_dir, map_name)
    map_data_dir = os.path.join(data_dir, map_name)
    if use_pending:
        pending = described['pending_source']
        if not pending:
            raise MapEditError('取り直した点群がありません。もう一度取り直してください。', 409)
        return os.path.join(map_data_dir, PENDING_SOURCE_FILE_NAME), pending, described
    source_file = os.path.join(map_data_dir, SOURCE_POINTS_FILE_NAME)
    path = source_file if os.path.isfile(source_file) else os.path.join(map_dir, f'{map_name}.pcd')
    return path, described['source'], described


def build_edited_map(map_dir, data_dir, map_name, pcd_params, pgm_params, use_pending=False,
                     need_pgm=True):
    """元の点群と設定から、PCD の点群と PGM を作る (ファイルには書かない)。"""
    source_path, source, described = _source_for(map_dir, data_dir, map_name, use_pending)
    source_spacing = float(source.get('spacing') or 0.0)
    pcd_params = normalize_pcd_params(pcd_params, source_spacing)
    pgm_params = normalize_pgm_params(pgm_params)

    cache_key = (_file_key(source_path), json.dumps(pcd_params, sort_keys=True))
    if _derived_cache.get('key') == cache_key:
        points = _derived_cache['points']
        colors = _derived_cache['colors']
        source_count = _derived_cache['source_count']
        fields = _derived_cache['fields']
    else:
        source_points, source_colors, fields = _load_points_cached(source_path)
        points, colors = derive_points(source_points, pcd_params, source_spacing, source_colors)
        source_count = int(source_points.shape[0])
        _derived_cache.clear()
        _derived_cache.update({'key': cache_key, 'points': points, 'colors': colors,
                               'source_count': source_count, 'fields': fields})
    if points.shape[0] == 0:
        raise MapEditError('この設定では点が 1 つも残りません。範囲や高さの制限を見直してください。')

    result = {
        'points': points,
        'colors': colors,
        'source_path': source_path,
        'source': source,
        'source_count': source_count,
        # PCD に書き出すのは x y z (と色) だけ
        'extra_fields': [f for f in fields if f not in ('x', 'y', 'z')
                         and not (colors is not None and f in ('rgb', 'rgba'))],
        'pcd_params': pcd_params,
        'pgm_params': pgm_params,
        'changes_points': pcd_params_change_points(pcd_params, source_spacing),
        'described': described,
    }
    if need_pgm:
        bounds = pgm_bounds_for(points, pcd_params, source.get('region'))
        pgm, map_info, stats = build_occupancy_pgm(points, bounds, **pgm_params)
        result.update({'pgm': pgm, 'map_info': map_info, 'pgm_stats': stats})
    return result


def save_pending_source(data_dir, map_name, points, info, colors=None):
    """Google から取り直した点群を、保存するまでの間 data/<名前>/ に置いておく。"""
    map_data_dir = os.path.join(data_dir, map_name)
    os.makedirs(map_data_dir, exist_ok=True)
    write_pcd_binary(os.path.join(map_data_dir, PENDING_SOURCE_FILE_NAME), points, colors)
    info = {**info, 'fetched_at': _now_iso(), 'points': int(points.shape[0]), 'has_color': colors is not None}
    text = json.dumps(info, ensure_ascii=False, indent=2) + '\n'
    _replace_file(os.path.join(map_data_dir, PENDING_SOURCE_INFO_NAME), lambda f: f.write(text.encode('utf-8')))
    return info


def save_edited_map(map_dir, data_dir, map_name, save_as, pcd_params, pgm_params,
                    use_pending=False, pgm_pixels=None, overwrite=False):
    """設定を変えて作り直した地図を保存する。

    pgm_pixels: ブラウザで PGM をさらにブラシで直した画素 (PGM ファイルと同じ並びのバイト列)。
                作り直した PGM と大きさが同じときだけ使う。
    """
    built = build_edited_map(map_dir, data_dir, map_name, pcd_params, pgm_params, use_pending)
    pgm = built['pgm']
    height, width = pgm.shape
    if pgm_pixels is not None:
        if len(pgm_pixels) != width * height:
            raise MapEditError('2Dマップの大きさがプレビューと合いません。プレビューし直してから保存してください。', 409)
        pgm = np.frombuffer(pgm_pixels, dtype=np.uint8).reshape(height, width)

    same_name = save_as == map_name
    src_data_dir = os.path.join(data_dir, map_name)
    dst_data_dir = os.path.join(data_dir, save_as)
    dst_pcd = os.path.join(map_dir, f'{save_as}.pcd')
    if not same_name and not overwrite:
        existing = [p for p in (dst_pcd, os.path.join(map_dir, f'{save_as}.pgm'),
                                os.path.join(map_dir, f'{save_as}.yaml'), dst_data_dir) if os.path.exists(p)]
        if existing:
            raise MapEditError('同じ名前の地図があります: ' + ', '.join(os.path.basename(p) for p in existing)
                               + '。上書きする場合は「同じ名前の地図があれば上書きする」にチェックしてください。', 409)

    os.makedirs(dst_data_dir, exist_ok=True)
    written = []
    source = dict(built['source'])
    src_pcd = os.path.join(map_dir, f'{map_name}.pcd')
    src_source_file = os.path.join(src_data_dir, SOURCE_POINTS_FILE_NAME)
    dst_source_file = os.path.join(dst_data_dir, SOURCE_POINTS_FILE_NAME)

    # 1. 元の点群を残す (PCD を書き換えても、あとで元の点群から作り直せるように)
    if use_pending:
        pending_file = os.path.join(src_data_dir, PENDING_SOURCE_FILE_NAME)
        pending_info = os.path.join(src_data_dir, PENDING_SOURCE_INFO_NAME)
        if same_name:
            os.replace(pending_file, dst_source_file)
            os.remove(pending_info)
        else:
            shutil.copyfile(pending_file, dst_source_file)
        written.append(dst_source_file)
        source_kept = True
    elif os.path.isfile(src_source_file):
        if not same_name:
            shutil.copyfile(src_source_file, dst_source_file)
            written.append(dst_source_file)
        source_kept = True
    elif built['changes_points']:
        # まだ元の点群を残していない (map/<名前>.pcd が元の点群) ので、書き換える前に残す
        shutil.copyfile(src_pcd, dst_source_file)
        written.append(dst_source_file)
        source_kept = True
    else:
        source_kept = False
    if not source_kept and not same_name and os.path.exists(dst_source_file):
        # 上書きした先に別の地図の元の点群が残っていると、そちらから作り直してしまう
        os.remove(dst_source_file)

    # 2. PCD
    # 元の点群が map/<名前>.pcd 以外 (source_points.pcd や取り直した点群) なら、作った点群を書く。
    # map/<名前>.pcd が元の点群で点が減らないなら、中身は同じなので書き換えない (別名なら複製)。
    derived_differs = use_pending or os.path.isfile(src_source_file) or built['changes_points']
    if derived_differs:
        write_pcd_binary(dst_pcd, built['points'], built['colors'])
    elif not same_name:
        # x y z 以外の項目も残るよう、元のファイルをそのまま複製する
        shutil.copyfile(src_pcd, dst_pcd)
    pcd_written = derived_differs or not same_name
    if pcd_written:
        written.append(dst_pcd)

    # 3. PGM / YAML
    pgm_path, yaml_path, map_info = write_pgm_and_yaml(map_dir, save_as, pgm, built['map_info'])
    written += [pgm_path, yaml_path]

    # 4. 原点・初期位置 (別名で保存するときは引き継ぐ)
    if not same_name:
        for name in ('init_lat_lon_alt.txt', 'init_pose.txt'):
            src = os.path.join(src_data_dir, name)
            if os.path.isfile(src):
                shutil.copyfile(src, os.path.join(dst_data_dir, name))
                written.append(os.path.join(dst_data_dir, name))

    # 5. 設定
    previous = load_params(data_dir, map_name) or {}
    source.pop('file', None)
    params = {
        'version': 1,
        'created_at': previous.get('created_at') or _now_iso(),
        'updated_at': _now_iso(),
        'origin': previous.get('origin'),
        'source': {k: v for k, v in source.items() if k not in ('points',)},
        'pcd': built['pcd_params'],
        'pgm': built['pgm_params'],
    }
    if params['origin'] is None:
        params.pop('origin')
    write_params(data_dir, save_as, params)
    written.append(params_path(data_dir, save_as))

    # PCD の中身が変わったので、キャッシュは次のプレビューで作り直す
    _derived_cache.clear()
    return {
        'files': written,
        'points': int(built['points'].shape[0]),
        'source_points': built['source_count'],
        'pcd_written': pcd_written,
        'extra_fields_dropped': built['extra_fields'] if derived_differs else [],
        'pgm': {'width': width, 'height': height,
                'occupied_cells': int((pgm == PGM_OCCUPIED).sum())},
        'yaml': map_info,
        'params': params,
    }
