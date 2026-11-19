#!/usr/bin/env python3
"""障害物検出設定 (検出範囲: 停止判定の範囲・不感帯・costmap 除外) を obstacle_monitor.yaml から読み書きする。

検出範囲はウェイポイントの属性ごとに zone.<属性>.<項目> のパラメータで持つ
(obstacle_monitor.cpp と同じ)。ウェイポイント個別の上書きはウェイポイントの属性の zone_<項目> に書く。
yaml のコメントを残すため、PyYAML で丸ごと書き直さず、対象のキーの行の値だけを
書き換える。実行中の obstacle_monitor へは `ros2 param load` で送る
(obstacle_monitor は zone.* を実行中に変更できる)。
"""
import json
import os
import re
import subprocess
import tempfile

import yaml

NODE_NAME = '/obstacle_monitor'

# ウェイポイントの属性 (waypoint_manager の type と同じ)
ZONE_TYPES = ('normal', 'slow', 'stop', 'wait', 'pass_through')

# 属性ごとの項目: 名前 -> (型, 最小, 最大, 説明)
ZONE_FIELDS = {
    'path_clearance_left': (float, 0.05, 2.0, '経路の中心線から左側この距離以内を停止判定する [m]'),
    'path_clearance_right': (float, 0.05, 2.0, '経路の中心線から右側この距離以内を停止判定する [m]'),
    'stop_distance': (float, 0.2, 10.0, 'プランに沿って前方この距離以内を停止判定する [m]'),
    'stop_distance_back': (float, 0.0, 3.0, 'ロボットの位置から後方この距離までを停止判定する [m]'),
    'min_blocking_cells': (int, 1, 1000, '検出帯のセルがこの数以上で「塞がれている」として止まる'),
    'deadband_left_ratio': (float, 0.0, 1.0, '不感帯の左側の幅 (左の幅 path_clearance_left に対する割合)'),
    'deadband_right_ratio': (float, 0.0, 1.0, '不感帯の右側の幅 (右の幅 path_clearance_right に対する割合)'),
    'costmap_margin': (float, 0.0, 1.5, '不感帯の外側で local_costmap に入れない幅 [m] (停止判定には使う)'),
    'lookahead': (float, 0.0, 20.0, '不感帯・costmap に入れない範囲を前方何 m のプランまで取るか [m]'),
    'lookbehind': (float, 0.0, 10.0, '不感帯・costmap に入れない範囲を後方何 m まで取るか [m]'),
}

# 既定値 (obstacle_monitor.cpp と同じ)。pass_through だけ不感帯・costmap 除外がある
ZONE_DEFAULTS = {
    'path_clearance_left': 0.45, 'path_clearance_right': 0.45, 'stop_distance': 2.0, 'stop_distance_back': 0.0,
    'min_blocking_cells': 2, 'deadband_left_ratio': 0.0, 'deadband_right_ratio': 0.0, 'costmap_margin': 0.0,
    'lookahead': 6.0, 'lookbehind': 1.0,
}
ZONE_DEFAULTS_BY_TYPE = {
    'pass_through': {'deadband_left_ratio': 1.0, 'deadband_right_ratio': 0.0, 'costmap_margin': 0.35},
}


def zone_key(zone_type, field):
    return f'zone.{zone_type}.{field}'


def zone_default(zone_type, field):
    return ZONE_DEFAULTS_BY_TYPE.get(zone_type, {}).get(field, ZONE_DEFAULTS[field])


# GUI から変更できるパラメータ: 名前 -> (型, 最小, 最大, 説明)
EDITABLE_PARAMS = {zone_key(t, f): spec for t in ZONE_TYPES for f, spec in ZONE_FIELDS.items()}
EDITABLE_PARAMS['zone_timeout'] = (float, 0.1, 10.0, '検出範囲の指令がこの時間途絶えたら normal の設定に戻す [s]')

# 描画に使う (変更はしない) パラメータ。後半は YVT で見える範囲の描画に使う
REFERENCE_PARAMS = (
    'footprint_min_x', 'footprint_max_x', 'footprint_half_width', 'ground_z', 'cell_size',
    'min_range', 'max_range', 'min_obstacle_height', 'max_obstacle_height', 'self_filter_margin',
    'sensor_min_elevation_deg', 'sensor_horizontal_fov_deg',
)


def _line_pattern(key):
    # "    key: value   # comment" の value 部分を取り出す
    return re.compile(r'^(?P<head>\s*' + re.escape(key) + r'\s*:\s*)(?P<value>[^#\n]*?)(?P<tail>\s*(#.*)?)$',
                      re.MULTILINE)


def load(yaml_path):
    """yaml から編集対象と参照用のパラメータを読む。書かれていない項目は既定値にする。"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = {}
    for node in data.values():
        if isinstance(node, dict) and isinstance(node.get('ros__parameters'), dict):
            params = node['ros__parameters']
            break
    values = {k: params.get(k) for k in EDITABLE_PARAMS}
    for t in ZONE_TYPES:
        for f in ZONE_FIELDS:
            if values[zone_key(t, f)] is None:
                values[zone_key(t, f)] = zone_default(t, f)
    if values['zone_timeout'] is None:
        values['zone_timeout'] = 1.0
    reference = {k: params.get(k) for k in REFERENCE_PARAMS}
    return values, reference


def validate(values):
    """GUI から来た値を検査し、型をそろえた dict を返す。不正なら ValueError。"""
    out = {}
    for key, raw in values.items():
        if key not in EDITABLE_PARAMS:
            raise ValueError(f'変更できないパラメータです: {key}')
        typ, lo, hi, _ = EDITABLE_PARAMS[key]
        try:
            v = float(raw)
        except (TypeError, ValueError):
            raise ValueError(f'{key} が数値ではありません: {raw!r}')
        if typ is int:
            if v != int(v):
                raise ValueError(f'{key} は整数にしてください: {raw!r}')
            v = int(v)
        if not (lo <= v <= hi):
            raise ValueError(f'{key} は {lo}〜{hi} の範囲にしてください: {v}')
        out[key] = v
    return out


def _format(key, v):
    # double のパラメータに "1" と書くと ROS が整数と解釈して起動に失敗するため、必ず小数点を付ける
    if EDITABLE_PARAMS[key][0] is int:
        return str(int(v))
    s = f'{v:.3f}'.rstrip('0')
    return s + '0' if s.endswith('.') else s


def save(yaml_path, values):
    """yaml の対象キーの値だけを書き換える (コメント・並びはそのまま)。書き換えたキーの一覧を返す。"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        text = f.read()
    changed = []
    for key, v in values.items():
        pat = _line_pattern(key)
        matches = list(pat.finditer(text))
        new_value = _format(key, v)
        if len(matches) > 1:
            raise ValueError(f'{key} が yaml に複数あります')
        if not matches:
            # 無ければ ros__parameters の末尾 (最後の行) に追加する
            indent = re.search(r'^(\s*)ros__parameters\s*:', text, re.MULTILINE)
            pad = (indent.group(1) if indent else '  ') + '  '
            text = text.rstrip('\n') + f'\n{pad}{key}: {new_value}\n'
            changed.append(key)
            continue
        m = matches[0]
        if m.group('value').strip() == new_value:
            continue
        text = text[:m.start('value')] + new_value + text[m.end('value'):]
        changed.append(key)
    if changed:
        tmp = yaml_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write(text)
        os.replace(tmp, yaml_path)
    return changed


def apply_live(values, timeout=20.0):
    """実行中の obstacle_monitor へパラメータを送る。(成功したか, 出力) を返す。"""
    params = {k: (int(v) if EDITABLE_PARAMS[k][0] is int else float(v)) for k, v in values.items()}
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8') as f:
        yaml.safe_dump({NODE_NAME: {'ros__parameters': params}}, f)
        path = f.name
    try:
        proc = subprocess.run(['ros2', 'param', 'load', NODE_NAME, path],
                              capture_output=True, text=True, timeout=timeout)
        out = (proc.stdout + proc.stderr).strip()
        ok = proc.returncode == 0 and 'Node not found' not in out and 'Failed' not in out
        return ok, out
    except FileNotFoundError:
        return False, 'ros2 コマンドが見つかりません (ROS 2 の環境が読み込まれていません)'
    except subprocess.TimeoutExpired:
        return False, 'ros2 param load が時間内に終わりませんでした'
    finally:
        os.unlink(path)


def read_live(timeout=15.0):
    """実行中の obstacle_monitor の値を読む。動いていなければ None。"""
    try:
        proc = subprocess.run(['ros2', 'param', 'dump', NODE_NAME],
                              capture_output=True, text=True, timeout=timeout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    try:
        data = yaml.safe_load(proc.stdout) or {}
    except yaml.YAMLError:
        return None
    for node in data.values():
        if isinstance(node, dict) and isinstance(node.get('ros__parameters'), dict):
            p = node['ros__parameters']
            return {k: p.get(k) for k in EDITABLE_PARAMS}
    return None


# ---------------------------------------------------------------------------
# ウェイポイント個別の設定 (ウェイポイントの属性の zone_<項目>)
#   waypoint_manager は、到着したウェイポイントから次のウェイポイントまでの区間で、
#   属性 (type) ごとの設定にこの値を重ねて obstacle_monitor に送る。
#   保存は zone_* のキーだけを書き換え、位置・向き・他の属性・緯度経度などはファイルのまま残す
#   (Map Viewer で同時に編集した内容を古い値で上書きしないように)。
# ---------------------------------------------------------------------------
WAYPOINT_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_.\-]+$')


def waypoint_path(wp_dir, name):
    """ウェイポイント名 (拡張子なし) からファイルのパスを返す。不正な名前なら ValueError。"""
    if not name or not WAYPOINT_NAME_PATTERN.match(name) or name.startswith('.'):
        raise ValueError(f'ウェイポイント名が不正です: {name!r}')
    return os.path.join(wp_dir, name + '.json')


def list_waypoint_files(wp_dir):
    try:
        names = os.listdir(wp_dir)
    except OSError:
        return []
    return sorted(n[:-5] for n in names if n.endswith('.json') and WAYPOINT_NAME_PATTERN.match(n[:-5]))


def _attribute(item):
    """ウェイポイント [位置, 向き, 属性, ...] の属性 dict (無ければ None)"""
    if len(item) < 3:
        return None
    attr = item[2]
    if isinstance(attr, list):
        attr = attr[0] if attr else None
    return attr if isinstance(attr, dict) else None


def load_waypoints(wp_dir, name):
    """ウェイポイントごとに {index, type, value, overrides} を返す。"""
    with open(waypoint_path(wp_dir, name), 'r', encoding='utf-8') as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise ValueError('ウェイポイントファイルの形式が違います (リストではありません)')
    out = []
    for i, item in enumerate(data):
        attr = _attribute(item) if isinstance(item, list) else None
        attr = attr or {}
        overrides = {f: attr['zone_' + f] for f in ZONE_FIELDS if attr.get('zone_' + f) not in (None, '')}
        pos = item[0] if isinstance(item, list) and item and isinstance(item[0], list) else [0, 0, 0]
        out.append({'index': i, 'type': attr.get('type', 'normal'), 'value': attr.get('value', 0),
                    'x': pos[0] if len(pos) > 0 else 0, 'y': pos[1] if len(pos) > 1 else 0,
                    'overrides': overrides})
    return out


def save_zone_overrides(wp_dir, name, edits):
    """edits = {index: {項目: 値 or None}} を zone_<項目> に書き込む (None は消す)。変更したウェイポイント数を返す。"""
    path = waypoint_path(wp_dir, name)
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    changed = 0
    for key, fields in (edits or {}).items():
        try:
            index = int(key)
        except (TypeError, ValueError):
            raise ValueError(f'ウェイポイント番号が不正です: {key!r}')
        if not (0 <= index < len(data)) or not isinstance(data[index], list):
            raise ValueError(f'ウェイポイント {index} がありません')
        item = data[index]
        while len(item) < 3:
            item.append({} if len(item) == 2 else [0, 0, 0, 1])
        attr = _attribute(item)
        if attr is None:
            attr = {'type': 'normal', 'value': 0}
            item[2] = attr
        for field, raw in (fields or {}).items():
            if field not in ZONE_FIELDS:
                raise ValueError(f'個別設定できない項目です: {field}')
            k = 'zone_' + field
            if raw is None or raw == '':
                attr.pop(k, None)
                continue
            typ, lo, hi, _ = ZONE_FIELDS[field]
            try:
                v = float(raw)
            except (TypeError, ValueError):
                raise ValueError(f'ウェイポイント {index} の {field} が数値ではありません: {raw!r}')
            if typ is int:
                if v != int(v):
                    raise ValueError(f'ウェイポイント {index} の {field} は整数にしてください')
                v = int(v)
            if not (lo <= v <= hi):
                raise ValueError(f'ウェイポイント {index} の {field} は {lo}〜{hi} にしてください: {v}')
            attr[k] = v
        changed += 1
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    os.replace(tmp, path)
    return changed
