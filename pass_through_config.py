#!/usr/bin/env python3
"""通過モード (pass_through) の設定を obstacle_monitor.yaml から読み書きする。

yaml のコメントを残すため、PyYAML で丸ごと書き直さず、対象のキーの行の値だけを
書き換える。実行中の obstacle_monitor へは `ros2 param load` で送る
(obstacle_monitor は停止判定・通過モードのパラメータを実行中に変更できる)。
"""
import os
import re
import subprocess
import tempfile

import yaml

NODE_NAME = '/obstacle_monitor'

# GUI から変更できるパラメータ: 名前 -> (型, 最小, 最大, 説明)
EDITABLE_PARAMS = {
    'path_clearance': (float, 0.05, 2.0, '経路の中心線から左右この距離以内を停止判定する [m] (通常走行にも影響)'),
    'stop_distance': (float, 0.2, 10.0, 'プラン前方この距離以内を停止判定する [m] (通常走行にも影響)'),
    'pass_through_deadband_left_ratio': (float, 0.0, 1.0, '不感帯の左側の幅 (path_clearance に対する割合)'),
    'pass_through_deadband_right_ratio': (float, 0.0, 1.0, '不感帯の右側の幅 (path_clearance に対する割合)'),
    'pass_through_costmap_margin': (float, 0.0, 1.5, '不感帯の外側で local_costmap に入れない幅 [m]'),
    'pass_through_lookahead': (float, 0.5, 20.0, '不感帯を前方何 m のプランまで取るか [m]'),
    'pass_through_min_blocking_cells': (int, 1, 1000, '通過モード中に「塞がれている」とするセル数'),
    'pass_through_timeout': (float, 0.1, 10.0, '指令がこの時間途絶えたら通過モードを終了する [s]'),
}

# 描画に使う (変更はしない) パラメータ
REFERENCE_PARAMS = (
    'footprint_min_x', 'footprint_max_x', 'footprint_half_width', 'ground_z', 'cell_size',
    'min_blocking_cells',
)


def _line_pattern(key):
    # "    key: value   # comment" の value 部分を取り出す
    return re.compile(r'^(?P<head>\s*' + re.escape(key) + r'\s*:\s*)(?P<value>[^#\n]*?)(?P<tail>\s*(#.*)?)$',
                      re.MULTILINE)


def load(yaml_path):
    """yaml から編集対象と参照用のパラメータを読む。"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = {}
    for node in data.values():
        if isinstance(node, dict) and isinstance(node.get('ros__parameters'), dict):
            params = node['ros__parameters']
            break
    values = {k: params.get(k) for k in EDITABLE_PARAMS}
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
