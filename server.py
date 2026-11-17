#!/usr/bin/env python3

import os
import shutil
import zipfile 
import subprocess
from threading import Thread, Lock
import sys
import asyncio
import time
import yaml
import pathlib
import csv # Added for CSV handling
import json # Added for JSON handling
import math
import re
import urllib.error
import urllib.parse
import urllib.request

from flask import Flask, request, render_template, redirect, jsonify, url_for, flash, send_from_directory
from flask_sockets import Sockets
import websockets
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
from geventwebsocket.websocket import WebSocket

from ament_index_python.packages import get_package_prefix, PackageNotFoundError

try:
    import map_from_points
except ImportError as e:
    print(f"Warning: map_from_points を読み込めません (Google から地図を作る機能は使えません): {e}")
    map_from_points = None

try:
    import map_rebase
except ImportError as e:
    print(f"Warning: map_rebase を読み込めません (初期位置に地図の原点を合わせる機能は使えません): {e}")
    map_rebase = None

try:
    from rosbag2_filter_core import get_topic_list, filter_rosbag
except ImportError as e:
    print(f"Error: Core logic file (rosbag2_filter_core.py) or ROS 2 libraries not found/sourced: {e}")

# ROS Bag の詳細表示・設定診断・実行ログ翻訳。
# ROS 環境が読み込まれていない場合でも GUI 自体は起動できるように、
# 読み込み失敗を握りつぶして機能だけを無効化する。
try:
    from rosbag_inspect import inspect_bag
except ImportError as e:
    inspect_bag = None
    print(f"Warning: rosbag_inspect could not be imported. ROS Bag details are disabled: {e}")

try:
    import mapping_diagnostics
except ImportError as e:
    mapping_diagnostics = None
    print(f"Warning: mapping_diagnostics could not be imported. Config check is disabled: {e}")

try:
    import log_insight
except ImportError as e:
    log_insight = None
    print(f"Warning: log_insight could not be imported. Log translation is disabled: {e}")

try:
    # ament_index_python を使って 'hokuyo_navigation2' パッケージのインストールプレフィックスを取得
    package_install_prefix = get_package_prefix('hokuyo_navigation2')
    
    # インストールプレフィックスからソースパスを堅牢に導出
    # 例: /path/to/ws/install/pkg -> /path/to/ws/src/pkg
    install_base_dir = os.path.dirname(package_install_prefix) # /path/to/ws/install
    workspace_root = os.path.dirname(install_base_dir) # /path/to/ws
    package_name = os.path.basename(package_install_prefix) # pkg
    
    HOKUYO_NAV2_PKG_PATH = os.path.join(workspace_root, 'src', package_name, package_name)
    BASE_PATH = os.path.join(HOKUYO_NAV2_PKG_PATH, 'scripts')

    if os.path.isdir(HOKUYO_NAV2_PKG_PATH) and 'src' in HOKUYO_NAV2_PKG_PATH:
        print(f"Successfully derived source path for 'hokuyo_navigation2' package at: {HOKUYO_NAV2_PKG_PATH}")
    else:
        print(f"Warning: Could not derive source path. Using install path: {HOKUYO_NAV2_PKG_PATH}")
except PackageNotFoundError:
    print("Warning: Could not find 'hokuyo_navigation2' package through ament. Falling back to hardcoded paths.")
    # フォールバックとして、元のハードコードされたパスを使用
    if 'DOCKER_ENV' in os.environ:
        BASE_PATH = "/home/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2/scripts/"
        HOKUYO_NAV2_PKG_PATH = "/home/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2" 
    else:
        BASE_PATH = "/home/hokuyo/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2/scripts/"
        HOKUYO_NAV2_PKG_PATH = "/home/hokuyo/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2/"

ROSBAG_ROOT_DIR = os.path.join(HOKUYO_NAV2_PKG_PATH, 'rosbag')
DOWNLOAD_FOLDER = ROSBAG_ROOT_DIR 
ALLOWED_EXTENSIONS = {'bag', 'db3', 'mcap'}

MAP_DIR = os.path.join(HOKUYO_NAV2_PKG_PATH, 'map')
WP_DIR = os.path.join(HOKUYO_NAV2_PKG_PATH, 'waypoints')
CONFIG_DIR = os.path.join(HOKUYO_NAV2_PKG_PATH, 'config')
# マップごとの付随データ置き場。自己位置推定の初期位置 (init_pose.txt) はここにある。
DATA_DIR = os.path.join(HOKUYO_NAV2_PKG_PATH, 'data')
# マッピングやPCD変換は別ターミナルで動くため、実行ログをここに書き出して
# ブラウザから読めるようにする。
LOG_DIR = os.path.join(HOKUYO_NAV2_PKG_PATH, 'log', 'gui')

ROSBRIDGE_URI = "ws://localhost:9090"
SERVER_HOST = '0.0.0.0'
SERVER_PORT = 5050

app = Flask(__name__)
sockets = Sockets(app)
app.secret_key = 'your_secret_key_here' 

current_mode = "stopped"

os.makedirs(ROSBAG_ROOT_DIR, exist_ok=True)
os.makedirs(DOWNLOAD_FOLDER, exist_ok=True)
os.makedirs(MAP_DIR, exist_ok=True)
os.makedirs(WP_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# 実行ログに終了コードを書き残すための目印。log_insight がこの行を読んで
# 処理の成功・失敗を判定する（通常は start_mapping.sh 側が書き出す）。
EXIT_MARKER = log_insight.EXIT_MARKER if log_insight else '[HOKUYO_GUI] exit_code='

# 実行中／実行済みジョブの一覧。
# job_id -> {'mode', 'name', 'log_path', 'started', 'seen_error'}
# ブラウザは job_id を使って /mapping_log から実行ログを取得する。
_jobs = {}
_jobs_lock = Lock()
_JOB_ID_PATTERN = re.compile(r'^[A-Za-z0-9_.-]+$')
# 1回のポーリングでブラウザに返すログの上限（バイト）。
LOG_CHUNK_LIMIT = 64 * 1024
# 保存しておく実行ログの本数。これより古いものは処理開始時に削除する。
MAX_KEPT_LOGS = 50

def zip_directory(path, zip_filename):
    """指定されたパスのディレクトリをZIPファイルに圧縮する"""
    with zipfile.ZipFile(zip_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
        root_dir_name = os.path.basename(path)
        for root, _, files in os.walk(path):
            for file in files:
                file_path = os.path.join(root, file)
                archive_path = os.path.join(root_dir_name, os.path.relpath(file_path, path))
                zipf.write(file_path, archive_path)

def _job_log_path(job_id):
    """job_id に対応する実行ログのパスを返す。不正な job_id なら None。"""
    if not job_id or not _JOB_ID_PATTERN.match(job_id):
        return None
    log_path = os.path.join(LOG_DIR, f'{job_id}.log')
    if not _is_safe_path(log_path, LOG_DIR):
        return None
    return log_path


def _prune_logs():
    """古い実行ログを削除して、ログフォルダが際限なく増えるのを防ぐ。"""
    try:
        logs = sorted(
            (entry.path for entry in os.scandir(LOG_DIR)
             if entry.is_file() and entry.name.endswith('.log')),
            key=os.path.getmtime, reverse=True)
    except OSError:
        return
    for old_log in logs[MAX_KEPT_LOGS:]:
        try:
            os.remove(old_log)
        except OSError:
            pass


def create_job(mode, name):
    """実行ログを1本のファイルに紐づけるためのジョブを作成する。

    戻り値: (job_id, ログファイルのパス)
    """
    _prune_logs()
    safe_name = re.sub(r'[^A-Za-z0-9_.-]', '_', name or 'job')[:60]
    job_id = f"{mode}_{safe_name}_{time.strftime('%Y%m%d-%H%M%S')}"
    log_path = os.path.join(LOG_DIR, f'{job_id}.log')
    header = (f"[HOKUYO_GUI] {time.strftime('%Y-%m-%d %H:%M:%S')} "
              f"{mode} の処理を開始します (出力名: {name})\n")
    try:
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write(header)
    except OSError as e:
        print(f"Warning: Failed to create log file {log_path}: {e}")
    with _jobs_lock:
        _jobs[job_id] = {
            'mode': mode,
            'name': name,
            'log_path': log_path,
            'started': time.time(),
            'seen_error': False,
        }
    return job_id, log_path


def _append_log(log_path, text):
    """実行ログに追記する。ログが書けなくても処理自体は続行する。"""
    if not log_path:
        return
    try:
        # 別ターミナル側の tee と同時に書き込むため、必ず追記モードで開く。
        with open(log_path, 'a', encoding='utf-8') as f:
            f.write(text)
    except OSError as e:
        print(f"Warning: Failed to write log file {log_path}: {e}")


def run_subprocess(command_list, log_path=None, extra_env=None):
    """別スレッドでサブプロセスを実行するための関数

    log_path が指定されている場合、標準出力を画面と同時にそのファイルへ書き出し、
    ブラウザから /mapping_log で読めるようにする。
    起動されるスクリプト側も環境変数 HOKUYO_GUI_LOG_FILE 経由で同じファイルに
    追記するため、別ターミナルで動く本処理のログもここに集まる。
    """
    env = os.environ.copy()
    if log_path:
        env['HOKUYO_GUI_LOG_FILE'] = log_path
    if extra_env:
        env.update(extra_env)
    try:
        # Popenを使い、新しいプロセスグループで実行する (preexec_fn=os.setsid)
        # これにより、killall等が親プロセスに影響を与えるのを防ぐ
        process = subprocess.Popen(
            command_list,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # 標準エラー出力を標準出力にリダイレクト
            text=True,
            bufsize=1,  # 行バッファリングを有効化
            preexec_fn=os.setsid,  # 新しいセッションでプロセスを開始
            env=env
        )
        
        # 標準出力をリアルタイムで読み込んで表示
        if process.stdout:
            for line in iter(process.stdout.readline, ''):
                # [stdout]や[stderr]のプレフィックスを付けずにそのまま出力
                print(line.strip())
                _append_log(log_path, line if line.endswith('\n') else line + '\n')

        process.wait() # プロセスの終了を待つ

        print(f"Subprocess finished with code {process.returncode}: {command_list}")
    except subprocess.CalledProcessError as e:
        print(f"Subprocess failed: {e}")
        print(f"Stdout: {e.stdout}")
        # 起動できなかった場合はスクリプト側が終了コードを書けないため、
        # ここで書いておく。これが無いとブラウザが完了を待ち続けてしまう。
        _append_log(log_path, f'ERROR: 処理の起動に失敗しました: {e}\n{EXIT_MARKER}1\n')
    except FileNotFoundError:
        print(f"Subprocess failed: Command not found or script path error: {command_list}")
        _append_log(log_path,
                    f'ERROR: 実行スクリプトが見つかりません: {command_list[0]}\n'
                    f'{EXIT_MARKER}1\n')


def _is_safe_path(full_path, root_dir):
    """ディレクトリトラバーサル攻撃を防ぐための安全なパスチェック"""
    absolute_root_dir = os.path.abspath(root_dir)
    absolute_full_path = os.path.abspath(full_path)
    return absolute_full_path.startswith(absolute_root_dir)


# 直近に解析した ROS Bag の情報。解析は数十msで終わるが、設定ファイルを
# 切り替えるたびに読み直すのは無駄なのでキャッシュする。
_bag_info_cache = {}
_bag_info_lock = Lock()
BAG_INFO_CACHE_SIZE = 8


def _bag_signature(bag_path):
    """ROS Bag の更新を検知するための署名（パスと更新時刻）を作る。"""
    try:
        return (os.path.abspath(bag_path), os.path.getmtime(bag_path))
    except OSError:
        return (os.path.abspath(bag_path), 0)


def get_bag_info(bag_path):
    """ROS Bag の詳細情報を取得する（同じBagならキャッシュを返す）。

    ROS 環境が無い場合や解析に失敗した場合は None を返し、
    呼び出し側は詳細表示なしで処理を続行する。
    """
    if inspect_bag is None:
        return None
    signature = _bag_signature(bag_path)
    with _bag_info_lock:
        cached = _bag_info_cache.get(signature)
    if cached is not None:
        return cached
    try:
        info = inspect_bag(bag_path)
    except Exception as e:
        print(f"Warning: Failed to inspect rosbag {bag_path}: {e}")
        return None
    with _bag_info_lock:
        if len(_bag_info_cache) >= BAG_INFO_CACHE_SIZE:
            _bag_info_cache.clear()
        _bag_info_cache[signature] = info
    return info


def _mapping_config_files():
    """マッピング用のヘッダを持つ設定CSVのファイル名を一覧する。"""
    config_files = []
    if os.path.exists(CONFIG_DIR):
        for filename in os.listdir(CONFIG_DIR):
            if filename.endswith('.csv') \
                    and _is_mapping_config(os.path.join(CONFIG_DIR, filename)):
                config_files.append(filename)
    config_files.sort()
    return config_files


def _rosbag_directories():
    """ROS Bag として使えるディレクトリを一覧する（候補の取得元を選ぶため）。

    戻り値: [{'name': 表示名, 'path': 絶対パス}, ...]
    """
    bags = []
    if not os.path.isdir(ROSBAG_ROOT_DIR):
        return bags
    for name in sorted(os.listdir(ROSBAG_ROOT_DIR)):
        full_path = os.path.join(ROSBAG_ROOT_DIR, name)
        if not os.path.isdir(full_path):
            continue
        try:
            entries = os.listdir(full_path)
        except OSError:
            continue
        if any(entry == 'metadata.yaml' or entry.endswith(('.db3', '.mcap'))
               for entry in entries):
            bags.append({'name': name, 'path': full_path})
    return bags


def _is_mapping_config(file_path):
    """マッピング用パラメータ設定CSV（見出しが一致するCSV）かどうかを判定する。"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.readline().strip() == "オプション,指定値,デフォルト値"
    except OSError:
        return False


def _resolve_config_path(config_filename):
    """設定ファイル名を CONFIG_DIR 内の絶対パスに変換する。不正なら空文字。"""
    if not config_filename:
        return ''
    config_path = os.path.join(CONFIG_DIR, os.path.basename(config_filename))
    if not _is_safe_path(config_path, CONFIG_DIR):
        return ''
    return config_path


def run_diagnostics(mode, config_filename, bag_info):
    """コンフィグと ROS Bag の食い違いを検査する。無効時は None。"""
    if mapping_diagnostics is None:
        return None
    try:
        return mapping_diagnostics.diagnose(
            mode, _resolve_config_path(config_filename), bag_info, scripts_dir=BASE_PATH)
    except Exception as e:
        print(f"Warning: Mapping diagnostics failed: {e}")
        return None

async def forward(ws, target):
    """WebSocket間でメッセージを転送する"""
    try:
        while True:
            message = await ws.recv() if isinstance(ws, websockets.legacy.client.WebSocketClientProtocol) else ws.receive()
            if message is None and isinstance(ws, WebSocket):
                break
            await target.send(message)
    except (websockets.exceptions.ConnectionClosedOK, websockets.exceptions.ConnectionClosedError, Exception):
        pass

async def proxy(websocket):
    """クライアントとROSBridge間のWebSocketプロキシを確立する"""
    ros_ws = None
    try:
        ros_ws = await websockets.connect(ROSBRIDGE_URI)
        await asyncio.gather(
            forward(websocket, ros_ws),
            forward(ros_ws, websocket),
        )
    except ConnectionRefusedError:
        print(f"Proxy: Error: Could not connect to ROSBridge at {ROSBRIDGE_URI}. Make sure it's running.")
    except Exception:
        pass
    finally:
        if ros_ws and not ros_ws.closed:
            await ros_ws.close()

@sockets.route('/ws')
def websocket_handler(ws):
    """WebSocket接続を処理し、asyncioイベントループでプロキシを実行する"""
    asyncio.run(proxy(ws))

@app.route('/')
@app.route('/gui')
def main_gui():
    """メインGUIページ (index.html)"""
    # request.host は 'hostname:port' 形式なので、ホスト名(IPアドレス)だけを抽出
    viewer_host = request.host.split(':')[0]
    return render_template('index.html', viewer_host=viewer_host)

@app.route('/get_mode')
def get_mode():
    """現在のシステムモードを返す (API)"""
    global current_mode
    return jsonify(mode=current_mode)

@app.route('/navigation_run_popup')
def navigation_run_popup():
    try:
        # MAP_DIRから.yamlファイルのベース名を取得
        map_files = sorted([
            os.path.splitext(f)[0] for f in os.listdir(MAP_DIR) if f.endswith('.yaml')
        ])
    except FileNotFoundError:
        map_files = []
        print(f"Warning: MAP_DIR not found at {MAP_DIR}")

    try:
        # WP_DIRから.jsonファイルのベース名を取得
        wp_files = sorted([os.path.splitext(f)[0] for f in os.listdir(WP_DIR) if f.endswith('.json')])
    except FileNotFoundError:
        wp_files = []
        print(f"Warning: WP_DIR not found at {WP_DIR}")

    try:
        # CONFIG_DIRから特定のヘッダーを持つ.csvファイル名を取得
        csv_files = []
        expected_header = "map_file,waypoint_file,nav_type,interval"
        for filename in os.listdir(CONFIG_DIR):
            if filename.endswith('.csv'):
                filepath = os.path.join(CONFIG_DIR, filename)
                with open(filepath, 'r', encoding='utf-8') as f:
                    header = f.readline().strip()
                    if header == expected_header:
                        csv_files.append(filename)
        csv_files.sort()
    except FileNotFoundError:
        csv_files = []
        print(f"Warning: CONFIG_DIR not found at {CONFIG_DIR}")
    return render_template('navigation_run_popup.html', map_files=map_files, wp_files=wp_files, csv_files=csv_files)

@app.route('/stop')
def stop_run():
    return render_template('stop.html')

@app.route('/mapping_executed')
def mapping_run():
    """マッピング実行後のメッセージ画面"""
    return render_template('ctrl_executed.html', message="マッピングが開始されました。")

@app.route('/mapping_popup')
def mapping_run_popup():
    return render_template('mapping_popup.html')

@app.route('/file_management_popup')
def file_management_popup():
    return render_template('file_management_popup.html')

@app.route('/ctrl_executed')
def ctrl_run():
    return render_template('ctrl_executed.html', message="センサデータの記録が開始されました。")

@app.route('/navigation_executed')
def navigation_executed():
    return render_template('navigation_executed.html', message="自律走行が開始されました。周囲の安全に気をつけて下さい。")

@app.route('/browse_rosbag', defaults={'path': ''}) 
@app.route('/browse_rosbag/<path:path>')
def browse_rosbag(path):
    """ROS Bag フィルタ/P2O/LIO-RAW用のファイルブラウザ"""
    full_path = os.path.join(ROSBAG_ROOT_DIR, path)
    
    if not _is_safe_path(full_path, ROSBAG_ROOT_DIR):
        flash("セキュリティ上の理由により、このディレクトリにはアクセスできません。", "error")
        return redirect(url_for('browse_rosbag'))
    
    if not os.path.isdir(full_path):
        flash(f"ディレクトリが見つかりません: {path}", "error")
        return redirect(url_for('browse_rosbag', path=os.path.dirname(path)))

    try:
        items = os.listdir(full_path)
        files = [item for item in items if os.path.isfile(os.path.join(full_path, item))]
        dirs = [item for item in items if os.path.isdir(os.path.join(full_path, item))]
        
        parent_path = os.path.dirname(path) if path else None
        
        p2o_mode_browse = request.args.get('mode') == 'p2o'
        lio_raw_mode_browse = request.args.get('mode') == 'lio_raw'
        
        return render_template('rosbag_browse.html', 
                               files=files, 
                               dirs=dirs, 
                               current_path=path, 
                               current_dir_name=os.path.basename(full_path) if path else ROSBAG_ROOT_DIR, 
                               root_dir=ROSBAG_ROOT_DIR,
                               parent_path=parent_path,
                               p2o_mode=p2o_mode_browse,
                               lio_raw_mode=lio_raw_mode_browse) 

    except (FileNotFoundError, PermissionError) as e:
        flash(f"ディレクトリ操作中にエラーが発生しました: {e}", "error")
        return redirect(url_for('browse_rosbag'))

@app.route('/select_rosbag', methods=['POST'])
def select_rosbag():
    """ファイル/ディレクトリパスからトピックリストを取得し、選択画面へ遷移する"""
    file_path = request.form.get('file_path') 
    
    if not file_path:
        mode = request.form.get('mode') 
        flash("ファイルまたはディレクトリが選択されていません。", "error")
        return redirect(url_for('browse_rosbag', mode=mode))
    
    full_path = os.path.join(ROSBAG_ROOT_DIR, file_path)

    if not _is_safe_path(full_path, ROSBAG_ROOT_DIR):
        flash("許可されていないパスへのアクセスが試行されました。", "error")
        return redirect(url_for('browse_rosbag', path=os.path.dirname(file_path)))
    
    is_p2o_mode = request.form.get('mode') == 'p2o' 
    is_lio_raw_mode = request.form.get('mode') == 'lio_raw' 

    is_rosbag = os.path.isdir(full_path) or full_path.lower().endswith(tuple(f'.{ext}' for ext in ALLOWED_EXTENSIONS))
    
    if is_rosbag:
        try:
            topics_info = get_topic_list(full_path)
            sorted_topics = sorted(topics_info.items())

            # ROS Bag の詳細（メッセージ数・周波数・座標系）を取得する。
            # 取得できた場合は記録時間もここから求め、遅い ros2 bag info を省く。
            bag_info = get_bag_info(full_path)
            bag_duration_sec = bag_info['duration_sec'] if bag_info else 0
            if bag_duration_sec == 0:
                command_list = ["ros2", "bag", "info", full_path]
                try:
                    result = subprocess.run(
                        command_list, 
                        capture_output=True, 
                        text=True, 
                        check=True, 
                        timeout=10
                    )
                    
                    for line in result.stdout.splitlines():
                        if "Duration:" in line:
                            match_direct = re.search(r'Duration:\s+([\d.]+?)s', line)
                            if match_direct:
                                bag_duration_sec = float(match_direct.group(1))
                                break
                            match_bracket = re.search(r'\(([\d.]+?)s\)', line)
                            if match_bracket:
                                bag_duration_sec = float(match_bracket.group(1))
                                break
                            
                except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
                    print(f"Warning: ros2 bag info failed or not found: {e}. Falling back to metadata.yaml.")

            if bag_duration_sec == 0 and os.path.isdir(full_path):
                metadata_path = pathlib.Path(full_path) / 'metadata.yaml'
                if metadata_path.exists():
                    with open(metadata_path, 'r') as f:
                        metadata = yaml.safe_load(f)
                        if 'duration' in metadata:
                            bag_duration_sec = metadata['duration'] / 1_000_000_000 
                        elif 'rosbag2_bagfile_information' in metadata and 'duration' in metadata['rosbag2_bagfile_information']:
                            bag_duration_sec = metadata['rosbag2_bagfile_information']['duration'] / 1_000_000_000
            
            if bag_duration_sec > 0:
                estimated_duration = round(bag_duration_sec)
            else:
                estimated_duration = 120
            
            config_files = _mapping_config_files()

            flash(f'ROS Bag "{file_path}" を読み込みました。', 'success')

            if is_p2o_mode:
                mapping_mode = 'p2o'
            elif is_lio_raw_mode:
                mapping_mode = 'lio_raw'
            else:
                mapping_mode = 'filter'

            # 設定チェックの結果は、設定ファイルを選ぶたびに変わるため
            # ブラウザ側から /rosbag_diagnostics を呼んで取得する。
            return render_template('rosbag_select_topics.html', 
                                   topics_info=sorted_topics, 
                                   input_bag_path=full_path,
                                   p2o_mode=is_p2o_mode, 
                                   lio_raw_mode=is_lio_raw_mode,
                                   bag_duration_sec=estimated_duration,
                                   config_files=config_files,
                                   bag_info=bag_info,
                                   mapping_mode=mapping_mode) 
            
        except NameError:
            flash("ROS Bagフィルタのコア機能がインポートされていません。ROS環境を確認してください。", 'error')
            return redirect(url_for('browse_rosbag', path=os.path.dirname(file_path)))
        except Exception as e:
            flash(f'ROS Bagの読み込み中にエラーが発生しました: {e}', 'error')
            return redirect(url_for('browse_rosbag', path=os.path.dirname(file_path)))
    else:
        # ディレクトリの場合はブラウズを続行
        if os.path.isdir(full_path):
            mode = request.form.get('mode')
            return redirect(url_for('browse_rosbag', path=file_path, mode=mode))
        
        flash("選択されたファイル形式はROS Bagとしてサポートされていません。", "error")
        return redirect(url_for('browse_rosbag', path=os.path.dirname(file_path), mode=request.form.get('mode')))

@app.route('/rosbag_diagnostics', methods=['POST'])
def rosbag_diagnostics():
    """選択中のROS Bagとパラメータ設定ファイルの食い違いを検査する (API)

    設定ファイルのプルダウンを変えるたびに呼ばれ、
    「このまま実行すると失敗するか」をブラウザ側に返す。
    """
    data = request.get_json(silent=True) or {}
    input_bag_path = data.get('input_bag_path', '')
    config_filename = os.path.basename(data.get('config_file', '') or '')
    mode = data.get('mode', 'p2o')

    if mode not in ('p2o', 'lio_raw', 'pcd2pgm', 'filter'):
        return jsonify({'status': 'error', 'message': '不明なモードが指定されました。'}), 400

    bag_info = None
    if input_bag_path:
        if not _is_safe_path(input_bag_path, ROSBAG_ROOT_DIR):
            return jsonify({'status': 'error', 'message': '許可されていないパスが指定されました。'}), 400
        if not os.path.exists(input_bag_path):
            return jsonify({'status': 'error', 'message': 'ROS Bag が見つかりません。'}), 400
        bag_info = get_bag_info(input_bag_path)

    diagnostics = run_diagnostics(mode, config_filename, bag_info)
    if diagnostics is None:
        return jsonify({'status': 'unavailable',
                        'message': '設定チェック機能を利用できません。'}), 200

    return jsonify({'status': 'success', 'diagnostics': diagnostics})


@app.route('/mapping_config', methods=['POST'])
def mapping_config():
    """編集用に、設定ファイルの各行と選択候補を返す (API)

    トピック名や座標系の候補は、選択中の ROS Bag に実際に記録されている
    ものから作る。利用者が手入力せずに選び直せるようにするため。
    """
    if mapping_diagnostics is None:
        return jsonify({'status': 'unavailable',
                        'message': '設定編集機能を利用できません。'}), 200

    data = request.get_json(silent=True) or {}
    config_filename = os.path.basename(data.get('config_file', '') or '')
    input_bag_path = data.get('input_bag_path', '')

    config_path = _resolve_config_path(config_filename)
    if not config_path or not os.path.isfile(config_path):
        return jsonify({'status': 'error',
                        'message': '編集する設定ファイルを選択してください。'}), 400

    bag_info = None
    if input_bag_path:
        if not _is_safe_path(input_bag_path, ROSBAG_ROOT_DIR):
            return jsonify({'status': 'error', 'message': '許可されていないパスが指定されました。'}), 400
        bag_info = get_bag_info(input_bag_path)

    config = mapping_diagnostics.load_mapping_config(config_path)
    choices = mapping_diagnostics.build_option_choices(bag_info, config['values'])

    rows = []
    for row in config['rows']:
        rows.append({
            'name': row['name'],
            'label': row['label'],
            'kind': mapping_diagnostics.option_kind(row['name']),
            'specified': row['specified'],
            'default': row['default'],
            'value': row['value'],
            'choices': choices.get(row['name'], []),
        })

    return jsonify({
        'status': 'success',
        'config_name': config_filename,
        'rows': rows,
        'has_bag_info': bag_info is not None,
    })


@app.route('/save_mapping_config', methods=['POST'])
def save_mapping_config():
    """編集した設定ファイルを保存する (API)

    行の並び順とデフォルト値の列は元のまま残し、「指定値」だけを書き換える。
    """
    if mapping_diagnostics is None:
        return jsonify({'status': 'error',
                        'message': '設定編集機能を利用できません。'}), 400

    data = request.get_json(silent=True) or {}
    source_filename = os.path.basename(data.get('config_file', '') or '')
    save_as = os.path.basename(data.get('save_as', '') or source_filename).strip()
    values = data.get('values') or {}

    source_path = _resolve_config_path(source_filename)
    if not source_path or not os.path.isfile(source_path):
        return jsonify({'status': 'error', 'message': '元の設定ファイルが見つかりません。'}), 400

    if not save_as.lower().endswith('.csv'):
        save_as = f'{save_as}.csv'
    if not re.match(r'^[^/\\]{1,100}\.csv$', save_as):
        return jsonify({'status': 'error',
                        'message': 'ファイル名に使えない文字が含まれています。'}), 400

    target_path = _resolve_config_path(save_as)
    if not target_path:
        return jsonify({'status': 'error', 'message': '保存先が不正です。'}), 400

    if not isinstance(values, dict):
        return jsonify({'status': 'error', 'message': '設定値の形式が正しくありません。'}), 400
    values = {str(k): str(v).strip() for k, v in values.items()}

    ok, errors = mapping_diagnostics.write_mapping_config(source_path, values, target_path)
    if not ok:
        return jsonify({'status': 'error', 'message': ' / '.join(errors)}), 400

    return jsonify({
        'status': 'success',
        'config_file': save_as,
        'message': f'設定ファイル「{save_as}」を保存しました。',
        'config_files': _mapping_config_files(),
    })


@app.route('/mapping_log', methods=['GET'])
def mapping_log():
    """実行ログの続きと、そこから読み取ったエラーの説明を返す (API)

    ブラウザは前回受け取った offset を送り、その続きだけを受け取る。
    """
    job_id = request.args.get('job_id', '')
    try:
        offset = max(0, int(request.args.get('offset', 0)))
    except ValueError:
        offset = 0

    log_path = _job_log_path(job_id)
    if log_path is None:
        return jsonify({'status': 'error', 'message': '不正なジョブIDです。'}), 400

    if not os.path.exists(log_path):
        return jsonify({'status': 'success', 'job_id': job_id, 'offset': offset,
                        'text': '', 'events': [], 'exit_code': None, 'finished': False})

    try:
        size = os.path.getsize(log_path)
        # ログが作り直された場合は先頭から読み直す。
        if offset > size:
            offset = 0
        with open(log_path, 'r', encoding='utf-8', errors='replace') as f:
            f.seek(offset)
            text = f.read(LOG_CHUNK_LIMIT)
            new_offset = f.tell()
    except OSError as e:
        return jsonify({'status': 'error', 'message': f'実行ログを読み込めません: {e}'}), 500

    lines = text.splitlines()
    events = []
    exit_code = None
    if log_insight is not None:
        events = log_insight.classify_lines(lines)
        exit_code = log_insight.find_exit_code(lines)

        # エラーは終了コードより前のチャンクに出ていることが多いため、
        # ジョブ単位で「エラーを見たか」を覚えておく。
        with _jobs_lock:
            job = _jobs.get(job_id)
            if job is not None:
                if any(event['level'] == 'error' for event in events):
                    job['seen_error'] = True
                has_error = job.get('seen_error', False)
            else:
                has_error = any(event['level'] == 'error' for event in events)

        mode_label = '処理'
        if job and mapping_diagnostics is not None:
            mode_label = mapping_diagnostics.MODE_LABELS.get(job['mode'], '処理')
        exit_event = log_insight.exit_code_finding(exit_code, mode_label, has_error)
        if exit_event:
            events.append(exit_event)

    return jsonify({
        'status': 'success',
        'job_id': job_id,
        'offset': new_offset,
        'text': text,
        'events': events,
        'exit_code': exit_code,
        'finished': exit_code is not None,
    })


@app.route('/convert', methods=['POST'])
def convert():
    """トピックフィルタリングを実行し、結果のダウンロードパスを返す (API)

    この処理はサーバ内で完結するため、応答が返った時点で変換は終わっている。
    """
    try:
        data = request.get_json()
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'リクエストJSONのパースエラー: {e}'}), 400

    input_bag_path = data.get('input_bag_path')
    selected_topics = data.get('topics', [])
    output_filename_base = data.get('output_filename')

    # 入力チェック 
    if not input_bag_path or not os.path.exists(input_bag_path):
        return jsonify({'status': 'error', 'message': '入力ファイルが見つかりません。パスを確認してください。'}), 400
    if not output_filename_base:
        return jsonify({'status': 'error', 'message': '出力ファイル名を入力してください。'}), 400
    if not selected_topics:
        return jsonify({'status': 'error', 'message': 'トピックフィルタリングにはトピックを一つ以上選択してください。'}), 400

    output_bag_dir = os.path.join(DOWNLOAD_FOLDER, output_filename_base)

    try:
        result_message = filter_rosbag(input_bag_path, output_bag_dir, selected_topics)

        return jsonify({
            'status': 'success',
            'message': result_message,
            'download_path': output_filename_base
        })
        
    except NameError:
        return jsonify({'status': 'error', 'message': 'ROS Bagフィルタのコア機能がインポートされていません。ROS環境を確認してください。'}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'処理中にエラーが発生しました: {e}'}), 500

@app.route('/p2o_mapping', methods=['POST'])
def p2o_mapping():
    """P2O SLAM 処理を開始し、結果のダウンロードパスを返す (API)"""
    return _start_mapping_process('p2o', request)

@app.route('/lio_raw_mapping', methods=['POST'])
def lio_raw_mapping():
    """LIO-RAW SLAM 処理を開始し、結果のダウンロードパスを返す (API)"""
    return _start_mapping_process('lio_raw', request)

def _start_mapping_process(mode, req):
    """
    指定されたモードでマッピング処理を開始する共通ヘルパー関数。
    mode: 'p2o' または 'lio_raw'
    """
    try:
        data = req.get_json()
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'リクエストJSONのパースエラー: {e}'}), 400

    input_bag_path = data.get('input_bag_path')
    output_map_name = data.get('output_map_name')
    config_filename = data.get('config_file', '')
    config_file_path = os.path.join(CONFIG_DIR, config_filename) if config_filename else ''

    if not input_bag_path or not os.path.exists(input_bag_path):
        return jsonify({'status': 'error', 'message': '入力ファイルが見つかりません。パスを確認してください。'}), 400
    if not output_map_name:
        return jsonify({'status': 'error', 'message': '出力マップ名を入力してください。'}), 400

    base_name = os.path.basename(input_bag_path)
    if os.path.isfile(input_bag_path):
        base_name = os.path.splitext(base_name)[0]
    else:
        base_name = os.path.basename(input_bag_path.rstrip('/'))

    flag_file_name_full = f'{output_map_name}.{mode.upper()}_DONE'
    
    try:
        script_path = os.path.join(BASE_PATH, "start_mapping.sh")
        command_list = [
            script_path, 
            mode,
            base_name,
            output_map_name,
            MAP_DIR,
            WP_DIR,
            flag_file_name_full,
            config_file_path
        ]
        
        job_id, log_path = create_job(mode, output_map_name)
        Thread(target=run_subprocess, args=(command_list, log_path)).start()
        result_message = f"{mode.upper()} マッピングスクリプトがバックグラウンドで開始されました。出力マップ名: {output_map_name}"
        
        return jsonify({
            'status': 'success',
            'message': result_message,
            'map_name': output_map_name,
            'job_id': job_id
        })
        
    except Exception as e:
        print(f"ERROR: {mode.upper()} Mapping failed with exception: {e}")
        return jsonify({'status': 'error', 'message': f'処理中にエラーが発生しました: {e}'}), 500

def _check_mapping_status(mode, req):
    """
    P2O または LIO-RAW SLAM 処理の完了ステータスをチェックする共通ヘルパー関数。
    mode: 'p2o' または 'lio_raw'
    """
    data = req.get_json()
    output_map_name = data.get('output_map_name')

    if not output_map_name:
        return jsonify({'status': 'error', 'message': '出力マップ名が指定されていません。'}), 400

    flag_file_name = f'{output_map_name}.{mode.upper()}_DONE'
    flag_file_path = os.path.join(MAP_DIR, flag_file_name) 
    
    if os.path.exists(flag_file_path):
        pcd_filename = f'{output_map_name}.pcd'
        pcd_file_path = os.path.join(MAP_DIR, pcd_filename)
        
        try:
            os.remove(flag_file_path)
            print(f"{mode.upper()} completion flag removed: {flag_file_path}")
        except Exception as e:
            print(f"Warning: Failed to remove flag file {flag_file_path}: {e}")
            
        if not os.path.exists(pcd_file_path):
            return jsonify({
                'status': 'error', 
                'message': f'PCDファイルが見つかりません。フラグは存在しましたが、"{pcd_filename}" が {MAP_DIR} に見つかりません。'
            })

        download_url = url_for('download_map', filename=pcd_filename)
        
        return jsonify({
            'status': 'finished', 
            'message': f'{mode.upper()} SLAM 処理が完了しました。PCDファイルをダウンロードできます。',
            'map_name': output_map_name,
            'download_url': download_url 
        })
    else:
        return jsonify({
            'status': 'in_progress', 
            'message': f'{mode.upper()} SLAM 処理を続行中です...'
        })

@app.route('/check_p2o_status', methods=['POST'])
def check_p2o_status():
    """P2O SLAM 処理の完了ステータスをチェックするAPI。"""
    return _check_mapping_status('p2o', request)

@app.route('/check_lio_raw_status', methods=['POST'])
def check_lio_raw_status():
    """LIO-RAW SLAM 処理の完了ステータスをチェックするAPI。"""
    return _check_mapping_status('lio_raw', request)

@app.route('/download/<path:filename>')
def download_file(filename):
    """フィルタリングされたBagディレクトリをZIP圧縮してダウンロードさせる (Filter用)"""
    bag_dir_path = os.path.join(DOWNLOAD_FOLDER, filename)
    zip_filename = f'{filename}.zip'
    zip_path = os.path.join(DOWNLOAD_FOLDER, zip_filename)
    
    if not os.path.isdir(bag_dir_path):
        flash('ダウンロード用のファイルが見つかりません。', 'error')
        return redirect(url_for('browse_rosbag'))

    try:
        zip_directory(bag_dir_path, zip_path)
        
        return send_from_directory(
            DOWNLOAD_FOLDER, 
            zip_filename, 
            as_attachment=True
        )
    except Exception as e:
        flash(f'ファイルのZIP化中にエラーが発生しました: {e}', 'error')
        return redirect(url_for('browse_rosbag'))
    finally:
        if os.path.exists(zip_path):
            os.remove(zip_path)

@app.route('/download_map/<filename>')
def download_map(filename):
    """
    PCDファイルをダウンロードさせる。(P2O/LIO-RAW完了後にフロントエンドから呼ばれる)
    """
    try:
        directory = MAP_DIR
        map_filename = filename 
        file_path = os.path.join(directory, map_filename)

        if not os.path.exists(file_path):
            flash(f'エラー: 指定されたマップファイル "{map_filename}" が {directory} に見つかりません。', 'error')
            return redirect(url_for('main_gui')) 

        return send_from_directory(
            directory, 
            map_filename, 
            as_attachment=True, 
            mimetype='application/octet-stream'
        )

    except Exception as e:
        flash(f'マップファイルのダウンロード中にエラーが発生しました: {e}', 'error')
        return redirect(url_for('main_gui'))

@app.route('/map_viewer', methods=['GET'])
def open_empty_viewer():
    """
    ファイル指定なしで、マップを後から選択できる空のビューアを開く。
    このエンドポイントは、クライアント側でロードモーダルを表示させることを目的とする。
    """
    # 既存の view_map エンドポイントと同じテンプレートを使用しますが、
    # データを空（またはデフォルト値）で渡します。
    
    # 利用可能なマップファイルリストを取得
    try:
        available_pcds = sorted([
            os.path.splitext(f)[0] for f in os.listdir(MAP_DIR) if f.endswith('.pcd')
        ])
    except FileNotFoundError:
        available_pcds = []
        print(f"Warning: MAP_DIR not found at {MAP_DIR}")

    try:
        available_yamls = sorted([
            os.path.splitext(f)[0] for f in os.listdir(MAP_DIR) if f.endswith('.yaml')
        ])
    except FileNotFoundError:
        available_yamls = []

    try:
        available_wps = sorted([
            os.path.splitext(f)[0] for f in os.listdir(WP_DIR) if f.endswith('.json')
        ])
    except FileNotFoundError:
        available_wps = []

    # マップ名: 空
    map_name = request.args.get('map_name', '')
    # YAMLデータ: 空の文字列
    yaml_data_string = ""
    
    # Waypointデータ: 空のJSON配列
    waypoints_data_string = "[]" 

    if map_name:
        # マップ名が指定されている場合のロードロジック
        print(f"INFO: マップ名 '{map_name}' が指定されました。ファイル読み込みを試行します。")
        
        # 1. PCDファイルの存在チェック (必須と仮定)
        pcd_filename = f'{map_name}.pcd'
        pcd_file_path = os.path.join(MAP_DIR, pcd_filename)
        
        if not os.path.exists(pcd_file_path):
            flash(f'エラー: マップファイル "{pcd_filename}" がサーバーに見つかりません。', 'error')
            # ファイルが存在しない場合は、マップ名が空のビューワを開く(またはリダイレクト)
            map_name = ""
            
        # 2. YAMLファイルの読み込み
        yaml_filename = f'{map_name}.yaml'
        yaml_file_path = os.path.join(MAP_DIR, yaml_filename)
        if os.path.exists(yaml_file_path):
            try:
                with open(yaml_file_path, 'r') as f:
                    yaml_data_string = f.read()
                print(f"INFO: YAML file '{yaml_filename}' loaded for viewer.")
            except Exception as e:
                print(f"ERROR: Failed to read YAML file '{yaml_file_path}': {e}")
                flash(f'警告: YAMLファイル "{yaml_filename}" の読み込みに失敗しました。', 'warning')
        # 3. ウェイポイントファイルの読み込み
        wp_filename = f'{map_name}.json'
        wp_file_path = os.path.join(WP_DIR, wp_filename)
        if os.path.exists(wp_file_path):
            try:
                with open(wp_file_path, 'r') as f:
                    waypoints_data_string = f.read()
                print(f"INFO: Waypoint file '{wp_filename}' loaded for viewer.")
            except Exception as e:
                print(f"ERROR: Failed to read Waypoint file '{wp_file_path}': {e}")
                flash(f'警告: Waypointファイル "{wp_filename}" の読み込みに失敗しました。', 'warning')
    else:
        print("INFO: マップ名が指定されていません。空のビューワを開きます。")
        
    return render_template(
        'map_viewer.html', # パラメータがなければ ""
        map_name=map_name, 
        yaml_data=yaml_data_string, 
        waypoints_data=waypoints_data_string,
        available_pcds=available_pcds,
        available_yamls=available_yamls,
        available_wps=available_wps
    )

@app.route('/view_map/<map_name>')
def view_map(map_name):
    """
    PCDビューアのHTMLページをレンダリングする。
    対応するPGMマップ用のYAMLファイル、ウェイポイントファイルがあれば、その内容も読み込みテンプレートに渡す。
    """
    if not map_name:
        flash("エラー: 表示するマップ名が指定されていません。", "error")
        return redirect(url_for('main_gui'))

    pcd_filename = f'{map_name}.pcd'
    pcd_file_path = os.path.join(MAP_DIR, pcd_filename)
    
    if not os.path.exists(pcd_file_path):
        flash(f'エラー: マップファイル "{pcd_filename}" がサーバーに見つかりません。', 'error')
        return redirect(url_for('main_gui'))
    
    yaml_filename = f'{map_name}.yaml'
    yaml_file_path = os.path.join(MAP_DIR, yaml_filename)
    yaml_data_string = ""
    
    if os.path.exists(yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as f:
                yaml_data_string = f.read()
            print(f"INFO: YAML file '{yaml_filename}' loaded for viewer.")
        except Exception as e:
            print(f"ERROR: Failed to read YAML file '{yaml_file_path}': {e}")
            flash(f'警告: YAMLファイル "{yaml_filename}" の読み込みに失敗しました。', 'warning')

    wp_filename = f'{map_name}.json'
    wp_file_path = os.path.join(WP_DIR, wp_filename)
    waypoints_data_string = "[]"
    
    if os.path.exists(wp_file_path):
        try:
            with open(wp_file_path, 'r') as f:
                waypoints_data_string = f.read()
            print(f"INFO: Waypoint file '{wp_filename}' loaded for viewer.")
        except Exception as e:
            print(f"ERROR: Failed to read Waypoint file '{wp_file_path}': {e}")
            flash(f'警告: Waypointファイル "{wp_filename}" の読み込みに失敗しました。', 'warning')

    return render_template('map_viewer.html', 
                           map_name=map_name,
                           yaml_data=yaml_data_string,
                           waypoints_data=waypoints_data_string)

@app.route('/map/<filename>')
def serve_map_file(filename):
    """
    map_viewer.htmlからリクエストされたPCDファイル、PGMファイル、
    およびウェイポイントファイルをブラウザに提供する。
    """
    ext = os.path.splitext(filename)[1].lower()
    
    if ext == '.pcd' or ext == '.yaml' or ext == '.pgm':
        target_dir = MAP_DIR
    elif ext == '.json':
        target_dir = WP_DIR
    else:
        return jsonify({'error': 'サポートされていないファイル形式です'}), 400

    try:
        full_path = os.path.join(target_dir, filename)
        if not _is_safe_path(full_path, HOKUYO_NAV2_PKG_PATH): 
            return jsonify({'error': '許可されていないパスへのアクセスです'}), 403

        return send_from_directory(
            target_dir, 
            filename, 
            as_attachment=False,
            mimetype='application/octet-stream'
        )
    except Exception as e:
        print(f"ERROR: ファイルの提供に失敗しました ({filename}): {e}")
        return jsonify({'error': 'ファイルが見つからないか、アクセスできません'}), 404
        
# 自己位置推定 (simple_fastlio_localization) の初期位置ファイルの既定値。
# scripts/navigation/nav_common.sh の load_initial_poses が
# data/<マップ名>/init_pose.txt の最終行を "x,y,z,qx,qy,qz,qw" として読む。
DEFAULT_INITIAL_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def _initial_pose_path(map_name):
    """マップ名から init_pose.txt のパスを返す。不正なマップ名なら None。"""
    if not map_name or os.path.basename(map_name) != map_name:
        return None
    pose_path = os.path.join(DATA_DIR, map_name, 'init_pose.txt')
    if not _is_safe_path(pose_path, DATA_DIR):
        return None
    return pose_path


@app.route('/initial_pose/<map_name>', methods=['GET'])
def get_initial_pose(map_name):
    """自己位置推定の初期位置を返す (API)

    ナビゲーション起動時は data/<マップ名>/init_pose.txt が使われるため、
    3Dビューアではこのファイルを読み書きする。
    """
    pose_path = _initial_pose_path(map_name)
    if pose_path is None:
        return jsonify({'status': 'error', 'message': '不正なマップ名です。'}), 400

    if not os.path.exists(pose_path):
        return jsonify({'status': 'success', 'exists': False,
                        'pose': DEFAULT_INITIAL_POSE,
                        'message': '初期位置ファイルがまだありません。既定値を表示しています。'})

    try:
        with open(pose_path, 'r', encoding='utf-8') as f:
            lines = [line.strip() for line in f if line.strip()]
    except OSError as e:
        return jsonify({'status': 'error', 'message': f'初期位置を読み込めません: {e}'}), 500

    if not lines:
        return jsonify({'status': 'success', 'exists': False,
                        'pose': DEFAULT_INITIAL_POSE,
                        'message': '初期位置ファイルが空です。既定値を表示しています。'})

    # nav_common.sh と同じく最終行を使う。
    try:
        pose = [float(v) for v in lines[-1].split(',')]
    except ValueError:
        return jsonify({'status': 'error',
                        'message': f'初期位置の書式が不正です: {lines[-1]}'}), 400

    if len(pose) < 7:
        pose = (pose + DEFAULT_INITIAL_POSE[len(pose):])[:7]

    return jsonify({'status': 'success', 'exists': True, 'pose': pose[:7]})


@app.route('/save_initial_pose', methods=['POST'])
def save_initial_pose():
    """3Dビューアで指定した初期位置を init_pose.txt に保存する (API)"""
    data = request.get_json(silent=True) or {}
    map_name = data.get('map_name', '')
    pose = data.get('pose')

    pose_path = _initial_pose_path(map_name)
    if pose_path is None:
        return jsonify({'status': 'error', 'message': '不正なマップ名です。'}), 400

    if not isinstance(pose, list) or len(pose) != 7:
        return jsonify({'status': 'error',
                        'message': '初期位置は x,y,z,qx,qy,qz,qw の7要素で指定してください。'}), 400
    try:
        pose = [float(v) for v in pose]
    except (TypeError, ValueError):
        return jsonify({'status': 'error', 'message': '初期位置に数値以外が含まれています。'}), 400

    try:
        os.makedirs(os.path.dirname(pose_path), exist_ok=True)
        with open(pose_path, 'w', encoding='utf-8') as f:
            f.write(','.join(f'{v:.6f}' for v in pose) + '\n')
    except OSError as e:
        return jsonify({'status': 'error', 'message': f'初期位置を保存できません: {e}'}), 500

    return jsonify({
        'status': 'success',
        'message': f'初期位置を保存しました: data/{map_name}/init_pose.txt',
    })


@app.route('/rebase_map_origin', methods=['POST'])
def rebase_map_origin():
    """初期位置を地図の新しい原点にする (API)

    JSON: {map_name, pose: [x, y, z, qx, qy, qz, qw], dry_run}
    初期位置 (x, y) が (0, 0) になるよう、地図 (PGM/YAML・PCD)・ウェイポイント・基準点 (init_lat_lon_alt.txt)・
    パラメータ調整の元の点群と設定を平行移動し、init_pose.txt は (0, 0, z) と向きで保存する。
    dry_run なら書き換えるファイルの一覧だけを返す。
    """
    if map_rebase is None:
        return jsonify({'status': 'error', 'message': '原点を合わせる機能に必要な Python パッケージ (numpy, PyYAML) がありません。'}), 500
    data = request.get_json(silent=True) or {}
    map_name = str(data.get('map_name') or '')
    pose = data.get('pose')
    if not _MAP_NAME_PATTERN.match(map_name) or _initial_pose_path(map_name) is None \
            or not _is_safe_path(os.path.join(MAP_DIR, f'{map_name}.yaml'), MAP_DIR) \
            or not _is_safe_path(os.path.join(WP_DIR, f'{map_name}.json'), WP_DIR):
        return jsonify({'status': 'error', 'message': '不正なマップ名です。'}), 400
    try:
        pose = [float(v) for v in pose]
        if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({'status': 'error', 'message': '初期位置は数値の x,y,z,qx,qy,qz,qw で指定してください。'}), 400

    relpath = lambda p: os.path.relpath(p, HOKUYO_NAV2_PKG_PATH)
    if data.get('dry_run'):
        try:
            map_rebase.validate_rebase(DATA_DIR, map_name, pose[0], pose[1], default_latlon=DEFAULT_LAT_LON_ALT)
        except map_rebase.RebaseError as e:
            return jsonify({'status': 'error', 'message': str(e)}), e.status
        plan = map_rebase.plan_rebase(MAP_DIR, DATA_DIR, WP_DIR, map_name)
        return jsonify({'status': 'success',
                        'files': [relpath(p) for _, p in plan] + [relpath(_initial_pose_path(map_name))]})
    try:
        result = map_rebase.rebase_map(MAP_DIR, DATA_DIR, WP_DIR, map_name, pose[0], pose[1], pose,
                                       default_latlon=DEFAULT_LAT_LON_ALT)
    except map_rebase.RebaseError as e:
        return jsonify({'status': 'error', 'message': str(e)}), e.status
    except (OSError, ValueError, yaml.YAMLError) as e:
        return jsonify({'status': 'error', 'message': f'地図の原点を合わせられませんでした (ファイルは変えていません): {e}'}), 500
    return jsonify({
        'status': 'success',
        'message': f'地図「{map_name}」の原点を初期位置に合わせました (x {result["dx"]:+.3f} m, y {result["dy"]:+.3f} m ずらしました)。',
        'files': [relpath(p) for p in result['files']],
        'dx': result['dx'],
        'dy': result['dy'],
        'origin': result['origin'],
        'notes': result['notes'],
    })


# 3Dビューアの背景 (Google の衛星写真 / 3D Tiles) に使う API キー。
# 環境変数を優先し、無ければ GUI フォルダのファイルから読む。どちらも git には含めない。
GOOGLE_MAPS_API_KEY_ENV = 'GOOGLE_MAPS_API_KEY'
GOOGLE_MAPS_API_KEY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        'google_maps_api_key.txt')
# 場所の検索 (Geocoding / Places) にサーバーから使うキー。ブラウザ用のキーをリファラで制限していると
# サーバーからは使えないので、別のキー (IP アドレスで制限) を置けるようにする。無ければブラウザ用のキーを使う。
GOOGLE_MAPS_SERVER_API_KEY_ENV = 'GOOGLE_MAPS_SERVER_API_KEY'
GOOGLE_MAPS_SERVER_API_KEY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               'google_maps_server_api_key.txt')
# マッピングで GNSS を使わなかったときに init_lat_lon_alt.txt へ書かれる既定値
# (scripts/mapping/lio_raw.bash, scripts/navigation/nav_common.sh と同じ値)。
# この値のときは実際の場所を表していないので、背景と位置を合わせられない。
DEFAULT_LAT_LON_ALT = (35.0, 135.0, 40.0)


def _read_api_key(env_name, file_path):
    """環境変数かファイル (最初の行) からキーを読む。(キー, 読み込み元) を返す。未設定なら ('', None)。"""
    key = os.environ.get(env_name, '').strip()
    if key:
        return key, 'env'
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = [line.strip() for line in f
                     if line.strip() and not line.lstrip().startswith('#')]
    except OSError:
        return '', None
    return (lines[0], 'file') if lines else ('', None)


def _google_maps_api_key():
    """ブラウザで使うキー (Map Tiles API)。(キー, 読み込み元) を返す。"""
    return _read_api_key(GOOGLE_MAPS_API_KEY_ENV, GOOGLE_MAPS_API_KEY_FILE)


def _google_maps_server_api_key():
    """サーバーから使うキー (場所の検索)。専用のキーが無ければブラウザ用のキーを使う。"""
    key, source = _read_api_key(GOOGLE_MAPS_SERVER_API_KEY_ENV, GOOGLE_MAPS_SERVER_API_KEY_FILE)
    return (key, source) if key else _google_maps_api_key()


@app.route('/google_maps_config', methods=['GET'])
def google_maps_config():
    """3Dビューアの Google 背景用の設定を返す (API)

    Map Tiles API はブラウザから直接呼ぶため、キーはブラウザに渡る。
    Google Cloud 側で「Map Tiles API のみ」「リファラ制限」を掛けておくこと。
    """
    key, source = _google_maps_api_key()
    return jsonify({
        'status': 'success',
        'api_key': key,
        'source': source,
        'env_name': GOOGLE_MAPS_API_KEY_ENV,
        'file_name': os.path.basename(GOOGLE_MAPS_API_KEY_FILE),
    })


# 場所の検索 (Map Viewer の「場所を検索」)。キーをブラウザに渡さずに済むよう、サーバーから Google に問い合わせる。
GEOCODE_URL = 'https://maps.googleapis.com/maps/api/geocode/json'
PLACES_TEXT_SEARCH_URL = 'https://places.googleapis.com/v1/places:searchText'
GEOCODE_TIMEOUT_SEC = 8
GEOCODE_MAX_RESULTS = 5


def _http_json(url, data=None, headers=None):
    """JSON を返す API を呼ぶ。(HTTP ステータス, JSON) を返す。通信できなければ例外。"""
    req = urllib.request.Request(url, data=data, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=GEOCODE_TIMEOUT_SEC) as res:
            return res.status, json.loads(res.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode('utf-8'))
        except ValueError:
            return e.code, {}


def _short_address(address):
    """「日本、〒130-0013 東京都墨田区錦糸」のような住所から、国名と郵便番号を省く (表示用)。"""
    return re.sub(r'^(日本、)?\s*(〒\d{3}-\d{4}\s*)?', '', address or '').strip()


def _geocode(query, key):
    """Geocoding API (住所に強い)。(候補, エラーメッセージ) を返す。"""
    params = urllib.parse.urlencode({'address': query, 'language': 'ja', 'region': 'jp', 'key': key})
    _, body = _http_json(f'{GEOCODE_URL}?{params}')
    status = body.get('status')
    if status == 'ZERO_RESULTS':
        return [], None
    if status != 'OK':
        return [], f'Geocoding API: {status} {body.get("error_message", "")}'.strip()
    return [{
        'name': _short_address(r.get('formatted_address', '')),
        'address': _short_address(r.get('formatted_address', '')),
        'lat': r['geometry']['location']['lat'],
        'lon': r['geometry']['location']['lng'],
        'source': 'geocoding',
        'partial': bool(r.get('partial_match')),
    } for r in body.get('results', [])[:GEOCODE_MAX_RESULTS]], None


def _places_text_search(query, key):
    """Places API (New) の Text Search (施設名・駅名などに強い)。(候補, エラーメッセージ) を返す。"""
    status, body = _http_json(
        PLACES_TEXT_SEARCH_URL,
        data=json.dumps({'textQuery': query, 'languageCode': 'ja', 'regionCode': 'JP',
                         'pageSize': GEOCODE_MAX_RESULTS}).encode('utf-8'),
        headers={'Content-Type': 'application/json', 'X-Goog-Api-Key': key,
                 'X-Goog-FieldMask': 'places.displayName,places.formattedAddress,places.location'})
    if status != 200:
        message = (body.get('error') or {}).get('message', '')
        return [], f'Places API: HTTP {status} {message}'.strip()
    return [{
        'name': (p.get('displayName') or {}).get('text', ''),
        'address': _short_address(p.get('formattedAddress', '')),
        'lat': p['location']['latitude'],
        'lon': p['location']['longitude'],
        'source': 'places',
        'partial': False,
    } for p in body.get('places', []) if p.get('location')][:GEOCODE_MAX_RESULTS], None


@app.route('/geocode', methods=['GET'])
def geocode():
    """住所・地名から緯度経度の候補を返す (API)

    まず Geocoding API (住所) で探し、見つからないか一部一致だけのときは Places API (施設名など) でも探す。
    """
    query = (request.args.get('q') or '').strip()
    if not query:
        return jsonify({'status': 'error', 'message': '検索する住所や地名を入力してください。'}), 400
    if len(query) > 200:
        return jsonify({'status': 'error', 'message': '検索する文字が長すぎます。'}), 400
    key, _ = _google_maps_server_api_key()
    if not key:
        return jsonify({'status': 'error', 'message': 'Google Maps の API キーが設定されていません。'}), 400

    errors = []
    try:
        results, error = _geocode(query, key)
        if error:
            errors.append(error)
        if not results or all(r['partial'] for r in results):
            places, error = _places_text_search(query, key)
            if error:
                errors.append(error)
            # 施設名で見つかったものを先に出す (同じ場所の重複は除く)
            merged = places + [r for r in results
                               if all(abs(r['lat'] - p['lat']) > 1e-4 or abs(r['lon'] - p['lon']) > 1e-4 for p in places)]
            results = merged[:GEOCODE_MAX_RESULTS]
    except (OSError, ValueError, KeyError) as e:
        return jsonify({'status': 'error', 'message': f'Google に問い合わせできません: {e}'}), 502

    if not results and errors:
        return jsonify({'status': 'error',
                        'message': '場所を検索できません (' + ' / '.join(errors) + ')。'
                                   'API キーで Geocoding API か Places API (New) を有効にしてください。'
                                   'ブラウザ用のキーをリファラで制限している場合は、サーバー用のキーを '
                                   f'{os.path.basename(GOOGLE_MAPS_SERVER_API_KEY_FILE)} か環境変数 '
                                   f'{GOOGLE_MAPS_SERVER_API_KEY_ENV} に設定してください。'}), 502
    return jsonify({'status': 'success', 'query': query, 'results': results})


def _read_geo_origin(map_name):
    """data/<マップ名>/init_lat_lon_alt.txt の最終行 (lat,lon,alt) を読む。

    nav_common.sh の load_initial_poses と同じ読み方。不正なマップ名なら None を返す。
    """
    if not map_name or os.path.basename(map_name) != map_name:
        return None
    latlon_path = os.path.join(DATA_DIR, map_name, 'init_lat_lon_alt.txt')
    if not _is_safe_path(latlon_path, DATA_DIR):
        return None

    if not os.path.exists(latlon_path):
        return {'exists': False, 'valid': False,
                'message': f'data/{map_name}/init_lat_lon_alt.txt がありません。'}
    try:
        with open(latlon_path, 'r', encoding='utf-8') as f:
            lines = [line.strip() for line in f if line.strip()]
        values = [float(v) for v in lines[-1].split(',')][:3]
    except (OSError, IndexError, ValueError) as e:
        return {'exists': True, 'valid': False,
                'message': f'init_lat_lon_alt.txt を読み込めません: {e}'}

    if len(values) < 2:
        return {'exists': True, 'valid': False,
                'message': 'init_lat_lon_alt.txt に緯度と経度がありません。'}
    lat, lon = values[0], values[1]
    alt = values[2] if len(values) > 2 else 0.0
    # マッピング時の UTM ゾーンが不正だと inf/nan が書かれることがある (hokuyo_slam.bash 参照)
    in_range = all(math.isfinite(v) for v in (lat, lon, alt)) \
        and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0
    is_default = all(abs(a - b) < 1e-9 for a, b in zip((lat, lon, alt), DEFAULT_LAT_LON_ALT))

    result = {'exists': True, 'lat': lat, 'lon': lon, 'alt': alt,
              'is_default': is_default, 'valid': in_range and not is_default}
    if not in_range:
        result['message'] = f'init_lat_lon_alt.txt の値が不正です: {lines[-1]}'
    elif is_default:
        result['message'] = ('原点が既定値 (35.0,135.0,40.0) のままです。GNSS を使わずに作った地図のため、'
                             '実際の場所は分かりません。')
    return result


@app.route('/geo_origin/<map_name>', methods=['GET'])
def get_geo_origin(map_name):
    """地図座標の原点の緯度経度高度を返す (API)"""
    origin = _read_geo_origin(map_name)
    if origin is None:
        return jsonify({'status': 'error', 'message': '不正なマップ名です。'}), 400
    return jsonify({'status': 'success', **origin})


@app.route('/geo_origins', methods=['GET'])
def list_geo_origins():
    """有効な原点 (GNSS で作った地図の init_lat_lon_alt.txt) を持つマップの一覧を返す (API)

    事前地図を読まずに Google 背景を表示するとき、同じ現場の原点を選べるようにするため。
    """
    origins = []
    try:
        names = sorted(os.listdir(DATA_DIR))
    except OSError:
        names = []
    for name in names:
        if not os.path.isdir(os.path.join(DATA_DIR, name)):
            continue
        origin = _read_geo_origin(name)
        if origin and origin.get('valid'):
            origins.append({'map_name': name, 'lat': origin['lat'],
                            'lon': origin['lon'], 'alt': origin['alt']})
    return jsonify({'status': 'success', 'origins': origins})


_MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_.-]+$')
# ブラウザから受け取る点群の上限 (点数)。Map Viewer 側でも同じ上限で止める。
MAX_UPLOADED_POINTS = 8_000_000


@app.route('/create_map_from_points', methods=['POST'])
def create_map_from_points():
    """Map Viewer で範囲を選んで作った点群から、地図ファイル一式を保存する (API)

    本文: 点群 (リトルエンディアン。point_format が xyz なら float32 x,y,z、xyzrgb なら続けて uint32 の色)。座標は範囲の中心を原点にした
          fix2xyz と同じ地図座標 [m]。
    クエリ meta: JSON 文字列
        map_name, overwrite, origin {lat, lon, alt}, bounds {min_x, min_y, max_x, max_y},
        pgm {resolution, obstacle_min, obstacle_max, ground_window, point_spacing, inflate (省略時は間隔の半分)},
        point_format: 'xyz' (1 点 12 バイト) / 'xyzrgb' (1 点 16 バイト、色 0x00RRGGBB 付き)
        init_pose {x, y, yaw}
    """
    try:
        meta = json.loads(request.args.get('meta', ''))
    except ValueError:
        return jsonify({'status': 'error', 'message': 'meta の JSON を読めません。'}), 400
    if map_from_points is None:
        return jsonify({'status': 'error',
                        'message': '地図の書き出しに必要な Python パッケージ (numpy, Pillow) がありません。'}), 500

    map_name = str(meta.get('map_name', '')).strip()
    if not _MAP_NAME_PATTERN.match(map_name):
        return jsonify({'status': 'error',
                        'message': 'マップ名は半角英数字と _ . - だけで指定してください。'}), 400

    targets = [os.path.join(MAP_DIR, f'{map_name}{ext}') for ext in ('.pcd', '.pgm', '.yaml')]
    targets.append(os.path.join(DATA_DIR, map_name))
    if not all(_is_safe_path(p, HOKUYO_NAV2_PKG_PATH) for p in targets):
        return jsonify({'status': 'error', 'message': '不正なマップ名です。'}), 400
    existing = [os.path.relpath(p, HOKUYO_NAV2_PKG_PATH) for p in targets if os.path.exists(p)]
    if existing and not meta.get('overwrite'):
        return jsonify({'status': 'exists',
                        'message': f'同じ名前の地図があります: {", ".join(existing)}。'
                                   '上書きする場合は「同じ名前の地図があれば上書きする」にチェックしてください。'}), 409

    try:
        origin = meta['origin']
        lat, lon, alt = float(origin['lat']), float(origin['lon']), float(origin['alt'])
        b = meta['bounds']
        bounds = (float(b['min_x']), float(b['min_y']), float(b['max_x']), float(b['max_y']))
        p = meta['pgm']
        point_spacing = float(p['point_spacing'])
        pgm_params = {
            'resolution': float(p['resolution']),
            'obstacle_min': float(p['obstacle_min']),
            'obstacle_max': float(p['obstacle_max']),
            'ground_window': float(p['ground_window']),
            # 点と点のすき間で壁が途切れないよう、点の間隔の半分だけ障害物を太らせる
            'inflate': float(p.get('inflate', point_spacing / 2.0)),
        }
        pose = meta.get('init_pose') or {}
        init_pose = (float(pose.get('x', 0.0)), float(pose.get('y', 0.0)), float(pose.get('yaw', 0.0)))
    except (KeyError, TypeError, ValueError) as e:
        return jsonify({'status': 'error', 'message': f'meta の値が不正です: {e}'}), 400
    values = [lat, lon, alt, *bounds, point_spacing, *pgm_params.values(), *init_pose]
    if not all(math.isfinite(v) for v in values) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return jsonify({'status': 'error', 'message': '緯度経度や範囲の値が不正です。'}), 400
    if pgm_params['resolution'] <= 0 or pgm_params['obstacle_max'] <= pgm_params['obstacle_min']:
        return jsonify({'status': 'error', 'message': 'PGM の解像度か障害物の高さの指定が不正です。'}), 400

    point_format = str(meta.get('point_format') or 'xyz')
    record_bytes = map_from_points.POINT_RECORD_BYTES.get(point_format)
    if record_bytes is None:
        return jsonify({'status': 'error', 'message': f'point_format {point_format} には対応していません。'}), 400
    body = request.get_data(cache=False)
    if len(body) == 0 or len(body) % record_bytes != 0:
        return jsonify({'status': 'error', 'message': '点群データの長さが不正です (x,y,z の float32 と色の並び)。'}), 400
    points, colors = map_from_points.points_from_bytes(body, point_format)
    if points.shape[0] > MAX_UPLOADED_POINTS:
        return jsonify({'status': 'error',
                        'message': f'点が多すぎます ({points.shape[0]} 点、上限 {MAX_UPLOADED_POINTS} 点)。'}), 400

    try:
        result = map_from_points.save_map_files(MAP_DIR, DATA_DIR, map_name, points, bounds,
                                                (lat, lon, alt), pgm_params, init_pose,
                                                source={'type': str(meta.get('source_type') or 'points'),
                                                        'spacing': point_spacing},
                                                colors=colors)
    except ValueError as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400
    except OSError as e:
        return jsonify({'status': 'error', 'message': f'地図ファイルを保存できません: {e}'}), 500

    return jsonify({
        'status': 'success',
        'message': f'地図「{map_name}」を保存しました。',
        'map_name': map_name,
        'points': int(points.shape[0]),
        'pgm': result['pgm'],
        'files': [os.path.relpath(f, HOKUYO_NAV2_PKG_PATH) for f in result['files']],
    })


def _map_edit_request(map_name):
    """地図のパラメータ調整 API の共通チェック。問題があれば (レスポンス, ステータス) を返す。"""
    if map_from_points is None:
        return jsonify({'status': 'error',
                        'message': '地図の作り直しに必要な Python パッケージ (numpy, Pillow) がありません。'}), 500
    if not _MAP_NAME_PATTERN.match(map_name or '') \
            or not _is_safe_path(os.path.join(MAP_DIR, f'{map_name}.pcd'), HOKUYO_NAV2_PKG_PATH) \
            or not _is_safe_path(os.path.join(DATA_DIR, map_name), HOKUYO_NAV2_PKG_PATH):
        return jsonify({'status': 'error', 'message': '不正なマップ名です。'}), 400
    return None


def _map_edit_error(error):
    status = getattr(error, 'status', 400)
    return jsonify({'status': 'error', 'message': str(error)}), status


def _map_edit_meta():
    """クエリ meta (JSON) を読む。本文を点群や画素に使うため、設定はクエリで受け取る。"""
    try:
        meta = json.loads(request.args.get('meta', '') or '{}')
    except ValueError:
        raise map_from_points.MapEditError('meta の JSON を読めません。')
    if not isinstance(meta, dict):
        raise map_from_points.MapEditError('meta の形式が不正です。')
    return meta


@app.route('/map_params/<map_name>', methods=['GET'])
def get_map_params(map_name):
    """地図を作ったときの設定 (無ければ既定値) と元の点群の情報を返す (API)"""
    problem = _map_edit_request(map_name)
    if problem:
        return problem
    try:
        return jsonify({'status': 'success',
                        **map_from_points.describe_map_params(MAP_DIR, DATA_DIR, map_name)})
    except ValueError as e:
        return _map_edit_error(e)


@app.route('/map_params/<map_name>/preview', methods=['POST'])
def preview_map_params(map_name):
    """設定を変えて作り直した PCD か PGM を返す (保存はしない) (API)

    クエリ what: 'pcd' (binary PCD) / 'pgm' (P5 の PGM)
    クエリ meta: {pcd: {spacing, crop, z_min, z_max}, pgm: {...}, use_pending_source}
    PGM のときは YAML の内容と統計を X-Map-Info ヘッダー (URL エンコードした JSON) で返す。
    """
    problem = _map_edit_request(map_name)
    if problem:
        return problem
    what = request.args.get('what', 'pgm')
    if what not in ('pcd', 'pgm'):
        return jsonify({'status': 'error', 'message': 'what は pcd か pgm です。'}), 400
    try:
        meta = _map_edit_meta()
        built = map_from_points.build_edited_map(
            MAP_DIR, DATA_DIR, map_name, meta.get('pcd'), meta.get('pgm'),
            use_pending=bool(meta.get('use_pending_source')), need_pgm=(what == 'pgm'))
    except (ValueError, OSError) as e:
        return _map_edit_error(e)

    info = {
        'points': int(built['points'].shape[0]),
        'source_points': built['source_count'],
        'changes_points': built['changes_points'],
        'extra_fields': built['extra_fields'],
        'pcd': built['pcd_params'],
        'pgm': built['pgm_params'],
    }
    if what == 'pcd':
        body = map_from_points.pcd_binary_bytes(built['points'], built['colors'])
    else:
        pgm = built['pgm']
        height, width = pgm.shape
        body = f'P5\n{width} {height}\n255\n'.encode('ascii') + pgm.tobytes()
        info.update({'map_info': built['map_info'], 'pgm_stats': built['pgm_stats']})
    response = app.response_class(body, mimetype='application/octet-stream')
    response.headers['X-Map-Info'] = urllib.parse.quote(json.dumps(info))
    response.headers['Cache-Control'] = 'no-store'
    return response


@app.route('/map_params/<map_name>/save', methods=['POST'])
def save_map_params(map_name):
    """設定を変えて作り直した PCD / PGM / YAML と設定を保存する (API)

    クエリ meta: {save_as, overwrite, pcd, pgm, use_pending_source}
    本文 (任意): ブラウザでブラシ編集した PGM の画素 (PGM ファイルと同じ並び)
    """
    problem = _map_edit_request(map_name)
    if problem:
        return problem
    try:
        meta = _map_edit_meta()
        save_as = str(meta.get('save_as') or map_name).strip()
        if save_as.lower().endswith('.pgm') or save_as.lower().endswith('.pcd'):
            save_as = save_as[:-4]
        problem = _map_edit_request(save_as)
        if problem:
            return jsonify({'status': 'error',
                            'message': '保存するマップ名は半角英数字と _ . - だけで指定してください。'}), 400
        pixels = request.get_data(cache=False)
        result = map_from_points.save_edited_map(
            MAP_DIR, DATA_DIR, map_name, save_as, meta.get('pcd'), meta.get('pgm'),
            use_pending=bool(meta.get('use_pending_source')),
            pgm_pixels=pixels if pixels else None,
            overwrite=bool(meta.get('overwrite')))
    except (ValueError, OSError) as e:
        return _map_edit_error(e)

    message = f'地図「{save_as}」を保存しました (点群 {result["points"]:,} 点、' \
              f'PGM {result["pgm"]["width"]}×{result["pgm"]["height"]} 画素)。'
    if not result['pcd_written']:
        message += ' 点群は変わらないため PCD はそのままです。'
    if result['extra_fields_dropped']:
        message += f' PCD は x y z だけで書き出したため、{" ".join(result["extra_fields_dropped"])} の項目は元の点群にだけ残っています。'
    return jsonify({
        'status': 'success',
        'message': message,
        'map_name': save_as,
        'points': result['points'],
        'pgm': result['pgm'],
        'yaml': result['yaml'],
        'params': result['params'],
        'files': [os.path.relpath(f, HOKUYO_NAV2_PKG_PATH) for f in result['files']],
    })


@app.route('/map_params/<map_name>/source', methods=['POST'])
def upload_map_source(map_name):
    """Google から取り直した点群を、保存するまでの元の点群として置いておく (API)

    クエリ meta: {type, spacing, region {min_x, min_y, max_x, max_y}, point_format ('xyz' / 'xyzrgb')}
    本文: 点群 (リトルエンディアン。float32 x,y,z、xyzrgb なら続けて uint32 の色 0x00RRGGBB。この地図の座標)
    """
    problem = _map_edit_request(map_name)
    if problem:
        return problem
    if not os.path.isfile(os.path.join(MAP_DIR, f'{map_name}.pcd')):
        return jsonify({'status': 'error', 'message': f'map/{map_name}.pcd がありません。'}), 404
    try:
        meta = _map_edit_meta()
        spacing = float(meta.get('spacing'))
        region = map_from_points._bounds_dict(map_from_points._bounds_tuple(meta.get('region')))
        if not (math.isfinite(spacing) and spacing > 0):
            raise map_from_points.MapEditError('点の間隔が不正です。')
    except (KeyError, TypeError, ValueError) as e:
        return _map_edit_error(e)
    point_format = str(meta.get('point_format') or 'xyz')
    record_bytes = map_from_points.POINT_RECORD_BYTES.get(point_format)
    if record_bytes is None:
        return jsonify({'status': 'error', 'message': f'point_format {point_format} には対応していません。'}), 400
    body = request.get_data(cache=False)
    if len(body) == 0 or len(body) % record_bytes != 0:
        return jsonify({'status': 'error', 'message': '点群データの長さが不正です (x,y,z の float32 と色の並び)。'}), 400
    points, colors = map_from_points.points_from_bytes(body, point_format)
    if points.shape[0] > MAX_UPLOADED_POINTS:
        return jsonify({'status': 'error',
                        'message': f'点が多すぎます ({points.shape[0]} 点、上限 {MAX_UPLOADED_POINTS} 点)。'}), 400
    try:
        info = map_from_points.save_pending_source(
            DATA_DIR, map_name, points,
            {'type': str(meta.get('type') or 'google_3d_tiles'), 'spacing': spacing, 'region': region},
            colors=colors)
    except OSError as e:
        return jsonify({'status': 'error', 'message': f'点群を保存できません: {e}'}), 500
    return jsonify({'status': 'success', 'source': info})


@app.route('/save_pgm', methods=['POST'])
def save_pgm():
    """3Dビューアで編集した2Dマップ (PGM) を保存する (API)

    本文には編集後のグレースケール値をそのまま (1画素1バイト) 送る。
    別名で保存する場合は、元のYAMLを引き継いだ .yaml も一緒に作る。
    """
    source_map = os.path.basename(request.args.get('source_map', '') or '')
    save_as = os.path.basename(request.args.get('save_as', '') or source_map)
    try:
        width = int(request.args.get('width', 0))
        height = int(request.args.get('height', 0))
    except ValueError:
        return jsonify({'status': 'error', 'message': '画像サイズが不正です。'}), 400

    if not save_as:
        return jsonify({'status': 'error', 'message': '保存するマップ名を入力してください。'}), 400
    if save_as.lower().endswith('.pgm'):
        save_as = save_as[:-4]
    if not re.match(r'^[A-Za-z0-9_.-]{1,100}$', save_as):
        return jsonify({'status': 'error',
                        'message': 'マップ名に使えない文字が含まれています。'}), 400
    if width <= 0 or height <= 0:
        return jsonify({'status': 'error', 'message': '画像サイズが不正です。'}), 400

    pixels = request.get_data()
    if len(pixels) != width * height:
        return jsonify({'status': 'error',
                        'message': f'画像データの長さが合いません '
                                   f'(期待 {width * height} バイト / 受信 {len(pixels)} バイト)。'}), 400

    pgm_path = os.path.join(MAP_DIR, f'{save_as}.pgm')
    yaml_path = os.path.join(MAP_DIR, f'{save_as}.yaml')
    if not _is_safe_path(pgm_path, MAP_DIR) or not _is_safe_path(yaml_path, MAP_DIR):
        return jsonify({'status': 'error', 'message': '保存先が不正です。'}), 400

    try:
        # P5 (バイナリ grayscale) 形式で書き出す。
        header = f'P5\n{width} {height}\n255\n'.encode('ascii')
        with open(pgm_path, 'wb') as f:
            f.write(header)
            f.write(pixels)
    except OSError as e:
        return jsonify({'status': 'error', 'message': f'PGMを保存できません: {e}'}), 500

    # 別名保存のときは、元のYAML（解像度・原点）を引き継いだものを作る。
    yaml_created = False
    if source_map and save_as != source_map:
        source_yaml = os.path.join(MAP_DIR, f'{source_map}.yaml')
        if os.path.isfile(source_yaml):
            try:
                with open(source_yaml, 'r', encoding='utf-8') as f:
                    map_info = yaml.safe_load(f) or {}
                map_info['image'] = f'{save_as}.pgm'
                with open(yaml_path, 'w', encoding='utf-8') as f:
                    yaml.dump(map_info, f, default_flow_style=True)
                yaml_created = True
            except (OSError, yaml.YAMLError) as e:
                return jsonify({'status': 'error',
                                'message': f'PGMは保存しましたが、YAMLを作成できませんでした: {e}'}), 500

    message = f'2Dマップを保存しました: {save_as}.pgm'
    if yaml_created:
        message += f' / {save_as}.yaml'
    return jsonify({'status': 'success', 'message': message, 'map_name': save_as})


@app.route('/save_waypoints', methods=['POST'])
def save_waypoints():
    """
    クライアントから送信されたウェイポイントデータをJSONファイルとして保存する。
    """
    try:
        data = request.get_json()
        map_name = data.get('map_name')
        waypoints_data = data.get('waypoints')

        if not map_name:
            return jsonify({'status': 'error', 'message': 'マップ名が指定されていません。'}), 400
        if waypoints_data is None:
            return jsonify({'status': 'error', 'message': 'ウェイポイントデータが含まれていません。'}), 400

        filename = f"{map_name}.json"
        file_path = os.path.join(WP_DIR, filename)

        # セキュリティチェック
        if not _is_safe_path(file_path, WP_DIR):
            return jsonify({'status': 'error', 'message': '不正なファイルパスです。'}), 403

        with open(file_path, 'w') as f:
            import json
            json.dump(waypoints_data, f, indent=4)

        return jsonify({'status': 'success', 'message': f'ウェイポイントファイル ({filename}) が正常に保存されました。'})

    except Exception as e:
        return jsonify({'status': 'error', 'message': f'ウェイポイントの保存中にエラーが発生しました: {e}'}), 500

@app.route('/browse_pcd')
def browse_pcd():
    """PCD2PGM変換用のPCDファイルブラウザ。MAP_DIRから.pcdファイルを取得。"""
    try:
        if not _is_safe_path(MAP_DIR, HOKUYO_NAV2_PKG_PATH):
             flash("セキュリティ上の理由により、このディレクトリにはアクセスできません。", "error")
             return redirect(url_for('main_gui'))

        items = os.listdir(MAP_DIR)
        # .pcd 拡張子を持つファイルのみを抽出
        pcd_files = sorted([item for item in items if item.lower().endswith('.pcd')])
        
        return render_template('pcd_browse.html', 
                               pcd_files=pcd_files,
                               map_dir=MAP_DIR) 

    except (FileNotFoundError, PermissionError) as e:
        flash(f"マップディレクトリの操作中にエラーが発生しました: {e}", "error")
        return redirect(url_for('main_gui'))

@app.route('/pcd2pgm_select', methods=['POST'])
def pcd2pgm_select():
    """選択されたPCDファイル名を受け取り、ウェイポイント選択画面へ遷移する"""
    selected_pcd_filename = request.form.get('pcd_filename') 
    
    if not selected_pcd_filename:
        flash("PCDファイルが選択されていません。", "error")
        return redirect(url_for('browse_pcd'))
    return redirect(url_for('browse_waypoint', 
                            input_pcd_filename=selected_pcd_filename))

@app.route('/browse_waypoint')
def browse_waypoint():
    """
    PCD2PGM変換に使用するウェイポイントファイル（.json）のリストを表示するページをレンダリングする。
    """
    input_pcd_filename = request.args.get('input_pcd_filename', 'PCDファイル名未指定')
    
    if not input_pcd_filename:
        flash("エラー: 変換元のPCDファイルが指定されていません。再度選択してください。", "error")
        return redirect(url_for('browse_pcd'))
    try:
        all_files = os.listdir(WP_DIR)
        
        waypoint_files = [f for f in all_files if f.endswith('.json')]
    except Exception:
        print(f"ERROR: Failed to list files in WP_DIR: {e}")
        waypoint_files = []

    return render_template('waypoint_browse.html',
                           input_pcd_filename=input_pcd_filename,
                           waypoint_files=waypoint_files)


@app.route('/pcd2pgm_select_waypoint', methods=['POST'])
def pcd2pgm_select_waypoint():
    """選択されたウェイポイントファイル名を受け取り、最終的な変換設定画面へ遷移する"""
    pcd_filename = request.form.get('pcd_filename')
    waypoint_filename = request.form.get('waypoint_filename')
    
    if not pcd_filename:
        flash("PCDファイルが選択されていません。", "error")
        return redirect(url_for('browse_pcd'))
    
    # ウェイポイントファイルは空文字列（スキップ）も許容
    if waypoint_filename:
        full_path = os.path.join(WP_DIR, waypoint_filename)
        if not os.path.exists(full_path):
             flash(f"警告: ウェイポイントファイル '{waypoint_filename}' が見つかりませんでした。ウェイポイント情報なしで変換を実行します。", "warning")
             waypoint_filename = '' # ファイルがない場合は空にして送信

    base_map_name = os.path.splitext(pcd_filename)[0]

    config_files = _mapping_config_files()

    # 最終的な変換設定画面 (pcd_pgm_convert.html) に遷移
    return render_template('pcd_pgm_convert.html', 
                            input_pcd_filename=pcd_filename,
                            input_waypoint_filename=waypoint_filename, # ウェイポイントファイル名を渡す
                            default_output_name=f'{base_map_name}',
                            config_files=config_files)

@app.route('/pcd2pgm_convert', methods=['POST'])
def pcd2pgm_convert():
    """PCD to PGM 変換処理を開始し、完了までポーリングするAPI"""
    try:
        data = request.get_json()
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'リクエストJSONのパースエラー: {e}'}), 400

    input_pcd_filename = data.get('input_pcd_filename')
    output_map_name = data.get('output_map_name') # PGMファイルのベース名
    waypoint_filename = data.get('waypoint_filename', '') # ウェイポイントファイル名を受け取る
    loop_waypoints = data.get('loop_waypoints', 'false').lower() # ループフラグを受け取るオプション
    config_filename = data.get('config_file', '')

    if not input_pcd_filename:
        return jsonify({'status': 'error', 'message': '入力PCDファイル名が指定されていません。'}), 400
    if not os.path.exists(os.path.join(MAP_DIR, input_pcd_filename)):
        return jsonify({'status': 'error', 'message': f'入力PCDファイルが見つかりません: {input_pcd_filename}'}), 400
    if not output_map_name:
        return jsonify({'status': 'error', 'message': '出力マップ名を入力してください。'}), 400
    
    # ウェイポイントファイルが存在するかチェック（存在しない場合は空文字列のままにする）
    if waypoint_filename and not os.path.exists(os.path.join(WP_DIR, waypoint_filename)):
        waypoint_filename = ''
    config_file_path = os.path.join(CONFIG_DIR, config_filename) if config_filename else ''
    
    FLAG_MODE = 'PCD2PGM'
    flag_file_name_full = f'{output_map_name}.{FLAG_MODE}_DONE'
    
    try:
        script_path = os.path.join(BASE_PATH, "start_mapping.sh")
        command_list = [
            script_path, 
            "pcd2pgm",
            input_pcd_filename,
            output_map_name,
            MAP_DIR,
            waypoint_filename,
            loop_waypoints,
            flag_file_name_full,
            config_file_path
        ]
        
        # 別スレッドで実行
        job_id, log_path = create_job('pcd2pgm', output_map_name)
        Thread(target=run_subprocess, args=(command_list, log_path)).start()
        result_message = f"PCD to PGM 変換スクリプトがバックグラウンドで開始されました。出力マップ名: {output_map_name}"
        
        return jsonify({
            'status': 'success',
            'message': result_message,
            'map_name': output_map_name,
            'job_id': job_id
        })
        
    except Exception as e:
        print(f"ERROR: PCD2PGM Mapping failed with exception: {e}")
        return jsonify({'status': 'error', 'message': f'処理中にエラーが発生しました: {e}'}), 500

@app.route('/check_pcd2pgm_status', methods=['POST'])
def check_pcd2pgm_status():
    """PCD to PGM 変換処理の完了ステータスをチェックするAPI。"""
    data = request.get_json()
    output_map_name = data.get('output_map_name')

    if not output_map_name:
        return jsonify({'status': 'error', 'message': '出力マップ名が指定されていません。'}), 400

    flag_file_name = f'{output_map_name}.PCD2PGM_DONE'
    flag_file_path = os.path.join(MAP_DIR, flag_file_name) 
    
    if os.path.exists(flag_file_path):
        pgm_filename = f'{output_map_name}.pgm'
        yaml_filename = f'{output_map_name}.yaml'
        pgm_file_path = os.path.join(MAP_DIR, pgm_filename)
        yaml_file_path = os.path.join(MAP_DIR, yaml_filename)
        
        try:
            os.remove(flag_file_path)
            print(f"PCD2PGM completion flag removed: {flag_file_path}")
        except Exception as e:
            print(f"Warning: Failed to remove flag file {flag_file_path}: {e}")
            
        if not os.path.exists(pgm_file_path) or not os.path.exists(yaml_file_path):
            return jsonify({
                'status': 'error', 
                'message': f'PGMまたはYAMLファイルが見つかりません。フラグは存在しましたが、"{pgm_filename}"または"{yaml_filename}"が{MAP_DIR}に見つかりません。'
            })
            
        return jsonify({
            'status': 'finished', 
            'message': 'PCD to PGM 変換処理が完了しました。マップファイルをダウンロードできます。',
            'map_name': output_map_name,
            'download_url': url_for('download_pgm_map', basename=output_map_name) 
        })
    else:
        return jsonify({
            'status': 'in_progress', 
            'message': 'PCD to PGM 変換処理を続行中です...'
        })

@app.route('/pcd_pgm_convert_page')
def pcd_pgm_convert_page():
    input_pcd_filename = request.args.get('input_pcd_filename')
    input_waypoint_filename = request.args.get('input_waypoint_filename', '')
    loop_waypoints = request.args.get('loop_waypoints', 'false')
    
    if not input_pcd_filename or input_pcd_filename == 'PCDファイル名未指定':
        flash('エラー: 変換元のPCDファイルが指定されていません。再度選択してください。', 'error')
        return redirect(url_for('index'))
    
    default_output_name = input_pcd_filename.replace('.pcd', '') if input_pcd_filename.endswith('.pcd') else f'{input_pcd_filename}'
    
    config_files = _mapping_config_files()

    return render_template('pcd_pgm_convert.html', 
                           input_pcd_filename=input_pcd_filename, 
                           default_output_name=default_output_name,
                           input_waypoint_filename=input_waypoint_filename,
                           loop_waypoints=loop_waypoints,
                           config_files=config_files)

@app.route('/download_pgm_map/<basename>')
def download_pgm_map(basename):
    """PGMとYAMLファイルをZIP圧縮してダウンロードさせる"""
    temp_dir = os.path.join(DOWNLOAD_FOLDER, f'temp_pgm_{basename}')
    zip_base = os.path.join(DOWNLOAD_FOLDER, basename + '_pgm_map')
    zip_path = zip_base + '.zip'
    os.makedirs(temp_dir, exist_ok=True)
    
    pgm_filename = f'{basename}.pgm'
    yaml_filename = f'{basename}.yaml'
    
    try:
        shutil.copy2(os.path.join(MAP_DIR, pgm_filename), temp_dir)
        shutil.copy2(os.path.join(MAP_DIR, yaml_filename), temp_dir)
        shutil.make_archive(zip_base, 'zip', temp_dir)
        
        zip_filename_only = os.path.basename(zip_path)

        return send_from_directory(
            DOWNLOAD_FOLDER, 
            zip_filename_only, 
            as_attachment=True
        )
    except FileNotFoundError:
        flash('ダウンロード用のマップファイルが見つかりません。', 'error')
        return redirect(url_for('main_gui'))
    except Exception as e:
        flash(f'ファイルのZIP化中にエラーが発生しました: {e}', 'error')
        return redirect(url_for('main_gui'))
    finally:
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)
        if os.path.exists(zip_path):
            os.remove(zip_path)

@app.route('/rename_file', methods=['POST'])
def rename_file():
    """ファイル名を変更するAPI"""
    dir_type = request.form.get('dir_type')
    old_filename = request.form.get('old_filename')
    new_filename = request.form.get('new_filename')

    dir_map = {'map': MAP_DIR, 'wp': WP_DIR, 'config': CONFIG_DIR}
    if dir_type not in dir_map:
        return jsonify({'status': 'error', 'message': '無効なディレクトリタイプです。'}), 400

    target_dir = dir_map[dir_type]

    if not old_filename or not new_filename:
        return jsonify({'status': 'error', 'message': 'ファイル名が指定されていません。'}), 400

    if old_filename == new_filename:
        return jsonify({'status': 'success', 'message': 'ファイル名は変更されませんでした。'})

    old_path = os.path.join(target_dir, old_filename)
    new_path = os.path.join(target_dir, new_filename)

    if not _is_safe_path(old_path, target_dir) or not _is_safe_path(new_path, target_dir):
        return jsonify({'status': 'error', 'message': '不正なファイルパスです。'}), 400

    if os.path.exists(new_path):
        return jsonify({'status': 'error', 'message': f'ファイル "{new_filename}" は既に存在します。'}), 400

    try:
        # マップファイルの場合、関連ファイルもリネーム
        if dir_type == 'map':
            old_base, old_ext = os.path.splitext(old_filename)
            new_base, new_ext = os.path.splitext(new_filename)

            related_exts = ['.pcd', '.yaml', '.pgm']
            for ext in related_exts:
                old_rel_path = os.path.join(target_dir, old_base + ext)
                new_rel_path = os.path.join(target_dir, new_base + ext)
                if os.path.exists(old_rel_path):
                    if os.path.exists(new_rel_path):
                         return jsonify({'status': 'error', 'message': '拡張子の変更はできません。ファイル名のみ変更してください。'}), 400
                    os.rename(old_rel_path, new_rel_path)
        else:
            # wp と config の場合も拡張子変更を禁止
            if dir_type == 'wp' or dir_type == 'config':
                old_ext = os.path.splitext(old_filename)[1]
                new_ext = os.path.splitext(new_filename)[1]
                if old_ext != new_ext:
                    return jsonify({'status': 'error', 'message': '拡張子の変更はできません。ファイル名のみ変更してください。'}), 400
            os.rename(old_path, new_path)

        return jsonify({'status': 'success', 'message': f'"{old_filename}" を "{new_filename}" に変更しました。'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'ファイル名の変更中にエラーが発生しました: {e}'}), 500

# Helper to get available files for dropdowns
def _get_available_files(directory, extension):
    """指定されたディレクトリから特定の拡張子のファイル名（拡張子なし）のリストを取得する"""
    try:
        files = sorted([
            os.path.splitext(f)[0] for f in os.listdir(directory) if f.endswith(extension)
        ])
        return files
    except FileNotFoundError:
        return []

@app.route('/browse_files/<dir_type>', defaults={'path': ''})
@app.route('/browse_files/<dir_type>/<path:path>')
def browse_files(dir_type, path):
    """
    指定されたタイプのディレクトリ内のファイルを表示する。
    dir_type: 'map', 'wp', 'config'
    """
    dir_map = {
        'map': {'path': MAP_DIR, 'title': 'マップファイル管理'},
        'wp': {'path': WP_DIR, 'title': 'ウェイポイントファイル管理'},
        'config': {'path': CONFIG_DIR, 'title': '設定ファイル管理'}
    }

    if dir_type not in dir_map:
        flash('無効なディレクトリタイプです。', 'error')
        return redirect(url_for('main_gui'))

    base_dir = dir_map[dir_type]['path']
    title = dir_map[dir_type]['title']
    target_dir = os.path.join(base_dir, path)

    if not _is_safe_path(target_dir, base_dir):
        flash("セキュリティ上の理由により、このディレクトリにはアクセスできません。", "error")
        return redirect(url_for('browse_files', dir_type=dir_type))

    try:
        items = sorted(os.listdir(target_dir))
        files_with_info = []
        parent_path = os.path.dirname(path) if path else None

        if dir_type == 'config':
            # configディレクトリの場合、ヘッダーをチェックして構造化編集可能か判定
            expected_header = "map_file,waypoint_file,nav_type,interval"
            csv_files = [f for f in items if f.lower().endswith('.csv')]
            for filename in csv_files:
                if os.path.isdir(os.path.join(target_dir, filename)): continue

                file_path = os.path.join(target_dir, filename)
                is_structured = False
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        header = f.readline().strip()
                        if header == expected_header:
                            is_structured = True
                except Exception as e:
                    print(f"Could not read or parse {filename}: {e}")
                files_with_info.append({
                    'name': filename, 
                    'is_structured': is_structured,
                    # マッピング用のパラメータ設定CSVは、専用のプルダウン編集画面を出す
                    'is_mapping_config': _is_mapping_config(file_path),
                    'is_dir': False, # configはファイルのみ
                    'path': os.path.join(path, filename)
                })
        else:
            # map と wp の場合
            for item_name in items:
                item_path = os.path.join(target_dir, item_name)
                is_dir = os.path.isdir(item_path)
                files_with_info.append({
                    'name': item_name,
                    'is_dir': is_dir,
                    'path': os.path.join(path, item_name)
                })

        return render_template('file_browser.html', 
                               files=files_with_info, 
                               dir_type=dir_type, 
                               title=title,
                               current_path=path,
                               parent_path=parent_path,
                               # 設定ファイル新規作成で「参考にするROS Bag」を選ぶため
                               rosbags=_rosbag_directories() if dir_type == 'config' else [])
    except (FileNotFoundError, PermissionError) as e:
        flash(f"ディレクトリの操作中にエラーが発生しました: {e}", "error")
        return redirect(url_for('main_gui'))

@app.route('/delete_item', methods=['POST'])
def delete_item():
    """ファイルまたはディレクトリを削除するAPI"""
    dir_type = request.form.get('dir_type') 
    item_paths = request.form.getlist('item_paths') # 複数アイテムに対応
    current_path = request.form.get('current_path', '')

    dir_map = {'map': MAP_DIR, 'wp': WP_DIR, 'config': CONFIG_DIR}
    if dir_type not in dir_map:
        flash('無効なディレクトリタイプです。', 'error')
        return redirect(url_for('main_gui'))

    if not item_paths:
        flash('削除するアイテムが選択されていません。', 'error')
        return redirect(url_for('browse_files', dir_type=dir_type, path=current_path))

    base_dir = dir_map[dir_type]
    deleted_count = 0
    error_count = 0

    for item_path_str in item_paths:
        # item_path_str is the relative path from base_dir
        full_path = os.path.join(base_dir, item_path_str)

        if not _is_safe_path(full_path, base_dir):
            flash(f'不正なパスです: {item_path_str}', 'error')
            error_count += 1
            continue

        try:
            if os.path.isdir(full_path):
                shutil.rmtree(full_path)
                deleted_count += 1
            elif os.path.isfile(full_path):
                os.remove(full_path)
                deleted_count += 1
            else:
                # Already deleted or does not exist
                pass
        except Exception as e:
            flash(f'アイテム "{os.path.basename(item_path_str)}" の削除中にエラーが発生しました: {e}', 'error')
            error_count += 1

    if deleted_count > 0:
        flash(f'{deleted_count}個のアイテムを削除しました。', 'success')
    if error_count == 0 and deleted_count == 0:
        flash('削除対象のアイテムが見つかりませんでした。', 'warning')

    return redirect(url_for('browse_files', dir_type=dir_type, path=current_path))

def _multi_map_template(bag_info=None):
    """マルチマップ走行シナリオCSVの雛形。"""
    return "map_file,waypoint_file,nav_type,interval\n"


def _mapping_config_template(bag_info=None):
    """マッピング用パラメータ設定CSVの雛形。

    全項目を既定値付きで書き出す。マッピングのスクリプトはCSVを行番号で
    読むため、行が欠けていると値がずれてしまう。
    参考にする ROS Bag が渡された場合は、トピック名と座標系をその内容に
    合わせて埋めておく。
    """
    if mapping_diagnostics is None:
        return ''
    return mapping_diagnostics.mapping_config_template(bag_info)


# 新規CSV作成で選べる形式。値 -> (表示名, 中身を作る関数)
CSV_TEMPLATES = {
    'mapping_config': ('マッピング用パラメータ設定', _mapping_config_template),
    'multi_map': ('マルチマップ走行シナリオ', _multi_map_template),
}


@app.route('/create_csv', methods=['POST'])
def create_csv():
    """新規CSVファイルを作成する"""
    filename = request.form.get('filename')
    csv_type = request.form.get('csv_type', 'mapping_config')
    if csv_type not in CSV_TEMPLATES:
        csv_type = 'mapping_config'
    reference_bag = request.form.get('reference_bag', '')

    if not filename:
        flash('ファイル名が入力されていません。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    if not filename.lower().endswith('.csv'):
        filename += '.csv'

    file_path = os.path.join(CONFIG_DIR, filename)

    if not _is_safe_path(file_path, CONFIG_DIR):
        flash('不正なファイルパスです。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    if os.path.exists(file_path):
        flash(f'ファイル "{filename}" は既に存在します。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    # 参考にする ROS Bag が選ばれていれば、その内容から初期値を決める。
    bag_info = None
    if reference_bag and csv_type == 'mapping_config':
        if _is_safe_path(reference_bag, ROSBAG_ROOT_DIR) and os.path.exists(reference_bag):
            bag_info = get_bag_info(reference_bag)
        else:
            flash('参考にするROS Bagが見つからないため、既定値で作成します。', 'warning')

    label, build_template = CSV_TEMPLATES[csv_type]
    content = build_template(bag_info)
    if not content:
        flash('マッピング用の雛形を作成できませんでした。ROS 2 環境を確認してください。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    try:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        flash(f'{label}ファイル "{filename}" を作成しました。', 'success')
    except Exception as e:
        flash(f'ファイルの作成中にエラーが発生しました: {e}', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    # マッピング用設定はそのままプルダウン編集画面へ進む。
    # 参考にした ROS Bag は編集画面でもそのまま候補の取得元として使う。
    if csv_type == 'mapping_config':
        return redirect(url_for('edit_mapping_config', filename=filename,
                                bag=reference_bag if bag_info else None))

    return redirect(url_for('browse_files', dir_type='config'))

@app.route('/create_plain_text')
def create_plain_text():
    """プレーンテキストエディタで新規CSVファイルを作成するページを表示"""
    csv_type = request.args.get('csv_type', 'mapping_config')
    if csv_type not in CSV_TEMPLATES:
        csv_type = 'mapping_config'

    bag_info = None
    reference_bag = request.args.get('reference_bag', '')
    if reference_bag and csv_type == 'mapping_config' \
            and _is_safe_path(reference_bag, ROSBAG_ROOT_DIR) \
            and os.path.exists(reference_bag):
        bag_info = get_bag_info(reference_bag)

    return render_template('plain_text_editor.html', filename='',
                           content=CSV_TEMPLATES[csv_type][1](bag_info), is_new=True)

@app.route('/edit_plain_text/<dir_type>/<path:filename>')
def edit_plain_text(dir_type, filename):
    """プレーンテキストエディタで既存のファイルを編集するページを表示"""
    dir_map = {'map': MAP_DIR, 'wp': WP_DIR, 'config': CONFIG_DIR}
    if dir_type not in dir_map:
        flash('無効なディレクトリタイプです。', 'error')
        return redirect(url_for('main_gui'))

    target_dir = dir_map[dir_type]
    file_path = os.path.join(target_dir, filename)

    if not _is_safe_path(file_path, target_dir):
        flash('不正なファイルパスです。', 'error')
        return redirect(url_for('browse_files', dir_type=dir_type))

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        return render_template('plain_text_editor.html', filename=filename, content=content, is_new=False, dir_type=dir_type)
    except Exception as e:
        flash(f'ファイルの読み込み中にエラーが発生しました: {e}', 'error')
        return redirect(url_for('browse_files', dir_type=dir_type))

@app.route('/save_plain_text', methods=['POST'])
def save_plain_text():
    """プレーンテキストエディタから送信された内容を保存"""
    dir_type = request.form.get('dir_type', 'config') # デフォルトはconfig
    filename = request.form.get('filename')
    content = request.form.get('content')

    dir_map = {'map': MAP_DIR, 'wp': WP_DIR, 'config': CONFIG_DIR}
    target_dir = dir_map.get(dir_type, CONFIG_DIR)
    file_path = os.path.join(target_dir, filename)

    if not _is_safe_path(file_path, target_dir):
        flash('不正なファイルパスです。', 'error')
        return redirect(url_for('browse_files', dir_type=dir_type))

    try:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content.replace('\r\n', '\n')) # 改行コードをLFに統一
        flash(f'ファイル "{filename}" を保存しました。', 'success')
    except Exception as e:
        flash(f'ファイルの保存中にエラーが発生しました: {e}', 'error')
    return redirect(url_for('browse_files', dir_type=dir_type))

@app.route('/edit_mapping_config/<path:filename>')
def edit_mapping_config(filename):
    """マッピング用パラメータ設定CSVを、プルダウンで編集する画面。

    ROS Bag を選ぶと、そこに記録されているトピック名・座標系が
    各項目の候補として表示される。
    """
    config_path = _resolve_config_path(filename)
    if not config_path or not os.path.isfile(config_path):
        flash(f'設定ファイルが見つかりません: {filename}', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    if not _is_mapping_config(config_path):
        flash('マッピング用のパラメータ設定ファイルではありません。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    # 新規作成直後などに、候補の取得元とする ROS Bag を引き継ぐ。
    selected_bag = request.args.get('bag', '')
    if selected_bag and not (_is_safe_path(selected_bag, ROSBAG_ROOT_DIR)
                             and os.path.exists(selected_bag)):
        selected_bag = ''

    return render_template('mapping_config_editor.html',
                           config_file=os.path.basename(config_path),
                           rosbags=_rosbag_directories(),
                           selected_bag=selected_bag)


@app.route('/edit_csv/<filename>')
def edit_csv(filename):
    """CSVファイルを編集するページを表示"""
    file_path = os.path.join(CONFIG_DIR, filename)

    if not _is_safe_path(file_path, CONFIG_DIR):
        flash('不正なファイルパスです。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    try:
        # CSVをパースして構造化データとして渡す
        csv_data = []
        with open(file_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                csv_data.append(row)

        # プルダウン用のファイルリストを取得
        available_map_files = _get_available_files(MAP_DIR, '.yaml')
        available_wp_files = _get_available_files(WP_DIR, '.json')

        return render_template('csv_editor.html', 
                               filename=filename, 
                               csv_data=csv_data,
                               available_map_files=available_map_files,
                               available_wp_files=available_wp_files)
    except Exception as e:
        flash(f'ファイルの読み込み中にエラーが発生しました: {e}', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

@app.route('/save_structured_csv', methods=['POST'])
def save_structured_csv():
    """編集されたCSVファイルの内容を保存"""
    filename = request.form.get('filename')
    rows_data_json = request.form.get('rows_data')
    file_path = os.path.join(CONFIG_DIR, filename)

    if not _is_safe_path(file_path, CONFIG_DIR):
        flash('不正なファイルパスです。', 'error')
        return redirect(url_for('browse_files', dir_type='config'))

    try:
        rows = json.loads(rows_data_json)
        
        if not rows:
            # 空の場合はヘッダーのみ書き込む
            with open(file_path, 'w', encoding='utf-8', newline='') as f:
                f.write("map_file,waypoint_file,nav_type,interval\n")
        else:
            with open(file_path, 'w', encoding='utf-8', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=["map_file", "waypoint_file", "nav_type", "interval"])
                writer.writeheader()
                writer.writerows(rows)

        flash(f'ファイル "{filename}" を保存しました。', 'success')
    except Exception as e:
        flash(f'ファイルの保存中にエラーが発生しました: {e}', 'error')

    return redirect(url_for('browse_files', dir_type='config'))

@app.route('/gui', methods=['GET', 'POST'])
def trigger_script():
    """GUIからのPOSTリクエストに基づき、対応するスクリプトを実行する。"""
    global current_mode
    if request.method == 'GET':
        return render_template('index.html') 
    
    command = request.form.get("command") or request.get_json().get("command")

    if command == "execute_navigation_run":
        confirmation_field = request.form.get("confirm_check") 
        arguments = request.form.get("arguments")
        mapfile = request.form.get("mapfile")
        wpfile = request.form.get("wpfile")
        csvfile = request.form.get("csvfile")
        if confirmation_field or request.form:
            script_path = os.path.join(BASE_PATH, "start_navigation.sh")
            command_list = [script_path]
            if arguments:
                command_list.append(arguments)
                if arguments == 'multi_map' and csvfile:
                    command_list.append(csvfile)
                elif mapfile:
                    command_list.append(mapfile)
                if wpfile:
                    command_list.append(wpfile)

            # シミュレータを使うかは GUI のトグルで決め、オプション CSV の use_unity_sim より優先させる
            use_unity_sim = 'true' if request.form.get("use_unity_sim") else 'false'
            Thread(target=run_subprocess, args=(command_list,),
                   kwargs={'extra_env': {'USE_UNITY_SIM': use_unity_sim}}).start()
            current_mode = "running"
            return redirect('/navigation_executed')
        else:
            return render_template('navigation_run_popup.html', error="開始を確認してください。")
    elif command == "execute_mapping_filter":
        current_mode = "stopped" 
        flash("ROS Bagフィルタリング機能に遷移します。フィルタ対象のROS Bagを選択してください。", "info")
        return redirect(url_for('browse_rosbag')) 
    
    elif command == "execute_mapping_p2o":
        current_mode = "stopped"
        flash("P2O マッピングに使用するROS Bagを選択してください。", "info")
        return redirect(url_for('browse_rosbag', mode='p2o'))

    elif command == "execute_mapping_lio_raw":
        current_mode = "stopped"
        flash("LIO-RAW マッピングに使用するROS Bagを選択してください。", "info")
        return redirect(url_for('browse_rosbag', mode='lio_raw'))

    elif command == "execute_mapping_pcd2pgm":
        current_mode = "stopped"
        flash("PGMマップに変換するPCDファイルを選択してください。", "info")
        return redirect(url_for('browse_pcd'))

    elif command.startswith("execute_mapping_"):
        mapping_type = command.replace("execute_mapping_", "")
        
        script_path = os.path.join(BASE_PATH, "start_mapping.sh")
        command_list = [script_path, mapping_type]
        
        Thread(target=run_subprocess, args=(command_list,)).start()
        current_mode = "mapping"
        return redirect('/mapping_executed')

    elif command == "stop":
        script_path = os.path.join(BASE_PATH, "ctrl/web_kill_all_rosnode.sh")
        Thread(target=run_subprocess, args=([script_path],)).start()
        current_mode = "stopped"
        return render_template('stop.html')
        
    elif command == "ctrl":
        script_path = os.path.join(BASE_PATH, "start_getting_rosbag.sh")
        Thread(target=run_subprocess, args=([script_path],)).start()
        current_mode = "ctrl"
        return redirect('/ctrl_executed')
        
    elif command == "map":
        return redirect('/mapping_popup')
        
    else:
        print(f"Unknown command received: {command}")
        return "Unknown command", 400

if __name__ == '__main__':
    # 証明書と秘密鍵を環境変数で指定すると HTTPS で起動する。
    # ブラウザの位置情報 (Map Viewer の「デバイスの現在地を原点にする」) は、
    # localhost 以外では HTTPS のページでしか使えないため。
    ssl_cert = os.environ.get('HOKUYO_GUI_SSL_CERT', '').strip()
    ssl_key = os.environ.get('HOKUYO_GUI_SSL_KEY', '').strip()
    ssl_args = {}
    if ssl_cert or ssl_key:
        if not (os.path.isfile(ssl_cert) and os.path.isfile(ssl_key)):
            sys.exit('Error: HOKUYO_GUI_SSL_CERT と HOKUYO_GUI_SSL_KEY には、'
                     f'存在する証明書と秘密鍵のファイルを指定してください (cert={ssl_cert!r}, key={ssl_key!r})')
        ssl_args = {'certfile': ssl_cert, 'keyfile': ssl_key}
    scheme = 'https' if ssl_args else 'http'
    print(f"Flask Server starting at {scheme}://{SERVER_HOST}:{SERVER_PORT}")
    print(f"WebSocket Proxy server started at {'wss' if ssl_args else 'ws'}://{SERVER_HOST}:{SERVER_PORT}/ws")
    server = pywsgi.WSGIServer((SERVER_HOST, SERVER_PORT), app, handler_class=WebSocketHandler, **ssl_args)
    server.serve_forever()