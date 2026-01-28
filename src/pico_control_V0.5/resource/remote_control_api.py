#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
遥操控制API服务 - systemctl版本
通过systemctl系统服务来控制遥操程序的启动和停止

使用方法：
python3 remote_control_api.py

API接口：
- GET  /status     - 获取遥操服务状态
- POST /start      - 启动遥操服务
- POST /stop       - 停止遥操服务
- POST /restart    - 重启遥操服务
- POST /kill       - 强制杀死所有遥操进程（备用）
- GET  /logs       - 获取日志
- GET  /health     - 健康检查

示例：
curl http://localhost:8081/status
curl -X POST http://localhost:8081/start
curl -X POST http://localhost:8081/stop
"""

import os
import sys
import json
import time
import psutil
import threading
import subprocess
from datetime import datetime
from flask import Flask, request, jsonify
from flask_cors import CORS

# 配置
API_HOST = '0.0.0.0'
API_PORT = 8081
SERVICE_NAME = 'vr-arm-control'
LOG_FILE = 'remote_control.log'

# 全局变量
app = Flask(__name__)
CORS(app)  # 允许跨域请求

log_messages = []

def log_message(message):
    """记录日志消息"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    log_entry = f"[{timestamp}] {message}"
    log_messages.append(log_entry)
    print(log_entry)

    # 限制日志数量
    if len(log_messages) > 100:
        log_messages.pop(0)

    # 写入文件
    try:
        with open(LOG_FILE, 'a', encoding='utf-8') as f:
            f.write(log_entry + '\n')
    except Exception as e:
        print(f"写入日志失败: {e}")

def check_remote_status():
    """检查遥操服务状态"""
    try:
        # 使用systemctl检查服务状态
        result = subprocess.run(
            ['systemctl', 'is-active', SERVICE_NAME],
            capture_output=True,
            text=True,
            timeout=5
        )

        is_active = result.returncode == 0 and result.stdout.strip() == 'active'

        if is_active:
            # 获取服务详细信息
            status_result = subprocess.run(
                ['systemctl', 'show', SERVICE_NAME, '--property=MainPID,ActiveEnterTimestamp'],
                capture_output=True,
                text=True,
                timeout=5
            )

            pid = None
            start_time = None

            if status_result.returncode == 0:
                for line in status_result.stdout.strip().split('\n'):
                    if line.startswith('MainPID='):
                        pid_str = line.split('=', 1)[1]
                        pid = int(pid_str) if pid_str.isdigit() else None
                    elif line.startswith('ActiveEnterTimestamp='):
                        timestamp_str = line.split('=', 1)[1]
                        try:
                            # 解析systemd时间戳格式
                            start_time = datetime.strptime(timestamp_str, '%a %Y-%m-%d %H:%M:%S %Z')
                        except:
                            start_time = datetime.now()

            return {
                'running': True,
                'pid': pid,
                'start_time': start_time.isoformat() if start_time else None,
                'uptime': str(datetime.now() - start_time) if start_time else None,
                'service': SERVICE_NAME
            }
        else:
            return {
                'running': False,
                'pid': None,
                'start_time': None,
                'uptime': None,
                'service': SERVICE_NAME
            }

    except Exception as e:
        log_message(f"检查服务状态失败: {e}")
        return {
            'running': False,
            'pid': None,
            'start_time': None,
            'uptime': None,
            'service': SERVICE_NAME,
            'error': str(e)
        }

def start_remote_control():
    """启动遥操服务"""
    status = check_remote_status()
    if status['running']:
        return {'success': False, 'message': '遥操服务已在运行中', 'status': status}

    try:
        log_message("正在启动遥操服务...")

        # 使用systemctl启动服务
        result = subprocess.run(
            ['sudo', 'systemctl', 'start', SERVICE_NAME],
            capture_output=True,
            text=True,
            timeout=30
        )

        if result.returncode == 0:
            log_message("遥操服务启动成功")
            # 等待一会儿让服务完全启动
            time.sleep(3)
            status = check_remote_status()
            return {'success': True, 'message': '遥操服务启动成功', 'status': status}
        else:
            error_msg = result.stderr.strip() or result.stdout.strip()
            log_message(f"遥操服务启动失败: {error_msg}")
            return {'success': False, 'message': f'启动失败: {error_msg}', 'status': check_remote_status()}

    except subprocess.TimeoutExpired:
        log_message("启动遥操服务超时")
        return {'success': False, 'message': '启动超时', 'status': check_remote_status()}
    except Exception as e:
        log_message(f"启动遥操服务失败: {e}")
        return {'success': False, 'message': f'启动失败: {str(e)}', 'status': check_remote_status()}

def stop_remote_control():
    """停止遥操服务"""
    status = check_remote_status()
    if not status['running']:
        return {'success': False, 'message': '遥操服务未在运行', 'status': status}

    try:
        log_message("正在停止遥操服务...")

        # 使用systemctl停止服务
        result = subprocess.run(
            ['sudo', 'systemctl', 'stop', SERVICE_NAME],
            capture_output=True,
            text=True,
            timeout=30
        )

        if result.returncode == 0:
            log_message("遥操服务已停止")
            status = check_remote_status()
            return {'success': True, 'message': '遥操服务已停止', 'status': status}
        else:
            error_msg = result.stderr.strip() or result.stdout.strip()
            log_message(f"遥操服务停止失败: {error_msg}")
            return {'success': False, 'message': f'停止失败: {error_msg}', 'status': check_remote_status()}

    except subprocess.TimeoutExpired:
        log_message("停止遥操服务超时")
        return {'success': False, 'message': '停止超时', 'status': check_remote_status()}
    except Exception as e:
        log_message(f"停止遥操服务失败: {e}")
        return {'success': False, 'message': f'停止失败: {str(e)}', 'status': check_remote_status()}

def restart_remote_control():
    """重启遥操服务"""
    log_message("正在重启遥操服务...")

    try:
        # 使用systemctl重启服务
        result = subprocess.run(
            ['sudo', 'systemctl', 'restart', SERVICE_NAME],
            capture_output=True,
            text=True,
            timeout=45
        )

        if result.returncode == 0:
            log_message("遥操服务重启成功")
            # 等待一会儿让服务完全重启
            time.sleep(3)
            status = check_remote_status()
            return {'success': True, 'message': '遥操服务重启成功', 'status': status}
        else:
            error_msg = result.stderr.strip() or result.stdout.strip()
            log_message(f"遥操服务重启失败: {error_msg}")
            return {'success': False, 'message': f'重启失败: {error_msg}', 'status': check_remote_status()}

    except subprocess.TimeoutExpired:
        log_message("重启遥操服务超时")
        return {'success': False, 'message': '重启超时', 'status': check_remote_status()}
    except Exception as e:
        log_message(f"重启遥操服务失败: {e}")
        return {'success': False, 'message': f'重启失败: {str(e)}', 'status': check_remote_status()}

def kill_all_remote_processes():
    """强制杀死所有遥操相关进程（备用方法）"""
    log_message("使用systemctl停止服务，如果失败则强制杀死进程...")

    # 先尝试正常停止
    stop_result = stop_remote_control()
    if stop_result['success']:
        return stop_result

    # 如果停止失败，尝试强制杀死
    killed = []
    try:
        log_message("正常停止失败，开始强制清理进程...")

        # 查找并杀死所有相关进程
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if proc.info['name'] == 'python3':
                    cmdline = proc.info['cmdline']
                    if cmdline and len(cmdline) > 1:
                        cmd = cmdline[-1]
                        # 检查是否是遥操相关进程
                        if ('vr_arm_control' in cmd or 'web_control.py' in cmd):
                            # 排除当前API进程
                            if proc.pid != os.getpid():
                                log_message(f"强制杀死进程: PID={proc.pid}, CMD={cmd}")
                                try:
                                    proc.kill()
                                    killed.append(f"PID:{proc.pid} ({cmd})")
                                except Exception as e:
                                    log_message(f"杀死进程 {proc.pid} 失败: {e}")
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    except Exception as e:
        log_message(f"强制清理进程时出错: {e}")
        return {'success': False, 'message': f'清理失败: {str(e)}'}

    message = f'已清理 {len(killed)} 个进程' if killed else '未发现需要清理的进程'
    log_message(message)

    return {'success': True, 'message': message, 'killed': killed}

# API路由
@app.route('/status', methods=['GET'])
def get_status():
    """获取遥操服务状态"""
    status = check_remote_status()
    return jsonify({
        'success': True,
        'data': status,
        'timestamp': datetime.now().isoformat()
    })

@app.route('/start', methods=['POST'])
def api_start():
    """启动遥操服务"""
    result = start_remote_control()
    return jsonify({
        'success': result['success'],
        'message': result['message'],
        'data': result.get('status'),
        'timestamp': datetime.now().isoformat()
    })

@app.route('/stop', methods=['POST'])
def api_stop():
    """停止遥操服务"""
    result = stop_remote_control()
    return jsonify({
        'success': result['success'],
        'message': result['message'],
        'data': result.get('status'),
        'timestamp': datetime.now().isoformat()
    })

@app.route('/restart', methods=['POST'])
def api_restart():
    """重启遥操服务"""
    result = restart_remote_control()
    return jsonify({
        'success': result['success'],
        'message': result['message'],
        'data': result.get('status'),
        'timestamp': datetime.now().isoformat()
    })

@app.route('/kill', methods=['POST'])
def api_kill():
    """强制杀死所有遥操相关进程"""
    result = kill_all_remote_processes()
    return jsonify({
        'success': result['success'],
        'message': result['message'],
        'data': result.get('killed', []),
        'timestamp': datetime.now().isoformat()
    })

@app.route('/logs', methods=['GET'])
def get_logs():
    """获取日志"""
    limit = request.args.get('limit', default=50, type=int)
    recent_logs = log_messages[-limit:] if limit > 0 else log_messages

    return jsonify({
        'success': True,
        'data': recent_logs,
        'total': len(log_messages),
        'timestamp': datetime.now().isoformat()
    })

@app.route('/health', methods=['GET'])
def health_check():
    """健康检查"""
    return jsonify({
        'success': True,
        'status': 'healthy',
        'service': 'remote_control_api',
        'timestamp': datetime.now().isoformat()
    })

def background_monitor():
    """后台监控线程"""
    while True:
        try:
            check_remote_status()
            time.sleep(5)  # 每5秒检查一次
        except Exception as e:
            log_message(f"监控线程错误: {e}")
            time.sleep(10)

def main():
    """主函数"""
    # 检查依赖
    try:
        import flask_cors
    except ImportError:
        print("错误: 缺少依赖包，请安装: pip install flask-cors psutil")
        sys.exit(1)

    # 初始化日志
    log_message("遥操控制API服务启动 (systemctl版本)")

    # 启动后台监控线程
    monitor_thread = threading.Thread(target=background_monitor, daemon=True)
    monitor_thread.start()

    # 启动API服务器
    print("🤖 遥操控制API服务启动 (systemctl版本)")
    print(f"📡 API地址: http://{API_HOST}:{API_PORT}")
    print("📋 可用接口:")
    print(f"  GET  /status   - 获取状态")
    print(f"  POST /start    - 启动遥操服务")
    print(f"  POST /stop     - 停止遥操服务")
    print(f"  POST /restart  - 重启遥操服务")
    print(f"  POST /kill     - 强制杀死所有遥操进程")
    print(f"  GET  /logs     - 获取日志")
    print(f"  GET  /health   - 健康检查")
    print("\n📝 使用示例:")
    print(f"  curl http://localhost:{API_PORT}/status")
    print(f"  curl -X POST http://localhost:{API_PORT}/start")
    print(f"  curl -X POST http://localhost:{API_PORT}/stop")
    print(f"  curl -X POST http://localhost:{API_PORT}/restart")
    print(f"  curl -X POST http://localhost:{API_PORT}/kill")

    try:
        app.run(host=API_HOST, port=API_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        log_message("API服务被用户中断")
    except Exception as e:
        log_message(f"API服务启动失败: {e}")

if __name__ == "__main__":
    main()

