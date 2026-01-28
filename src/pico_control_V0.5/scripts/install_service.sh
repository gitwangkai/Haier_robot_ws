#!/bin/bash

# 一键安装VR遥操控制systemctl服务

echo "🔧 一键安装VR遥操控制systemctl服务..."

# 检查是否为root用户
if [[ $EUID -eq 0 ]]; then
   echo "❌ 请不要使用root用户运行此脚本，将使用sudo权限"
   exit 1
fi

# 获取当前用户名
USERNAME=$(whoami)
SERVICE_NAME="vr-arm-control"

# 检查服务文件是否存在
VR_SERVICE_FILE="vr-arm-control.service"
API_SERVICE_FILE="remote-control-api.service"

if [ ! -f "$VR_SERVICE_FILE" ]; then
    echo "❌ 找不到VR服务文件: $VR_SERVICE_FILE"
    exit 1
fi

if [ ! -f "$API_SERVICE_FILE" ]; then
    echo "❌ 找不到API服务文件: $API_SERVICE_FILE"
    exit 1
fi

echo "📋 复制服务文件到systemd目录..."
sudo cp "$VR_SERVICE_FILE" /etc/systemd/system/
sudo cp "$API_SERVICE_FILE" /etc/systemd/system/

echo "🔄 重新加载systemd配置..."
sudo systemctl daemon-reload

echo "✅ 启用服务..."
sudo systemctl enable remote-control-api
sudo systemctl disable vr-arm-control  # VR服务不设置开机自启

echo "🚀 启动服务API..."
sudo systemctl start remote-control-api

echo "🔑 配置sudo免密权限..."
echo "这将允许 $USERNAME 用户无需密码即可控制systemctl服务"

# 配置sudo免密权限
SUDOERS_LINE="$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/systemctl start $SERVICE_NAME"
SUDOERS_LINE2="$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/systemctl stop $SERVICE_NAME"
SUDOERS_LINE3="$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart $SERVICE_NAME"
SUDOERS_LINE4="$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/systemctl status $SERVICE_NAME"

# 添加sudoers配置
echo "$SUDOERS_LINE" | sudo tee -a /etc/sudoers > /dev/null
echo "$SUDOERS_LINE2" | sudo tee -a /etc/sudoers > /dev/null
echo "$SUDOERS_LINE3" | sudo tee -a /etc/sudoers > /dev/null
echo "$SUDOERS_LINE4" | sudo tee -a /etc/sudoers > /dev/null

echo "📊 设置日志轮转..."
sudo mkdir -p /var/log/vr-arm-control
sudo chown aidlux:aidlux /var/log/vr-arm-control

# 创建logrotate配置
sudo tee /etc/logrotate.d/vr-arm-control > /dev/null <<EOF2
/var/log/vr-arm-control/*.log {
    daily
    missingok
    rotate 7
    compress
    delaycompress
    notifempty
    create 644 aidlux aidlux
    postrotate
        systemctl reload vr-arm-control
    endscript
}
EOF2

echo ""
echo "🎉 一键安装完成！"
echo ""
echo "�� 服务管理命令:"
echo "  启动API服务: sudo systemctl start remote-control-api"
echo "  停止API服务: sudo systemctl stop remote-control-api"
echo "  重启API服务: sudo systemctl restart remote-control-api"
echo "  查看API状态: sudo systemctl status remote-control-api"
echo "  查看API日志: sudo journalctl -u remote-control-api -f"
echo ""
echo "📋 VR遥操服务管理命令:"
echo "  启动VR服务: sudo systemctl start vr-arm-control"
echo "  停止VR服务: sudo systemctl stop vr-arm-control"
echo "  重启VR服务: sudo systemctl restart vr-arm-control"
echo "  查看VR状态: sudo systemctl status vr-arm-control"
echo "  查看VR日志: sudo journalctl -u vr-arm-control -f"
echo ""
echo "🔗 API接口地址:"
echo "  API地址: http://localhost:8083"
echo "  可用接口:"
echo "    GET  /status      - 获取VR服务状态"
echo "    POST /start       - 启动VR服务"
echo "    POST /stop        - 停止VR服务"
echo "    POST /restart     - 重启VR服务"
echo "    POST /kill        - 强制杀死进程"
echo "    GET  /config/ip   - 获取VR服务器IP"
echo "    POST /config/ip   - 设置VR服务器IP"
echo "    GET  /logs        - 获取日志"
echo "    GET  /health      - 健康检查"
echo ""
echo "💡 使用提示:"
echo "1. API服务会自动启动，您可以通过API远程控制VR服务"
echo "2. VR服务不会开机自启，需要通过API手动启动"
echo "3. 首次运行前，请确保VR服务端IP地址已正确配置"
echo "4. 测试API: curl http://localhost:8083/health"
echo ""
echo "⚠️  注意: 系统重启后，API服务会自动启动，VR服务需要手动通过API启动"
