# VR遥操控制系统

这是一个基于systemd服务的VR遥操控制系统，包含API控制服务和VR遥操服务。

## 🚀 快速开始

### 1. 一键安装
```bash
cd scripts
./install_service.sh
```

此脚本将自动：
- ✅ 安装API控制服务和VR遥操服务
- ✅ 配置sudo免密权限
- ✅ 设置API服务开机自启
- ✅ 配置日志轮转

### 2. 启动服务
系统重启后，API服务会自动启动，VR服务需要通过API手动启动。或者手动启动API服务：
```bash
sudo systemctl start remote-control-api
```

## 📋 服务管理

### API服务 (remote-control-api)
```bash
# 启动
sudo systemctl start remote-control-api

# 停止
sudo systemctl stop remote-control-api

# 重启
sudo systemctl restart remote-control-api

# 查看状态
sudo systemctl status remote-control-api

# 查看日志
sudo journalctl -u remote-control-api -f
```

### VR遥操服务 (vr-arm-control)
```bash
# 启动
sudo systemctl start vr-arm-control

# 停止
sudo systemctl stop vr-arm-control

# 重启
sudo systemctl restart vr-arm-control

# 查看状态
sudo systemctl status vr-arm-control

# 查看日志
sudo journalctl -u vr-arm-control -f
```

## 🔗 API接口

API服务启动后，通过以下接口控制VR服务：

**基础地址**: `http://localhost:8083`

| 接口 | 方法 | 说明 |
|------|------|------|
| `/status` | GET | 获取VR服务状态 |
| `/start` | POST | 启动VR服务 |
| `/stop` | POST | 停止VR服务 |
| `/restart` | POST | 重启VR服务 |
| `/kill` | POST | 强制杀死进程 |
| `/config/ip` | GET | 获取VR服务器IP |
| `/config/ip` | POST | 设置VR服务器IP |
| `/logs` | GET | 获取日志 |
| `/health` | GET | 健康检查 |

### 使用示例
```bash
# 检查API健康状态
curl http://localhost:8083/health

# 获取VR服务状态
curl http://localhost:8083/status

# 启动VR服务
curl -X POST http://localhost:8083/start

# 停止VR服务
curl -X POST http://localhost:8083/stop

# 重启VR服务
curl -X POST http://localhost:8083/restart

# 强制杀死VR进程
curl -X POST http://localhost:8083/kill

# 获取VR服务器IP配置
curl http://localhost:8083/config/ip

# 设置VR服务器IP
curl -X POST http://localhost:8083/config/ip \
     -H "Content-Type: application/json" \
     -d '{"ip":"192.168.0.104"}'

# 获取VR服务日志
curl http://localhost:8083/logs
```

## ⚙️ 配置说明

### VR服务器IP配置
首次使用前，请确保VR服务器IP正确配置：
```bash
# 查看当前配置
curl http://localhost:8083/config/ip

# 修改IP地址
curl -X POST http://localhost:8083/config/ip \
     -H "Content-Type: application/json" \
     -d '{"ip":"YOUR_VR_SERVER_IP"}'
```

### 服务文件位置
- API服务: `/etc/systemd/system/remote-control-api.service`
- VR服务: `/etc/systemd/system/vr-arm-control.service`

## 🛠️ 故障排除

### API服务无法启动
```bash
# 检查状态
sudo systemctl status remote-control-api

# 查看详细日志
sudo journalctl -u remote-control-api -n 50
```

### VR服务无法启动
```bash
# 检查API权限
curl http://localhost:8083/health

# 检查VR服务状态
sudo systemctl status vr-arm-control

# 查看详细日志
sudo journalctl -u vr-arm-control -n 50
```

### 缺少依赖
确保ROS2环境正确配置，rclpy模块可用。

## 📊 系统架构

```
┌─────────────────┐    HTTP API    ┌─────────────────┐
│   客户端应用    │◄─────────────►│   API服务        │
│                 │                │ (remote-control)│
└─────────────────┘                └─────────────────┘
                                         │
                                         │ systemctl
                                         ▼
                                ┌─────────────────┐
                                │   VR遥操服务    │
                                │ (vr-arm-control)│
                                └─────────────────┘
```

## 🔒 安全注意事项

- API服务监听所有地址，生产环境建议配置防火墙
- sudo权限仅限于systemctl命令控制
- 日志文件可能包含敏感信息，妥善保管

## 📞 技术支持

如遇问题，请检查：
1. 服务状态: `sudo systemctl status <service-name>`
2. 系统日志: `sudo journalctl -u <service-name> -f`
3. API响应: `curl http://localhost:8083/health`