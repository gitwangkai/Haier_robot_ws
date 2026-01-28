# Next.js 页面集成控制接口说明

本文档说明如何在 Next.js 的 `page.tsx` 中集成四个核心控制接口。

## 修改概述

在 `/home/aidlux/Haier_robot_ws/src/robot_control_html/src/app/page.tsx` 中添加了以下功能：

1. **Socket.IO 连接管理**
2. **四个核心控制按钮**
3. **动作文件选择和播放控制**
4. **机械臂状态实时显示**
5. **播放进度显示**

---

## 新增功能

### 1. Socket.IO 连接管理

```typescript
import { io, Socket } from 'socket.io-client';

const [socket, setSocket] = useState<Socket | null>(null);
const [isConnected, setIsConnected] = useState(false);

useEffect(() => {
  const socketInstance = io('http://localhost:8082', {
    transports: ['websocket', 'polling'],
  });

  socketInstance.on('connect', () => {
    console.log('Socket.IO 已连接');
    setIsConnected(true);
    socketInstance.emit('get_action_files');
  });

  socketInstance.on('disconnect', () => {
    console.log('Socket.IO 已断开');
    setIsConnected(false);
  });

  setSocket(socketInstance);

  return () => {
    socketInstance.disconnect();
  };
}, []);
```

**连接地址**: `http://localhost:8082`  
**传输方式**: WebSocket 和 Polling 双模式

---

### 2. 四个核心控制按钮

#### 2.1 开始播放按钮

```typescript
const handleStartPlayback = () => {
  if (!selectedPlaybackFile) {
    alert('请先选择动作文件');
    return;
  }
  if (!socket) {
    alert('Socket.IO 未连接');
    return;
  }
  
  socket.emit('start_playback', {
    filename: selectedPlaybackFile,
    speed: playbackSpeed,
  });
};
```

**触发事件**: `start_playback`  
**参数**:
- `filename`: 动作文件名
- `speed`: 播放速度（0.1-3.0）

#### 2.2 停止播放按钮

```typescript
const handleStopPlayback = () => {
  if (!socket) {
    alert('Socket.IO 未连接');
    return;
  }
  
  socket.emit('stop_playback');
};
```

**触发事件**: `stop_playback`  
**参数**: 无

#### 2.3 重置位姿按钮

```typescript
const handleResetPose = () => {
  if (!socket) {
    alert('Socket.IO 未连接');
    return;
  }
  
  if (confirm('确定要重置双臂位姿到初始位置吗？')) {
    socket.emit('reset_pose');
  }
};
```

**触发事件**: `reset_pose`  
**参数**: 无  
**确认提示**: 显示确认对话框

#### 2.4 刷新状态按钮

```typescript
const handleRefreshStatus = () => {
  if (!socket) {
    alert('Socket.IO 未连接');
    return;
  }
  
  socket.emit('get_status');
};
```

**触发事件**: `get_status`  
**参数**: 无

---

### 3. Socket.IO 事件监听

#### 3.1 动作文件列表

```typescript
socketInstance.on('action_files_list', (data: { files: string[] }) => {
  setPlaybackFiles(data.files);
  if (data.files.length > 0 && !selectedPlaybackFile) {
    setSelectedPlaybackFile(data.files[0]);
  }
});
```

**事件**: `action_files_list`  
**用途**: 获取可用的动作文件列表

#### 3.2 播放开始

```typescript
socketInstance.on('playback_started', (data: { filename: string; speed: number }) => {
  console.log('播放开始:', data.filename);
  setIsPlayingBack(true);
  setPlaybackProgress({ current: 0, total: 0, progress: 0 });
  setStatusMessage(`开始播放: ${data.filename}`);
});
```

**事件**: `playback_started`  
**用途**: 播放开始时更新UI状态

#### 3.3 播放进度

```typescript
socketInstance.on('playback_progress', (data: { current: number; total: number; progress: number }) => {
  setPlaybackProgress({
    current: data.current,
    total: data.total,
    progress: data.progress || (data.current / data.total) * 100
  });
});
```

**事件**: `playback_progress`  
**用途**: 实时更新播放进度条

#### 3.4 播放停止

```typescript
socketInstance.on('playback_stopped', () => {
  console.log('播放已停止');
  setIsPlayingBack(false);
  setPlaybackProgress({ current: 0, total: 0, progress: 0 });
  setStatusMessage('播放已停止');
});
```

**事件**: `playback_stopped`  
**用途**: 播放停止时重置UI状态

#### 3.5 播放失败

```typescript
socketInstance.on('playback_failed', (data: { error: string }) => {
  console.error('播放失败:', data.error);
  setIsPlayingBack(false);
  setStatusMessage(`播放失败: ${data.error}`);
  alert('播放失败: ' + data.error);
});
```

**事件**: `playback_failed`  
**用途**: 显示播放失败错误信息

#### 3.6 位姿重置完成

```typescript
socketInstance.on('pose_reset_complete', (data: { success: boolean }) => {
  console.log('位姿重置完成');
  setStatusMessage('位姿已重置到初始位置');
});
```

**事件**: `pose_reset_complete`  
**用途**: 位姿重置完成时更新状态

#### 3.7 机械臂状态

```typescript
socketInstance.on('arm_status', (data: { right: any; left: any }) => {
  setArmStatus(data);
  setStatusMessage('状态已刷新');
});
```

**事件**: `arm_status`  
**用途**: 更新机械臂状态显示

#### 3.8 状态更新

```typescript
socketInstance.on('status_update', (data: { msg: string; level: string }) => {
  setStatusMessage(data.msg);
});
```

**事件**: `status_update`  
**用途**: 显示系统状态消息

---

### 4. UI 组件

#### 4.1 连接状态显示

```tsx
<div className="mb-6 rounded-lg border-2 bg-white dark:bg-slate-900 p-4 shadow-sm">
  <div className="flex items-center justify-between">
    <div className="flex items-center gap-3">
      <div className={`h-3 w-3 rounded-full animate-pulse ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
      <span className="text-lg font-semibold">
        Socket.IO 连接状态：
        <span className={isConnected ? 'text-green-600 dark:text-green-400' : 'text-red-600 dark:text-red-400'}>
          {isConnected ? '已连接' : '未连接'}
        </span>
      </span>
    </div>
    {statusMessage && (
      <div className="flex items-center gap-2 text-sm text-muted-foreground">
        <Activity className="h-4 w-4" />
        <span>{statusMessage}</span>
      </div>
    )}
  </div>
</div>
```

**显示内容**:
- Socket.IO 连接状态（绿色=已连接，红色=未连接）
- 当前状态消息

#### 4.2 动作播放控制卡片

```tsx
<Card className="border-2 shadow-sm">
  <CardHeader>
    <CardTitle className="flex items-center gap-2">
      <FileVideo className="h-5 w-5 text-blue-600" />
      动作播放控制
    </CardTitle>
    <CardDescription>控制动作文件的播放和机械臂状态</CardDescription>
  </CardHeader>
  <CardContent>
    <div className="space-y-4">
      <div className="grid gap-4 sm:grid-cols-2">
        <div className="space-y-2">
          <Label>选择动作文件</Label>
          <Select
            value={selectedPlaybackFile}
            onValueChange={setSelectedPlaybackFile}
            disabled={!isConnected || isPlayingBack}
          >
            <SelectTrigger>
              <SelectValue placeholder="请选择文件" />
            </SelectTrigger>
            <SelectContent>
              {playbackFiles.map((file) => (
                <SelectItem key={file} value={file}>
                  {file}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
        
        <div className="space-y-2">
          <Label>播放速度: {playbackSpeed}x</Label>
          <Slider
            value={[playbackSpeed]}
            onValueChange={(value) => setPlaybackSpeed(value[0])}
            min={0.1}
            max={3.0}
            step={0.1}
            disabled={!isConnected || isPlayingBack}
            className="cursor-pointer"
          />
        </div>
      </div>

      {isPlayingBack && (
        <div className="space-y-2">
          <div className="flex items-center justify-between text-sm">
            <span>播放进度</span>
            <span className="font-medium">
              {playbackProgress.current} / {playbackProgress.total}
            </span>
          </div>
          <div className="h-2 w-full overflow-hidden rounded-full bg-slate-200 dark:bg-slate-800">
            <div
              className="h-full bg-blue-600 transition-all duration-300"
              style={{ width: `${playbackProgress.progress}%` }}
            />
          </div>
        </div>
      )}

      <div className="grid gap-3 sm:grid-cols-4">
        <Button
          onClick={handleStartPlayback}
          disabled={!isConnected || !selectedPlaybackFile || isPlayingBack}
          className="w-full"
        >
          <Play className="mr-2 h-4 w-4" />
          开始播放
        </Button>
        
        <Button
          onClick={handleStopPlayback}
          disabled={!isConnected || !isPlayingBack}
          variant="outline"
          className="w-full"
        >
          <Pause className="mr-2 h-4 w-4" />
          停止播放
        </Button>
        
        <Button
          onClick={handleResetPose}
          disabled={!isConnected || isPlayingBack}
          variant="outline"
          className="w-full"
        >
          <RotateCcw className="mr-2 h-4 w-4" />
          重置位姿
        </Button>
        
        <Button
          onClick={handleRefreshStatus}
          disabled={!isConnected}
          variant="outline"
          className="w-full"
        >
          <RefreshCw className="mr-2 h-4 w-4" />
          刷新状态
        </Button>
      </div>
    </div>
  </CardContent>
</Card>
```

**包含功能**:
- 动作文件选择下拉框
- 播放速度滑块（0.1-3.0x）
- 播放进度条
- 四个控制按钮

#### 4.3 机械臂状态卡片

```tsx
<Card className="border-2 shadow-sm">
  <CardHeader>
    <CardTitle className="flex items-center gap-2">
      <Activity className="h-5 w-5 text-purple-600" />
      机械臂状态
    </CardTitle>
    <CardDescription>实时显示机械臂的运行状态</CardDescription>
  </CardHeader>
  <CardContent>
    <div className="grid gap-4 sm:grid-cols-2">
      <div className="space-y-2">
        <h3 className="font-semibold">右臂状态</h3>
        <div className="space-y-1 text-sm">
          {Object.keys(armStatus.right).length > 0 ? (
            Object.entries(armStatus.right).map(([servoId, info]: [string, any]) => (
              <div key={servoId} className="flex justify-between rounded bg-slate-100 dark:bg-slate-800 p-2">
                <span>舵机 {servoId}</span>
                <span className={info.pos !== null ? 'text-green-600' : 'text-red-600'}>
                  {info.pos !== null ? `位置: ${info.pos}` : '离线'}
                </span>
              </div>
            ))
          ) : (
            <p className="text-muted-foreground">暂无数据</p>
          )}
        </div>
      </div>
      
      <div className="space-y-2">
        <h3 className="font-semibold">左臂状态</h3>
        <div className="space-y-1 text-sm">
          {Object.keys(armStatus.left).length > 0 ? (
            Object.entries(armStatus.left).map(([servoId, info]: [string, any]) => (
              <div key={servoId} className="flex justify-between rounded bg-slate-100 dark:bg-slate-800 p-2">
                <span>舵机 {servoId}</span>
                <span className={info.pos !== null ? 'text-green-600' : 'text-red-600'}>
                  {info.pos !== null ? `位置: ${info.pos}` : '离线'}
                </span>
              </div>
            ))
          ) : (
            <p className="text-muted-foreground">暂无数据</p>
          )}
        </div>
      </div>
    </div>
  </CardContent>
</Card>
```

**显示内容**:
- 右臂各舵机状态
- 左臂各舵机状态
- 在线/离线状态指示

---

## 使用说明

### 前置条件

1. **安装依赖**
   ```bash
   npm install socket.io-client
   # 或
   pnpm add socket.io-client
   ```

2. **启动后端服务**
   ```bash
   cd /home/aidlux/Haier_robot_ws/src/pico_control_V0.5
   python3 web_control.py
   ```

3. **启动前端服务**
   ```bash
   cd /home/aidlux/Haier_robot_ws/src/robot_control_html
   npm run dev
   ```

### 操作步骤

1. **打开页面**
   - 访问 `http://localhost:3000`
   - 等待 Socket.IO 连接成功（绿色指示灯）

2. **开始播放**
   - 从下拉框选择动作文件
   - 调整播放速度（可选）
   - 点击"开始播放"按钮
   - 观察播放进度条

3. **停止播放**
   - 点击"停止播放"按钮
   - 播放将立即停止

4. **重置位姿**
   - 点击"重置位姿"按钮
   - 确认对话框
   - 双臂将移动到初始位置

5. **刷新状态**
   - 点击"刷新状态"按钮
   - 查看最新的机械臂状态

---

## 状态说明

### 连接状态
- **绿色**: Socket.IO 已连接到后端
- **红色**: Socket.IO 未连接

### 按钮状态
- **启用**: 可以正常点击
- **禁用**: 当前条件不允许操作（如未连接、正在播放等）

### 播放状态
- **播放中**: 显示进度条，"开始播放"按钮禁用
- **未播放**: 进度条隐藏，"停止播放"按钮禁用

---

## 注意事项

1. **Socket.IO 地址**
   - 默认连接到 `http://localhost:8082`
   - 如需修改，更新 `io()` 函数的第一个参数

2. **CORS 配置**
   - 确保后端 `web_control.py` 已配置 CORS
   - 允许前端域名访问

3. **错误处理**
   - 所有按钮都有错误检查
   - 未连接时会显示提示

4. **性能优化**
   - 使用 React 的 `useEffect` 管理生命周期
   - 组件卸载时自动断开连接

---

## 故障排除

### 问题1: Socket.IO 连接失败

**可能原因**:
- 后端服务未启动
- 端口 8082 被占用
- 防火墙阻止连接

**解决方案**:
```bash
# 检查后端服务
ps aux | grep web_control.py

# 检查端口
netstat -tlnp | grep 8082

# 重启后端服务
python3 /home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py
```

### 问题2: 动作文件列表为空

**可能原因**:
- `arm_data` 目录不存在
- 目录中没有 JSON 文件

**解决方案**:
```bash
# 创建目录
mkdir -p /home/aidlux/Haier_robot_ws/src/pico_control_V0.5/arm_data

# 检查文件
ls -la /home/aidlux/Haier_robot_ws/src/pico_control_V0.5/arm_data
```

### 问题3: 播放失败

**可能原因**:
- 文件不存在
- 文件格式错误
- 舵机未连接

**解决方案**:
- 检查文件是否存在
- 验证 JSON 格式
- 检查串口连接

---

## 扩展建议

1. **添加更多控制功能**
   - 单舵机控制
   - 力矩控制
   - 动作录制

2. **改进UI**
   - 添加动画效果
   - 优化响应式布局
   - 添加主题切换

3. **增强功能**
   - 动作文件预览
   - 动作编辑器
   - 多动作队列

---

**文档版本**: 1.0  
**最后更新**: 2026-01-27  
**维护者**: Pico Control System Team
