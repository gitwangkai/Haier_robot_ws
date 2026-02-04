'use client';

import { useState, useEffect } from 'react';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Label } from '@/components/ui/label';
import { Slider } from '@/components/ui/slider';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Play, Pause, RotateCcw, RefreshCw, FileVideo, Activity } from 'lucide-react';
import { io, Socket } from 'socket.io-client';
import { useRouter } from 'next/navigation';

export default function RecordPage() {
  const router = useRouter();
  const [socket, setSocket] = useState<Socket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isRecording, setIsRecording] = useState(false);
  const [recordingName, setRecordingName] = useState('');
  const [recordingProgress, setRecordingProgress] = useState({ time: '0.00s', frames: 0 });
  const [selectedArms, setSelectedArms] = useState<string[]>(['right', 'left']);
  const [isFiltering, setIsFiltering] = useState(false);
  const [selectedFilterFile, setSelectedFilterFile] = useState('');
  const [smoothingFactor, setSmoothingFactor] = useState(0.3);
  const [deadzoneThreshold, setDeadzoneThreshold] = useState(2);
  const [playbackFiles, setPlaybackFiles] = useState<string[]>([]);
  const [statusMessage, setStatusMessage] = useState<string>('');

  useEffect(() => {
    // Get the server's hostname from the current URL
    const serverHostname = window.location.hostname;
    const socketInstance = io(`http://${serverHostname}:8082`, {
      transports: ['websocket', 'polling'],
      reconnection: true,
      reconnectionAttempts: 5,
      reconnectionDelay: 1000,
      reconnectionDelayMax: 5000,
      timeout: 20000,
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

    socketInstance.on('action_files_list', (data: { files: string[] }) => {
      setPlaybackFiles(data.files);
      if (data.files.length > 0 && !selectedFilterFile) {
        setSelectedFilterFile(data.files[0]);
      }
    });

    socketInstance.on('recording_started', (data: { filename: string }) => {
      console.log('录制开始:', data.filename);
      setIsRecording(true);
      setRecordingProgress({ time: '0.00s', frames: 0 });
      setStatusMessage(`开始录制: ${data.filename}`);
    });

    socketInstance.on('recording_progress', (data: { time: string; frames: number }) => {
      setRecordingProgress({
        time: data.time,
        frames: data.frames
      });
    });

    socketInstance.on('recording_stopped', () => {
      console.log('录制已停止');
      setIsRecording(false);
      setRecordingProgress({ time: '0.00s', frames: 0 });
      setStatusMessage('录制已停止');
      // 刷新文件列表
      socketInstance.emit('get_action_files');
    });

    socketInstance.on('recording_failed', (data: { error: string }) => {
      console.error('录制失败:', data.error);
      setIsRecording(false);
      setStatusMessage(`录制失败: ${data.error}`);
      alert('录制失败: ' + data.error);
    });

    socketInstance.on('status_update', (data: { msg: string; level: string }) => {
      setStatusMessage(data.msg);
    });

    setSocket(socketInstance);

    return () => {
      socketInstance.disconnect();
    };
  }, []);

  const handleStartRecording = () => {
    if (!socket) {
      alert('Socket.IO 未连接');
      return;
    }
    if (!recordingName.trim()) {
      alert('请输入录制文件名');
      return;
    }
    if (selectedArms.length === 0) {
      alert('请至少选择一个手臂进行录制');
      return;
    }
    
    socket.emit('start_recording', {
      filename: recordingName.trim(),
      arms: selectedArms
    });
  };

  const handleStopRecording = () => {
    if (!socket) {
      alert('Socket.IO 未连接');
      return;
    }
    
    socket.emit('stop_recording');
  };

  const handleResetPose = () => {
    if (!socket) {
      alert('Socket.IO 未连接');
      return;
    }
    
    if (confirm('确定要重置双臂位姿到初始位置吗？')) {
      socket.emit('reset_pose');
    }
  };

  const handleRefreshStatus = () => {
    if (!socket) {
      alert('Socket.IO 未连接');
      return;
    }
    
    socket.emit('get_status');
  };

  const handleFilterFile = () => {
    if (!socket) {
      alert('Socket.IO 未连接');
      return;
    }
    if (!selectedFilterFile) {
      alert('请选择要滤波的文件');
      return;
    }
    
    setIsFiltering(true);
    socket.emit('filter_file', {
      filename: selectedFilterFile,
      smoothingFactor: smoothingFactor,
      deadzoneThreshold: deadzoneThreshold
    });
  };

  const handleArmToggle = (arm: string) => {
    setSelectedArms(prev =>
      prev.includes(arm)
        ? prev.filter(a => a !== arm)
        : [...prev, arm]
    );
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 via-blue-50 to-indigo-50 dark:from-slate-950 dark:via-blue-950 dark:to-indigo-950 p-4 md:p-8">
      <div className="mx-auto max-w-4xl">
        <div className="mb-8">
          <div className="flex items-center gap-4">
            <Button
              onClick={() => router.back()}
              variant="outline"
            >
              返回
            </Button>
            <h1 className="text-3xl md:text-4xl font-bold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">
              动作录制
            </h1>
          </div>
          <p className="mt-2 text-muted-foreground">录制和编辑机器人的动作序列</p>
        </div>

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

        <Card className="border-2 shadow-lg">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <FileVideo className="h-5 w-5 text-red-600" />
              动作录制
            </CardTitle>
            <CardDescription>录制机器人的动作序列</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-6">
              {/* 录制设置 */}
              <div className="space-y-4">
                <h3 className="text-lg font-semibold">录制设置</h3>
                
                <div className="grid gap-4 sm:grid-cols-2">
                  <div className="space-y-2">
                    <Label>录制文件名</Label>
                    <input
                      type="text"
                      value={recordingName}
                      onChange={(e) => setRecordingName(e.target.value)}
                      placeholder="请输入文件名"
                      disabled={isRecording}
                      className="w-full rounded-md border-2 border-gray-200 dark:border-slate-700 px-3 py-2 focus:border-blue-500 focus:outline-none"
                    />
                  </div>
                  
                  <div className="space-y-2">
                    <Label>选择手臂</Label>
                    <div className="flex gap-4">
                      <div className="flex items-center gap-2 cursor-pointer">
                        <div
                          className={`flex h-5 w-5 items-center justify-center rounded border-2 transition-all ${
                            selectedArms.includes('right')
                              ? 'border-blue-500 bg-blue-500'
                              : 'border-gray-300 dark:border-slate-600'
                          }`}
                          onClick={() => !isRecording && handleArmToggle('right')}
                        >
                          {selectedArms.includes('right') && (
                            <svg
                              className="h-3.5 w-3.5 text-white"
                              fill="currentColor"
                              viewBox="0 0 20 20"
                            >
                              <path
                                fillRule="evenodd"
                                d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
                                clipRule="evenodd"
                              />
                            </svg>
                          )}
                        </div>
                        <span>右臂</span>
                      </div>
                      <div className="flex items-center gap-2 cursor-pointer">
                        <div
                          className={`flex h-5 w-5 items-center justify-center rounded border-2 transition-all ${
                            selectedArms.includes('left')
                              ? 'border-blue-500 bg-blue-500'
                              : 'border-gray-300 dark:border-slate-600'
                          }`}
                          onClick={() => !isRecording && handleArmToggle('left')}
                        >
                          {selectedArms.includes('left') && (
                            <svg
                              className="h-3.5 w-3.5 text-white"
                              fill="currentColor"
                              viewBox="0 0 20 20"
                            >
                              <path
                                fillRule="evenodd"
                                d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
                                clipRule="evenodd"
                              />
                            </svg>
                          )}
                        </div>
                        <span>左臂</span>
                      </div>
                    </div>
                  </div>
                </div>

                {isRecording && (
                  <div className="space-y-2">
                    <div className="flex items-center justify-between text-sm">
                      <span>录制进度</span>
                      <span className="font-medium">
                        {recordingProgress.time} | {recordingProgress.frames} 帧
                      </span>
                    </div>
                    <div className="h-2 w-full overflow-hidden rounded-full bg-slate-200 dark:bg-slate-800">
                      <div
                        className="h-full bg-red-600 transition-all duration-300 animate-pulse"
                      />
                    </div>
                  </div>
                )}

                <div className="grid gap-3 sm:grid-cols-3">
                  <Button
                    onClick={handleStartRecording}
                    disabled={!isConnected || isRecording || selectedArms.length === 0}
                    className="w-full"
                  >
                    <Play className="mr-2 h-4 w-4" />
                    开始录制
                  </Button>
                  
                  <Button
                    onClick={handleStopRecording}
                    disabled={!isConnected || !isRecording}
                    variant="outline"
                    className="w-full"
                  >
                    <Pause className="mr-2 h-4 w-4" />
                    停止录制
                  </Button>
                  
                  <Button
                    onClick={handleResetPose}
                    disabled={!isConnected || isRecording}
                    variant="outline"
                    className="w-full"
                  >
                    <RotateCcw className="mr-2 h-4 w-4" />
                    重置位姿
                  </Button>
                </div>
              </div>

              {/* 动作平滑 */}
              <div className="space-y-4 pt-4 border-t-2 border-gray-100 dark:border-slate-800">
                <h3 className="text-lg font-semibold">动作平滑</h3>
                
                <div className="grid gap-4 sm:grid-cols-2">
                  <div className="space-y-2">
                    <Label>选择要滤波的文件</Label>
                    <Select
                      value={selectedFilterFile}
                      onValueChange={setSelectedFilterFile}
                      disabled={isFiltering}
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
                </div>

                <div className="grid gap-4 sm:grid-cols-2">
                  <div className="space-y-2">
                    <Label>平滑因子: {smoothingFactor}</Label>
                    <Slider
                      value={[smoothingFactor]}
                      onValueChange={(value) => setSmoothingFactor(value[0])}
                      min={0.1}
                      max={0.9}
                      step={0.1}
                      disabled={isFiltering}
                    />
                  </div>
                  
                  <div className="space-y-2">
                    <Label>死区阈值: {deadzoneThreshold}</Label>
                    <Slider
                      value={[deadzoneThreshold]}
                      onValueChange={(value) => setDeadzoneThreshold(value[0])}
                      min={0}
                      max={10}
                      step={1}
                      disabled={isFiltering}
                    />
                  </div>
                </div>

                <div className="grid gap-3 sm:grid-cols-2">
                  <Button
                    onClick={handleFilterFile}
                    disabled={!isConnected || isFiltering || !selectedFilterFile}
                    className="w-full"
                  >
                    <RefreshCw className="mr-2 h-4 w-4" />
                    {isFiltering ? '正在滤波...' : '开始滤波'}
                  </Button>
                  
                  <Button
                    onClick={() => socket?.emit('get_action_files')}
                    disabled={!isConnected}
                    variant="outline"
                    className="w-full"
                  >
                    <RefreshCw className="mr-2 h-4 w-4" />
                    刷新文件列表
                  </Button>
                </div>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
