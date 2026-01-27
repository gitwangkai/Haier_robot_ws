'use client';

import { useState, useRef, useEffect } from 'react';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Label } from '@/components/ui/label';
import { Checkbox } from '@/components/ui/checkbox';
import { Slider } from '@/components/ui/slider';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Play, Pause, Rotate3D, Hand, Music, Clock, AlertCircle, RotateCcw, RefreshCw, FileVideo, Activity } from 'lucide-react';
import { io, Socket } from 'socket.io-client';

export default function RobotPerformancePage() {
  const [isPerforming, setIsPerforming] = useState(false);
  const [selectedActions, setSelectedActions] = useState<string[]>([]);
  const [selectedReset, setSelectedReset] = useState<boolean>(false);
  const [selectedAudio, setSelectedAudio] = useState<string>('audio1');
  const [performanceTime, setPerformanceTime] = useState<number>(30);
  const [playingAudio, setPlayingAudio] = useState<string>('');

  const [socket, setSocket] = useState<Socket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [selectedPlaybackFile, setSelectedPlaybackFile] = useState<string>('');
  const [playbackSpeed, setPlaybackSpeed] = useState<number>(1.0);
  const [playbackFiles, setPlaybackFiles] = useState<string[]>([]);
  const [isPlayingBack, setIsPlayingBack] = useState(false);
  const [playbackProgress, setPlaybackProgress] = useState({ current: 0, total: 0, progress: 0 });
  const [armStatus, setArmStatus] = useState<{ right: any; left: any }>({ right: {}, left: {} });
  const [statusMessage, setStatusMessage] = useState<string>('');

  const audioRef = useRef<HTMLAudioElement | null>(null);

  const actionOptions = [
    { id: 'rotate', label: '旋转', icon: Rotate3D, isExclusive: false },
    { id: 'wave', label: '挥手', icon: Hand, isExclusive: false },
    { id: 'reset', label: '恢复初始状态', icon: RotateCcw, isExclusive: true },
  ];

  const audioOptions = [
    { id: 'audio1', label: '音频1', url: 'https://www.soundhelix.com/examples/mp3/SoundHelix-Song-1.mp3' },
    { id: 'audio2', label: '音频2', url: 'https://www.soundhelix.com/examples/mp3/SoundHelix-Song-2.mp3' },
    { id: 'audio3', label: '音频3', url: 'https://www.soundhelix.com/examples/mp3/SoundHelix-Song-3.mp3' },
  ];

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

    socketInstance.on('action_files_list', (data: { files: string[] }) => {
      setPlaybackFiles(data.files);
      if (data.files.length > 0 && !selectedPlaybackFile) {
        setSelectedPlaybackFile(data.files[0]);
      }
    });

    socketInstance.on('playback_started', (data: { filename: string; speed: number }) => {
      console.log('播放开始:', data.filename);
      setIsPlayingBack(true);
      setPlaybackProgress({ current: 0, total: 0, progress: 0 });
      setStatusMessage(`开始播放: ${data.filename}`);
    });

    socketInstance.on('playback_progress', (data: { current: number; total: number; progress: number }) => {
      setPlaybackProgress({
        current: data.current,
        total: data.total,
        progress: data.progress || (data.current / data.total) * 100
      });
    });

    socketInstance.on('playback_stopped', () => {
      console.log('播放已停止');
      setIsPlayingBack(false);
      setPlaybackProgress({ current: 0, total: 0, progress: 0 });
      setStatusMessage('播放已停止');
    });

    socketInstance.on('playback_failed', (data: { error: string }) => {
      console.error('播放失败:', data.error);
      setIsPlayingBack(false);
      setStatusMessage(`播放失败: ${data.error}`);
      alert('播放失败: ' + data.error);
    });

    socketInstance.on('pose_reset_complete', (data: { success: boolean }) => {
      console.log('位姿重置完成');
      setStatusMessage('位姿已重置到初始位置');
    });

    socketInstance.on('arm_status', (data: { right: any; left: any }) => {
      setArmStatus(data);
      setStatusMessage('状态已刷新');
    });

    socketInstance.on('status_update', (data: { msg: string; level: string }) => {
      setStatusMessage(data.msg);
    });

    setSocket(socketInstance);

    return () => {
      socketInstance.disconnect();
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current = new Audio('');
      }
    };
  }, []);

  const handleActionToggle = (actionId: string, isExclusive: boolean) => {
    if (isExclusive) {
      if (selectedReset) {
        setSelectedReset(false);
      } else {
        setSelectedActions([]);
        setSelectedReset(true);
      }
    } else {
      setSelectedReset(false);
      setSelectedActions(prev =>
        prev.includes(actionId)
          ? prev.filter(id => id !== actionId)
          : [...prev, actionId]
      );
    }
  };

  const handleAudioPlay = (audioId: string, audioUrl: string) => {
    if (playingAudio === audioId) {
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current.currentTime = 0;
      }
      setPlayingAudio('');
    } else {
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current.currentTime = 0;
      }
      audioRef.current = new Audio(audioUrl);
      audioRef.current.onended = () => setPlayingAudio('');
      audioRef.current.play();
      setPlayingAudio(audioId);
    }
  };

  const handlePerformanceToggle = () => {
    if (!isPerforming) {
      if (selectedActions.length === 0 && !selectedReset) {
        alert('请至少选择一个表演动作');
        return;
      }
      if (!selectedAudio) {
        alert('请选择一个背景音频');
        return;
      }
      
      if (selectedActions.includes('rotate')) {
        const rotationTimeInSeconds = performanceTime * 60;
        
        fetch('/api/rotate', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ duration: rotationTimeInSeconds }),
        })
        .then(response => response.json())
        .then(data => {
          console.log('旋转服务调用结果:', data);
        })
        .catch(error => {
          console.error('调用旋转服务失败:', error);
          alert('调用旋转服务失败，请检查后端服务是否运行');
        });
      }
    } else {
      fetch('/api/stop', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      })
      .then(response => response.json())
      .then(data => {
        console.log('停止服务调用结果:', data);
      })
      .catch(error => {
        console.error('调用停止服务失败:', error);
        alert('调用停止服务失败，请检查后端服务是否运行');
      });
    }
    setIsPerforming(!isPerforming);
  };

  const formatTime = (minutes: number): string => {
    if (minutes >= 60) {
      const hours = Math.floor(minutes / 60);
      const mins = minutes % 60;
      return `${hours}小时${mins > 0 ? ` ${mins}分钟` : ''}`;
    }
    return `${minutes}分钟`;
  };

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

  const handleStopPlayback = () => {
    if (!socket) {
      alert('Socket.IO 未连接');
      return;
    }
    
    socket.emit('stop_playback');
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

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 via-blue-50 to-indigo-50 dark:from-slate-950 dark:via-blue-950 dark:to-indigo-950 p-4 md:p-8">
      <div className="mx-auto max-w-4xl">
        <div className="mb-8 text-center">
          <h1 className="text-3xl md:text-4xl font-bold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">
            机器人表演设置
          </h1>
          <p className="mt-2 text-muted-foreground">配置机器人的表演参数和动作</p>
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

        <div className="grid gap-6">
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

          <Card className="border-2 shadow-sm">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Rotate3D className="h-5 w-5 text-blue-600" />
                表演动作选择
              </CardTitle>
              <CardDescription>选择机器人将要执行的动作（可多选）</CardDescription>
            </CardHeader>
            <CardContent>
              <div className="grid gap-4 sm:grid-cols-2">
                {actionOptions.map((action) => {
                  const Icon = action.icon;
                  const isSelected = action.isExclusive ? selectedReset : selectedActions.includes(action.id);
                  return (
                    <div
                      key={action.id}
                      className={`flex items-center gap-3 rounded-lg border-2 p-4 transition-all cursor-pointer ${
                        isSelected
                          ? 'border-blue-500 bg-blue-50 dark:bg-blue-950/20'
                          : 'border-gray-200 dark:border-slate-800'
                      } ${isPerforming ? 'opacity-50 cursor-not-allowed' : 'hover:border-blue-300 dark:hover:border-slate-700'}`}
                      onClick={() => !isPerforming && handleActionToggle(action.id, action.isExclusive)}
                    >
                      {action.isExclusive ? (
                        <div
                          className={`flex h-5 w-5 items-center justify-center rounded-full border-2 transition-all ${
                            isSelected
                              ? 'border-blue-500 bg-blue-500'
                              : 'border-gray-300 dark:border-slate-600'
                          }`}
                        >
                          {isSelected && (
                            <div className="h-2.5 w-2.5 rounded-full bg-white" />
                          )}
                        </div>
                      ) : (
                        <div
                          className={`flex h-5 w-5 items-center justify-center rounded border-2 transition-all ${
                            isSelected
                              ? 'border-blue-500 bg-blue-500'
                              : 'border-gray-300 dark:border-slate-600'
                          }`}
                        >
                          {isSelected && (
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
                      )}
                      <div className="flex-1">
                        <div className="flex items-center gap-2 text-base font-medium">
                          <Icon className="h-4 w-4" />
                          {action.label}
                        </div>
                      </div>
                    </div>
                  );
                })}
              </div>
              <div className="mt-4 flex items-center justify-between">
                {selectedActions.length === 0 && !selectedReset && !isPerforming && (
                  <div className="flex items-center gap-2 text-sm text-amber-600 dark:text-amber-400">
                    <AlertCircle className="h-4 w-4" />
                    <span>请至少选择一个动作</span>
                  </div>
                )}
              </div>
            </CardContent>
          </Card>

          <Card className="border-2 shadow-sm">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Music className="h-5 w-5 text-purple-600" />
                音频选择
              </CardTitle>
              <CardDescription>选择背景音乐并试听</CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <Select
                  value={selectedAudio}
                  onValueChange={(value) => !isPerforming && setSelectedAudio(value)}
                  disabled={isPerforming}
                >
                  <SelectTrigger>
                    <SelectValue placeholder="请选择音频" />
                  </SelectTrigger>
                  <SelectContent>
                    {audioOptions.map((audio) => (
                      <SelectItem key={audio.id} value={audio.id}>
                        <div className="flex items-center gap-2">
                          <Music className="h-4 w-4" />
                          {audio.label}
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>

                {selectedAudio && (
                  <div className="flex items-center justify-between rounded-lg border-2 border-gray-200 dark:border-slate-800 p-4 bg-slate-50 dark:bg-slate-800/50">
                    <div className="flex items-center gap-3">
                      <span className="text-base font-medium">
                        当前选择：{audioOptions.find(a => a.id === selectedAudio)?.label}
                      </span>
                      {playingAudio === selectedAudio && (
                        <span className="text-xs text-purple-600 dark:text-purple-400">
                          播放中
                        </span>
                      )}
                    </div>
                    <Button
                      variant={playingAudio === selectedAudio ? 'secondary' : 'outline'}
                      size="sm"
                      onClick={() => {
                        const audio = audioOptions.find(a => a.id === selectedAudio);
                        if (audio) {
                          handleAudioPlay(audio.id, audio.url);
                        }
                      }}
                      disabled={isPerforming}
                      className="min-w-[80px]"
                    >
                      {playingAudio === selectedAudio ? (
                        <>
                          <Pause className="mr-2 h-4 w-4" />
                          停止
                        </>
                      ) : (
                        <>
                          <Play className="mr-2 h-4 w-4" />
                          试听
                        </>
                      )}
                    </Button>
                  </div>
                )}
              </div>
            </CardContent>
          </Card>

          <Card className="border-2 shadow-sm">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Clock className="h-5 w-5 text-green-600" />
                表演时间设置
              </CardTitle>
              <CardDescription>设置机器人的表演时长</CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div className="relative pt-6 pb-2">
                  <Slider
                    value={[performanceTime]}
                    onValueChange={(value) => !isPerforming && setPerformanceTime(value[0])}
                    min={0}
                    max={60}
                    step={1}
                    disabled={isPerforming}
                    className="cursor-pointer"
                  />
                  <div className="mt-3 flex justify-between text-xs text-muted-foreground">
                    <span>0分钟</span>
                    <span>30分钟</span>
                    <span>60分钟</span>
                  </div>
                </div>
                <div className="flex items-center justify-between rounded-lg bg-slate-100 dark:bg-slate-800 p-4">
                  <span className="text-sm font-medium">已选择时长：</span>
                  <span className="text-2xl font-bold text-green-600 dark:text-green-400">
                    {formatTime(performanceTime)}
                  </span>
                </div>
              </div>
            </CardContent>
          </Card>

          <div className="flex justify-center pt-4">
            <Button
              size="lg"
              onClick={handlePerformanceToggle}
              disabled={!isPerforming && (selectedActions.length === 0 && !selectedReset)}
              className={`h-14 px-12 text-lg font-semibold transition-all ${
                isPerforming
                  ? 'bg-amber-500 hover:bg-amber-600 shadow-lg shadow-amber-500/30'
                  : 'bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-700 hover:to-indigo-700 shadow-lg shadow-blue-500/30'
              }`}
            >
              {isPerforming ? (
                <>
                  <Pause className="mr-2 h-5 w-5" />
                  暂停表演
                </>
              ) : (
                <>
                  <Play className="mr-2 h-5 w-5" />
                  开始表演
                </>
              )}
            </Button>
          </div>
        </div>
      </div>
    </div>
  );
}
