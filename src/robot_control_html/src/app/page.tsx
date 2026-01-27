'use client';

import { useState, useRef, useEffect } from 'react';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Label } from '@/components/ui/label';
import { Checkbox } from '@/components/ui/checkbox';
import { Slider } from '@/components/ui/slider';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Play, Pause, Rotate3D, Hand, Music, Clock, AlertCircle, RotateCcw } from 'lucide-react';

export default function RobotPerformancePage() {
  const [isPerforming, setIsPerforming] = useState(false);
  const [selectedActions, setSelectedActions] = useState<string[]>([]); // 用于旋转和挥手
  const [selectedReset, setSelectedReset] = useState<boolean>(false); // 用于恢复初始状态
  const [selectedAudio, setSelectedAudio] = useState<string>('audio1'); // 默认选择音频1
  const [performanceTime, setPerformanceTime] = useState<number>(30);
  const [playingAudio, setPlayingAudio] = useState<string>('');

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

  const handleActionToggle = (actionId: string, isExclusive: boolean) => {
    if (isExclusive) {
      // 互斥选项：恢复初始状态
      if (selectedReset) {
        setSelectedReset(false);
      } else {
        setSelectedActions([]); // 清空其他选项
        setSelectedReset(true); // 选中恢复初始状态
      }
    } else {
      // 多选选项：旋转和挥手
      setSelectedReset(false); // 清除恢复初始状态
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

  useEffect(() => {
    return () => {
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current = new Audio('');
      }
    };
  }, []);

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
      
      // 如果选择了旋转动作，调用旋转服务
      if (selectedActions.includes('rotate')) {
        // 将分钟转换为秒
        const rotationTimeInSeconds = performanceTime * 60;
        
        // 调用旋转服务
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
      // 停止表演，调用停止服务
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
              <div className={`h-3 w-3 rounded-full animate-pulse ${isPerforming ? 'bg-green-500' : 'bg-blue-500'}`} />
              <span className="text-lg font-semibold">
                当前状态：
                <span className={isPerforming ? 'text-green-600 dark:text-green-400' : 'text-blue-600 dark:text-blue-400'}>
                  {isPerforming ? '表演中' : '空闲中'}
                </span>
              </span>
            </div>
            {isPerforming && (
              <div className="flex items-center gap-2 text-sm text-muted-foreground">
                <Clock className="h-4 w-4" />
                <span>预计时长：{formatTime(performanceTime)}</span>
              </div>
            )}
          </div>
        </div>

        <div className="grid gap-6">
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
                        // 单选按钮样式（恢复初始状态）
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
                        // 复选框样式（旋转和挥手）
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
