import { NextRequest, NextResponse } from 'next/server';
import { exec } from 'child_process';

// 处理 POST 请求，停止机器人所有动作并恢复到初始状态
export async function POST(request: NextRequest) {
  try {
    // 执行停止服务调用命令
    // 这里我们可以通过发布一个速度为 0 的 Twist 消息来停止机器人
    const command = `cd /home/aidlux/Haier_robot_ws && source install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`;
    
    // 使用 exec 执行命令
    const result = await new Promise<string>((resolve, reject) => {
      exec(command, (error, stdout, stderr) => {
        if (error) {
          console.error('执行停止命令失败:', error);
          console.error('错误输出:', stderr);
          reject(error);
        } else {
          console.log('停止命令执行成功:', stdout);
          resolve(stdout);
        }
      });
    });
    
    // 返回成功响应
    return NextResponse.json({
      success: true,
      message: '机器人已停止所有动作并恢复到初始状态',
      result: result
    });
  } catch (error) {
    console.error('处理停止请求失败:', error);
    return NextResponse.json(
      { error: '停止机器人动作失败，请检查后端服务是否运行' },
      { status: 500 }
    );
  }
}
