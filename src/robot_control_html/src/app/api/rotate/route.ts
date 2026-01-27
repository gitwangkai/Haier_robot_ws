import { NextRequest, NextResponse } from 'next/server';
import { exec } from 'child_process';

// 处理 POST 请求，调用旋转服务
export async function POST(request: NextRequest) {
  try {
    // 解析请求体
    const body = await request.json();
    const { duration } = body;
    
    // 验证参数
    if (!duration || typeof duration !== 'number') {
      return NextResponse.json(
        { error: 'Invalid duration parameter' },
        { status: 400 }
      );
    }
    
    // 执行旋转服务调用命令
    const command = `cd /home/aidlux/Haier_robot_ws && source install/setup.bash && ros2 service call /robot_mover/rotate robot_mover/srv/Rotate "{duration: ${duration}}"`;
    
    // 使用 exec 执行命令
    const result = await new Promise<string>((resolve, reject) => {
      exec(command, (error, stdout, stderr) => {
        if (error) {
          console.error('执行旋转服务命令失败:', error);
          console.error('错误输出:', stderr);
          reject(error);
        } else {
          console.log('旋转服务命令执行成功:', stdout);
          resolve(stdout);
        }
      });
    });
    
    // 返回成功响应
    return NextResponse.json({
      success: true,
      message: '旋转服务调用成功',
      result: result
    });
  } catch (error) {
    console.error('处理旋转请求失败:', error);
    return NextResponse.json(
      { error: '调用旋转服务失败，请检查后端服务是否运行' },
      { status: 500 }
    );
  }
}
