# 🚗 Line_Follower ROS Package

 [中文](README.md)  | [English](README_en.md) 

![tip](https://badgen.net/badge/C++/g++/red?icon=github)![tip](https://badgen.net/badge/ROS/Melodic/blue?icon=github)![tip](https://badgen.net/badge/OpenCV/3.3.1/green?icon=github)

这个ROS包是由**长江大学极客班**的极客👨‍💻👩‍💻开源，是第19届科大讯飞智能车救援组巡线比赛定制的方案，也可以用于其他双线内巡线的参考的ROS包方案。

![demo](demo.gif)

## 🌟 一起开源，推动科技进步

在这个快速变化的时代，开源是推动技术进步和创新的关键力量。通过分享我们的知识和代码，我们不仅可以加速技术的发展，还能培养新一代的工程师和开发者。我们鼓励大家加入开源社区，分享你的创意和成果，让更多人受益。

无论你是一个初学者还是经验丰富的专家，都可以在开源项目中找到你的位置。每一行代码、每一个建议都能为项目注入新的活力。让我们携手合作，共同推动科技的进步，创造一个更加开放和互联的世界。



## 🌟 闪亮特性

- 🧠 智能图像处理：就像给你的小车装了一双火眼金睛
- 🎛️ 灵活PID控制：平滑巡线，宛如小车在跳华尔兹
- 🔧 超强可配置性：可以自由的调教你的小车



## 🛠️ 你需要准备的工具箱

在开始之前，请确保你的系统已经安装了以下环境和依赖：

- ROS（我们在Noetic上反复蹂躏测试过）
- OpenCV（没有它，你的小车就像失明了一样）
- 其他一些厉害的库：cv_bridge、image_transport、sensor_msgs、geometry_msgs
- Markdown环境（用于查看和编辑本README文件）

> 💡 提示：如果你还没有安装Markdown环境，可以使用VS Code或Typora等编辑器，它们对Markdown有很好的支持。



## 📁 包的内部构造

```
line_follower/
├── src/
│   └── line_follower_node.cpp  (核心代码，小车的大脑！)
├── config/
│   ├── image.cfg  (调教你的火眼金睛)
│   └── pid.cfg   (调教你的平衡感)
├── launch/
│   └── line_follower.launch  (一键启动，像魔法一样简单)
├── rviz/
│   └── img_follow_rviz_config.rviz  (RViz配置文件，可视化小车的路径跟踪)
├── include/
│   └── line_follower/  (头文件，用于代码的模块化和重用)
├── CMakeLists.txt  (CMake配置文件，定义了如何构建项目)
└── package.xml  (包配置文件，定义了包的元数据，如依赖关系、版本等)

```



## ⚙️ 如何调教你的小车

1. 找个安静的地方，克隆这个包：

   ```shell
   cd ~/catkin_ws/src
   git clone https://gitee.com/changjiang-university_2/Line_Follower.git
   ```

2. 给它点营养，编译一下：

   ```shell
   cd ~/catkin_ws
   catkin_make
   ```

3. 在Ubuntu中安装C++ OpenCV依赖：

   ```shell
   sudo apt update
   sudo apt install libopencv-dev
   ```

4. 告诉它该干活了：

   ```shell
   source ~/catkin_ws/devel/setup.bash
   ```

5. 打开小车的眼睛

   需要启动你小车的摄像头，并且订阅对应的话题，在`config/pid.cfg`里面修改对应的话题名称
   
   ```shell
   image_topic=/new_camera/image # 更改图像话题名称以匹配新设备
   ```
   
6. 启动动力能源

   需要启动你小车的底盘，让小车能够进行运动，比如科大讯飞的ucar-mini一代车启动实例如下

   ```shell
   roslaunch ucar_controller base_driver.launch
   ```

7. 唤醒你的小车：

   ```shell
   roslaunch line_follower line_follower.launch
   ```

## 🧠 核心代码解密

我们的核心代码藏在 `src/line_follower_node.cpp` 里面。来看看它有什么秘密：

1. `ImageProcessor` 类：这是小车的眼睛🧐
   
- 它能在杂乱的画面中精准找到那条线，就像在人海中一眼认出自己的对象✨
  
2. `LineFollower` 类：这是小车的大脑和肌肉💪
   
- 它决定小车该怎么走，左转还是右转，快跑还是慢走🚗
  
3. 参数读取：我们让小车变得很听话，你说什么它就怎么做👂
   
4. PID控制：这就像是给小车装了一个平衡器，让它走起路来虎虎生风🏋️‍♂️

5. 调试输出：让你随时了解小车在想什么，做什么💡

6. 图像话题：订阅`/img_follow`，就像打开小车眼睛的滤镜，一窥`ImageProcessor`如何把混乱变清晰，小车视角的独家揭秘，就在这里！👀

   

### 🎛️ PID调参详解

PID控制是这个包的核心功能之一。在 `config/pid.cfg` 文件中，你可以找到以下关键参数：

- `Kp`：比例系数，控制转向的快速响应
- `Ki`：积分系数，用于消除稳态误差
- `Kd`：微分系数，用于抑制振荡

调参小贴士：
- 先调 `Kp`：增大 `Kp` 直到系统开始震荡，然后减小到震荡消失
- 再调 `Kd`：增大 `Kd` 以减小过冲，但不要太大以免引入高频噪声
- 最后调 `Ki`：缓慢增加 `Ki` 以消除稳态误差，但要注意不要引起振荡

此外，我们的PID控制系统不仅仅是一个简单的PID控制器，它包含了多个高级特性，使得小车能够更好地适应各种赛道条件。以下是对这些特性的详细介绍：

以下是一个完整的PID配置实例：

```shell
# 巡线机器人PID配置文件

# 基本参数
max_linear_speed=0.25   # 最大线速度 (m/s)
max_angular_speed=1.0   # 最大角速度 (rad/s)
Kp=0.005                # 比例增益
Ki=0.000005             # 积分增益
Kd=0.009                # 微分增益

# 高级参数
deadzone=30             # 死区 (像素)
integral_limit=0.06     # 积分限幅，防止积分饱和
error_threshold=60      # 积分分离阈值，大误差时停止积分
soft_limit_lower=-1.0   # 软限幅下限
soft_limit_upper=1.0    # 软限幅上限
filter_coefficient=0.7  # 输入滤波系数，用于平滑输入信号

# 其他参数
pid_debug_output=false  # 控制是否打印调试信息
end_audio_msg=/home/ucar/Desktop/ucar/src/main/navacation/voice_packge/任务完成/任务完成.wav # 结束之后播报的音频
```

此外，我们的PID控制系统不仅仅是一个简单的PID控制器，它包含了多个高级特性，使得小车能够更好地适应各种赛道条件。以下是对这些特性的详细介绍：

### 1. 输入滤波

```c++
double filteredError = filter_coefficient_ * error + (1 - filter_coefficient_) * lastFilteredError;
```

**作用**：减少传感器噪声对控制系统的影响，使控制更加平滑。

**参数调整**：

- `filter_coefficient`：范围从0到1。值越大，滤波效果越弱；值越小，滤波效果越强。
- 建议从0.7开始，根据小车的反应进行微调。

### 2. 死区控制

```c++
if (abs(filteredError) < deadzone_)
{
    filteredError = 0;
}
else
{
    filteredError = (filteredError > 0) ? filteredError - deadzone_ : filteredError + deadzone_;
}
```

**作用**：防止小误差引起的频繁抖动，提高系统稳定性。

**参数调整**：

- `deadzone`：根据你的赛道宽度和摄像头分辨率来设定。
- 从误差值的1-2%开始，逐步增加直到小车在直线上能保持稳定。

### 3. 自适应PID参数

```c++
void adaptPIDParameters(double error)
{
    if (abs(error) > error_threshold_ * 2)
    {
        adaptive_Kp_ = Kp_ * 1.5;
        adaptive_Ki_ = Ki_ * 0.5;
        adaptive_Kd_ = Kd_ * 2;
    }
    else
    {
        adaptive_Kp_ = Kp_;
        adaptive_Ki_ = Ki_;
        adaptive_Kd_ = Kd_;
    }
}
```

**作用**：根据误差大小动态调整PID参数，使得小车在不同情况下都能有最佳表现。

**参数调整**：

- `error_threshold`：决定何时启用自适应参数。建议设置为正常误差范围的2倍左右。
- 调整系数（如1.5，0.5，2）：根据小车在大弯道和直道上的表现来调整。

### 4. 积分分离

```c++
if (abs(filteredError) < error_threshold_)
{
    integral += filteredError;
}
else
{
    integral = 0;
}
```

**作用**：防止积分饱和，减少大幅度转向时的过冲现象。

**参数调整**：

- `error_threshold`：与自适应PID中的阈值可以保持一致。

### 5. 积分限幅

```C++
integral = max(-integral_limit_, min(integral_limit_, integral));
```

**作用**：进一步防止积分饱和，提高系统稳定性。

**参数调整**：

- `integral_limit_`：开始时可以设置为最大允许转向角的10-20%，然后根据小车表现进行调整。

### 6. 微分先行

```C++
int derivative = filteredError - lastError;
```

**作用**：提高系统响应速度，减少超调。

**参数调整**：

- 主要通过调整 `Kd_` 来控制微分作用的强度。

### 7. 软限幅

```C++
steeringAngle = max(soft_limit_lower_, min(soft_limit_upper_, steeringAngle));
```

**作用**：防止输出饱和，使转向更加平滑。

**参数调整**：

- `soft_limit_lower` 和 `soft_limit_upper`：根据你的舵机或电机的实际转向范围来设定。

### 参数调整总体建议

1. 从基础PID参数（`Kp_`, `Ki_`, `Kd_`）开始调整，使小车能基本循线。
2. 逐步引入高级特性，每次只调整一个参数，观察小车表现。
3. 使用调试输出(`pid_debug_output_`)来监控误差和输出值，帮助你更好地理解参数变化对系统的影响。
4. 在不同类型的赛道上反复测试，找到一个能适应大多数情况的参数组合。
5. 记录每次调整的结果，这将帮助你更快地找到最佳参数。



## 🖼️ 图像处理局限性

请注意，当前的图像处理算法主要针对**白线和蓝底的巡线方案**进行了优化。如果你的赛道条件不同，可能需要修改 `ImageProcessor` 类中的 `processImage` 函数。

特别是，你可能需要调整`config/pid.cfg`以下部分：

- **颜色阈值**：调整 `line_threshold` 值。这个部分较为死板，推荐使用 HSV 图像进行优化，然后自己添加参数。
- **图像分割方法**：如果你的赛道有不同的颜色或纹理，可能需要使用更复杂的图像分割算法。

### 配置示例

```shell
line_threshold=220            # 提高阈值以检测更亮的线
range_y=4                     # 增加垂直分割比例以检测更大的图像区域
center_p=3                    # 扩展中心线参数以适应更宽的线条
boundary_check_width=2        # 增加边界检测宽度以增强边界识别
img_debug_output=false        # 禁用图像调试信息以提高性能
image_topic=/new_camera/image # 更改图像话题名称以匹配新设备
end_dist=7                    # 增加停止阈值以适应更长的赛道
```

### 适应其他场景的建议

- **调整颜色阈值**：根据赛道的实际颜色情况调整 `line_threshold` 值，使用 HSV 色彩空间可以更灵活地处理不同的颜色。
- **改进图像分割**：尝试使用不同的图像分割技术，如区域生长、边缘检测或深度学习方法，以适应不同的赛道条件。
- **增强边界检测**：调整 `boundary_check_width` 以增强边界检测的鲁棒性，特别是在复杂背景下。

我们鼓励你发挥创意，尝试不同的图像处理技术，以适应你的特定应用场景。



## 📜 许可证

我们采用MIT许可证，这意味着你可以随意使用、修改、分发这个项目，但请保留我们的版权信息哦！



## 🙏 致谢

- 感谢科大讯飞智能车比赛给了我们这个大显身手的机会
- 感谢ROS和OpenCV社区，没有你们，我们的小车就是个瞎子
- 感谢所有为开源社区做出贡献的开发者们，你们的智慧和奉献让技术变得更加开放和共享。





