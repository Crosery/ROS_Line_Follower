# 🚗 Line_Follower ROS Package

 [中文](README.md)  | [English](README_en.md) 

![tip](https://badgen.net/badge/C++/g++/red?icon=github)![tip](https://badgen.net/badge/ROS/Melodic/blue?icon=github)![tip](https://badgen.net/badge/OpenCV/3.3.1/green?icon=github)

This ROS package is open-sourced by the Geek Class 👨‍💻👩‍💻 of Yangtze University. It's a customized solution for the 19th iFLYTEK Intelligent Car Rescue Group Line-following Competition and can also be used as a reference ROS package solution for other dual-line following scenarios.

![demo](demo.gif)

## 🌟 Let's Open Source Together, Driving Technological Progress

In this rapidly changing era, open source is a key force driving technological progress and innovation. By sharing our knowledge and code, we can not only accelerate technological development but also nurture a new generation of engineers and developers. We encourage everyone to join the open-source community, share your ideas and achievements, and benefit more people.

Whether you're a beginner or an experienced expert, you can find your place in open-source projects. Every line of code, every suggestion can inject new vitality into the project. Let's work together to promote technological progress and create a more open and interconnected world.

## 🌟 Shining Features

- 🧠 Intelligent Image Processing: Like giving your car a pair of keen eyes
- 🎛️ Flexible PID Control: Smooth line following, as if your car is dancing a waltz
- 🔧 Highly Configurable: You can freely tune your car

## 🛠️ Tools You Need to Prepare

Before starting, please ensure your system has the following environments and dependencies installed:

- ROS (We've repeatedly tested it on Noetic)
- OpenCV (Without it, your car would be like blind)
- Other powerful libraries: cv_bridge, image_transport, sensor_msgs, geometry_msgs
- Markdown environment (for viewing and editing this README file)

> 💡 Tip: If you haven't installed a Markdown environment yet, you can use editors like VS Code or Typora, which have good support for Markdown.

## 📁 Package Structure

```
line_follower/
├── src/
│   └── line_follower_node.cpp  (Core code, the brain of your car!)
├── config/
│   ├── image.cfg  (Tune your keen eyes)
│   └── pid.cfg   (Tune your sense of balance)
├── launch/
│   └── line_follower.launch  (One-click launch, as simple as magic)
├── rviz/
│   └── img_follow_rviz_config.rviz  (RViz configuration file, visualize your car's path tracking)
├── include/
│   └── line_follower/  (Header files, for code modularity and reuse)
├── CMakeLists.txt  (CMake configuration file, defines how to build the project)
└── package.xml  (Package configuration file, defines package metadata, such as dependencies, version, etc.)
```

## ⚙️ How to Train Your Car

1. Find a quiet place and clone this package:

   ```shell
   cd ~/catkin_ws/src
   git clone https://gitee.com/changjiang-university_2/Line_Follower.git
   ```

2. Give it some nutrition, compile it:

   ```shell
   cd ~/catkin_ws
   catkin_make
   ```

3. Install C++ OpenCV dependencies in Ubuntu:

   ```shell
   sudo apt update
   sudo apt install libopencv-dev
   ```

4. Tell it it's time to work:

   ```shell
   source ~/catkin_ws/devel/setup.bash
   ```

5. Open your car's eyes:

   You need to start your car's camera and subscribe to the corresponding topic. Modify the corresponding topic name in `config/pid.cfg`

   ```shell
   image_topic=/new_camera/image # Change the image topic name to match the new device
   ```

6. Start the power source:

   You need to start your car's chassis to allow the car to move. For example, here's how to start the first-generation ucar-mini from iFLYTEK:

   ```shell
   roslaunch ucar_controller base_driver.launch
   ```

7. Wake up your car:

   ```shell
   roslaunch line_follower line_follower.launch
   ```

## 🧠 Core Code Decryption

Our core code is hidden in `src/line_follower_node.cpp`. Let's see what secrets it holds:

1. `ImageProcessor` class: This is the car's eyes 🧐

- It can precisely find that line in a cluttered scene, just like spotting your partner in a sea of people ✨

2. `LineFollower` class: This is the car's brain and muscles 💪

- It decides how the car should move, turn left or right, run fast or slow 🚗

3. Parameter reading: We make the car very obedient, it does whatever you say 👂

4. PID control: This is like installing a balancer on the car, making it run with vigor 🏋️‍♂️

5. Debug output: Let you know what the car is thinking and doing at any time 💡

6. Image topic: Subscribe to `/img_follow`, it's like opening a filter for the car's eyes, a glimpse of how `ImageProcessor` turns chaos into clarity, an exclusive reveal of the car's perspective, right here! 👀

### 🎛️ PID Tuning Details

PID control is one of the core functions of this package. In the `config/pid.cfg` file, you can find the following key parameters:

- `Kp`: Proportional coefficient, controls the quick response of steering
- `Ki`: Integral coefficient, used to eliminate steady-state error
- `Kd`: Derivative coefficient, used to suppress oscillation

Tuning tips:

- First adjust `Kp`: Increase `Kp` until the system starts to oscillate, then reduce it until the oscillation disappears
- Then adjust `Kd`: Increase `Kd` to reduce overshoot, but not too much to avoid introducing high-frequency noise
- Finally adjust `Ki`: Slowly increase `Ki` to eliminate steady-state error, but be careful not to cause oscillation

Moreover, our PID control system is not just a simple PID controller, it includes multiple advanced features that allow the car to better adapt to various track conditions. Here's a detailed introduction to these features:

Here's a complete PID configuration example:

```shell
# Line-following robot PID configuration file

# Basic parameters
max_linear_speed=0.25   # Maximum linear speed (m/s)
max_angular_speed=1.0   # Maximum angular speed (rad/s)
Kp=0.005                # Proportional gain
Ki=0.000005             # Integral gain
Kd=0.009                # Derivative gain

# Advanced parameters
deadzone=30             # Dead zone (pixels)
integral_limit=0.06     # Integral limit, prevents integral saturation
error_threshold=60      # Integral separation threshold, stops integration at large errors
soft_limit_lower=-1.0   # Soft limit lower bound
soft_limit_upper=1.0    # Soft limit upper bound
filter_coefficient=0.7  # Input filter coefficient, used to smooth input signal

# Other parameters
pid_debug_output=false  # Controls whether to print debug information
end_audio_msg=/home/ucar/Desktop/ucar/src/main/navacation/voice_packge/任务完成/任务完成.wav # Audio to play after task completion
```

In addition, our PID control system is not just a simple PID controller, it includes multiple advanced features that allow the car to better adapt to various track conditions. Here's a detailed introduction to these features:

### 1. Input Filtering

```c++
double filteredError = filter_coefficient_ * error + (1 - filter_coefficient_) * lastFilteredError;
```

**Function**: Reduces the impact of sensor noise on the control system, making control smoother.

**Parameter adjustment**:

- `filter_coefficient`: Range from 0 to 1. The larger the value, the weaker the filtering effect; the smaller the value, the stronger the filtering effect.
- Suggested to start from 0.7 and fine-tune based on the car's response.

### 2. Dead Zone Control

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

**Function**: Prevents frequent jittering caused by small errors, improving system stability.

**Parameter adjustment**:

- `deadzone`: Set based on your track width and camera resolution.
- Start from 1-2% of the error value and gradually increase until the car can maintain stability on a straight line.

### 3. Adaptive PID Parameters

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

**Function**: Dynamically adjusts PID parameters based on error size, allowing the car to perform optimally in different situations.

**Parameter adjustment**:

- `error_threshold`: Determines when to enable adaptive parameters. Suggested to set to about 2 times the normal error range.
- Adjustment coefficients (like 1.5, 0.5, 2): Adjust based on the car's performance on large curves and straight roads.

### 4. Integral Separation

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

**Function**: Prevents integral saturation, reduces overshoot phenomenon during large turns.

**Parameter adjustment**:

- `error_threshold`: Can be kept consistent with the threshold in adaptive PID.

### 5. Integral Limit

```C++
integral = max(-integral_limit_, min(integral_limit_, integral));
```

**Function**: Further prevents integral saturation, improves system stability.

**Parameter adjustment**:

- `integral_limit_`: Can be set to 10-20% of the maximum allowed steering angle at the beginning, then adjusted based on the car's performance.

### 6. Derivative on Measurement

```C++
int derivative = filteredError - lastError;
```

**Function**: Improves system response speed, reduces overshoot.

**Parameter adjustment**:

- Mainly control the intensity of the derivative action by adjusting `Kd_`.

### 7. Soft Limiting

```C++
steeringAngle = max(soft_limit_lower_, min(soft_limit_upper_, steeringAngle));
```

**Function**: Prevents output saturation, making steering smoother.

**Parameter adjustment**:

- `soft_limit_lower` and `soft_limit_upper`: Set based on the actual steering range of your servo or motor.

### Overall Parameter Adjustment Suggestions

1. Start by adjusting the basic PID parameters (`Kp_`, `Ki_`, `Kd_`) to make the car basically follow the line.
2. Gradually introduce advanced features, adjusting only one parameter at a time, observing the car's performance.
3. Use debug output (`pid_debug_output_`) to monitor error and output values, helping you better understand how parameter changes affect the system.
4. Repeatedly test on different types of tracks to find a parameter combination that can adapt to most situations.
5. Record the results of each adjustment, this will help you find the optimal parameters faster.

## 🖼️ Image Processing Limitations

Please note that the current image processing algorithm has been optimized primarily for **white line and blue background line-following schemes**. If your track conditions are different, you may need to modify the `processImage` function in the `ImageProcessor` class.

In particular, you may need to adjust the following parts in `config/pid.cfg`:

- **Color thresholds**: Adjust the `line_threshold` value. This part is relatively rigid, it's recommended to use HSV images for optimization, then add your own parameters.
- **Image segmentation method**: If your track has different colors or textures, you may need to use more complex image segmentation algorithms.

### Configuration Example

```shell
line_threshold=220            # Increase threshold to detect brighter lines
range_y=4                     # Increase vertical segmentation ratio to detect larger image areas
center_p=3                    # Expand center line parameter to adapt to wider lines
boundary_check_width=2        # Increase boundary detection width to enhance boundary recognition
img_debug_output=false        # Disable image debug info to improve performance
image_topic=/new_camera/image # Change image topic name to match new device
end_dist=7                    # Increase stop threshold to adapt to longer tracks
```

### Suggestions for Adapting to Other Scenarios

- **Adjust color thresholds**: Adjust the `line_threshold` value based on the actual color conditions of the track. Using HSV color space can more flexibly handle different colors.
- **Improve image segmentation**: Try different image segmentation techniques, such as region growing, edge detection, or deep learning methods, to adapt to different track conditions.
- **Enhance boundary detection**: Adjust `boundary_check_width` to enhance the robustness of boundary detection, especially in complex backgrounds.

We encourage you to be creative and try different image processing techniques to adapt to your specific application scenario.

## 📜 License

We use the MIT license, which means you can freely use, modify, and distribute this project, but please retain our copyright information!

## 🙏 Acknowledgements

- Thanks to the iFLYTEK Intelligent Car Competition for giving us this opportunity to show our skills
- Thanks to the ROS and OpenCV communities, without you, our car would be blind
- Thanks to all the developers who have contributed to the open-source community, your wisdom and dedication have made technology more open and shared.
