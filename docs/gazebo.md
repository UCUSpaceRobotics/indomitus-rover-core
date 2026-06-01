# RViz Launch для марсохода Indomitus

## Мета

Цей launch-файл піднімає модель марсохода у ROS2 та дозволяє візуалізувати її в RViz2.
Він також запускає:

robot_state_publisher — транслює TF для всіх лінків робота

joint_state_publisher або joint_state_publisher_gui — для керування суглобами робота

## RViz2 — для візуалізації моделі

⚠️ rviz.launch.py в indomitus_rover_viz не запускає реальні драйвери або контролери — тільки віртуальну модель.

#### 1. Підготовка

Залежності:
```bash
sudo apt install -y \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2
```

Перед запуском переконайтеся, що workspace зібраний:

```bash
cd ~/opt/ws
colcon build --symlink-install
source install/setup.bash
```

#### 2. Запуск launch-файлу
```bash
ros2 launch indomitus_rover_viz rviz.launch.py
```

За замовчуванням запускається RViz і joint_state_publisher_gui.

### 3. Аргументи launch-файлу

| Аргумент | Тип Опис	За замовчуванням
name	string	Префікс TF для робота	''
use_rviz	bool	Чи запускати RViz2	true
use_joint_state_publisher_gui	bool	Використовувати GUI для керування суглобами	true

| Аргумент                      | Тип    | Опис                                        | За замовчуванням |
|-------------------------------|--------|---------------------------------------------|------------------|
| name                          | string | Префікс TF для робота                       | ''               |
| use_rviz                      | bool   | Чи запускати RViz2                          | true             |
| use_joint_state_publisher_gui | bool   | Використовувати GUI для керування суглобами | true             |


Приклади використання

Запуск без RViz:
```bash
ros2 launch indomitus_rover_viz rviz.launch.py use_rviz:=false
```

Запуск без GUI для суглобів:
```bash
ros2 launch indomitus_rover_viz rviz.launch.py use_joint_state_publisher_gui:=false
```

Використання TF префіксу:
```bash
ros2 launch indomitus_rover_viz rviz.launch.py name:=mars_rover
```

4. Що відбувається після запуску

Robot State Publisher читає URDF з пакета indomitus_rover_description і публікує TF для всіх лінків.

Joint State Publisher / GUI дозволяє рухати суглоби вручну.

RViz2 відкривається з конфігурацією robot.rviz і візуалізує модель робота та його TF дерево.


# Simulation Setup

## Installation

### Ubuntu 22 / ROS2 Humble
```bash
# ROS2 tools
sudo apt install -y \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-teleop-twist-keyboard

# Gazebo Harmonic
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic ros-${ROS_DISTRO}-ros-gzharmonic
```


### Project dependencies
```bash
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

## Build and setup
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running

To run URDF model:
```bash
ros2 launch indomitus_rover_sim sim_gz_urdf.launch.py
```

#### Additional Parameters to Launch File
```bash
ros2 launch indomitus_rover_sim sim_gz_urdf.launch.py \
    world_name:=world_demo \
    model_name:=indomitus_rover
```

| Parameter | Description | Default |
|---|---|---|
| `world_name` | Name of a world located in `indomitus_rover_sim/world/` | `world_demo` |
| `model_name` | Name of a model | `indomitus_rover` |

## Camera on marsrover

To visualize camera streams from the rover, you can use `rqt_image_view`, a GUI tool that subscribes to image topics and displays them in real time

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rqt-image-view

ros2 run rqt_image_view rqt_image_view
```
After launching program you should select topic of camera you are interested in. `/camera/front_rgb/image_raw` for instance

## Driving rover with keyboard

To control the rover manually using your keyboard, use the `teleop_twist_keyboard` node. This publishes velocity commands to the robot

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
