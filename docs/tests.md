# Testing Guide — Indomitus Rover

## Структура тестів

```
src/
  <package>/test/          ← unit-тести (в кожному пакеті)
  indomitus_rover_bringup/test/   ← integration-тести
```

В корені немає бути скриптів для тестів

### Unit-тести — в пакеті

Тестують логіку одного пакету ізольовано, без запуску нод і без реального заліза.

```
indomitus_rover_control/test/
    test_kinematics.py       # математика cmd_vel → WheelTargets

chassis_driver/test/
    test_protocols.cpp       # формування CAN-фреймів (Damiao)
```

### Integration-тести — в bringup/test

Тестують взаємодію між нодами. Запускають реальні ноди через `launch_testing`, публікують топіки і перевіряють вихід.

```
indomitus_rover_bringup/test/
    test_pipeline.py         # ланцюг: cmd_vel → kinematics → chassis_driver
    test_driver.py           # поведінка драйвера на різні WheelTargets
```

---

## Запуск тестів

```bash
# всі тести
colcon test

# конкретний пакет
colcon test --packages-select indomitus_rover_control

# з виводом у термінал
colcon test --event-handlers console_direct+

# переглянути результати після запуску
colcon test-result --verbose
```

> Для цього в cmake/setup.py та package.xml треба вказати які файли є тестами

---

## Unit-тести (Python)

Використовуй `pytest`. ROS2 підхоплює його автоматично якщо файл називається `test_*.py`.

### Шаблон

```python
# src/indomitus_rover_control/test/test_kinematics.py

import pytest
from indomitus_rover_control.rover_kinematics_node import compute_wheel_targets

def test_straight_line():
    """Пряма їзда — швидкості лівих і правих коліс рівні."""
    result = compute_wheel_targets(vx=1.0, omega=0.0)
    assert result.front_left == pytest.approx(result.front_right, abs=1e-6)

def test_zero_command():
    """Нульова команда — всі колеса стоять."""
    result = compute_wheel_targets(vx=0.0, omega=0.0)
    assert all(v == 0.0 for v in result)

def test_max_speed_clamped():
    """Швидкість не перевищує максимум."""
    result = compute_wheel_targets(vx=999.0, omega=0.0)
    assert all(abs(v) <= MAX_WHEEL_SPEED for v in result)
```

### Реєстрація в CMakeLists.txt / setup.py

**Python (setup.py):**
```python
tests_require=['pytest'],
```

**C++ (CMakeLists.txt):**
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocols test/test_protocols.cpp)
  target_link_libraries(test_protocols chassis_driver_lib)
endif()
```

---

## Unit-тести (C++)

Використовуй `gtest`. Файли в `test/`, підключаються через `CMakeLists.txt`.

### Шаблон

```cpp
// src/chassis_driver/test/test_protocols.cpp
#include <gtest/gtest.h>
#include "chassis_driver/damiao_protocol.hpp"

TEST(DamiaoProtocol, ZeroSpeedFrame) {
    auto frame = DamiaoProtocol::build_frame(0.0f);
    EXPECT_EQ(frame.can_id, 0x01);
    EXPECT_EQ(frame.data[0], 0x00);
}

TEST(DamiaoProtocol, MaxSpeedClamped) {
    auto frame = DamiaoProtocol::build_frame(9999.0f);
    float decoded = DamiaoProtocol::decode_speed(frame);
    EXPECT_LE(decoded, MAX_SPEED);
}
```

---

## Integration-тести (launch_testing)

Запускають ноди програмно і перевіряють поведінку системи цілком.

### Шаблон

```python
# src/indomitus_rover_bringup/test/test_pipeline.py

import unittest
import launch
import launch_ros.actions
import launch_testing
import rclpy
from geometry_msgs.msg import Twist
from indomitus_msgs.msg import WheelTargets

@launch_testing.decorators.keep_alive
def generate_test_description():
    kinematics_node = launch_ros.actions.Node(
        package='indomitus_rover_control',
        executable='rover_kinematics_node',
    )
    return launch.LaunchDescription([kinematics_node]), {'kinematics': kinematics_node}


class TestPipeline(unittest.TestCase):
    def test_cmd_vel_produces_wheel_targets(self):
        rclpy.init()
        node = rclpy.create_node('test_node')

        received = []
        node.create_subscription(WheelTargets, '/wheel_targets',
                                  lambda msg: received.append(msg), 10)

        pub = node.create_publisher(Twist, '/cmd_vel', 10)
        cmd = Twist()
        cmd.linear.x = 0.5
        pub.publish(cmd)

        # чекаємо відповідь
        deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=2)
        while not received and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        self.assertTrue(len(received) > 0, "Не отримано WheelTargets")
        self.assertGreater(received[0].front_left, 0.0)

        node.destroy_node()
        rclpy.shutdown()
```

---

## Що тестувати (пріоритети)

| Пакет | Що | Тип | Пріоритет |
|---|---|---|---|
| `indomitus_rover_control` | Формула кінематики | unit | 🔴 обов'язково |
| `indomitus_rover_control` | Граничні значення, від'ємні швидкості | unit | 🔴 обов'язково |
| `chassis_driver` | Формування CAN-фреймів | unit | 🔴 обов'язково |
| `chassis_driver` | Декодування відповіді мотора | unit | 🟡 бажано |
| `bringup` | cmd_vel → WheelTargets end-to-end | integration | 🟡 бажано |
| `bringup` | Поведінка при втраті зв'язку | integration | 🟢 колись |

---

## Правила

- Назва файлу — `test_*.py` або `*_test.cpp`
- Назва функції/тесту описує **що** тестується і **який очікується результат**: `test_straight_line_equal_wheel_speeds` а не `test_1`
- Unit-тест не запускає ноди, не використовує реальний CAN
- Для CAN mock'ай через dependency injection або підміну транспорту
- Кожен тест — незалежний, не покладається на стан від попереднього