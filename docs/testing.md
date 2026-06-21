# Testing

## Running Tests

### Local Manual Testing

**Workspace preparation:**

Tests should generally be executed inside the development Docker container to ensure all dependencies and simulation tools are present.

1. Start the local development container or run tests natively.

2. [Enter the container](scripts/enter_container.md) or navigate to your workspace root.

3. Prepare the workspace:
```bash
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

> Note: if you are using other ROS 2 distribution rather than humble you will need to specify it in the command


**Run all tests:**

Executes all suites, including functional logic and linters (code style, copyright, PEP257).

```bash
colcon test --event-handlers console_cohesion+
colcon test-result --all
```

**Run only functional tests (code style, docs, and copyright excluded):**

To see results for *only* the current test run, clear the previous test results first. Otherwise, cached results from past runs may also be displayed:

```bash
rm -rf build/*/test_results/
```

Run tests:

```bash
colcon test --event-handlers console_cohesion+ \
  --pytest-args -m "not linter" \
  --ctest-args -LE linter
colcon test-result --all
```

### Remote Automated Testing

**Automated testing is handled by GitHub workflows:**

* **Triggers:** Automatically runs when a Pull Request is opened or updated against the `main` or `develop` branches.
* **What Is Tested:** Workflows execute only functional tests, actively ignoring code style, documentation, and copyright checks.
* **Reporting:** Test results are published to the GitHub PR dashboard. Any functional test failure will block the merge. 


**Finding the Test Results Dashboard in GitHub:**

1. **Open your Pull Request** in GitHub.
2. **Scroll down** to the workflow checks section at the bottom of the page.
3. **Locate the job** named `PR Pipeline / run-tests (pull_request)`.
4. **Click the three dots (`...`)** next to the job name, then select **View details**.
5. **Select Summary** from the left sidebar.
6. **Scroll down** to view the complete test results and identify any failures.

---

## Creating Tests

Place test files in the `test/` directory of your ROS 2 package.

**Recommended Frameworks/Libraries:**

| Language | Type | Framework | ROS 2 Wrapper | Description |
| --- | --- | --- | --- | --- |
| **C++** | **Unit** | `gtest` / `gmock` | `ament_cmake_gtest` | Isolated testing of C++ functions and mock hardware. |
| **C++** | **Integration** | `launch_testing` | `launch_testing_ament_cmake` | Spins up real nodes to test pub/sub and service calls. |
| **Python** | **Unit** | `pytest` | `ament_pytest` | Fast, isolated logic testing using fixtures and asserts. |
| **Python** | **Integration** | `launch_testing` | `launch_testing_ros` | Launches Python nodes to test real network interactions. |
| **Both** | **Linters** | `ament_lint_auto` | `ament_cpplint`, `ament_flake8` | Enforces coding standards, formatting, and copyright. |

---

### Configuration Guide

To enable these testing frameworks, declare them in your package's configuration files:

* **`package.xml`:** Add the required testing frameworks (e.g., `ament_lint_auto`, `ament_cmake_gtest`, or `pytest`) as `<test_depend>` tags.
* **`CMakeLists.txt` (C++):** Wrap all test configurations and target linkages inside an `if(BUILD_TESTING)` block at the bottom of the file (immediately before `ament_package()`).
* **`setup.py` (Python):** Add `pytest` to the `tests_require` array. No further configuration is needed; `colcon` will automatically discover and execute your tests.
