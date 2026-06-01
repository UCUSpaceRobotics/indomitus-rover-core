# Testing

## Structure

```bash
src/<package>/test/               # unit tests (per package)
src/rover_bringup/test/ # integration tests
```

## Running

```bash
colcon test                                             # all tests
colcon test --packages-select rover_control   # single package
colcon test --event-handlers console_direct+            # with terminal output
colcon test-result --verbose                            # view results
```

## Unit Tests

**Python** — `pytest`, file must be named `test_*.py`  
**C++** — `gtest`, registered in `CMakeLists.txt` via `ament_add_gtest`

## Integration Tests

TODO:  
Use `launch_testing` in `rover_bringup/test/`. Spins up real nodes and checks topics.
