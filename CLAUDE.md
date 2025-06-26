# Scenario Simulator V2 - Project Context

## Overview
This is the scenario_simulator_v2 project - a scenario testing framework for Autoware (autonomous driving platform). The project provides simulation tools for testing autonomous driving scenarios using the OpenSCENARIO standard.

## Project Structure
- **common/** - Shared utilities (math, exceptions, parameter handling)
- **openscenario/** - OpenSCENARIO interpreter and preprocessor
- **simulation/** - Traffic simulator, sensor simulator, behavior plugins
- **test_runner/** - Scenario and random test runners
- **rviz_plugins/** - RViz visualization plugins
- **map/** - Example map data (Kashiwanoha, simple cross)
- **mock/** - C++ mock scenarios for testing
- **external/** - External dependencies (concealer, embree, zmqpp vendors)

## Technology Stack
- **Language**: C++ (primary) with Python components
- **Framework**: ROS 2
- **Build System**: Colcon/CMake
- **Scenario Format**: OpenSCENARIO 1.2/1.3

## Git Conventions
- **Commit messages**: Write in English, keep concise and clear
- **Pull requests**: Follow PULL_REQUEST_TEMPLATE.md structure:
  - **Abstract**: [Required] Short, clear summary
  - **Background**: [Optional] Context and circumstances (use N/A if not applicable)
  - **Details**: [Optional] Detailed explanation of changes (use N/A if not applicable)
  - **References**: [Optional] Standards, algorithms, articles referenced (use N/A if not applicable)
  - **Destructive Changes**: [Optional] Breaking changes with migration guide (use N/A if not applicable)
  - **Known Limitations**: [Optional] Limitations of the implementation (use N/A if not applicable)

## Development Commands

### Formatting
```bash
ament_clang_format --reformat --clang-format-version 14
```

### Build
```bash
# Standard ROS 2 colcon build
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build specific package with its dependencies
colcon build --packages-up-to <package_name>

# Build packages that depend on a specific package
colcon build --packages-above <package_name>

# Build packages with dependencies and their dependents
colcon build --packages-above-and-dependencies <package_name>

# Skip specific package and its dependencies
colcon build --packages-skip-up-to <package_name>

# Build only previously failed packages
colcon build --packages-select-build-failed

# Skip already built packages
colcon build --packages-skip-build-finished

# Build using regex pattern
colcon build --packages-select-regex "traffic_simulator.*"

# Build starting from a specific package
colcon build --packages-start <package_name>

# Build up to a specific package
colcon build --packages-end <package_name>
```

### Testing
```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select <package_name>

# Test specific package with its dependencies
colcon test --packages-up-to <package_name>

# Test packages that depend on a specific package
colcon test --packages-above <package_name>

# Test only previously failed packages
colcon test --packages-select-test-failures

# Skip already passed tests
colcon test --packages-skip-test-passed

# Test using regex pattern
colcon test --packages-select-regex "traffic_simulator.*"

# Test starting from a specific package
colcon test --packages-start <package_name>

# Test up to a specific package
colcon test --packages-end <package_name>

# Check test results (delete old results first)
colcon test-result --delete-yes
colcon test-result
# Note: colcon test-result does not support package selection options
```

## Key Components

### 1. OpenSCENARIO Interpreter
- Interprets OpenSCENARIO XML files for scenario execution
- Located in `openscenario/openscenario_interpreter/`
- Supports OpenSCENARIO 1.2 and 1.3

### 2. Traffic Simulator
- Simulates traffic entities (vehicles, pedestrians)
- Located in `simulation/traffic_simulator/`
- Manages entity behaviors and interactions

### 3. Simple Sensor Simulator
- Simulates sensor data for perception testing
- Located in `simulation/simple_sensor_simulator/`
- Generates point clouds and detected objects

### 4. Scenario Test Runner
- Executes scenario tests
- Located in `test_runner/scenario_test_runner/`
- Supports YAML and OpenSCENARIO formats

### 5. Random Test Runner
- Generates random test scenarios
- Located in `test_runner/random_test_runner/`
- Used for robustness testing

## Important Files
- Main meta-package: `scenario_simulator_v2/`
- Documentation: `docs/` directory
- Example scenarios: `test_runner/scenario_test_runner/scenario/`
- Mock test scenarios: `mock/cpp_mock_scenarios/src/`

## Notes
- The project uses ROS 2 launch files (`.launch.py`)
- Visualization is done through RViz with custom plugins
- Communication between components uses ZeroMQ
- The project supports both Autoware Universe and legacy Autoware
