# Robocup Demo
## Introduction
The Booster T1 Robocup official demo allows the robot to make autonomous decisions to kick the ball and complete the full Robocup match. It includes three programs: vision, brain, and game_controller.

- vision
    - The Robocup vision recognition program, based on Yolo-v8, detects objects such as robots, soccer balls, and the field, and calculates their positions in the robot's coordinate system using geometric relationships.
- brain
    - The Robocup decision-making program reads visual data and GameController game control data, integrates all available information, makes judgments, and controls the robot to perform corresponding actions, completing the match process.
- game_controller
    - Reads the game control data packets broadcast by the referee machine on the local area network, converts them into ROS2 topic messages, and makes them available for the brain to use.

## Build and Run
```
# Build the programs
./scripts/build.sh

# Run in the simultion environment
./scripts/sim_start.sh

# Run on the actual robot
./scripts/start.sh
```

## Documents
[Chinese Version](https://booster.feishu.cn/wiki/P5kJw6nDGib5wskZ3Yfc289lnIg)

[English Version](https://booster.feishu.cn/wiki/XY6Kwrq1bizif4kq7X9c14twnle)
