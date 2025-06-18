# robocup game_controller
## Function Description
It mainly receives the UDP packets sent by the Robocup referee machine GameController to the local area network and converts them into Ros2 Topic messages for the robocup brain to use.

## Startup Method

```
# Enter the robocup_demo directory
> cd robocup_demo
# Compile
>./script/build.sh
# Run
>./start_game_controller.sh
# View topic information
> ros2 topic info -v /robocup/game_controller
```
