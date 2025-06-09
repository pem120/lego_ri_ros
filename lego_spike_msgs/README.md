# Lego Spike Messages

This package contains additional message types tailored for use with the Lego Spike Prime and Lego Mindstorms robots.

# Building lego_spike_msgs

1. Source ROS2 Toolchain
2. CD into the lego_spike_msgs directory
3. Run

```
 colcon build --packages-select lego_spike_msgs
```

## Installing the msgs package

1. Run the setup script in the install directory created by the previous step
2. To verify that the installation was sucessful, run `ros2 pkg list | grep lego_spike_msgs`
