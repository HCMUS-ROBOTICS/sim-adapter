# sim-adapter
This is the package to fit simulators from the outside to our packages

# Installlation

Updating...

# DiRA Simulator

This simulation publishes the below topics which could be subsribed to by rosbridge-server package. In order to use this simulator, you should map the topics respectively.

**Subscribe topics**:
- `dira_rgb_topic`: a rgb image topic published by the simulator. It follows the pattern: `/<team>/camera/rgb/compressed`
- `cmd_topic`: a topic to set driving info. Default is `/cmd_vel`

where `<team>` is the team infomation set in the simulator.

**Publish topics**:
- `rgb_topic`: a topic to publish the image from the simulator. Default is `/camera/rgb/image/compressed`
- `dira_speed_topic`: a topic to set driving speed of the simulator. It follows the pattern: `/<team>/set_speed`
- `dira_angle_topic`: a topic to set driving angle of the simulator. It follows the pattern: `/<team>/set_angle`

where `<team>` is the team infomation set in the simulator.

**(Optional) Arguments/Parameters**:
- `is_show_image`: whether to use `cv2.imshow` or not. Default is `False`
- `queue_size`: the publish queue size. Default is `10`


**Build**:
```bash
cd catkin_ws
catkin_make
```

**Example running command**:
```bash
rosrun sim_adapter node.py _dira_rgb_topic:=/team220/camera/rgb/compressed _dira_speed_topic:=/team220/set_speed _dira_angle_topic:=/team220/set_angle _rgb_topic:=/camera/rgb/image/compressed _is_show_image:=true
```

You could also write a launch file to set these things.
