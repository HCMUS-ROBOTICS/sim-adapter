# sim-adapter
This is the package to fit simulators from the outside to our packages

# Installlation

Updating...

# Current Supported Simulators

Currently, there are two types of simulation adapters we support: DiRA and UIT. To choose different adapters, you should add `adapter` argument (please see [bin/node.py](bin/node.py)).

**Publish topics**:
- `rgb_topic`: a topic to publish the image to the self-driving system. Default is `/camera/rgb/image/compressed`

**Subscribe topics**:
- `cmd_topic`: a topic to set driving info sent from self-driving system. Default is `/cmd_vel`

**Arguments/Parameters**:
- `adapter`: the value of `adapter` argument must be in `[dira, uit]`.
- (optional) `is_show_image`: whether to use `cv2.imshow` or not. Default is `False`
- (optional) `queue_size`: the publish queue size. Default is `10`

**Build**:
```bash
cd catkin_ws
catkin_make
```

**Example**:
```bash
rosrun sim_adapter node.py _adapter:=uit _is_show_image:=True
```


## DiRA Simulator

This simulation publishes the below topics which could be subsribed to by rosbridge-server package. In order to use this simulator, you should map the topics respectively.
The publishing and subscribing topics belows are used for internal interacting with this simulation only.

**Subscribe topics**:
- `dira_rgb_topic`: a rgb image topic published by the simulator. It follows the pattern: `/<team>/camera/rgb/compressed`

where `<team>` is the team infomation set in the simulator.

**Publish topics**:
- `dira_speed_topic`: a topic to set driving speed of the simulator. It follows the pattern: `/<team>/set_speed`
- `dira_angle_topic`: a topic to set driving angle of the simulator. It follows the pattern: `/<team>/set_angle`

where `<team>` is the team infomation set in the simulator.

**Example running command**:
```bash
rosrun sim_adapter node.py _dira_rgb_topic:=/team220/camera/rgb/compressed _dira_speed_topic:=/team220/set_speed _dira_angle_topic:=/team220/set_angle _rgb_topic:=/camera/rgb/image/compressed _is_show_image:=true
```

You could also write a launch file to set these things.

## UIT Simulator

This simulation send message via websocket. In order to use this simulator, you should choose the hostname and port number respectively.

**Parameters**:
- `uit_hostname`: hostname for the simulator to access. Default is `4567`
- `uit_port`: port number for the simulator to access. Default is `127.0.0.1`

**Example running command**:
```bash
rosrun sim_adapter node.py _adapter:=uit _uit_hostname:=127.0.0.1 _uit_port:=4567 _is_show_image:=True
```