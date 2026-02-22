# ros2_rtsp

ROS 2 package for pulling RTSP video/audio from one or more cameras.

## Features

- Publishes video frames per camera:
  - `/camnet/cam_<index>/image_raw` (`sensor_msgs/msg/Image`)
  - `/camnet/cam_<index>/camera_info` (`sensor_msgs/msg/CameraInfo`)
- Publishes raw audio bytes per camera:
  - `/camnet/cam_<index>/audio` (`std_msgs/msg/UInt8MultiArray`)
  - Format: `pcm_s16le`, 16 kHz, mono
- Includes `audio_monitor` node to play audio live from a selected topic.

## Build

```bash
cd /home/nils/dev_ws
colcon build --packages-select ros2_rtsp
source install/setup.bash
```

## Run RTSP Capture

```bash
ros2 launch ros2_rtsp rtsp_multi.launch.py \
  ip_addresses:="['192.168.1.10','192.168.1.11']" \
  user_name:=admin \
  user_password:=secret \
  rtsp_path:=stream2 \
  audio_rtsp_path:=stream1
```

If `audio_rtsp_path` is empty, it falls back to `rtsp_path`.

## Listen To Audio Live

```bash
ros2 run ros2_rtsp audio_monitor --ros-args -p topic:=/camnet/cam_0/audio
```

Optional parameters:

- `player` (`auto`, `ffplay`, `aplay`) default: `auto`
- `sample_rate` default: `16000`
- `channels` default: `1`

Example forcing ALSA:

```bash
ros2 run ros2_rtsp audio_monitor --ros-args \
  -p topic:=/camnet/cam_0/audio \
  -p player:=aplay
```

## Notes

- `rtsp_multi` requires `ffmpeg` for audio extraction.
- `audio_monitor` requires either `ffplay` or `aplay` available on `PATH`.
