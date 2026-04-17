# Physical-ai-automatic-car
Physical AI 기기, Jetson orin nano super, 를 이용하여 자율주행차를 만드는 것이 목적이다. 이 실험을 통해 향후 V2V, V2I V2X를 구현하고자 한다.

## Run

Build and source the ROS 2 workspace:

```bash
cd /home/kown/jetcar_ws
colcon build --symlink-install
source install/setup.bash
```

Run the full autonomous stack. It now starts in `MANUAL` with autonomy disabled,
so launching it should not immediately drive the motors:

```bash
ros2 launch jetcar_research full_autonomous.launch.py
```

Before arming autonomy, keep the wheels off the ground and check status topics:

```bash
ros2 topic echo /autonomy/status
ros2 topic echo /system/selected_control_source
ros2 topic echo /vehicle/state
```

Arm autonomy only after the status is stable:

```bash
ros2 topic pub --once /system/autonomy_enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /system/drive_mode_cmd std_msgs/msg/String "{data: AUTONOMOUS}"
```

To immediately return to safe manual output:

```bash
ros2 topic pub --once /system/autonomy_enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /system/drive_mode_cmd std_msgs/msg/String "{data: MANUAL}"
ros2 topic pub --once /system/estop_cmd std_msgs/msg/Bool "{data: true}"
```

Run the manual web controller:

```bash
ros2 launch jetcar_base manual_web.launch.py
```

Open it from the SSH client computer with port forwarding:

```bash
ssh -L 5000:localhost:5000 kown@JETCAR_IP
```

Then open `http://localhost:5000`.

## YOLO Web View

Install runtime packages on the Jetson if they are not installed:

```bash
sudo apt install python3-opencv python3-flask
pip install ultralytics
```

Add a YOLO model file before expecting detections. Put one of these files in the workspace:

```text
/home/kown/jetcar_ws/models/yolov8n.engine
/home/kown/jetcar_ws/models/yolov8n.pt
```

If no local file is present, `model_path: auto` falls back to `yolov8n.pt` so
Ultralytics can use its cache or download the default model. For offline use,
put the `.pt` or `.engine` file in `/home/kown/jetcar_ws/models`.

Run YOLO detection with MJPEG web streaming:

```bash
cd /home/kown/jetcar_ws
source install/setup.bash
ros2 launch jetcar_perception yolo_web.launch.py
```

Open it from the SSH client computer with port forwarding:

```bash
ssh -L 8080:localhost:8080 kown@JETCAR_IP
```

Then open `http://localhost:8080`.

The default camera source is `auto`. It tries Jetson CSI camera sensor 0, CSI camera sensor 1, USB `/dev/video0`, then USB `/dev/video1`. CSI inputs use `nvarguscamerasrc` and convert the frame to BGR for OpenCV. USB inputs request MJPEG/RGB conversion to avoid the all-green raw camera view that can happen when a camera is opened with the wrong format.

To force a USB camera, change `camera_source` in `src/jetcar_perception/config/yolo_web.yaml`:

```yaml
camera_source: "0"
```

To force a CSI camera:

```yaml
camera_source: csi
camera_sensor_id: 0
```

If the web page says `YOLO model unavailable`, check that `ultralytics` is installed and that a model file exists or the Jetson has network access for the first `yolov8n.pt` download. The default `model_path: auto` checks `/home/kown/jetcar_ws/models`, the current launch directory, and the installed package share before falling back to Ultralytics' default `yolov8n.pt`.
