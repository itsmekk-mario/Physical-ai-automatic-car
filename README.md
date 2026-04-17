# Physical-ai-automatic-car
Physical AI 기기, Jetson orin nano super, 를 이용하여 자율주행차를 만드는 것이 목적이다. 이 실험을 통해 향후 V2V, V2I V2X를 구현하고자 한다.

## Run

Build and source the ROS 2 workspace:

```bash
cd /home/kown/jetcar_ws
colcon build --symlink-install
source install/setup.bash
```

Run the full autonomous experiment:

```bash
ros2 launch jetcar_research full_autonomous.launch.py
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

The default camera source is `csi`, which uses Jetson `nvarguscamerasrc` and converts the frame to BGR for OpenCV. This avoids the all-green raw camera view that can happen when a CSI camera is opened as plain `/dev/video0`.

For a USB camera, change `camera_source` in `src/jetcar_perception/config/yolo_web.yaml`:

```yaml
camera_source: "0"
```

If the web page says `YOLO model unavailable`, check that `ultralytics` is installed and that a model file exists. The default `model_path: auto` looks for `models/yolov8n.engine`, `yolov8n.engine`, `models/yolov8n.pt`, and `yolov8n.pt`.
