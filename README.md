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

The default camera source is `0` and the default model is `models/yolov8n.engine`. Change them in `src/jetcar_perception/config/yolo_web.yaml` when using a CSI/GStreamer pipeline, a different TensorRT engine, or a `.pt` model.
