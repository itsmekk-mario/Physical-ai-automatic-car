from pathlib import Path

import rclpy
from rclpy.node import Node


class YoloEngineBuilderNode(Node):
    def __init__(self):
        super().__init__('yolo_engine_builder')

        self.declare_parameter('model_path', '/home/kown/jetcar_ws/models/yolov8n.pt')
        self.declare_parameter('image_size', 320)
        self.declare_parameter('half', True)
        self.declare_parameter('int8', False)
        self.declare_parameter('workspace_gb', 2.0)
        self.declare_parameter('device', '0')

        model_path = Path(str(self.get_parameter('model_path').value)).expanduser().resolve()
        image_size = int(self.get_parameter('image_size').value)
        half = bool(self.get_parameter('half').value)
        int8 = bool(self.get_parameter('int8').value)
        workspace = float(self.get_parameter('workspace_gb').value)
        device = str(self.get_parameter('device').value).strip()

        from ultralytics import YOLO

        if not model_path.exists():
            raise FileNotFoundError(f'model file not found: {model_path}')

        self.get_logger().info(
            f'exporting TensorRT engine | model={model_path}, imgsz={image_size}, '
            f'half={half}, int8={int8}, workspace_gb={workspace}, device={device}'
        )
        model = YOLO(str(model_path))
        output = model.export(
            format='engine',
            imgsz=image_size,
            half=half,
            int8=int8,
            workspace=workspace,
            device=device,
            simplify=True,
        )
        self.get_logger().info(f'engine export complete: {output}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = YoloEngineBuilderNode()
    except Exception as exc:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f'engine export failed: {exc}')
        else:
            print(f'engine export failed: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
