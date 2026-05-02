from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

import cv2
import numpy as np


CULANE_ROW_ANCHORS = [
    121,
    131,
    141,
    150,
    160,
    170,
    180,
    189,
    199,
    209,
    219,
    228,
    238,
    248,
    258,
    267,
    277,
    287,
]

TUSIMPLE_ROW_ANCHORS = [
    64,
    68,
    72,
    76,
    80,
    84,
    88,
    92,
    96,
    100,
    104,
    108,
    112,
    116,
    120,
    124,
    128,
    132,
    136,
    140,
    144,
    148,
    152,
    156,
    160,
    164,
    168,
    172,
    176,
    180,
    184,
    188,
    192,
    196,
    200,
    204,
    208,
    212,
    216,
    220,
    224,
    228,
    232,
    236,
    240,
    244,
    248,
    252,
    256,
    260,
    264,
    268,
    272,
    276,
    280,
    284,
]


class UfldConfigurationError(RuntimeError):
    pass


@dataclass
class UfldLane:
    lane_index: int
    xs: np.ndarray
    ys: np.ndarray
    confidence: float


@dataclass
class UfldPrediction:
    lanes: List[UfldLane]
    point_count: int
    confidence: float
    runtime: str


class UfldLaneDetector:
    """Runs UFLD v1 exported models and decodes lane anchor logits.

    Supported model formats:
    - TorchScript: output [1, griding_num + 1, row_anchor_count, lane_count]
    - ONNX: same output layout, or any 3-D layout where the grid axis can be
      identified by griding_num + 1.
    """

    def __init__(
        self,
        model_path: str,
        runtime: str = 'auto',
        model_type: str = 'culane',
        input_width: int = 800,
        input_height: int = 288,
        griding_num: int = 200,
        lane_indices: Sequence[int] = None,
        row_anchors: Sequence[int] = None,
        min_points_per_lane: int = 4,
        presence_threshold: float = 0.5,
        device: str = 'cuda:0',
        prefer_half: bool = True,
        normalize_mean: Sequence[float] = (0.485, 0.456, 0.406),
        normalize_std: Sequence[float] = (0.229, 0.224, 0.225),
    ):
        self.model_path = Path(model_path).expanduser()
        self.runtime = runtime.lower().strip()
        self.model_type = model_type.lower().strip()
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.griding_num = int(griding_num)
        self.lane_indices = list(lane_indices or [1, 2])
        self.row_anchors = list(row_anchors or self.default_row_anchors(self.model_type))
        self.min_points_per_lane = max(1, int(min_points_per_lane))
        self.presence_threshold = max(0.0, min(1.0, float(presence_threshold)))
        self.device_name = str(device).strip()
        self.prefer_half = bool(prefer_half)
        self.normalize_mean = np.asarray(normalize_mean, dtype=np.float32).reshape(3, 1, 1)
        self.normalize_std = np.asarray(normalize_std, dtype=np.float32).reshape(3, 1, 1)

        if self.input_width <= 0 or self.input_height <= 0:
            raise UfldConfigurationError('ufld input size must be positive')
        if self.griding_num <= 1:
            raise UfldConfigurationError('ufld_griding_num must be greater than 1')
        if len(self.row_anchors) == 0:
            raise UfldConfigurationError('ufld row anchors are empty')
        if not self.model_path.exists():
            raise FileNotFoundError(f'UFLD model file not found: {self.model_path}')

        if self.runtime == 'auto':
            self.runtime = 'onnx' if self.model_path.suffix.lower() == '.onnx' else 'torchscript'

        self.model = None
        self.session = None
        self.input_name = ''
        self.torch = None
        self.torch_device = None

        if self.runtime in ('torch', 'torchscript', 'jit'):
            self.runtime = 'torchscript'
            self.load_torchscript()
        elif self.runtime == 'onnx':
            self.load_onnx()
        else:
            raise UfldConfigurationError(f'unsupported ufld_runtime: {runtime}')

    @staticmethod
    def default_row_anchors(model_type: str) -> List[int]:
        if model_type == 'tusimple':
            return list(TUSIMPLE_ROW_ANCHORS)
        return list(CULANE_ROW_ANCHORS)

    def load_torchscript(self):
        try:
            import torch
        except Exception as exc:
            raise UfldConfigurationError(f'PyTorch is required for TorchScript UFLD: {exc}') from exc

        self.torch = torch
        use_cuda = self.device_name.startswith('cuda') and torch.cuda.is_available()
        self.torch_device = torch.device(self.device_name if use_cuda else 'cpu')
        self.model = torch.jit.load(str(self.model_path), map_location=self.torch_device)
        self.model.eval()
        if use_cuda and self.prefer_half:
            try:
                self.model.half()
            except Exception:
                self.prefer_half = False
        else:
            self.prefer_half = False

    def load_onnx(self):
        try:
            import onnxruntime as ort
        except Exception as exc:
            raise UfldConfigurationError(f'onnxruntime is required for ONNX UFLD: {exc}') from exc

        providers = ['CPUExecutionProvider']
        if self.device_name.startswith('cuda'):
            available = set(ort.get_available_providers())
            if 'CUDAExecutionProvider' in available:
                providers.insert(0, 'CUDAExecutionProvider')
        self.session = ort.InferenceSession(str(self.model_path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.prefer_half = False

    def detect(self, frame: np.ndarray) -> UfldPrediction:
        output = self.run_model(frame)
        lanes = self.decode_output(output, frame.shape[1], frame.shape[0])
        point_count = sum(int(lane.xs.size) for lane in lanes)
        confidence = 0.0
        if lanes:
            confidence = float(sum(lane.confidence for lane in lanes) / len(lanes))
        return UfldPrediction(
            lanes=lanes,
            point_count=point_count,
            confidence=confidence,
            runtime=self.runtime,
        )

    def run_model(self, frame: np.ndarray) -> np.ndarray:
        image = self.preprocess(frame)
        if self.runtime == 'onnx':
            outputs = self.session.run(None, {self.input_name: image[np.newaxis, ...]})
            return np.asarray(outputs[0])

        tensor = self.torch.from_numpy(image).unsqueeze(0).to(self.torch_device)
        if self.prefer_half:
            tensor = tensor.half()
        with self.torch.no_grad():
            output = self.model(tensor)
        if isinstance(output, (tuple, list)):
            output = output[0]
        return output.detach().float().cpu().numpy()

    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        resized = cv2.resize(frame, (self.input_width, self.input_height), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        image = rgb.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        return (image - self.normalize_mean) / self.normalize_std

    def decode_output(self, output: np.ndarray, frame_width: int, frame_height: int) -> List[UfldLane]:
        logits = self.normalize_output_layout(output)
        probabilities = self.softmax(logits, axis=0)
        present_confidence = 1.0 - probabilities[self.griding_num, :, :]
        grid_probabilities = self.softmax(logits[: self.griding_num, :, :], axis=0)
        grid_positions = np.arange(1, self.griding_num + 1, dtype=np.float32).reshape(-1, 1, 1)
        weighted_positions = np.sum(grid_probabilities * grid_positions, axis=0)
        argmax_positions = np.argmax(logits, axis=0)

        lanes = []
        lane_count = logits.shape[2]
        selected_indices = self.lane_indices or list(range(lane_count))
        for lane_index in selected_indices:
            if lane_index < 0 or lane_index >= lane_count:
                continue
            valid = argmax_positions[:, lane_index] != self.griding_num
            valid &= present_confidence[:, lane_index] >= self.presence_threshold
            if int(np.count_nonzero(valid)) < self.min_points_per_lane:
                continue

            x_model = self.grid_to_model_x(weighted_positions[valid, lane_index])
            y_model = np.asarray(self.row_anchors, dtype=np.float32)[valid]
            xs = x_model * (float(frame_width) / float(self.input_width))
            ys = y_model * (float(frame_height) / float(self.input_height))
            confidence = float(np.mean(present_confidence[valid, lane_index]))
            lanes.append(
                UfldLane(
                    lane_index=int(lane_index),
                    xs=np.clip(xs, 0.0, float(frame_width - 1)).astype(np.float32),
                    ys=np.clip(ys, 0.0, float(frame_height - 1)).astype(np.float32),
                    confidence=confidence,
                )
            )
        return lanes

    def normalize_output_layout(self, output: np.ndarray) -> np.ndarray:
        logits = np.asarray(output)
        while logits.ndim > 3 and logits.shape[0] == 1:
            logits = logits[0]
        if logits.ndim != 3:
            raise ValueError(f'UFLD output must be 3-D after batch squeeze, got shape {logits.shape}')

        grid_size = self.griding_num + 1
        grid_axes = [index for index, size in enumerate(logits.shape) if size == grid_size]
        if grid_axes:
            grid_axis = grid_axes[0]
        else:
            grid_axis = int(np.argmax(logits.shape))
        logits = np.moveaxis(logits, grid_axis, 0)
        if logits.shape[0] != grid_size:
            raise ValueError(
                f'UFLD grid axis mismatch: expected {grid_size}, got output shape {output.shape}'
            )

        row_count = len(self.row_anchors)
        if logits.shape[1] == row_count:
            return logits.astype(np.float32, copy=False)
        if logits.shape[2] == row_count:
            return np.transpose(logits, (0, 2, 1)).astype(np.float32, copy=False)
        raise ValueError(
            f'UFLD row anchor mismatch: expected {row_count} rows, got output shape {output.shape}'
        )

    def grid_to_model_x(self, weighted_positions: np.ndarray) -> np.ndarray:
        zero_based = np.maximum(weighted_positions.astype(np.float32) - 1.0, 0.0)
        scale = float(self.input_width - 1) / float(self.griding_num - 1)
        return zero_based * scale

    @staticmethod
    def softmax(values: np.ndarray, axis: int = 0) -> np.ndarray:
        shifted = values - np.max(values, axis=axis, keepdims=True)
        exp = np.exp(shifted)
        return exp / np.sum(exp, axis=axis, keepdims=True)
