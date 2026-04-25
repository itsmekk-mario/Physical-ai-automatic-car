#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def image_msg_to_bgr(msg):
    if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    channels = 1 if msg.encoding == 'mono8' else 3
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


LEFT_LANE_CLASS = 0
RIGHT_LANE_CLASS = 1


def fit_lane(xs, ys, min_pixels):
    if xs.size < min_pixels:
        return None
    sample = max(1, xs.size // 1200)
    fit = np.polyfit(ys[::sample].astype(float), xs[::sample].astype(float), 1)
    return float(fit[0]), float(fit[1]), int(xs.size)


def line_to_polygon(fit, roi_y, roi_height, width, half_width):
    slope, intercept, _ = fit
    y0 = max(0, int(roi_height * 0.10))
    y1 = max(0, roi_height - 1)
    x0 = slope * y0 + intercept
    x1 = slope * y1 + intercept
    if (x0 < -width and x1 < -width) or (x0 > 2 * width and x1 > 2 * width):
        return None

    p0 = np.array([np.clip(x0, 0, width - 1), roi_y + y0], dtype=np.float32)
    p1 = np.array([np.clip(x1, 0, width - 1), roi_y + y1], dtype=np.float32)
    direction = p1 - p0
    norm = float(np.linalg.norm(direction))
    if norm < 1.0:
        return None
    normal = np.array([-direction[1], direction[0]], dtype=np.float32) / norm
    polygon = np.array([
        p0 + normal * half_width,
        p1 + normal * half_width,
        p1 - normal * half_width,
        p0 - normal * half_width,
    ], dtype=np.float32)
    polygon[:, 0] = np.clip(polygon[:, 0], 0, width - 1)
    polygon[:, 1] = np.clip(polygon[:, 1], roi_y, roi_y + roi_height - 1)
    return polygon


def detect_lane_polygons(frame, roi_top_fraction, min_pixels, half_width):
    height, width = frame.shape[:2]
    roi_y = int(max(0.0, min(0.95, roi_top_fraction)) * height)
    roi = frame[roi_y:, :]
    if roi.size == 0:
        return []

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 160]), np.array([180, 95, 255]))
    yellow = cv2.inRange(hsv, np.array([14, 45, 65]), np.array([45, 255, 255]))
    mask = cv2.bitwise_or(white, yellow)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    ys, xs = np.nonzero(mask)
    if xs.size < min_pixels:
        return []

    center_x = width / 2.0
    fits = []
    left = xs < center_x
    for class_id, group_xs, group_ys in (
        (LEFT_LANE_CLASS, xs[left], ys[left]),
        (RIGHT_LANE_CLASS, xs[~left], ys[~left]),
    ):
        fit = fit_lane(group_xs, group_ys, min_pixels)
        if fit is not None:
            fits.append((class_id, fit))

    polygons = []
    for class_id, fit in fits:
        polygon = line_to_polygon(fit, roi_y, roi.shape[0], width, half_width)
        if polygon is not None:
            polygons.append((class_id, polygon))
    return polygons


def yolo_seg_line(class_id, polygon, width, height):
    values = []
    for x, y in polygon:
        values.append(f'{float(x) / width:.6f}')
        values.append(f'{float(y) / height:.6f}')
    return f'{class_id} ' + ' '.join(values)


def draw_preview(frame, polygons):
    output = frame.copy()
    for class_id, polygon in polygons:
        color = (255, 0, 0) if class_id == LEFT_LANE_CLASS else (255, 80, 0)
        cv2.polylines(output, [polygon.astype(np.int32)], True, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.fillPoly(output, [polygon.astype(np.int32)], color)
    return output


def reset_dataset_dirs(output_dir):
    for split in ('train', 'val'):
        (output_dir / 'images' / split).mkdir(parents=True, exist_ok=True)
        (output_dir / 'labels' / split).mkdir(parents=True, exist_ok=True)
    (output_dir / 'preview').mkdir(parents=True, exist_ok=True)


def write_dataset_yaml(output_dir):
    yaml_text = (
        f'path: {output_dir.resolve()}\n'
        'train: images/train\n'
        'val: images/val\n'
        'names:\n'
        '  0: left_lane\n'
        '  1: right_lane\n'
    )
    (output_dir / 'lane.yaml').write_text(yaml_text, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description='Build weakly-labeled YOLO segmentation lane dataset from a ROS2 bag.')
    parser.add_argument('--bag', default='lane_left')
    parser.add_argument('--topic', default='/sensors/stereo/left/image_raw')
    parser.add_argument('--output', default='datasets/jetcar_lane_auto')
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--val-every', type=int, default=5)
    parser.add_argument('--roi-top', type=float, default=0.55)
    parser.add_argument('--min-pixels', type=int, default=90)
    parser.add_argument('--half-width', type=float, default=5.0)
    parser.add_argument('--preview-every', type=int, default=25)
    args = parser.parse_args()

    output_dir = Path(args.output)
    reset_dataset_dirs(output_dir)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        raise RuntimeError(f'topic not in bag: {args.topic}; available={sorted(topic_types)}')
    msg_type = get_message(topic_types[args.topic])

    seen = 0
    saved = 0
    labeled = 0
    empty = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        seen += 1
        if args.stride > 1 and (seen - 1) % args.stride != 0:
            continue

        msg = deserialize_message(data, msg_type)
        frame = image_msg_to_bgr(msg)
        height, width = frame.shape[:2]
        polygons = detect_lane_polygons(frame, args.roi_top, args.min_pixels, args.half_width)

        split = 'val' if saved % max(args.val_every, 2) == 0 else 'train'
        stem = f'lane_{saved:06d}'
        image_path = output_dir / 'images' / split / f'{stem}.jpg'
        label_path = output_dir / 'labels' / split / f'{stem}.txt'

        cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        label_lines = [yolo_seg_line(class_id, polygon, width, height) for class_id, polygon in polygons]
        label_path.write_text('\n'.join(label_lines) + ('\n' if label_lines else ''), encoding='utf-8')

        if label_lines:
            labeled += 1
        else:
            empty += 1
        if args.preview_every > 0 and saved % args.preview_every == 0:
            preview = draw_preview(frame, polygons)
            cv2.imwrite(str(output_dir / 'preview' / f'{stem}.jpg'), preview)
        saved += 1

    write_dataset_yaml(output_dir)
    print(f'bag={args.bag}')
    print(f'topic={args.topic}')
    print(f'frames_seen={seen}')
    print(f'images_saved={saved}')
    print(f'labeled_images={labeled}')
    print(f'empty_images={empty}')
    print(f'dataset={output_dir.resolve()}')
    print(f'data_yaml={(output_dir / "lane.yaml").resolve()}')


if __name__ == '__main__':
    main()
