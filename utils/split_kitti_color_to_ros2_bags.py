import os
import cv2
import argparse
import shutil
from math import floor
from cv_bridge import CvBridge
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.time import Time as RosTime


def load_times(times_path):
    with open(times_path, 'r') as f:
        return [float(line.strip()) for line in f.readlines()]


def write_bag(out_dir, frames, times, left_imgs, right_imgs):
    bridge = CvBridge()

    if os.path.exists(out_dir):
        print(f"[WARN] Removing existing bag directory: {out_dir}")
        shutil.rmtree(out_dir)

    storage_options = StorageOptions(uri=out_dir, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    writer.create_topic(TopicMetadata(
        0,
        '/stereo_camera/left/image_rect_color',
        'sensor_msgs/msg/Image',
        'cdr',
        [],
        ''
    ))
    writer.create_topic(TopicMetadata(
        0,
        '/stereo_camera/right/image_rect_color',
        'sensor_msgs/msg/Image',
        'cdr',
        [],
        ''
    ))

    for i, idx in enumerate(frames):
        t = RosTime(seconds=int(times[idx]), nanoseconds=int((times[idx] % 1) * 1e9)).to_msg()

        left = cv2.imread(os.path.join(left_imgs, f"{idx:06}.png"))
        right = cv2.imread(os.path.join(right_imgs, f"{idx:06}.png"))

        if left is None or right is None:
            print(f"[ERROR] Missing image at index {idx}")
            continue

        left_msg = bridge.cv2_to_imgmsg(left, encoding='bgr8')
        left_msg.header.stamp = t
        left_msg.header.frame_id = 'camera_left'

        right_msg = bridge.cv2_to_imgmsg(right, encoding='bgr8')
        right_msg.header.stamp = t
        right_msg.header.frame_id = 'camera_right'

        writer.write('/stereo_camera/left/image_rect_color', left_msg, t)
        writer.write('/stereo_camera/right/image_rect_color', right_msg, t)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True, help="KITTI sequence path (e.g. /color_dataset/sequences/00)")
    parser.add_argument('--output_dir', required=True, help="Output base dir for ROS 2 bags")
    parser.add_argument('--robots', type=int, default=5, help="Number of robots")

    args = parser.parse_args()

    times = load_times(os.path.join(args.dataset, "times.txt"))
    total = len(times)
    frames_per_robot = floor(total / args.robots)

    left_path = os.path.join(args.dataset, "image_2")
    right_path = os.path.join(args.dataset, "image_3")

    print(f"[INFO] Total frames: {total} — {frames_per_robot} per robot (except last)")

    for r in range(args.robots):
        start = r * frames_per_robot
        end = (r + 1) * frames_per_robot if r < args.robots - 1 else total
        frames = list(range(start, end))

        bag_dir = os.path.join(args.output_dir, f"KITTI00-{r}")
        print(f"[INFO] Writing bag for robot {r} — frames {start} to {end-1}")
        write_bag(bag_dir, frames, times, left_path, right_path)

    print("\n✅ All ROS 2 bags generated.")


if __name__ == "__main__":
    main()
