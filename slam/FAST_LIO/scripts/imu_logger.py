#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多主题 IMU 记录与分析工具。

默认订阅 /livox/imu 与 /trunk_imu，将原始数据写入 CSV，并在退出时输出
每个主题的统计信息（采样时间、均值、标准差、模长等），可快速对比楼梯场景下的
IMU 表现。
"""

import csv
import os
import signal
from dataclasses import dataclass, field
from datetime import datetime
from typing import List

import numpy as np
import rospy
from sensor_msgs.msg import Imu


CSV_HEADER = [
    "stamp",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
    "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z",
]


def sanitize_topic(topic: str) -> str:
    cleaned = topic.strip("/").replace("/", "_")
    return cleaned if cleaned else "root"


@dataclass
class TopicLogger:
    topic: str
    output_path: str
    writer: csv.writer
    records: List[List[float]] = field(default_factory=list)
    subscriber: rospy.Subscriber = field(init=False)

    def __post_init__(self):
        self.subscriber = rospy.Subscriber(self.topic, Imu, self.callback, queue_size=200)
        rospy.loginfo("IMU logger subscribing %s → %s", self.topic, self.output_path)

    def callback(self, msg: Imu):
        stamp = msg.header.stamp.to_sec()
        row = [
            stamp,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        ]
        self.writer.writerow(row)
        self.records.append(row)

    def close(self):
        self.subscriber.unregister()


class ImuMultiLogger:
    def __init__(self):
        rospy.init_node("imu_logger")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_topics = ["/livox/imu", "/trunk_imu"]
        topics = rospy.get_param("~topics", default_topics)
        if isinstance(topics, str):
            topics = [topics]
        output_dir = rospy.get_param("~output_dir", os.path.join(os.path.expanduser("~"), "imu_logs"))
        os.makedirs(output_dir, exist_ok=True)

        self.summary_path = os.path.join(output_dir, f"imu_summary_{timestamp}.txt")
        self.loggers: List[TopicLogger] = []
        self.open_files = []

        for topic in topics:
            filename = f"{sanitize_topic(topic)}_{timestamp}.csv"
            path = os.path.join(output_dir, filename)
            f = open(path, "w", newline="")
            self.open_files.append(f)
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
            self.loggers.append(TopicLogger(topic=topic, output_path=path, writer=writer))

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

    def shutdown(self, *_):
        rospy.loginfo("IMU logger shutting down, computing statistics…")
        for logger, f in zip(self.loggers, self.open_files):
            logger.close()
            f.flush()
            f.close()

        self.write_summary()
        rospy.signal_shutdown("imu_logger completed")

    def write_summary(self):
        lines = []
        for logger in self.loggers:
            if not logger.records:
                lines.append(f"Topic {logger.topic}: no data recorded.\n")
                continue

            data = np.array(logger.records, dtype=np.float64)
            stamps = data[:, 0]
            gyro = data[:, 5:8]
            acc = data[:, 8:11]

            duration = stamps[-1] - stamps[0] if stamps.size > 1 else 0.0
            gyro_mean = gyro.mean(axis=0)
            gyro_std = gyro.std(axis=0)
            acc_mean = acc.mean(axis=0)
            acc_std = acc.std(axis=0)
            acc_norm = np.linalg.norm(acc, axis=1)
            acc_norm_mean = acc_norm.mean()
            acc_norm_std = acc_norm.std()

            summary = [
                f"Topic: {logger.topic}",
                f"  Samples   : {len(logger.records)}",
                f"  Duration  : {duration:.3f} s",
                f"  Gyro mean : [{gyro_mean[0]: .4f}, {gyro_mean[1]: .4f}, {gyro_mean[2]: .4f}] rad/s",
                f"  Gyro std  : [{gyro_std[0]: .4f}, {gyro_std[1]: .4f}, {gyro_std[2]: .4f}]",
                f"  Acc mean  : [{acc_mean[0]: .4f}, {acc_mean[1]: .4f}, {acc_mean[2]: .4f}] m/s^2",
                f"  Acc std   : [{acc_std[0]: .4f}, {acc_std[1]: .4f}, {acc_std[2]: .4f}]",
                f"  |Acc| mean: {acc_norm_mean:.4f} m/s^2",
                f"  |Acc| std : {acc_norm_std:.4f}",
                "",
            ]
            lines.extend(summary)
            for line in summary:
                rospy.loginfo(line)

        with open(self.summary_path, "w") as summary_file:
            summary_file.write("\n".join(lines))
        rospy.loginfo("Summary saved to %s", self.summary_path)


def main():
    ImuMultiLogger()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
