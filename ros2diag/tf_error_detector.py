#!/usr/bin/env python3
"""
TF Error Detector - Monitor TF transforms and detect timing issues
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import statistics
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

NANOSEC_TO_SEC = 1e9  # conversion factor from nanoseconds to seconds
MSEC_TO_SEC = 1000.0  # conversion factor from seconds to milliseconds
QOS_DEPTH = 10  # queue depth for ROS2 subscriptions
MAX_TIMESTAMPS = 100  # max number of timestamps to keep per transform
MAX_TIMESTAMP_WARNINGS = 3  # max warnings to store per frame pair
TIMESTAMP_THRESHOLD_SEC = 0.025  # threshold for timestamp warnings in seconds
TIMER_PERIOD_SEC = 0.1  # period for lookup attempts in seconds
STARTUP_DELAY_SEC = 1.0  # delay before starting lookups to allow static TFs to load
RUN_DURATION_SEC = 10.0  # total monitoring duration in seconds
SUCCESS_RATE_OK = 90  # success rate threshold for OK status
SUCCESS_RATE_WARN = 50  # success rate threshold for WARN status
SEPARATOR_WIDTH = 80  # width of separator lines in output


class TFExceptionTracker:
    """Tracks and manages TF exception information, organized by exception type."""

    def __init__(self):
        self.exceptions_by_type = {
            "LookupException": [],
            "ConnectivityException": [],
            "ExtrapolationException": [],
        }
        self.exceptions_by_frame_pair = defaultdict(list)
        self.exception_count_by_type = defaultdict(int)

    def record_exception(
        self,
        exception_type,
        source_frame,
        target_frame,
        lookup_time,
        error_message,
        latest_tf_time,
        **extra_context,
    ):
        frame_pair = f"{source_frame}->{target_frame}"

        record = {
            "exception_type": exception_type,
            "source_frame": source_frame,
            "target_frame": target_frame,
            "frame_pair": frame_pair,
            "lookup_time": lookup_time,
            "error_message": error_message,
            "latest_tf_time": latest_tf_time,
        }

        if latest_tf_time is not None:
            record["time_diff"] = lookup_time - latest_tf_time
            if record["time_diff"] < 0:
                record["diagnosis"] = "future"
            else:
                record["diagnosis"] = "past"

        record.update(extra_context)
        self.exceptions_by_type[exception_type].append(record)
        self.exceptions_by_frame_pair[frame_pair].append(record)
        self.exception_count_by_type[exception_type] += 1

    def get_frame_pairs_with_errors(self):
        return sorted(self.exceptions_by_frame_pair.keys())

    def get_exceptions_for_pair(self, frame_pair):
        return self.exceptions_by_frame_pair[frame_pair]

    def get_total_count_for_pair(self, frame_pair):
        return len(self.exceptions_by_frame_pair[frame_pair])

    def has_exceptions(self):
        return len(self.exceptions_by_frame_pair) > 0

    def get_exceptions_by_type_for_pair(self, frame_pair):
        exceptions = self.exceptions_by_frame_pair[frame_pair]
        by_type = {}
        for exc in exceptions:
            exc_type = exc["exception_type"]
            if exc_type not in by_type:
                by_type[exc_type] = {"count": 0, "example": exc}
            by_type[exc_type]["count"] += 1
        return by_type


class TFTimingMonitor(Node):
    def __init__(self):
        super().__init__("tf_timing_monitor")

        self.tf_sub = self.create_subscription(
            TFMessage, "/tf", self.tf_callback, QOS_DEPTH
        )

        static_qos = QoSProfile(
            depth=QOS_DEPTH, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self.tf_static_callback, static_qos
        )

        self.timing_data = defaultdict(
            lambda: {
                "timestamps": [],
                "gaps": [],
                "is_static": False,
            }
        )
        self.lookup_attempts = defaultdict(lambda: {"successes": 0, "failures": 0})
        self.exception_tracker = TFExceptionTracker()
        self.timestamp_warnings = defaultdict(list)
        self.timestamp_threshold = TIMESTAMP_THRESHOLD_SEC
        self.run_duration = RUN_DURATION_SEC
        self.start_time = None
        self.lookups_enabled = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.startup_timer = self.create_timer(STARTUP_DELAY_SEC, self.enable_lookups)
        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.attempt_lookups)

        self.start_time = self.get_clock().now()

        print(
            f"TF Timing Monitor started - will run for {self.run_duration} seconds..."
        )

    def tf_callback(self, msg):
        self.analyze_transforms(msg, is_static=False)

    def tf_static_callback(self, msg):
        self.analyze_transforms(msg, is_static=True)

    def enable_lookups(self):
        self.lookups_enabled = True
        self.startup_timer.cancel()

    def analyze_transforms(self, msg, is_static):
        current_ros_time = self.get_clock().now().nanoseconds / NANOSEC_TO_SEC

        for transform in msg.transforms:
            frame_id = transform.header.frame_id
            child_frame_id = transform.child_frame_id
            pair = f"{frame_id}->{child_frame_id}"

            stamp = transform.header.stamp
            timestamp = stamp.sec + stamp.nanosec / NANOSEC_TO_SEC

            time_diff = abs(current_ros_time - timestamp)
            if (
                time_diff > self.timestamp_threshold
                and len(self.timestamp_warnings[pair]) < MAX_TIMESTAMP_WARNINGS
            ):
                self.timestamp_warnings[pair].append(
                    {
                        "tf_timestamp": timestamp,
                        "ros_time": current_ros_time,
                        "diff": time_diff,
                        "ahead": current_ros_time < timestamp,
                    }
                )

            data = self.timing_data[pair]
            data["is_static"] = is_static
            data["timestamps"].append(timestamp)

            if len(data["timestamps"]) > MAX_TIMESTAMPS:
                data["timestamps"].pop(0)

            if len(data["timestamps"]) > 1:
                gap = data["timestamps"][-1] - data["timestamps"][-2]
                data["gaps"].append(gap)
                if len(data["gaps"]) > MAX_TIMESTAMPS:
                    data["gaps"].pop(0)

    def attempt_lookups(self):
        if not self.lookups_enabled:
            return

        lookup_time = Time()

        for pair in self.timing_data:
            frame_id, child_frame_id = pair.split("->")

            try:
                transform = self.tf_buffer.lookup_transform(
                    child_frame_id, frame_id, lookup_time
                )
                self.lookup_attempts[pair]["successes"] += 1

            except LookupException as e:
                self.lookup_attempts[pair]["failures"] += 1
                self.handle_exception("LookupException", pair, str(e))
            except ConnectivityException as e:
                self.lookup_attempts[pair]["failures"] += 1
                self.handle_exception("ConnectivityException", pair, str(e))
            except ExtrapolationException as e:
                self.lookup_attempts[pair]["failures"] += 1
                self.handle_exception("ExtrapolationException", pair, str(e))

    def handle_exception(self, exc_type, pair, error_msg):
        frame_id, child_frame_id = pair.split("->")
        data = self.timing_data[pair]
        lookup_time_sec = self.get_clock().now().nanoseconds / NANOSEC_TO_SEC
        latest_tf_time = data["timestamps"][-1] if data["timestamps"] else None

        extra_context = {}
        if data["gaps"]:
            extra_context["avg_gap"] = statistics.mean(data["gaps"])
            extra_context["publish_rate"] = 1 / extra_context["avg_gap"]

        self.exception_tracker.record_exception(
            exc_type,
            frame_id,
            child_frame_id,
            lookup_time_sec,
            error_msg,
            latest_tf_time,
            **extra_context,
        )

    def format_timestamp(self, unix_timestamp):
        dt = datetime.fromtimestamp(unix_timestamp)
        return dt.strftime("%H:%M:%S.%f")[:-3]

    def calc_success_rate(self, pair):
        attempts = self.lookup_attempts.get(pair, {"successes": 0, "failures": 0})
        total = attempts["successes"] + attempts["failures"]
        return (attempts["successes"] / total * 100) if total > 0 else 0

    def get_status_label(self, success_rate):
        if success_rate > SUCCESS_RATE_OK:
            return "[OK]  "
        if success_rate > SUCCESS_RATE_WARN:
            return "[WARN]"
        return "[FAIL]"

    def print_transform_summary(self, pair, data):
        rate = 1 / statistics.mean(data["gaps"]) if data["gaps"] else 0
        success_rate = self.calc_success_rate(pair)
        status = self.get_status_label(success_rate)
        exception_count = self.exception_tracker.get_total_count_for_pair(pair)
        print(
            f"{status} {pair:40s} | {rate:6.2f} Hz | # exceptions on lookup: {exception_count}"
        )

    def print_error_record(self, pair, error_record):
        print(f"    {error_record['exception_type']}")
        if (
            "latest_tf_time" in error_record
            and error_record["latest_tf_time"] is not None
        ):
            print(
                f"    System: {self.format_timestamp(error_record['lookup_time'])} | "
                f"TF: {self.format_timestamp(error_record['latest_tf_time'])} | "
                f"Diff: {error_record['time_diff'] * MSEC_TO_SEC:.1f}ms"
            )
        if "publish_rate" in error_record:
            rate_info = f"{error_record['publish_rate']:.1f} Hz"
            success = self.calc_success_rate(pair)
            print(f"    Rate: {rate_info} | Success: {success:.1f}%")

    def print_clock_sync_info(self):
        if not self.timestamp_warnings:
            return
        print("\n" + "=" * SEPARATOR_WIDTH)
        print("CLOCK SYNC INFO")
        print("=" * SEPARATOR_WIDTH)
        for pair, warnings in sorted(self.timestamp_warnings.items()):
            for w in warnings:
                direction = "ahead" if w["ahead"] else "behind"
                print(f"{pair}: TF {direction} by {w['diff'] * MSEC_TO_SEC:.1f}ms")
        print("=" * SEPARATOR_WIDTH)

    def print_error_report(self):
        if not self.exception_tracker.has_exceptions():
            print("\nNo timing errors detected.")
            print("=" * SEPARATOR_WIDTH + "\n")
            return

        print("\n" + "=" * SEPARATOR_WIDTH)
        print("ERROR REPORT - TRANSFORMS WITH TIMING ISSUES")
        print("=" * SEPARATOR_WIDTH)
        for pair in self.exception_tracker.get_frame_pairs_with_errors():
            by_type = self.exception_tracker.get_exceptions_by_type_for_pair(pair)
            total_count = self.exception_tracker.get_total_count_for_pair(pair)
            if total_count > 0:
                print(f"\n{pair}: {total_count} errors, {len(by_type)} types")
                for exc_type, info in sorted(by_type.items()):
                    if len(by_type) > 1:
                        print(f"  {exc_type}: {info['count']} occurrences")
                    self.print_error_record(pair, info["example"])
        error_pairs = [
            p
            for p in self.exception_tracker.get_frame_pairs_with_errors()
            if self.exception_tracker.get_total_count_for_pair(p) > 0
        ]
        if error_pairs:
            print(f"\nTotal transforms with errors: {len(error_pairs)}")
            print("=" * SEPARATOR_WIDTH + "\n")

    def print_summary(self):
        if not self.timing_data:
            print("\nNo TF data collected.")
            return

        print("\n" + "=" * SEPARATOR_WIDTH)
        print("TF MONITORING SUMMARY - ALL TRANSFORMS")
        print("=" * SEPARATOR_WIDTH)

        for pair, data in sorted(self.timing_data.items()):
            if data["timestamps"]:
                self.print_transform_summary(pair, data)

        print("=" * SEPARATOR_WIDTH)
        self.print_clock_sync_info()
        self.print_error_report()


def main(args=None):
    import sys

    # Check for help flag
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print("Usage: tf_error_detector")
        print("\nMonitor TF transforms and detect timing issues")
        print("\nThis tool monitors /tf and /tf_static topics for 10 seconds and reports:")
        print("  - All TF frames and their publishers")
        print("  - Publishing rates")
        print("  - Clock synchronization issues")
        print("  - Transform lookup errors (connectivity, extrapolation, etc.)")
        sys.exit(0)

    rclpy.init(args=args)
    node = TFTimingMonitor()

    try:
        # Run until duration expires
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check if we've exceeded run duration
            current_time = node.get_clock().now()
            elapsed = (current_time.nanoseconds - node.start_time.nanoseconds) / 1e9
            if elapsed >= node.run_duration:
                print(f"\nRun duration ({node.run_duration}s) complete.")
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Print final summary report
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
