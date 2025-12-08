#!/usr/bin/env python3
# tf_checker.py - Checks specific TF transform pairs for problems
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import time

import rclpy
import tf2_ros
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

TF_PAIRS_TO_CHECK = [
    ("odom", "base_link"),
    ("odom", "base_footprint"),
    ("odom", "laser"),
    ("odom", "imu"),
    ("base_link", "base_footprint"),
    ("base_link", "laser"),
    ("base_link", "imu"),
    ("base_footprint", "laser"),
    ("base_footprint", "imu"),
    ("map", "odom"),
]

MAX_AGE_WARNING = 1.0
MAX_AGE_ERROR = 5.0
SAMPLE_DURATION = 2.0


class TfChecker(Node):
    """Checks health of specific TF transform pairs."""

    def __init__(self):
        super().__init__("tf_checker")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def check_transform_pair(self, source_frame, target_frame):
        """Check a single transform pair and return status."""
        result = {
            "source": source_frame,
            "target": target_frame,
            "available": False,
            "age": None,
            "chain": [],
            "error": None,
        }

        try:
            # Use current time for lookup to get accurate timestamps
            # (using time zero causes static transforms to return with timestamp=0)
            lookup_time = self.get_clock().now()
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, lookup_time
            )

            result["available"] = True

            current_time = time.time()
            transform_time = (
                transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            )
            result["age"] = current_time - transform_time

            chain = self._tf_buffer._getFrameStrings()
            result["chain"] = self._build_chain(source_frame, target_frame, chain)

        except LookupException as e:
            result["error"] = f"Lookup failed: {e!s}"
        except ConnectivityException as e:
            result["error"] = f"Connectivity error: {e!s}"
        except ExtrapolationException as e:
            result["error"] = f"Extrapolation error: {e!s}"
        except Exception as e:
            result["error"] = f"Unknown error: {e!s}"

        return result

    def _build_chain(self, source, target, all_frames):
        """Build the transform chain from source to target."""
        try:
            # _chainAsVector returns the complete chain from source to target
            # Parameters: target_frame, target_time, source_frame, source_time, fixed_frame
            chain = self._tf_buffer._chainAsVector(
                target, rclpy.time.Time(), source, rclpy.time.Time(), target
            )
            if chain and len(chain) > 0:
                return list(chain)
            # If _chainAsVector returns empty, just show source and target
            return [source, target]
        except Exception as e:
            # If chain lookup fails, try to at least show the direct connection
            self.get_logger().debug(f"Chain lookup failed for {source} -> {target}: {e}")
            return [source, target]

    def check_all_pairs(self):
        """Check all configured TF pairs."""
        print(f"Sampling TFs for {SAMPLE_DURATION} seconds...")
        start_time = time.time()
        while time.time() - start_time < SAMPLE_DURATION:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("\nChecking TF Transform Pairs")
        print("=" * 50)
        print()

        results = []
        for source, target in TF_PAIRS_TO_CHECK:
            result = self.check_transform_pair(source, target)
            results.append(result)
            self._print_pair_result(result)

        self._print_summary(results)

    def _format_age(self, seconds):
        """Format age in seconds as hh:mm:ss.ms."""
        abs_seconds = abs(seconds)
        hours = int(abs_seconds // 3600)
        minutes = int((abs_seconds % 3600) // 60)
        secs = abs_seconds % 60
        sign = "-" if seconds < 0 else ""
        return f"{sign}{hours:02d}:{minutes:02d}:{secs:06.3f}"

    def _print_pair_result(self, result):
        """Print the result for a single pair."""
        print(f"Pair: {result['source']} → {result['target']}")

        if result["available"]:
            print("  ✓ Transform available")

            age = result["age"]
            if age < MAX_AGE_WARNING:
                status = "GOOD"
                symbol = "✓"
            elif age < MAX_AGE_ERROR:
                status = "STALE"
                symbol = "⚠"
            else:
                status = "OLD"
                symbol = "✗"

            age_str = self._format_age(age)
            print(f"  {symbol} Age: {age_str} ({status})")

            if result["chain"]:
                chain_str = " → ".join(result["chain"])
                print(f"  → Chain: {chain_str}")
        else:
            print("  ✗ Transform NOT available")
            if result["error"]:
                print(f"  → Error: {result['error']}")

        print()

    def _print_summary(self, results):
        """Print summary of all checks."""
        total = len(results)
        available = sum(1 for r in results if r["available"])
        warnings = sum(
            1
            for r in results
            if r["available"] and MAX_AGE_WARNING <= r["age"] < MAX_AGE_ERROR
        )
        errors = sum(
            1
            for r in results
            if not r["available"] or (r["available"] and r["age"] >= MAX_AGE_ERROR)
        )

        print("=" * 50)
        print(f"Summary: {available}/{total} pairs available", end="")
        if warnings > 0:
            print(f", {warnings} warning(s)", end="")
        if errors > 0:
            print(f", {errors} error(s)", end="")
        print()


def check_tf_pairs():
    """Check configured TF transform pairs."""
    rclpy.init()
    checker = TfChecker()
    checker.check_all_pairs()
    checker.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """Main entry point for CLI."""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print("Usage: tf_checker")
        print("\nCheck TF transform pairs for connectivity and freshness")
        print("\nThis tool checks predefined transform pairs and reports:")
        print("  - Availability of transforms")
        print("  - Age of transform data")
        print("  - Transform chain from source to target")
        sys.exit(0)

    check_tf_pairs()


if __name__ == "__main__":
    main()
