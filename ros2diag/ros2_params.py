#!/usr/bin/env python3
"""
ROS2 parameter finder script with timeout handling and multithreading.
Finds all nodes, queries their parameters in parallel, and searches for matching strings.
"""

import subprocess
import sys
import threading
import time
from queue import Queue
from threading import Lock

import rclpy
from rclpy.node import Node


class ParameterFinder(Node):
    def __init__(self):
        super().__init__("parameter_finder")
        self.timeout_seconds = 2.0
        self.max_threads = 8
        self.results_lock = Lock()

    def get_all_nodes(self):
        """Get all running nodes using ros2 node list command."""
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                nodes = set(
                    line.strip() for line in result.stdout.split("\n") if line.strip()
                )
                return nodes
            self.get_logger().error("Failed to get node list")
            return set()
        except subprocess.TimeoutExpired:
            self.get_logger().error("Timeout getting node list")
            return set()
        except Exception as e:
            self.get_logger().error(f"Error getting node list: {e}")
            return set()

    def get_node_parameters_threaded(self, node_name):
        """Get all parameters for a specific node with timeout."""
        parameters = {}

        # Use ros2 param list to get parameter names
        try:
            result = subprocess.run(
                ["ros2", "param", "list", node_name],
                check=False,
                capture_output=True,
                text=True,
                timeout=self.timeout_seconds,
            )

            if result.returncode != 0:
                return parameters

            param_names = [
                line.strip() for line in result.stdout.split("\n") if line.strip()
            ]

            # Get each parameter value - limit concurrency to avoid overwhelming ROS2
            param_results = {}
            param_lock = Lock()
            max_param_threads = 5

            def get_param(pname):
                value = self.get_parameter_value_with_timeout(node_name, pname)
                if value is not None:
                    with param_lock:
                        param_results[pname] = value

            for i in range(0, len(param_names), max_param_threads):
                batch = param_names[i:i + max_param_threads]
                param_threads = []

                for param_name in batch:
                    t = threading.Thread(target=get_param, args=(param_name,), daemon=True)
                    t.start()
                    param_threads.append(t)

                # Wait for batch to complete
                for t in param_threads:
                    t.join(timeout=self.timeout_seconds + 1)

            return param_results

        except subprocess.TimeoutExpired:
            return parameters
        except Exception:
            return parameters

    def get_parameter_value_with_timeout(self, node_name, param_name):
        """Get a single parameter value with timeout handling."""
        try:
            start = time.time()
            result = subprocess.run(
                ["ros2", "param", "get", node_name, param_name],
                check=False,
                capture_output=True,
                text=True,
                timeout=self.timeout_seconds,
            )
            elapsed = time.time() - start

            if result.returncode == 0:
                output = result.stdout.strip()
                if ":" in output:
                    return output.split(":", 1)[1].strip()
                return output

            if elapsed > self.timeout_seconds * 0.9:
                self.get_logger().debug(f"Slow param {node_name}/{param_name}: {elapsed:.2f}s")

            return None

        except subprocess.TimeoutExpired:
            self.get_logger().debug(f"Timeout {node_name}/{param_name} after {self.timeout_seconds}s")
            return None
        except Exception as e:
            self.get_logger().debug(f"Error {node_name}/{param_name}: {e}")
            return None

    def query_nodes_threaded(self, nodes, progress_callback):
        """Query all nodes in parallel using worker threads."""
        all_parameters = {}
        work_queue = Queue()

        # Add all nodes to work queue
        for node_name in nodes:
            work_queue.put(node_name)

        completed = {"count": 0}
        total = len(nodes)

        def worker():
            while True:
                try:
                    node_name = work_queue.get_nowait()
                except:
                    break

                progress_callback("querying", node_name, None, completed["count"], total)
                params = self.get_node_parameters_threaded(node_name)

                with self.results_lock:
                    if params:
                        all_parameters[node_name] = params
                    completed["count"] += 1
                    progress_callback("completed", node_name, params, completed["count"], total)

                work_queue.task_done()

        # Create and start worker threads
        threads = []
        for _ in range(min(self.max_threads, total)):
            t = threading.Thread(target=worker, daemon=True)
            t.start()
            threads.append(t)

        # Wait for all threads to complete
        for t in threads:
            t.join()

        return all_parameters

    def search_parameters(self, search_string, parameters_dict):
        """Search through all parameters and print matches in table format."""
        search_lower = search_string.lower()
        matches_found = False

        for node_name in sorted(parameters_dict.keys()):
            params = parameters_dict[node_name]
            first_match = True
            for param_name, param_value in sorted(params.items()):
                if (
                    search_lower in param_name.lower()
                    or search_lower in str(param_value).lower()
                ):
                    node_col = node_name if first_match else ""
                    print(f"{node_col:<35} {param_name:<35} {param_value!s:<30}")
                    first_match = False
                    matches_found = True

        if not matches_found:
            print(f"No parameters found matching: '{search_string}'")


def main(args=None):
    # Check for help flag first
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print("Usage: ros2_params <search_string> [timeout_seconds] [max_threads]")
        print("\nSearch ROS2 parameters across all nodes")
        print("\nArguments:")
        print("  search_string     String to search for in parameter names and values")
        print("  timeout_seconds   Timeout per parameter query (default: 2.0)")
        print("  max_threads       Maximum parallel threads (default: 8)")
        print("\nExamples:")
        print("  ros2_params 'frame_id'")
        print("  ros2_params 'topic' 3.0 16")
        sys.exit(0)

    if len(sys.argv) < 2:
        print(
            "Usage: ros2_params <search_string> [timeout_seconds] [max_threads]"
        )
        print("\nExample:")
        print("  ros2_params 'frame_id'")
        print("  ros2_params 'topic' 3.0 16")
        print("\nUse --help for more information")
        sys.exit(1)

    search_string = sys.argv[1]
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    max_threads = int(sys.argv[3]) if len(sys.argv) > 3 else 8

    rclpy.init()

    finder = ParameterFinder()
    finder.timeout_seconds = timeout
    finder.max_threads = max_threads

    print(f"Searching for parameters matching: '{search_string}'")
    print(f"Timeout per parameter: {timeout}s")
    print(f"Max threads: {max_threads}")
    print("Querying all nodes in parallel...\n")

    # Get all nodes
    nodes = finder.get_all_nodes()

    if not nodes:
        print("No nodes found running")
        rclpy.shutdown()
        sys.exit(1)

    print(f"Found {len(nodes)} nodes.\n")

    status_lock = Lock()
    in_progress = 0
    completed_count = 0
    matching_count = 0

    def progress_callback(status, node_name, params, completed, total):
        nonlocal in_progress, completed_count, matching_count
        with status_lock:
            if status == "querying":
                in_progress += 1
            elif status == "completed":
                in_progress -= 1
                completed_count += 1
                if params:
                    search_lower = search_string.lower()
                    for param_name, param_value in params.items():
                        if (
                            search_lower in param_name.lower()
                            or search_lower in str(param_value).lower()
                        ):
                            matching_count += 1
                            break
            print(
                f"\rStatus: total nodes: {total}, in progress: {in_progress}, "
                f"completed: {completed_count}, matching query: {matching_count}",
                end="",
                flush=True
            )

    all_parameters = finder.query_nodes_threaded(sorted(nodes), progress_callback)

    print("\n")  # Move to next line after status updates

    # Print all parameters in table format
    print(f"{'=' * 100}")
    print(f"{'Node':<35} {'Parameter':<35} {'Value':<30}")
    print(f"{'=' * 100}")

    for node_name in sorted(all_parameters.keys()):
        params = all_parameters[node_name]
        if params:
            first = True
            for param_name, param_value in sorted(params.items()):
                node_col = node_name if first else ""
                print(f"{node_col:<35} {param_name:<35} {param_value!s:<30}")
                first = False
            print(f"{'-' * 100}")

    total_params = sum(len(params) for params in all_parameters.values())
    print(f"Total: {len(all_parameters)} nodes, {total_params} parameters")
    print(f"{'=' * 100}\n")

    # Search and print matching results
    print(f"\n{'=' * 100}")
    print(f"Search Results for: '{search_string}'")
    print(f"{'=' * 100}")
    print(f"{'Node':<35} {'Parameter':<35} {'Value':<30}")
    print(f"{'=' * 100}")

    finder.search_parameters(search_string, all_parameters)

    # Print summary
    total_params = sum(len(params) for params in all_parameters.values())
    print(f"\n{'=' * 70}")
    print(
        f"Summary: {len(all_parameters)} nodes, {total_params} total parameters queried"
    )
    print(f"{'=' * 70}\n")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
