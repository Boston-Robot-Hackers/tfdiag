#!/usr/bin/env python3
# ros2diag_help.py - Help and information for the ros2diag package
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

def main(args=None):
    """Display help information for all ros2diag commands."""
    import sys

    print("ros2diag - ROS2 Diagnostic Tools")
    print("=" * 70)
    print("\nA collection of diagnostic tools for ROS2 TF transforms,")
    print("parameters, and network latency measurement.")
    print("\n" + "=" * 70)
    print("AVAILABLE COMMANDS")
    print("=" * 70)

    print("\nTF Transform Diagnostics:")
    print("  tf_lister [--static]")
    print("    List all TF frames in the system with their publishers and rates")
    print("    Use --static to include static transforms")
    print()
    print("  tf_checker")
    print("    Check predefined TF transform pairs for connectivity and freshness")
    print("    Reports availability, age, and transform chains")
    print()
    print("  tf_topics")
    print("    Monitor /tf and /tf_static topics")
    print("    Shows published transform pairs and their rates")
    print()
    print("  tf_timechecker")
    print("    Check message timestamps against ROS2 and system time")
    print("    Detects clock synchronization issues")
    print()
    print("  tf_error_detector")
    print("    Monitor TF transforms for 10 seconds and detect timing issues")
    print("    Reports lookup errors, clock sync problems, and publishing rates")
    print()

    print("Utility Tools:")
    print("  ros2_params <search_string> [timeout] [threads]")
    print("    Search ROS2 parameters across all nodes")
    print("    Example: ros2_params 'frame_id'")
    print()
    print("  net_latency <node1> <node2>")
    print("    Measure network latency between two ROS2 nodes")
    print("    Run on two machines/terminals with swapped arguments")
    print()

    print("=" * 70)
    print("USAGE")
    print("=" * 70)
    print("\nAll commands support --help for detailed usage:")
    print("  tf_lister --help")
    print("  ros2_params --help")
    print("  etc.")
    print()
    print("After sourcing your ROS2 workspace, commands are available directly:")
    print("  source ~/ros2_ws/install/setup.bash")
    print("  tf_lister --static")
    print("  tf_checker")
    print()

    print("=" * 70)
    print("PACKAGE INFORMATION")
    print("=" * 70)
    print("Package: ros2diag")
    print("Version: 0.0.0")
    print("License: MIT")
    print("Maintainer: pitosalas@gmail.com")
    print()
    print("For more information, visit the package documentation.")
    print("=" * 70)


if __name__ == "__main__":
    main()
