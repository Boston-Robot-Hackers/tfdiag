# ros2diag

## Context

A CLI tool for ROS2 that analyzes existing TF transforms and identifies problems. Uses cyclopts for the command-line interface.

## Current Status

The tool successfully lists TF frames with the following features:
- Displays frame hierarchy (parent/child relationships)
- Shows publisher nodes for each frame
- Reports frame rates and buffer information
- Supports filtering static frames via `--include-static` flag

## Implementation

**Single implementation approach:** The improved publisher detection logic from `tf_lister_accurate.py` has been integrated into `tf_lister.py` to provide a more accurate and robust solution.
- `tf_lister_accurate.py` has been removed.

**CLI options:**
- `ros2 run ros2diag ros2diag list` - List dynamic TF frames only (default)
- `ros2 run ros2diag ros2diag list --include-static` - Include static frames

## Architecture

- **main.py** - Cyclopts CLI entry point
- **tf_lister.py** - TF frame detection and display logic
- Samples TF data for 5 seconds to gather comprehensive frame information

## Reference Documentation
* [rules.md](rules.md) - Coding standards
* [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md) - Technical implementation details
* [Cyclopts docs](https://cyclopts.readthedocs.io)
* [TF2 Introduction](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
* [TF2 BufferCore API](https://docs.ros2.org/foxy/api/tf2/classtf2_1_1BufferCore.html)