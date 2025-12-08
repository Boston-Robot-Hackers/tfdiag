# ros2diag

## Context

A comprehensive collection of diagnostic tools for ROS2 TF transforms, parameters, and network latency measurement. All tools are accessible as standalone CLI commands.

## Current Status

The package provides 8 standalone CLI commands:

### Package Help
- **ros2diag** - Display comprehensive help for all tools

### TF Transform Diagnostics
- **tf_lister** - List all TF frames with publishers and rates
- **tf_checker** - Check predefined TF transform pairs
- **tf_topics** - Monitor /tf and /tf_static topics
- **tf_timechecker** - Check message timestamps vs ROS2/system time
- **tf_error_detector** - Monitor TF for 10 seconds detecting timing issues

### Utility Tools
- **ros2_params** - Search ROS2 parameters across all nodes
- **net_latency** - Measure network latency between two nodes

All commands support `--help` for detailed usage information.

## Implementation

**Standalone CLI Architecture:** Each tool is a self-contained Python script with its own `main()` function and argument parsing. No external CLI framework dependencies.

**Usage Pattern:**
```bash
# Direct command execution (no ros2 run needed)
tf_lister --static
tf_checker
ros2_params 'frame_id'
net_latency node1 node2
```

## Architecture

### Hybrid Build System
- **ament_cmake** - Installs scripts to `bin/` (automatically added to PATH)
- **ament_python** - Python package management
- **CMakeLists.txt** - Configures and installs all CLI commands
- **setup.py** - Python package setup (no console_scripts needed)

### Script Installation
Scripts are installed to `install/ros2diag/bin/` which colcon automatically adds to PATH when you source the workspace. This allows direct command execution without `ros2 run`.

### Module Structure
All diagnostic modules are in `ros2diag/` directory:
- `ros2diag_help.py` - Main package help
- `tf_lister.py` - TF frame listing
- `tf_checker.py` - TF pair checking
- `tf_topics.py` - TF topic monitoring
- `tf_timechecker.py` - Timestamp checking
- `tf_error_detector.py` - TF error detection
- `ros2_params.py` - Parameter search
- `net_latency.py` - Latency measurement

## Recent Changes

### Major Refactoring (Current)
- Removed cyclopts dependency and main.py entry point
- Converted all modules to standalone CLI commands
- Implemented hybrid ament_cmake/ament_python build
- Added --help support to all commands
- Created ros2diag help command for package overview

### Previous Updates
- Integrated `tf_lister_accurate.py` logic into `tf_lister.py`
- Removed obsolete IMPLEMENTATION_NOTES.md

## Reference Documentation
* [ros2_cli_howto.md](ros2_cli_howto.md) - Guide for implementing CLI commands in ROS2
* [rules.md](rules.md) - Coding standards
* [TF2 Introduction](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
* [TF2 BufferCore API](https://docs.ros2.org/foxy/api/tf2/classtf2_1_1BufferCore.html)