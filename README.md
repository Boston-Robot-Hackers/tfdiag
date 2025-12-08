# ros2diag - ROS2 Diagnostic Tools

A comprehensive collection of diagnostic tools for ROS2 TF transforms, parameters, and network latency measurement.

## Features

- **Standalone CLI commands** - All tools accessible directly without `ros2 run`
- **TF Transform diagnostics** - Monitor, analyze, and debug transform issues
- **Parameter searching** - Fast parallel search across all ROS2 parameters
- **Network latency measurement** - Measure communication delays between nodes
- **Built-in help** - Every command supports `--help`

## Installation

```bash
cd ~/ros2_ws/src
git clone <repository-url> ros2diag
cd ~/ros2_ws
colcon build --packages-select ros2diag
source install/setup.bash
```

## Quick Start

After sourcing your workspace, all commands are available directly:

```bash
# Show all available tools
ros2diag

# List TF frames
tf_lister --static

# Check TF transform pairs
tf_checker

# Search parameters
ros2_params 'frame_id'
```

## Available Commands

### Package Help
- **`ros2diag`** - Display comprehensive help for all tools

### TF Transform Diagnostics

- **`tf_lister [--static]`**
  List all TF frames with publishers and rates
  Use `--static` to include static transforms

- **`tf_checker`**
  Check predefined TF transform pairs
  Reports availability, age, and transform chains

- **`tf_topics`**
  Monitor /tf and /tf_static topics
  Shows published transform pairs and rates

- **`tf_timechecker`**
  Check message timestamps vs ROS2/system time
  Detects clock synchronization issues

- **`tf_error_detector`**
  Monitor TF for 10 seconds detecting timing issues
  Reports lookup errors, clock sync, publishing rates

### Utility Tools

- **`ros2_params <search_string> [timeout] [threads]`**
  Search ROS2 parameters across all nodes
  Example: `ros2_params 'frame_id'`

- **`net_latency <node1> <node2>`**
  Measure network latency between two nodes
  Run on two machines/terminals with swapped arguments

## Usage Examples

### Diagnose TF Issues

```bash
# Get overview of all TF frames
tf_lister

# Include static frames
tf_lister --static

# Check if common transform pairs are working
tf_checker

# Monitor what's being published
tf_topics

# Check for timestamp synchronization issues
tf_timechecker

# Run comprehensive 10-second monitoring
tf_error_detector
```

### Search Parameters

```bash
# Find all parameters containing "frame"
ros2_params 'frame'

# Search with custom timeout and threads
ros2_params 'topic' 3.0 16
```

### Measure Network Latency

```bash
# Terminal 1 (or machine 1):
net_latency node1 node2

# Terminal 2 (or machine 2):
net_latency node2 node1
```

## Getting Help

All commands support `--help`:

```bash
tf_lister --help
ros2_params --help
net_latency --help
# etc.
```

## Architecture

This package uses a **hybrid build system**:
- **ament_cmake** - For installing scripts to `bin/` (added to PATH)
- **ament_python** - For Python package management

Scripts are installed to `install/ros2diag/bin/` which is automatically added to PATH when you source the workspace.

See [md/ros2_cli_howto.md](md/ros2_cli_howto.md) for detailed implementation guide.

## Development

### Package Structure

```
ros2diag/
├── CMakeLists.txt              # Hybrid build configuration
├── package.xml                 # Package metadata
├── setup.py                    # Python package setup
├── ros2diag/                   # Python source
│   ├── ros2diag_help.py       # Main help command
│   ├── tf_lister.py           # TF frame lister
│   ├── tf_checker.py          # TF pair checker
│   ├── tf_topics.py           # TF topic monitor
│   ├── tf_timechecker.py      # Timestamp checker
│   ├── tf_error_detector.py   # TF error detector
│   ├── ros2_params.py         # Parameter searcher
│   └── net_latency.py         # Latency measurement
├── md/
│   ├── ros2_cli_howto.md      # CLI implementation guide
│   └── current.md             # Current status
└── test/                       # Unit tests
```

### Building

```bash
cd ~/ros2_ws
colcon build --packages-select ros2diag

# For development (symlinks):
colcon build --packages-select ros2diag --symlink-install
```

### Testing

```bash
cd ~/ros2_ws
colcon test --packages-select ros2diag
colcon test-result --verbose
```

## Requirements

- ROS2 (Jazzy or later)
- Python 3.10+
- rclpy
- tf2_ros
- tf2_tools

## License

MIT License

## Maintainer

Pito Salas <pitosalas@gmail.com>

## Contributing

See [md/ros2_cli_howto.md](md/ros2_cli_howto.md) for information on how the CLI commands are implemented.
