# How to Create CLI Commands in ROS2 Python Packages

## Objective

This guide explains how to create Python-based command-line tools in a ROS2 package that can be run directly from the shell (like `my_command`) instead of requiring `ros2 run package_name my_command`. This makes your tools more convenient and intuitive to use.

**What you'll achieve:**
- Create Python scripts with ROS2 functionality
- Make them accessible directly from the command line after sourcing your workspace
- Automatically add them to PATH using colcon's hook system

## Background

By default, ROS2 Python packages use `console_scripts` entry points in `setup.py`, which installs executables to `install/<package>/lib/<package>/`. This directory is NOT automatically added to PATH, so you must use `ros2 run` to execute them.

The solution is to use a **hybrid package** approach (combining ament_cmake and ament_python) to install scripts to `install/<package>/bin/`, which colcon automatically adds to PATH.

## Step 1: Write Your Python Script

Create your Python script as you normally would for a ROS2 package. The key requirements:

1. **Shebang line**: Start with `#!/usr/bin/env python3`
2. **Main function**: Define a `main(args=None)` function
3. **ROS2 initialization**: Call `rclpy.init(args=args)` in your main function
4. **Module structure**: Can import from your package using standard Python imports

**Example:** `ros2diag/my_tool.py`

```python
#!/usr/bin/env python3
"""
My ROS2 diagnostic tool
"""

import sys
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: my_tool <argument>")
        sys.exit(1)

    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Step 2: Create/Update setup.py

Keep your standard `setup.py` for Python package installation:

```python
from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Package description',
    license='License',
    entry_points={
        'console_scripts': [
            # Optional: you can still define console_scripts
            # but they won't be in PATH automatically
        ],
    },
)
```

## Step 3: Create CMakeLists.txt

Create a `CMakeLists.txt` file in your package root. This enables the hybrid build approach:

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (from setup.py)
ament_python_install_package(${PROJECT_NAME})

# Copy scripts and install to bin/ directory
# This directory automatically gets added to PATH by colcon
configure_file(my_package/my_tool.py ${CMAKE_CURRENT_BINARY_DIR}/my_tool COPYONLY)
configure_file(my_package/another_tool.py ${CMAKE_CURRENT_BINARY_DIR}/another_tool COPYONLY)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/my_tool
  ${CMAKE_CURRENT_BINARY_DIR}/another_tool
  DESTINATION bin
)

ament_package()
```

**Key points:**
- `configure_file()` copies your `.py` files and renames them (removes `.py` extension)
- `install(PROGRAMS ...)` marks them as executable and installs to `bin/`
- `DESTINATION bin` is the magic - colcon creates PATH hooks for this directory

## Step 4: Update package.xml

Update your `package.xml` to declare the package as `ament_cmake` (not `ament_python`):

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>Package description</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>License</license>

  <!-- Build tool dependencies for hybrid package -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <!-- Your package dependencies -->
  <depend>rclpy</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <!-- IMPORTANT: Must be ament_cmake, not ament_python -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Key change:** `<build_type>ament_cmake</build_type>` instead of `ament_python`

## Step 5: Build Your Package

Build your package with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```

After building, check that:
1. Scripts are installed: `ls install/my_package/bin/`
2. PATH hook exists: `ls install/my_package/share/my_package/hook/path.sh`

## Step 6: Use Your CLI Commands

Source your workspace and run commands directly:

```bash
source ~/ros2_ws/install/setup.bash

# Now you can run directly:
my_tool argument1
another_tool argument2

# No need for:
# ros2 run my_package my_tool argument1
```

## How It Works

1. **CMake copies and installs scripts** to `install/<package>/bin/`
2. **Colcon automatically generates hooks** when files are installed to `bin/`:
   - Creates `install/<package>/share/<package>/hook/path.sh`
   - This hook contains: `_colcon_prepend_unique_value PATH "$COLCON_CURRENT_PREFIX/bin"`
3. **When you source the workspace**, the setup script sources all package hooks
4. **Your bin directory is added to PATH**, making commands available directly

## Directory Structure

After following this guide, your package structure will look like:

```
my_package/
├── CMakeLists.txt           # NEW: Enables hybrid build
├── package.xml              # MODIFIED: build_type is ament_cmake
├── setup.py                 # Unchanged: standard Python setup
├── setup.cfg
├── resource/
│   └── my_package
├── my_package/
│   ├── __init__.py
│   ├── my_tool.py           # Your CLI script
│   └── another_tool.py      # Another CLI script
└── test/
```

And after building:

```
install/my_package/
├── bin/                     # NEW: Your executables in PATH
│   ├── my_tool
│   └── another_tool
├── lib/
│   └── python3.12/site-packages/
│       └── my_package/      # Your Python package
└── share/my_package/
    └── hook/
        ├── path.sh          # NEW: Auto-generated PATH hook
        ├── pythonpath.sh
        └── ...
```

## Troubleshooting

**Commands not found after sourcing?**
- Check that scripts were installed: `ls install/<package>/bin/`
- Check that PATH hook exists: `cat install/<package>/share/<package>/hook/path.sh`
- Verify hook adds bin to PATH: should contain `_colcon_prepend_unique_value PATH "$COLCON_CURRENT_PREFIX/bin"`
- Source workspace in a new shell to ensure hooks are loaded

**Python imports not working?**
- Make sure `ament_python_install_package(${PROJECT_NAME})` is in CMakeLists.txt
- Check that package is installed: `ls install/<package>/lib/python3.*/site-packages/<package>/`

**Script has wrong permissions?**
- Use `install(PROGRAMS ...)` not `install(FILES ...)` in CMakeLists.txt
- Ensure source `.py` files have executable permission: `chmod +x my_package/*.py`

## Real-World Example

This technique is used by the `better_launch` package in the ROS2 ecosystem. Check its implementation:
- [better_launch CMakeLists.txt](https://github.com/v4hn/better_launch/blob/main/CMakeLists.txt)

## Summary

The hybrid package approach (ament_cmake + ament_python) allows you to:
- Write Python code as normal
- Install executables to `bin/` via CMake
- Automatically get PATH hooks from colcon
- Run commands directly from the shell

This makes your ROS2 tools more accessible and user-friendly, following the same pattern as system tools and other CLI utilities.
