# Example Package ROS


This ROS2 package contains nodes that utilize the `Example` class from the ROS-independent `example_package`.

## Prerequisites

Ensure you have [ROS2 installed](https://docs.ros.org/en/iron/Installation.html). This guide also assumes you have set up a [ROS workspace](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) which will be named `your_ros_workspace` for the purpose of this guide.

Example to create a workspace (in the example, using the `iron` ROS2 distribution):
```
# create your workspace and navigate to it
mkdir -p your_ros_workspace/src
cd your_ros_workspace

# source your ROS2 distribution
source /opt/ros/iron/setup.bash

# build the empty workspace
colcon build

# source the Workspace's Setup Script
source install/local_setup.bash
```

## Dependencies

- **example_package**: ROS-independent package containing the `Example` class.

## Building the Package

The ROS package requires the `example_package`, which you may have installed in the virtual environment.
However, when using a virtual environment together with ROS, note:

> **Important**: when using virtual environments with ROS2, as it might not recognize packages installed in a virtual environment. It's often recommended to install Python packages system-wide or in the same environment as ROS2.

If you have installed `example_package` system-wide, then you only need to add `example_package_ros` to your ROS workspace. If you haven't, it is recommended
to **not** use the virtual environment, and instead put the `example_package` in the ROS workspace as well.

> **Note**: See separate section below for a guide on how to install a package to your ROS2 environment. In short, it requires the package to have a `package.xml`. Conveniently, the `example_package` already has a package.xml so you can simply include it in your ROS workspace.

Move the `example_package_ros` directory (or create a softlink to it) to your ROS workspace (if it is not already there):

```
cd <your-ros-workspace>/src
ln -s <path-to>/example_package_ros
```

Do the same for `example_package` if you haven't  installed it system-wide.

Build the ROS package:

```bash
# From the root of your ROS workspace
cd <your-ros-workspace>
colcon build
```

And source the setup script:

```bash
source install/local_setup.bash
```

Note that you will need to source your ROS workspace (`source <your_ros_workspace>/install/local_setup.bash`) each time before you run the ROS nodes.
Tipp: You may want to put the sourcing command into your `.bashrc`.

## Running the Nodes

> You will need to source your ROS workspace (`source <your_ros_workspace>/install/local_setup.bash`) each time before you run the ROS nodes.

Launch `ExampleNode` with default parameters:

```bash
ros2 launch example_package_ros example_node_launch.py
```

Specify a custom message:

```bash
ros2 launch example_package_ros example_node.launch.py message:="Custom Message"
```

From another terminal, you may call the service:

```bash
ros2 service call /example_service example_package_msgs/srv/Example "{request_message: 'Test message'}"
```

... or the action:

```bash
ros2 action send_goal /example_action example_package_msgs/action/Example "{goal_message: 'Start the action'}"
```

Launch the other example `AdvancedExampleNode` (brings up several nodes that interact):

```bash
ros2 launch example_package_ros advanced_example_node.launch.py
```

## Running Tests

```bash
# From the root of your ROS workspace
colcon test --packages-select example_package_ros
colcon test-result --verbose
```



# Appendix

### [optional] Installing `example_package` into the ROS Environment (instead of the virtual)

To ensure that `example_package` is available in the ROS environment, you can include it in your ROS workspace and build it using `colcon`. This approach integrates the package into the ROS build system and avoids issues with virtual environments.

#### Step 1: Place `example_package` in Your ROS Workspace

- Clone this repository (or create a softlink to it) to your ROS workspace's `src` directory

  ```
  your_ros_workspace/
  └── src/
      ├── example_package/        # The ROS-independent package
      ├────── example_package/        # The ROS-independent package
      └────── example_package_ros/    # The ROS-dependent package
  ```

#### Step 2: Add a Minimal `package.xml` to `example_package`

Since `colcon` builds packages based on the presence of a `package.xml` file, you need to add one to `example_package`.

- **Create `example_package/package.xml`:**

  ```xml
  <?xml version="1.0"?>
  <package format="3">
    <name>example_package</name>
    <version>0.0.1</version>
    <description>ROS-independent example package</description>
    <maintainer email="somename@example.com">A Name</maintainer>
    <license>TODO: License</license>
    <buildtool_depend>ament_python</buildtool_depend>
    <export></export>
  </package>
  ```

#### Step 3: Update `setup.py` in `example_package`

Ensure that `setup.py` in `example_package` is configured correctly for ROS builds.

- **Add an empty `entry_points` section to your `setup.py` if it's not already present.**

  This helps `colcon` recognize and build the package correctly.

  **Modification to `setup.py`:**

  Add the following argument to the `setup()` function:

  ```python
  entry_points={
      'console_scripts': [],
  },
  ```

  Place this after your existing arguments within the `setup()` function.

- **Example Change:**

  ```diff
  setup(
      name=package_name,
      version='0.0.1',
      packages=[package_name],
      install_requires=[
          'setuptools',
          'pyfiglet>=0.8.post1',
      ],
      zip_safe=True,
      author='Your Name',
      author_email='you@example.com',
      description='ROS-independent example package',
      license='TODO',
  +   entry_points={
  +       'console_scripts': [],
  +   },
  )
  ```


#### Step 4: Build the Workspace

```bash
# Navigate to the root of your ROS workspace
cd your_ros_workspace

# Build the workspace
colcon build
```

This will build both `example_package` and `example_package_ros` and install them into the ROS environment.

#### Step 5: Source the Setup Script

```bash
source install/setup.bash
```

Now, both packages should be available in your ROS environment.


