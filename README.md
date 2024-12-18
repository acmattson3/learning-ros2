# How to do ROS2 (Humble) things!

## Installation

Visit [the installation documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (or the most recent documentation online).

## Creating a ROS2 Workspace

Creating a workspace is the first step to doing things in ROS2. Workspaces are what you can share with others so they can use what you make or collaborate.

1. Make a directory for your workspace (i.e., `mkdir ros2_ws`).
2. Make the source folder for this workspace (i.e., `cd ros2_ws`, then `mkdir src`).
3. Build the workspace (`colcon build` in the ros2_ws directory). This will initialize the workspace, creating some other directories and files.
4. To use the contents of this workspace across your environment, you must source the file (i.e., `source .../ros2_ws/install/setup.bash`).
    1. Optionally, add this source line to your ~/.bashrc file to automatically have access to this workspace on startup.

That is it! You have created a ROS2 workspace. You can now put this workspace into a GitHub repository to collaborate with others if you would like.


## Creating a ROS2 Package

**Important:** This follows how to do things in Python. Look for notes below on how to do things in C++.

Packages are fundamental to ROS. They are what allow you to implement custom functionalities like controlling robots via single commands.

**Note:** This requires that you are in a [previously created workspace](#creating-a-ros2-workspace).

1. Initialize the package (`ros2 pkg create my_package_name --build-type ament_python --dependencies <dep_name(s)>`).
    1. `ament_python` may be replaced with `ament_cmake` for C++ packages.
    2. A common dependency (probably always used) is `rclpy`. *You should add this as a dependency*.
2. You should now have your package in the src folder. Now, build the project again (project root, `colcon build`). You should see "Summary: 1 package finished"

That's it! It isn't very useful yet, so check out [how to create a node](#creating-a-ros2-node).


## Creating a ROS2 Node

**Important:** This follows how to do things in Python. Syntax will differ in C++.

Nodes are the building blocks of packages. They are what you run to do things like send/receive messages or commands.

**Note:** This requires that you have created both [a workspace](#creating-a-ros2-workspace) and [a package](#creating-a-ros2-package) within that workspace.

1. Within your package directory, enter the directory whose name is the same as your package (i.e., `cd ros2_ws/src/my_package/my_package`).
2. Create a new python file (i.e., `touch my_node.py`).
    1. Within your python script, add a shebang line (usually `#!/usr/bin/env python3` or `python3.10` in my case).
    2. Make your script executable (`sudo chmod +x my_node.py`) so it uses the interpreter you specified.
3. Write your script based on the boilerplate below:
    ```python
    #!/usr/bin/env python3.10
    import rclpy
    from rclpy.node import Node

    # The place where your node's logic goes
    class MyNode(Node):
        def __init__(self):
            super().__init__("first_node") # You can name the node to your liking
            self.get_logger().info("Hello from ROS2!") # Output to terminal
    
    # The function you want called
    def main(args=None):
        rclpy.init(args=args)
        
        node = MyNode()
        rclpy.spin(node) # Keep the node alive
        
        rclpy.shutdown()

    if __name__ == "__main__":
        main()
    ```
4. Within your package root, in `setup.py`, under the `console_scripts` list, add an element: `"example_node = my_package.my_node:main"`
    1. Syntax here is `ros2_executable_name = package_name.node_name:function_name`
    2. You can test your code before moving on by doing `./my_node.py`.
5. Within your workspace root, run `colcon build` to compile your node.
    1. Optionally, to not need to run `colcon build` each time you make changes to your node, run `colcon build --symlink-install`.
    2. *This may not work with C++!*
6. At this point, to run your node via `ros2`, you will need to `source` your file again (or rerun your .bashrc via `. ~/.bashrc` if sources are defined there).
7. Finally, you can run your node with `ros2 run my_package example_node`!

That is it!
