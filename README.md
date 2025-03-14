# Here are my personal notes on how to do things in ROS2 Humble!

## The tutorials I am following
| Tutorial                                                             | Status      |
|----------------------------------------------------------------------|-------------|
| [ROS2 Humble Tutorial](https://www.youtube.com/watch?v=Gg25GfA456o)  | Completed |
| [MoveIt Quickstart in RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | Completed |
| [Your First C++ Project](https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html) | Completed |
| [Create a URDF with ROS2](https://www.youtube.com/watch?v=dZ_CyyEvBE0) | In Progress |


## Table of Contents
* [The Basic Necessities](#the-basic-necessities)
  * [Installation](#installation)
  * [Creating a ROS2 Workspace](#creating-a-ros2-workspace)
  * [Creating a ROS2 Package](#creating-a-ros2-package)
  * [Creating a ROS2 Node](#creating-a-ros2-node)
* [General Information](#general-information)
  * [General Tools](#general-tools)
  * [All About Topics](#all-about-topics)
  * [All About Services](#all-about-services)
* [URDF Files](#urdf-files)

# The Basic Necessities

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
5. Add any dependencies from your new script to your `package.xml` under a depend tag.
6. Within your workspace root, run `colcon build` to compile your node.
    1. Optionally, to not need to run `colcon build` each time you make changes to your node, run `colcon build --symlink-install`.
    2. *This may not work with C++!*
7. At this point, to run your node via `ros2`, you will need to `source` your file again (or rerun your .bashrc via `. ~/.bashrc` if sources are defined there).
8. Finally, you can run your node with `ros2 run my_package example_node`!

That is it!


# General Information

## General Tools
* `rqt_graph` -> See running nodes and topics.
* `ros2 interface show <interface>` -> See the interfaces (essentially structs) that make up a given interface, and the interfaces that make those up, etc., all the way down to basic types.
* `ros2 node list` -> List all running nodes.

## All About Topics

### What are Topics?

Topics are to ROS2 what global event bus signals are to Godot 4.

### Commands for Topics

* `ros2 topic list` -> List all topics.
* `ros2 topic info /topic_name` -> See information about certain topics.
* `ros2 topic echo /topic_name` -> See traffic in a topic.
* `ros2 topic hz /topic_name` -> See the frequency of the topic publishing.

### Publishers

Publishers are what send information to topics (for [subscribers](#subscribers) to handle). To create a publisher:

* Within a pre-existing [node's](#creating-a-ros2-node) class script (in this case Python):
    1. In `__init__`, create a publisher: 
    ```python
    def __init__(self):
        ...
        self._my_publisher = self.create_publisher(Type, "topic_name", queue_size)
        ...
    ```
    2. Later in the code (say, in a timer-called function, or elsewhere), publish some data:
    ```python
    data = Type()
    # Be sure to update the type however necessary before publishing!
    self._my_publisher.publish(data)
    ```
* That is it!

### Subscribers

Subscribers are what catch information sent to topics (by [publishers](#publishers)). To create a subscriber:

* Within a pre-existing [node's](#creatings-a-ros2-node) class script (in this case Python):
    1. In `__init__`, create a subscription:
    ```python
    def __init__(self):
        ...
        self._my_subscriber = self.create_subscription(Type, "topic_name", self.callback_function, queue_size)
        ...
    ```
    2. Then you need to write your callback function in your class:
    ```python
    def callback_function(self, msg: Type):
        # Handle the msg as necessary.
    ```
* That is it!

## All About Services

Services are for client-server relationships, where one (client) node sends a request to another (server) node, who then responds. [Topics](#all-about-topics) are not built for these sorts of relationships.
There is only ever one service for a particular thing (like there is one server for example.com) but there can be many clients sending requests to that one server.
There are primarily two uses for services:
1. Doing computations (i.e., given `a` and `b`, responds with `sum=a+b`)
2. Managing one particular thing. For example, changing settings (i.e., a "settings" service that takes requests from other nodes to change particular settings) (in the same way that one server manages the website for example.com).

### Commands for Services
* `ros2 service list` -> List the currently running services.
* `ros2 service type /service_name` -> List the service's interface(s).
* `ros2 service call /service_name interface_name "{'param1': val, 'param2': val, ...}"` -> Call a (running) service manually and get a response.

### Using Services
#### Create a Client

To create a client: `client = self.create_client(Type, "service_name")`
To use a client:
```python
# To wait for service (logging every second)
while not client.wait_for_service(1.0):
    self.get_logger().warn("Waiting for service...")

request = Type.Request()
# Set the members of the type here

# Send the request (asynchronously; allows us to not be blocked by failed request)
future = client.call_async(request)
# Ensure `from functools import partial`; set the callback function to call when request received.
future.add_done_callback(partial(self.my_callback_function))
```

Within `my_callback_function`:
```python
def my_callback_function(self, future):
    try:
        response = future.result()
    except Exception as e:
        self.get_logger().error("Service call failed: "+str(e))
    # We can now deal with the response if applicable.
```

# URDF Files
