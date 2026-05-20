# Python Node Template

## `<pkg>/<node>_py.py`

```python
#!/usr/bin/env python3
# The shebang is required. Without it, the installed script won't be executable
# and `ros2 run <pkg> <node>_py.py` will fail with "not found".

import os
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from interfaces.msg import VCON as MyMsg
# from std_msgs.msg import Float64


class <Node>Publisher(Node):
    def __init__(self):
        super().__init__("<node>_py")

        self.load_config()

        self.publisher_ = self.create_publisher(MyMsg, "<topic/name>", 10)
        self.create_timer(1.0, self.run)   # seconds between callbacks

    def load_config(self):
        # get_package_share_directory resolves to the install space, so this
        # works regardless of where the workspace is sourced.
        pkg_share = get_package_share_directory("<pkg>")
        file_path = os.path.join(pkg_share, "config.yaml")

        with open(file_path, "r") as f:
            data = yaml.safe_load(f)

        # Example: read a nested value.
        # self.my_param = float(data["section"]["key"])

        self.get_logger().info(f"Config loaded from: {file_path}")

    def run(self):
        msg = MyMsg()
        # populate msg fields here
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = <Node>Publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## Subscriber Variant

Add to `__init__`:

```python
self.subscription_ = self.create_subscription(
    IncomingMsg,
    "<input_topic>",
    self.on_message,
    10
)
```

And the callback:

```python
def on_message(self, msg):
    # process msg
    pass
```

## Python `__init__.py`

The `<pkg>/__init__.py` file should be empty (or nearly so). Its only purpose is to mark the directory as a Python package so that `ament_python_install_package` can register it:

```python
# intentionally empty
```

## Logging Conventions

Use `self.get_logger()` — never `print()`. The logger integrates with `ros2 topic echo /rosout` and supports runtime severity filtering.

```python
self.get_logger().info("Node started")
self.get_logger().warn("Something unexpected")
self.get_logger().error("Failed to load config")
self.get_logger().debug("High-frequency detail (disabled by default)")
```

## Config File Access Pattern

When a node needs to find its own config file:

```python
from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory("<pkg>")
config_path = os.path.join(pkg_share, "config.yaml")
```

This resolves correctly in both development (symlink install) and deployed environments.

## Timer vs. Subscription-driven

| Pattern | When to use |
|---------|-------------|
| `create_timer(dt, callback)` | Periodic publishing at a fixed rate |
| `create_subscription(...)` | React to incoming messages |
| Both | State machine: receive input, publish output at fixed rate |

For periodic publishers, prefer `create_timer` over a manual `while` loop — it plays nicely with `rclpy.spin()` and the executor.
