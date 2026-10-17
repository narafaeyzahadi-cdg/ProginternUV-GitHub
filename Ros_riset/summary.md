# ROS 2 Research Summary

A complete beginner-to-intermediate guide to **ROS 2 (Robot Operating System 2)**: what it is, how it is built, and an explanation of every core concept — nodes, topics, publishers, subscribers, services, actions, parameters, launch files, and more.

---

## Table of Contents

1. [What is ROS 2?](#1-what-is-ros-2)
2. [ROS 1 vs ROS 2](#2-ros-1-vs-ros-2)
3. [Distributions (Versions)](#3-distributions-versions)
4. [Architecture: How ROS 2 is Built](#4-architecture-how-ros-2-is-built)
5. [The ROS 2 Graph](#5-the-ros-2-graph)
6. [Nodes](#6-nodes)
7. [Topics](#7-topics)
8. [Publishers and Subscribers (explained in detail)](#8-publishers-and-subscribers-explained-in-detail)
9. [Messages and Interfaces (.msg / .srv / .action)](#9-messages-and-interfaces)
10. [Services (Client / Server)](#10-services-client--server)
11. [Actions (Action Client / Action Server)](#11-actions-action-client--action-server)
12. [Parameters](#12-parameters)
13. [Quality of Service (QoS)](#13-quality-of-service-qos)
14. [Executors and Callbacks](#14-executors-and-callbacks)
15. [Lifecycle (Managed) Nodes](#15-lifecycle-managed-nodes)
16. [Components (Composable Nodes)](#16-components-composable-nodes)
17. [Launch Files](#17-launch-files)
18. [Workspaces, Packages, colcon and ament](#18-workspaces-packages-colcon-and-ament)
19. [Namespaces and Remapping](#19-namespaces-and-remapping)
20. [TF2 (Coordinate Frames)](#20-tf2-coordinate-frames)
21. [Time, Clocks and Timers](#21-time-clocks-and-timers)
22. [ROS Domain ID and Networking](#22-ros-domain-id-and-networking)
23. [Common Tools (CLI, rqt, RViz, rosbag2, Gazebo)](#23-common-tools)
24. [Ecosystem: Nav2, MoveIt 2, ros2_control](#24-ecosystem-nav2-moveit-2-ros2_control)
25. [Communication Patterns: When to Use What](#25-communication-patterns-when-to-use-what)
26. [Cheat Sheet of Commands](#26-cheat-sheet-of-commands)
27. [Glossary](#27-glossary)
28. [References](#28-references)

---

## 1. What is ROS 2?

**ROS 2** is **not an operating system** in the usual sense (like Linux or Windows). It is a **middleware framework + set of libraries and tools** for writing robot software. It runs *on top of* a normal OS (mainly Ubuntu Linux, also Windows and macOS).

ROS 2 gives you:

- **A communication system** — so different programs (nodes) can send data to each other (topics, services, actions).
- **Standard message formats** — e.g. `sensor_msgs/msg/LaserScan`, `geometry_msgs/msg/Twist`, so everyone's code speaks the same "language".
- **Tools** — for visualizing (RViz), recording data (rosbag2), debugging (rqt, `ros2` CLI), and simulating (Gazebo).
- **A huge ecosystem** of ready-made packages — navigation (Nav2), arm planning (MoveIt 2), hardware control (ros2_control), SLAM, perception, etc.

The core idea: **split a robot's software into many small, independent programs that talk to each other.** One program reads the camera, another detects objects, another plans a path, another drives the motors.

---

## 2. ROS 1 vs ROS 2

| Feature | ROS 1 | ROS 2 |
|---|---|---|
| Central master | Needs `roscore` (single point of failure) | **No master** — nodes discover each other automatically (DDS discovery) |
| Middleware | Custom TCPROS/UDPROS | **DDS** (Data Distribution Service), an industry standard; pluggable via RMW |
| Real-time | Not designed for it | Designed to support real-time systems |
| Quality of Service | None | **QoS policies** (reliability, durability, history, ...) |
| Security | None | **SROS2** — authentication, encryption, access control |
| Multi-robot | Hard | Natural (domains, namespaces, no master) |
| Platforms | Mostly Ubuntu | Ubuntu, Windows, macOS, RTOS / microcontrollers (micro-ROS) |
| Node lifecycle | No | **Managed / lifecycle nodes** |
| Multiple nodes per process | Nodelets (awkward) | **Components** (first class) |
| Build system | catkin | **ament + colcon** |
| Launch files | XML only | **Python**, XML, or YAML |
| Status | ROS 1 Noetic reached end of life in **May 2025** | Actively developed |

**Bottom line:** new projects should use ROS 2.

---

## 3. Distributions (Versions)

ROS 2 releases a new distribution every year on **May 23** (World Turtle Day). Even-year releases are **LTS** (Long-Term Support, 5 years); odd-year releases get ~1.5 years.

| Distro | Release | Type | Ubuntu | Notes |
|---|---|---|---|---|
| Humble Hawksbill | May 2022 | LTS | 22.04 | Supported until May 2027, still very widely used |
| Iron Irwini | May 2023 | Normal | 22.04 | End of life |
| **Jazzy Jalisco** | May 2024 | **LTS** | 24.04 | Supported until May 2029 — a safe default today |
| Kilted Kaiju | May 2025 | Normal | 24.04 | |
| Lyrical Luth | May 2026 | LTS | 26.04 | Newest LTS |
| Rolling Ridley | continuous | Development | latest | Bleeding edge, always changing |

Tip: Match the distro to your Ubuntu version, and pick an **LTS** for real projects.

---

## 4. Architecture: How ROS 2 is Built

ROS 2 is built in **layers**:

```
┌─────────────────────────────────────────────┐
│          Your code (nodes)                  │
├──────────────────────┬──────────────────────┤
│  rclcpp (C++)        │  rclpy (Python)      │  ← client libraries
├──────────────────────┴──────────────────────┤
│  rcl  (ROS Client Library, written in C)    │  ← shared logic
├─────────────────────────────────────────────┤
│  rmw  (ROS MiddleWare interface)            │  ← abstraction layer
├─────────────────────────────────────────────┤
│  DDS / middleware implementation            │
│  (Fast DDS, Cyclone DDS, Connext, Zenoh)    │
├─────────────────────────────────────────────┤
│  Operating system / network (UDP, SHM)      │
└─────────────────────────────────────────────┘
```

- **rclcpp / rclpy** — the libraries you actually write code with (C++ or Python). Others exist too (rclc for microcontrollers, rclrs for Rust, rclnodejs...).
- **rcl** — a common C core so that C++ and Python behave the same way.
- **rmw** — an interface that hides *which* middleware is used. You can switch middleware with an environment variable, e.g. `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- **DDS** — the actual networking. It handles **discovery** (finding other nodes automatically), **serialization**, and **delivery** of data. Default is **eProsima Fast DDS**; **Cyclone DDS** is a popular alternative. Newer distros (Jazzy and later) also support **Zenoh** (`rmw_zenoh`) as a non-DDS option that works better over Wi‑Fi and the internet.

---

## 5. The ROS 2 Graph

The **ROS graph** is the network of all ROS 2 elements running at the same time and how they connect. It contains:

- **Nodes** — the programs (the "boxes")
- **Topics, Services, Actions** — the communication channels (the "arrows")
- **Parameters** — the configuration stored in each node

Example of a simple mobile robot graph:

```
 [camera_node] --/image_raw--> [detector_node] --/detections--> [planner_node]
                                                                    |
 [lidar_node] ---/scan-----------------------------------------------+
                                                                    v
                                                    /cmd_vel  (Twist message)
                                                                    |
                                                                    v
                                                          [motor_driver_node]
```

You can see the live graph with `rqt_graph`.

---

## 6. Nodes

### What is a node?

A **node** is a single, independent program (process or part of a process) that does **one specific job** in the robot. Think of nodes as workers in a factory — each one does one task and passes results to others.

Examples:
- `camera_driver` — reads frames from the camera and publishes images
- `lidar_driver` — publishes laser scans
- `object_detector` — subscribes to images, publishes detections
- `path_planner` — computes a path
- `motor_controller` — receives velocity commands and drives the wheels

### Why split into nodes?

- **Modularity** — replace one part without touching the others (swap a simulated camera for a real one).
- **Fault isolation** — if one node crashes, the others keep running.
- **Reusability** — use someone else's node (e.g. a LiDAR driver) directly.
- **Distribution** — nodes can run on different computers and still talk to each other.
- **Language freedom** — a Python node can talk to a C++ node.

### What can a node contain?

One node can have any number of:
- Publishers
- Subscribers
- Service servers / clients
- Action servers / clients
- Parameters
- Timers

### Minimal node in Python

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')          # node name
        self.get_logger().info('Hello from ROS 2!')

def main():
    rclpy.init()                 # start ROS 2 communication
    node = MyNode()
    rclpy.spin(node)             # keep the node alive and process callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Minimal node in C++

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    RCLCPP_INFO(get_logger(), "Hello from ROS 2!");
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
```

### Useful node commands

```bash
ros2 run <package> <executable>     # start a node
ros2 node list                      # list running nodes
ros2 node info /my_node             # show its pubs, subs, services, actions
```

---

## 7. Topics

### What is a topic?

A **topic** is a **named channel (a "bus")** over which nodes exchange **messages**. It is like a radio station or a group chat:

- Anyone can **broadcast** on it (publishers).
- Anyone can **listen** to it (subscribers).
- Senders and receivers **don't need to know about each other** — they only need to agree on the topic **name** and the message **type**.

Key properties:
- **Many-to-many**: any number of publishers and subscribers on the same topic.
- **Asynchronous / one-way**: the publisher sends and continues; it does not wait for an answer.
- **Continuous streams of data**: best for sensor data, robot state, commands that update frequently.
- **Strongly typed**: every topic has exactly one message type (e.g. `/cmd_vel` uses `geometry_msgs/msg/Twist`).

### Common standard topics

| Topic | Message type | Meaning |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command to drive the robot |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR data |
| `/image_raw` | `sensor_msgs/msg/Image` | Camera image |
| `/odom` | `nav_msgs/msg/Odometry` | Robot position estimate from wheels |
| `/imu` | `sensor_msgs/msg/Imu` | IMU acceleration / angular velocity |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Coordinate frame transforms |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint positions of a robot arm |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy map |

### Topic commands

```bash
ros2 topic list -t                    # list topics with their types
ros2 topic echo /scan                 # print messages as they arrive
ros2 topic info /cmd_vel -v           # who publishes / subscribes, with QoS
ros2 topic hz /scan                   # measure publishing rate
ros2 topic bw /image_raw              # measure bandwidth
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

---

## 8. Publishers and Subscribers (explained in detail)

> The "other one" you forgot is the **Publisher**. Publishers and Subscribers are the two sides of a **topic**. This pattern is called **Publish / Subscribe ("pub/sub")**.

### The analogy

Think of a **YouTube channel**:
- The **publisher** is the YouTuber — they upload videos to the channel (topic) whenever they want.
- The **subscribers** are the viewers — they get notified and receive every new video.
- The YouTuber doesn't know or care how many viewers there are. Viewers can join or leave any time.
- If nobody is subscribed, the video is still "published" — it just goes nowhere.

### Publisher

A **publisher** is an object inside a node that **sends messages to a topic**.

To create one you specify:
1. **Message type** (e.g. `String`, `Twist`, `LaserScan`)
2. **Topic name** (e.g. `/chatter`)
3. **QoS** (Quality of Service) — at minimum a queue depth, e.g. `10`

Then you call `publish(msg)` whenever you have new data — often from a **timer** (e.g. 10 times per second) or right after reading a sensor.

**Python publisher example (the classic "talker"):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        # create_publisher(msg_type, topic_name, qos_depth)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # every 0.5 s
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)                 # send it!
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()
```

**C++ publisher:**

```cpp
auto pub = this->create_publisher<std_msgs::msg::String>("chatter", 10);
std_msgs::msg::String msg;
msg.data = "Hello World";
pub->publish(msg);
```

### Subscriber

A **subscriber** (also called a **subscription**) is an object inside a node that **receives messages from a topic**.

To create one you specify:
1. **Message type** — must match the publisher's type
2. **Topic name** — must match the publisher's topic
3. **Callback function** — the function ROS 2 calls **automatically every time a new message arrives**
4. **QoS** — must be *compatible* with the publisher's QoS

The subscriber does **not** ask for data. It just waits; when a message arrives, ROS 2 calls your callback with the message as its argument. That only happens while the node is **spinning** (`rclpy.spin(node)`).

**Python subscriber example (the classic "listener"):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        # create_subscription(msg_type, topic_name, callback, qos_depth)
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)

    def listener_callback(self, msg):          # called for every message
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()
```

**C++ subscriber:**

```cpp
auto sub = this->create_subscription<std_msgs::msg::String>(
  "chatter", 10,
  [this](const std_msgs::msg::String & msg) {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str());
  });
```

### Try it right now (built-in demo)

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker
# Terminal 2
ros2 run demo_nodes_py listener
```

A C++ talker and a Python listener talk without any extra setup — that's the power of topics.

### How it works under the hood

1. Each node starts and announces itself on the network via **DDS discovery**.
2. The publisher announces "I publish `String` on `/chatter`". The subscriber announces "I want `String` from `/chatter`".
3. DDS **matches** them if the **name**, **type**, and **QoS** are compatible.
4. A direct connection is made (no master in the middle). Each `publish()` serializes the message and sends it to every matched subscriber.
5. On the subscriber side, the message waits in a queue until the **executor** runs the callback.

### Publisher vs Subscriber at a glance

| | Publisher | Subscriber |
|---|---|---|
| Role | Sends data | Receives data |
| Created with | `create_publisher(type, topic, qos)` | `create_subscription(type, topic, callback, qos)` |
| Main action | `publish(msg)` | Your **callback** runs automatically |
| Who triggers it | Your code (often a timer or sensor read) | Arrival of a new message |
| Knows the other side? | No | No |
| How many per topic? | 0..N | 0..N |

### A node can be both

Most real nodes are **both** subscriber and publisher. Example: an obstacle-avoidance node **subscribes** to `/scan`, decides what to do, and **publishes** to `/cmd_vel`:

```python
class Avoider(Node):
    def __init__(self):
        super().__init__('avoider')
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def on_scan(self, scan):
        cmd = Twist()
        if min(scan.ranges) < 0.5:   # obstacle closer than 0.5 m
            cmd.angular.z = 0.8      # turn
        else:
            cmd.linear.x = 0.3       # go forward
        self.pub.publish(cmd)
```

### Common mistakes

- **Topic name typo** (`/cmd_vel` vs `/cmdvel`) — they silently never connect. Check with `ros2 topic info`.
- **Type mismatch** — both sides must use exactly the same message type.
- **QoS incompatibility** — e.g. subscriber requires `RELIABLE` but publisher is `BEST_EFFORT` → no data. (Very common with sensor topics; use `qos_profile_sensor_data`.)
- **Forgetting to spin** — the callback never runs if the node isn't spinning.
- **Doing slow work inside a callback** — blocks other callbacks; keep callbacks short.
- **Different `ROS_DOMAIN_ID`** on two machines — they can't see each other.

---

## 9. Messages and Interfaces

ROS 2 data structures are called **interfaces**. There are three kinds, defined in simple text files:

| File | Used by | Structure |
|---|---|---|
| `.msg` | Topics | Fields only |
| `.srv` | Services | Request `---` Response |
| `.action` | Actions | Goal `---` Result `---` Feedback |

**Example `.msg`** (`my_interfaces/msg/Temperature.msg`):
```
std_msgs/Header header
float64 celsius
string sensor_name
```

**Example `.srv`** (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

**Example `.action`** (`Fibonacci.action`):
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Primitive field types: `bool, byte, char, int8..int64, uint8..uint64, float32, float64, string`, arrays (`int32[]`, `float64[3]`), and other messages.

Common interface packages: `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `trajectory_msgs`, `visualization_msgs`, `action_msgs`, `std_srvs`.

```bash
ros2 interface list
ros2 interface show geometry_msgs/msg/Twist
```

---

## 10. Services (Client / Server)

### What is a service?

A **service** is a **request → response** communication, like a function call across nodes, or like asking a question and waiting for the answer.

- **Service server** — the node that **provides** the service and computes the answer.
- **Service client** — the node that **calls** the service and receives the answer.
- **One-to-one per call**: only one server per service name; many clients can call it.
- Best for **quick, occasional** operations: "reset the odometry", "get the current map", "turn the LED on", "spawn a robot".
- **Not** for continuous data (use a topic) and **not** for long tasks (use an action).

### Python server

```python
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle)

    def handle(self, request, response):
        response.sum = request.a + request.b
        return response
```

### Python client

```python
class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.cli.wait_for_service()

    def send(self, a, b):
        req = AddTwoInts.Request(a=a, b=b)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().sum
```

```bash
ros2 service list -t
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

---

## 11. Actions (Action Client / Action Server)

### What is an action?

An **action** is for **long-running tasks** that need:
1. a **goal** (what to do),
2. continuous **feedback** (progress while running),
3. a final **result**,
4. and the ability to **cancel** in the middle.

Examples: "navigate to the kitchen", "move the arm to this pose", "rotate 180°", "dock to the charger".

- **Action server** — accepts goals, executes them, publishes feedback, returns the result.
- **Action client** — sends goals, receives feedback and result, can cancel.

Under the hood an action is built from **3 services + 2 topics**:

```
Client                                   Server
  | --- send_goal (service) -------------> |
  | <-- accepted / rejected -------------- |
  | <-- feedback (topic) ----------------- |  (repeated)
  | <-- status (topic) ------------------- |
  | --- get_result (service) ------------> |
  | <-- result --------------------------- |
  | --- cancel_goal (service, optional) -> |
```

### Python action server (sketch)

```python
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci

class FibServer(Node):
    def __init__(self):
        super().__init__('fib_server')
        self._srv = ActionServer(self, Fibonacci, 'fibonacci', self.execute)

    def execute(self, goal_handle):
        seq = [0, 1]
        fb = Fibonacci.Feedback()
        for i in range(1, goal_handle.request.order):
            seq.append(seq[i] + seq[i - 1])
            fb.partial_sequence = seq
            goal_handle.publish_feedback(fb)       # progress
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)     # final result
```

```bash
ros2 action list -t
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback
```

---

## 12. Parameters

**Parameters** are **configuration values stored inside a node** — like settings. Each node owns its own parameters.

- Types: `bool, int, double, string, byte[]`, and arrays of these.
- Can be set at startup (command line, YAML file, launch file) and **changed at runtime**.
- Examples: `max_speed`, `camera_fps`, `frame_id`, `use_sim_time`.

```python
class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.declare_parameter('max_speed', 0.5)            # name, default
        speed = self.get_parameter('max_speed').value
        self.add_on_set_parameters_callback(self.on_change)  # react to changes
```

YAML file (`params.yaml`):
```yaml
driver:
  ros__parameters:
    max_speed: 1.2
    frame_id: "base_link"
```

```bash
ros2 run my_pkg driver --ros-args -p max_speed:=1.0
ros2 run my_pkg driver --ros-args --params-file params.yaml
ros2 param list
ros2 param get /driver max_speed
ros2 param set /driver max_speed 0.8
ros2 param dump /driver > saved.yaml
```

---

## 13. Quality of Service (QoS)

**QoS** lets you tune **how** messages are delivered. It is one of ROS 2's biggest improvements over ROS 1.

| Policy | Options | Meaning |
|---|---|---|
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | Guarantee delivery (retries) vs. send and forget (faster, may drop) |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | Late subscribers get nothing vs. get the last saved messages (a "latched" topic) |
| **History** | `KEEP_LAST(n)` / `KEEP_ALL` | Keep the last *n* messages in the queue vs. all |
| **Depth** | integer | Queue size for `KEEP_LAST` |
| **Deadline** | duration | Expected max time between messages |
| **Lifespan** | duration | Messages older than this are dropped |
| **Liveliness** | `AUTOMATIC` / `MANUAL_BY_TOPIC` | How to detect a publisher is still alive |

**Predefined profiles:**
- Default — `RELIABLE`, `VOLATILE`, `KEEP_LAST(10)`
- `qos_profile_sensor_data` — `BEST_EFFORT`, small queue (for camera/LiDAR: newest data matters, old data is useless)
- `qos_profile_services_default`, `qos_profile_parameters`
- Map / robot_description style — `RELIABLE` + `TRANSIENT_LOCAL` (late joiners still get the map)

**Compatibility rule (offered vs. requested):** the publisher must offer **at least** what the subscriber requests.

| Publisher | Subscriber | Works? |
|---|---|---|
| RELIABLE | RELIABLE | ✅ |
| RELIABLE | BEST_EFFORT | ✅ |
| BEST_EFFORT | BEST_EFFORT | ✅ |
| BEST_EFFORT | RELIABLE | ❌ no connection |
| TRANSIENT_LOCAL | VOLATILE | ✅ |
| VOLATILE | TRANSIENT_LOCAL | ❌ no connection |

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(LaserScan, 'scan', self.cb, qos_profile_sensor_data)
```

---

## 14. Executors and Callbacks

Everything in ROS 2 is **callback-driven**: subscription callbacks, timer callbacks, service handlers, action handlers. The **executor** is the engine that waits for events and runs the right callback.

- `rclpy.spin(node)` — uses a default single-threaded executor.
- **SingleThreadedExecutor** — callbacks run one at a time (simple, safe).
- **MultiThreadedExecutor** — callbacks can run in parallel threads.
- **Callback groups** control what may run in parallel:
  - `MutuallyExclusiveCallbackGroup` — callbacks in the group never run at the same time.
  - `ReentrantCallbackGroup` — callbacks may run concurrently.

Common pitfall: calling a service **synchronously inside a callback** with a single-threaded executor → **deadlock**. Use `call_async` or a multi-threaded executor with separate callback groups.

---

## 15. Lifecycle (Managed) Nodes

A **lifecycle node** has well-defined **states**, so a system can start up in a controlled, predictable order (important for real robots and safety).

```
          create
            |
     [Unconfigured] --configure--> [Inactive] --activate--> [Active]
            ^                          |   ^                     |
            |------- cleanup ----------|   |---- deactivate -----|
                                    (any) --shutdown--> [Finalized]
```

- **Unconfigured** — just created, nothing allocated
- **Inactive** — configured (params read, resources allocated) but not processing
- **Active** — fully running, publishing/processing
- **Finalized** — ready to be destroyed

Nav2 uses lifecycle nodes heavily (with a "lifecycle manager" that brings all nodes up in order).

```bash
ros2 lifecycle list /my_node
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
```

---

## 16. Components (Composable Nodes)

Normally each node is its own process. **Components** let you load **multiple nodes into one process** (a "component container"). Benefits:

- **Zero-copy / intra-process communication** — messages are passed by pointer, no serialization. Huge speedup for images and point clouds.
- Less memory and CPU overhead.
- Decide at **launch time** whether nodes share a process — no code change needed.

```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager composition composition::Talker
ros2 component list
```

---

## 17. Launch Files

A **launch file** starts **many nodes at once** with their parameters, remappings and namespaces — instead of opening 10 terminals. Can be written in **Python** (most flexible), **XML**, or **YAML**.

**Python launch example (`robot.launch.py`):**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker', name='talker'),
        Node(package='demo_nodes_py', executable='listener', name='listener',
             remappings=[('chatter', 'my_chatter')]),
        Node(package='my_pkg', executable='driver',
             parameters=[{'max_speed': 1.0}],
             namespace='robot1'),
    ])
```

**XML version:**

```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
  <node pkg="demo_nodes_py" exec="listener" name="listener"/>
</launch>
```

```bash
ros2 launch my_pkg robot.launch.py
```

Launch features: arguments (`DeclareLaunchArgument`), conditions (`IfCondition`), including other launch files (`IncludeLaunchDescription`), event handlers, groups, setting environment variables.

---

## 18. Workspaces, Packages, colcon and ament

- **Package** — the unit of organizing code (nodes, launch files, configs, interfaces). Has a `package.xml` (metadata + dependencies).
  - **ament_cmake** — C++ (and interface) packages, uses `CMakeLists.txt`
  - **ament_python** — Python packages, uses `setup.py` / `setup.cfg`
- **Workspace** — a folder containing many packages, usually `~/ros2_ws/src/...`
- **colcon** — the build tool that builds all packages in the right order.
- **ament** — the build system/conventions underneath colcon.
- **rosdep** — installs system dependencies listed in `package.xml`.

Typical layout:

```
ros2_ws/
├── src/
│   └── my_pkg/
│       ├── package.xml
│       ├── setup.py            (Python)  or CMakeLists.txt (C++)
│       ├── my_pkg/             (Python source)
│       ├── launch/
│       └── config/
├── build/      (generated)
├── install/    (generated)
└── log/        (generated)
```

Workflow:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ros2 pkg create my_pkg --build-type ament_python --dependencies rclpy std_msgs
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash          # IMPORTANT: "overlay" your workspace
ros2 run my_pkg my_node
```

**Underlay vs overlay:** `/opt/ros/<distro>/setup.bash` is the *underlay* (installed ROS). Your workspace's `install/setup.bash` is an *overlay* on top of it.

---

## 19. Namespaces and Remapping

**Namespaces** group names, like folders. Useful for running two identical robots:

```
/robot1/cmd_vel     /robot2/cmd_vel
/robot1/scan        /robot2/scan
```

Name types:
- `cmd_vel` — **relative** (gets the node's namespace prepended)
- `/cmd_vel` — **absolute** (global, unchanged)
- `~/status` — **private** (becomes `/<node_name>/status`)

**Remapping** renames topics/services/nodes at startup without changing code:

```bash
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=my_topic -r __node:=my_talker -r __ns:=/robot1
```

---

## 20. TF2 (Coordinate Frames)

A robot has many **coordinate frames**: `map`, `odom`, `base_link`, `laser`, `camera_link`, `gripper`... **TF2** keeps track of the position/rotation between all frames **over time**, as a tree.

```
map
 └── odom
      └── base_link
           ├── laser
           ├── camera_link
           └── imu_link
```

- Transforms are published on `/tf` (moving) and `/tf_static` (fixed).
- **Broadcaster** publishes a transform; **Listener / Buffer** looks up any transform between two frames at any time (e.g. "where is the obstacle seen by the laser, in the map frame?").
- `robot_state_publisher` reads the robot's **URDF** model and publishes the frames.

Standard conventions (REP-105): `map` → `odom` → `base_link`.

```bash
ros2 run tf2_ros static_transform_publisher --x 0.1 --z 0.2 --frame-id base_link --child-frame-id laser
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames      # generates a PDF of the tree
```

---

## 21. Time, Clocks and Timers

- **Timers** run a callback periodically: `self.create_timer(0.1, self.cb)` → 10 Hz.
- **Clock types:** system time, steady time, and **ROS time**.
- **Simulation time:** with `use_sim_time:=true`, nodes use the `/clock` topic from the simulator or a rosbag instead of the wall clock — essential for Gazebo and replaying bags.
- Messages carry a **`std_msgs/Header`** with a `stamp` (time) and `frame_id` (TF frame).

---

## 22. ROS Domain ID and Networking

- All nodes on the same network with the same **`ROS_DOMAIN_ID`** (default `0`) see each other automatically.
- Use different IDs (0–101 safe range) to isolate groups, e.g. in a classroom: `export ROS_DOMAIN_ID=7`.
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Iron and later) limits discovery to your own computer.
- Multi-machine: be on the same subnet; multicast must be allowed. Wi-Fi can be troublesome with DDS — consider a **discovery server** (Fast DDS) or **Zenoh**.
- **SROS2** adds security: encryption, authentication, access control (`ros2 security ...`).
- **micro-ROS** brings ROS 2 to microcontrollers (ESP32, STM32) through an agent.

---

## 23. Common Tools

| Tool | What it does |
|---|---|
| `ros2` CLI | Inspect/control everything (`node`, `topic`, `service`, `action`, `param`, `bag`, `launch`, `doctor`, `interface`) |
| **rqt** | GUI plugins: `rqt_graph` (graph view), `rqt_plot`, `rqt_console` (logs), `rqt_reconfigure` (params), `rqt_image_view` |
| **RViz2** | 3D visualization: robot model, TF, LiDAR, point clouds, maps, paths, markers |
| **rosbag2** | Record and replay topic data (`ros2 bag record -a`, `ros2 bag play my_bag`). Default storage format **MCAP** since Iron |
| **Gazebo** (new Gazebo, formerly Ignition) | Physics simulator; connected to ROS 2 via `ros_gz_bridge`. Gazebo Classic is end-of-life |
| **Foxglove** | Web/desktop visualization and debugging, works well with MCAP files |
| `ros2 doctor` | Checks your setup for common problems |

---

## 24. Ecosystem: Nav2, MoveIt 2, ros2_control

- **Nav2** — the navigation stack for mobile robots: localization (AMCL), global/local planners, controllers, costmaps, **behavior trees**, recovery behaviors. Send a goal pose → robot drives there avoiding obstacles. Uses actions (`/navigate_to_pose`) and lifecycle nodes.
- **SLAM Toolbox / Cartographer** — build maps while localizing.
- **MoveIt 2** — motion planning for robot arms: inverse kinematics, collision checking, trajectory generation.
- **ros2_control** — standard framework connecting controllers (diff drive, joint trajectory) to real or simulated hardware through a hardware interface.
- **robot_localization** — sensor fusion (EKF/UKF) of odometry, IMU, GPS.
- **image_pipeline / vision_opencv (cv_bridge)** — camera processing, converting ROS images ↔ OpenCV.
- **perception_pcl** — point cloud processing.
- **URDF / xacro** — XML description of the robot's links, joints, sensors and geometry.

---

## 25. Communication Patterns: When to Use What

| Need | Use | Pattern | Example |
|---|---|---|---|
| Continuous data stream | **Topic** (publisher → subscriber) | Many-to-many, async, one-way | Sensor data, `/cmd_vel`, robot state |
| Quick question → answer | **Service** (client → server) | 1 server, request/response | Reset odometry, get a value, toggle a mode |
| Long task with progress & cancel | **Action** (client → server) | Goal, feedback, result, cancel | Navigate to a point, move an arm |
| Node configuration | **Parameter** | Key-value per node | Max speed, frame names, gains |

Rule of thumb:
- If it **keeps flowing** → topic.
- If it **finishes fast** and you need an answer → service.
- If it **takes time** and you want progress or to cancel → action.
- If it's a **setting** → parameter.

---

## 26. Cheat Sheet of Commands

```bash
# Environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# Nodes
ros2 run <pkg> <exe>
ros2 node list
ros2 node info /node

# Topics
ros2 topic list -t
ros2 topic echo /topic
ros2 topic info /topic -v
ros2 topic hz /topic
ros2 topic pub --once /topic <type> "<yaml>"

# Services
ros2 service list -t
ros2 service call /srv <type> "<yaml>"

# Actions
ros2 action list -t
ros2 action send_goal /action <type> "<yaml>" --feedback

# Parameters
ros2 param list
ros2 param get /node name
ros2 param set /node name value

# Interfaces
ros2 interface show <type>

# Build
colcon build --symlink-install
colcon build --packages-select my_pkg
ros2 pkg create <name> --build-type ament_python|ament_cmake

# Launch / bag / debug
ros2 launch <pkg> <file.launch.py>
ros2 bag record -a -o my_bag
ros2 bag play my_bag
ros2 doctor
rqt_graph
rviz2
```

---

## 27. Glossary

| Term | Meaning |
|---|---|
| **Node** | A program that does one job in the robot system |
| **Topic** | Named channel for streaming messages |
| **Publisher** | Sends messages on a topic |
| **Subscriber / Subscription** | Receives messages from a topic via a callback |
| **Message** | Typed data structure sent on a topic (`.msg`) |
| **Service** | Request/response call (`.srv`) |
| **Service server / client** | Provides / calls a service |
| **Action** | Long-running goal with feedback, result, cancel (`.action`) |
| **Parameter** | Configuration value owned by a node |
| **Callback** | Function ROS calls automatically on an event |
| **Executor** | Runs callbacks when events happen ("spinning") |
| **QoS** | Rules for message delivery (reliability, durability, history...) |
| **DDS** | Networking middleware standard underneath ROS 2 |
| **RMW** | Interface allowing different middleware implementations |
| **Package** | Unit of code organization |
| **Workspace** | Folder of packages built together |
| **colcon** | Build tool |
| **Launch file** | Starts and configures many nodes at once |
| **TF2** | Library that tracks coordinate frames over time |
| **URDF** | XML robot model description |
| **Lifecycle node** | Node with managed states (unconfigured / inactive / active) |
| **Component** | Node that can be loaded into a shared process |
| **Domain ID** | Number that isolates groups of ROS 2 nodes on a network |
| **rosbag** | Recording of topic data for replay |

---

## 28. References

- Official ROS 2 documentation — https://docs.ros.org/en/jazzy/
- Concepts overview — https://docs.ros.org/en/jazzy/Concepts.html
- Beginner CLI tutorials (nodes, topics, services, params, actions) — https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html
- Writing a simple publisher and subscriber (Python) — https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- About Quality of Service settings — https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- Releases / distributions — https://docs.ros.org/en/rolling/Releases.html
- Managed nodes design — https://design.ros2.org/articles/node_lifecycle.html
- Nav2 — https://docs.nav2.org/
- MoveIt 2 — https://moveit.picknik.ai/
- ros2_control — https://control.ros.org/
- REP-105 (coordinate frames for mobile platforms) — https://www.ros.org/reps/rep-0105.html
