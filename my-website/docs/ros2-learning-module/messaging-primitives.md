# Messaging Primitives in ROS 2

Communication is the backbone of any distributed robotic system. In ROS 2, nodes communicate using several fundamental messaging primitives, primarily **Topics** and **Services**. These mechanisms allow different parts of a robot's software to exchange information and coordinate actions efficiently.

## Topics: The Publish-Subscribe Model

**Topics** are the most common way for nodes to exchange data in ROS 2. They implement a **publish-subscribe** (pub/sub) model, which is ideal for streaming data that doesn't require a direct, immediate response.

### How Topics Work:

1.  **Publisher Node**: A node that wants to share information creates a "publisher" for a specific topic. It then continuously sends (publishes) messages to that topic.
2.  **Subscriber Node**: A node that wants to receive information from that topic creates a "subscriber." It listens for and receives messages published to that topic.
3.  **Message Type**: Every topic has a defined **message type**. This type specifies the structure and data fields of the messages that can be sent over that topic. For example, a `sensor_msgs/msg/LaserScan` message type will contain fields for laser range data, angles, and timestamps.
4.  **Decentralized**: There is no central message broker. Publishers and subscribers connect directly via the underlying DDS layer. Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic.

### Key Characteristics of Topics:

-   **Asynchronous**: Publishers don't wait for subscribers to receive messages.
-   **One-way**: Information flows from publisher to subscriber.
-   **Many-to-many**: A single topic can have multiple publishers and multiple subscribers.
-   **No acknowledgment**: Publishers generally don't know if their messages were received by any subscribers (though QoS settings can influence this).

### When to Use Topics:

-   Continuous data streams (e.g., sensor readings like cameras, LiDAR, IMU).
-   Broadcasting state information (e.g., robot's current pose, joint states).
-   Sending commands that don't require an immediate, guaranteed response.

## Services: The Request-Reply Model

**Services** provide a **request-reply** model for communication. This is used when a node needs to send a request to another node and expects a specific response back. Services are synchronous, meaning the client node will typically block (wait) until it receives a response from the server node.

### How Services Work:

1.  **Service Server Node**: A node that offers a specific service creates a "service server." It defines a callback function that will be executed when a client requests the service.
2.  **Service Client Node**: A node that needs a specific action performed creates a "service client." It sends a request message to the service server and waits for a response message.
3.  **Service Type**: Similar to topics, services have a defined **service type**. This type specifies the structure of both the request message and the response message.

### Key Characteristics of Services:

-   **Synchronous**: The client typically waits for the server's response.
-   **Two-way**: Information flows as a request from client to server, and a response from server to client.
-   **One-to-one**: A client typically interacts with a single service server for a given request.
-   **Guaranteed response**: The client expects and waits for a response.

### When to Use Services:

-   Triggering a specific action (e.g., "take a picture," "open gripper").
-   Querying information that requires computation (e.g., "calculate inverse kinematics," "get map data").
-   Changing a node's configuration (e.g., "set camera resolution").
-   Any discrete, blocking operation where the client needs a result.

## Comparing Topics and Services

| Feature            | Topics                                 | Services                               |
| :----------------- | :------------------------------------- | :------------------------------------- |
| **Communication**  | Asynchronous (publish-subscribe)       | Synchronous (request-reply)            |
| **Direction**      | Unidirectional (publisher to subscriber) | Bidirectional (client-server)          |
| **Usage**          | Streaming data, continuous updates     | Discrete actions, queries              |
| **Response**       | No direct response expected            | Response expected                      |
| **Blocking**       | Non-blocking for publisher             | Blocking for client (typically)        |
| **Example**        | Sensor readings, robot pose            | Take a photo, set motor speed          |

Understanding when to use topics versus services is crucial for designing effective and efficient ROS 2 applications.
