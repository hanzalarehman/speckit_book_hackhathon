# Quiz: ROS 2 Fundamentals

This quiz assesses your understanding of the core concepts of ROS 2, including nodes, topics, and services.

## Questions

### Question 1: Understanding ROS 2 Components

Which of the following best describes a **ROS 2 Node**?

A) A physical robot.
B) An executable process that performs computations.
C) A channel for communication between processes.
D) A type of sensor used in ROS 2.

### Question 2: Communication in ROS 2

You want to send continuous sensor data (e.g., IMU readings) from one part of your robot's software to another without expecting an immediate response. Which ROS 2 communication primitive would be most appropriate for this task?

A) Service
B) Action
C) Topic
D) Parameter

### Question 3: ROS 2 Service vs. Topic

Explain the primary difference between a **ROS 2 Topic** and a **ROS 2 Service** in terms of communication patterns and when you would choose one over the other.

---

### Question 4: Identifying ROS 2 Communication Needs

Consider a scenario where a robot needs to:
1.  Continuously publish its current wheel odometry data.
2.  Receive a command to move to a specific (X, Y) coordinate, and respond once it has reached the destination or encountered an obstacle.

For each of these requirements, identify the most suitable ROS 2 communication primitive (Topic or Service) and briefly explain why.

## Answer Key (for self-assessment)

### Question 1:
**Answer**: B) An executable process that performs computations.

### Question 2:
**Answer**: C) Topic
**Explanation**: Topics are designed for continuous, asynchronous, many-to-many communication where no direct response is expected. Sensor data is a classic use case.

### Question 3:
**Answer**:
-   **ROS 2 Topic**:
    -   **Communication Pattern**: Asynchronous, one-way (publish/subscribe). Publishers send messages without expecting a direct reply, and subscribers receive messages when available. Many-to-many communication is possible.
    -   **When to Use**: For streaming data (e.g., sensor readings, video feeds), continuous status updates, or any situation where data needs to be broadcast to multiple listeners.
-   **ROS 2 Service**:
    -   **Communication Pattern**: Synchronous, request/reply. A client sends a request to a service server and waits for a response. One-to-one communication.
    -   **When to Use**: For discrete, blocking operations where a specific result or acknowledgment is needed (e.g., triggering an action, querying a database, setting a parameter).

### Question 4:
1.  **Continuously publish its current wheel odometry data**:
    -   **Primitive**: **ROS 2 Topic**
    -   **Reasoning**: Odometry data is a continuous stream of information. A Topic allows the robot to broadcast this data efficiently to any interested subscribers (e.g., navigation stack, logging nodes) without needing a direct response for each update.

2.  **Receive a command to move to a specific (X, Y) coordinate, and respond once it has reached the destination or encountered an obstacle**:
    -   **Primitive**: **ROS 2 Service**
    -   **Reasoning**: This is a discrete operation where the robot is commanded to perform an action and the sender needs to know the outcome (success, failure, obstacle). A Service provides the synchronous request-reply mechanism required for this interaction.
