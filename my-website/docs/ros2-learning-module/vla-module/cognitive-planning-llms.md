# Chapter 2: Cognitive Planning with LLMs

Building upon the Voice-to-Action pipeline, this chapter explores how Large Language Models (LLMs) can elevate a humanoid robot's autonomy by enabling sophisticated cognitive planning. We'll delve into task decomposition and how LLM-generated plans can be translated into executable ROS 2 actions.

## Task Decomposition

Human tasks are often complex and high-level, such as "clean the kitchen" or "prepare coffee." For a robot, these need to be broken down into a sequence of simpler, executable steps. This process is known as **task decomposition**.

LLMs excel at this. Given a high-level goal, an LLM can leverage its vast knowledge to:
-   **Interpret context**: Understand the nuances of the request and the robot's capabilities.
-   **Generate sub-tasks**: Break down the complex goal into a logical sequence of atomic actions.
-   **Handle preconditions and postconditions**: Implicitly reason about the state changes required for each step.

Example: For the high-level goal "make a cup of tea":
An LLM might decompose this into:
1.  `find_object("mug")`
2.  `pick_up("mug")`
3.  `move_to_location("kettle")`
4.  `fill_kettle_with_water()`
5.  `turn_on_kettle()`
6.  `wait_until_kettle_boils()`
7.  `move_mug_to_kettle()`
8.  `pour_water_into_mug()`
9.  `find_object("tea_bag")`
10. `pick_up("tea_bag")`
11. `place_object_in_mug("tea_bag")`
12. `stir_mug()`
13. `move_mug_to_location("user")`

This list of sub-tasks forms a robust plan that the robot can then attempt to execute.

## LLM-to-ROS 2 Action Sequencing

The output of an LLM is typically natural language text. For a robot to execute this plan, the textual sub-tasks need to be mapped to a standardized robotic action framework, such as ROS 2 actions.

**ROS 2 Actions** are a type of ROS 2 communication for long-running, goal-oriented tasks that provide feedback. They are ideal for robot behaviors because they:
-   **Define a Goal**: The desired outcome (e.g., `move_to_pose(x, y, z)`).
-   **Provide Feedback**: Information about the action's progress.
-   **Allow for Cancellation**: The action can be preempted if needed.

The process of converting an LLM's decomposed plan into ROS 2 action calls involves:
1.  **Semantic Parsing**: The LLM's generated sub-task text is parsed to identify the corresponding ROS 2 action name and its parameters.
2.  **Parameter Extraction**: Entities (like `red_cube` or `table`) are extracted and formatted into the required data types for the ROS 2 action goal message.
3.  **Action Dispatch**: The appropriate ROS 2 action client is called with the constructed goal.

This mapping can be facilitated by a "skill manager" or "action server" that exposes a set of robot capabilities as callable functions that the LLM understands.

## LLM Planning Flow Diagram

The cognitive planning flow for a humanoid robot using LLMs can be depicted as:

```text
[High-Level Human Goal (Text)]
      |
      V
[Large Language Model (LLM): Task Decomposition]
      |
      V
[Sequence of Textual Sub-Tasks]
      |
      V
[Skill Manager/Semantic Parser: Map Sub-Tasks to ROS 2 Actions]
      |
      V
[ROS 2 Action Calls (e.g., move_base, pick_object)]
```
*Text-described diagram illustrating the LLM-based cognitive planning flow.*

This intelligent planning layer significantly enhances a humanoid robot's ability to understand and execute complex, multi-step instructions from humans.

## Key Takeaways
-   LLMs are powerful tools for task decomposition, breaking high-level goals into executable sub-tasks for robots.
-   Task decomposition involves interpreting context, generating logical sub-tasks, and reasoning about state changes.
-   Mapping LLM-generated plans to ROS 2 actions enables robots to execute complex instructions.
-   ROS 2 actions provide a robust framework for goal-oriented, feedback-driven robot behaviors.
