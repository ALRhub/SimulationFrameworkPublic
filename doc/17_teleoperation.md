# 17. Real Robot Teleopartion

- [17. Real Robot Teleopartion](#17-real-robot-teleopartion)
  - [17.1. Forcefeedback Warning](#171-forcefeedback-warning)
  - [17.2. User Input](#172-user-input)
  - [17.3 Known Pitfalls](#173-known-pitfalls)

Please use the demos found in `alr_sim.sims.sl.multibot_teleop` as guidance.

We have demos for three scenarios:

- `demo_teacher.py` for kinesthetic teaching of robots. The real robot is put into gravity compensation mode and can be guided by gently pushing the robot arm.

- `demo_teachermulti.py` for kinesthetic teaching of multiple robots, e.g. in a bimanual setup.

- `demo_teleop.py` for teleoperation. One robot is designated as primary robot in the aforementioned "human teacher" mode. A second robot is designated as replicant and mirrors the movements of the first robot.
- `demo_vtwin.py` for a virtual twin setup. In contrast to teleoperation, the replicant robot exists only in simulation. This is a useful mode to itneract with objects in simulation without requiring a CV setup for object tracking.

The virtual twin setup is the most complete for now, showcasing a complete experiment with object interaction, logging capabilities and scene resets.

The code makes heavy use of multibot features to control mupltiple robot movements within one process. Please also refer to [07 Multibot Docu](07_multibot.md).

## 17.1. Forcefeedback Warning
Teleoperation supports Forcefeedback. We do not recommend enabling Forcefeedback in a virtual twin setting, as the simulated forces might behave strangely.

## 17.2. User Input
The `demo_vtwin.py` shows how you can use a simple CLI text input to control the process, such as starting and stopping logging or resetting the scene.

## 17.3 Known Pitfalls
The [HumanController](../alr_sim/sims/sl/multibot_teleop/src/human_controller.py) class has an optional `regularize=True` argument. When set to true, the controller contains a weak force regularizer to keep the robot in a centered position. This is often helpful to get cleaner movements, but might introduce a bias into your data. The regularization also actively fights against reaching exotic joint configurations.

In such cases, remember to turn off the regularization term.