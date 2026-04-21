"""Minimal ROS 2 runtime for an exported EasyTrainer checkpoint.

This package contains:

- ``simple_agent.SimpleAgent`` — single-arm robot wrapper that subscribes to
  the joint-state topic and publishes joint commands. Topic-based ``write_type``
  only. No IK, no service / action goals, no tool linkage.
- ``simple_env.SimpleEnv`` — wraps N camera subscribers and N agents into a
  single ``get_observation()`` call.
- ``image_parser`` — verbatim copy of EasyTrainer's image_parser utility for
  ROS Image / CompressedImage decoding and ``fetch_image_with_config``.

The bundled ``ros_inference.py`` script wires these together with the
single-step ``CheckpointInference`` class from the parent ``inference.py`` to
run a closed-loop inference loop without needing the full EasyTrainer stack.
"""
