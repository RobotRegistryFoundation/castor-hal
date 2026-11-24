"""Reference transport adapters shipped with castor-hal.

Generic, device-agnostic transports live here as optional extras (so the core
stays dependency-free):

  * ``castor_hal.adapters.j1939`` — read-only CAN/J1939 telemetry tap
    (``pip install castor-hal[j1939]`` for python-can).
  * ``castor_hal.adapters.ros2`` — bounded ROS 2 (``/cmd_vel``, ``/joint_states``)
    transport with DDS heartbeat health and latching software estop.

Device-SPECIFIC drivers (e.g. SO-ARM101) live in their own packages.
"""
