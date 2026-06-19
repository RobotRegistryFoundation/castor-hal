"""Reference transport adapters shipped with castor-hal.

Generic, device-agnostic transports live here as optional extras (so the core
stays dependency-free):

  * ``castor_hal.adapters.j1939`` — read-only CAN/J1939 telemetry tap
    (``pip install castor-hal[j1939]`` for python-can).

Device-SPECIFIC drivers (e.g. SO-ARM101) live in their own packages.
"""
