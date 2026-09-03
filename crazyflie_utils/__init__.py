"""Helper and test tooling for Crazyflie fleets on ROS 2 (crazyswarm2).

The package is self-contained: it talks to a running ``crazyflie_server``
through ``crazyflie_interfaces`` services/topics and nothing else.

Modules:
    config          layered YAML configuration (everything is configurable)
    cf_fleet        instrumented service/topic client for the fleet
    link_monitor    passive downlink (telemetry) health tracking
    metrics         latency / loss / telemetry bookkeeping
    report          console, JSON and CSV rendering
    runner          scenario lifecycle, pre-flight checks and abort handling
    scenarios       the stress tests themselves
"""

__version__ = '0.1.0'
