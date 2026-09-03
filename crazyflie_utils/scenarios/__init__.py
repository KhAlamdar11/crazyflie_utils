"""Stress scenario registry.

``test.type`` in the YAML config selects one of these by :attr:`name`. Adding
a scenario is: subclass :class:`~crazyflie_utils.scenarios.base.StressScenario`
in a new module, then add it to :data:`SCENARIOS` below.
"""

from typing import Dict, List, Type

from crazyflie_utils.scenarios.base import AbortRequested, StressScenario
from crazyflie_utils.scenarios.goto_flood import GoToFloodScenario
from crazyflie_utils.scenarios.service_ping import ServicePingScenario
from crazyflie_utils.scenarios.setpoint_stream import SetpointStreamScenario
from crazyflie_utils.scenarios.sync_vs_async import SyncVsAsyncScenario
from crazyflie_utils.scenarios.takeoff_land_cycle import TakeoffLandCycleScenario

SCENARIOS: Dict[str, Type[StressScenario]] = {
    scenario.name: scenario
    for scenario in (
        ServicePingScenario,
        TakeoffLandCycleScenario,
        SyncVsAsyncScenario,
        GoToFloodScenario,
        SetpointStreamScenario,
    )
}


def get_scenario(name: str) -> Type[StressScenario]:
    """Look up a scenario class by ``test.type``."""
    try:
        return SCENARIOS[name]
    except KeyError:
        raise KeyError(
            f"Unknown test.type '{name}'. Available: {', '.join(sorted(SCENARIOS))}"
        ) from None


def scenario_listing() -> List[str]:
    """``name -- description`` lines for ``--list``."""
    return [f"{name:<20} {cls.description}"
            for name, cls in sorted(SCENARIOS.items())]


__all__ = [
    'SCENARIOS', 'get_scenario', 'scenario_listing',
    'StressScenario', 'AbortRequested',
]
