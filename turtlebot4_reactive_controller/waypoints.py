"""Hard-coded waypoints in the `map` frame for D* Lite planner experiments.

Captured via RViz "Publish Point" on the area_map on 2026-04-19.
Orientation is not used; yaw defaults to 0.0.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0


WAYPOINTS: tuple[Waypoint, ...] = (
    Waypoint(name="A", x=-1.4907296895980835, y=1.1794612407684326),
    Waypoint(name="B", x=-3.8711657524108887, y=0.8193239569664001),
    Waypoint(name="C", x=-0.19871702790260315, y=-0.7073472142219543),
    Waypoint(name="D", x=0.53733891248703, y=0.7723487019538879),
)


def by_name(name: str) -> Waypoint:
    for wp in WAYPOINTS:
        if wp.name == name:
            return wp
    raise KeyError(f"no waypoint named {name!r}")
