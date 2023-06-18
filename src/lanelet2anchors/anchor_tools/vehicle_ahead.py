from dataclasses import dataclass
from typing import Dict, List, Optional

from lanelet2.core import Lanelet
from shapely.geometry import Point

from ..anchor_generation import Anchor
from ..anchor_tools.anchor2linestring import _anchor2linestring


@dataclass
class Vehicle:
    """Vehicle state representation on map.

    Args:
        identifier (int): Vehicle unique identifer.
        position (shapely.geometry.Point): 2D position of vehicle.
    """

    identifier: int
    position: Point


@dataclass
class VehicleAhead:
    """Preceeding vehicle information.

    Args:
        vehicle (Vehicle): Preceeding vehicle identifer and position.
        distance (float): Distance from ego vehicle along the anchor in meters.
    """

    vehicle: Vehicle
    distance: float


def find_vehicle_ahead(
    anchors: List[Anchor],
    lanelet2vehicle: Dict[Lanelet, List[Vehicle]],
    current_position: Point,
    min_ahead_distance: float = 0.01,
) -> Dict[Anchor, Optional[VehicleAhead]]:
    """Provided a list of anchors, this function checks if any vehicle is ahead per anchor along the anachor path. If a vehicle ahead is found, the identifier, position and distance is returned.

    Args:
        anchors (List[Anchor]): List of anchors to check.
        lanelet2vehicle (Dict[Lanelet, List[Vehicle]]): Positinal information for all vehicles in the scene. The mapping assigns a list of vehicles to each lanelet.
        current_position (Point): Point of the ego vehicle.
        min_ahead_distance (float, optional): Minimal distance in meters to next vehicle ahead to prevent self-detection. Defaults to 0.01.

    Returns:
        Dict[Anchor, Optional[VehicleAhead]]: Returns the vehicle ahead along each anchor per anchor, if any exists.
    """
    return {
        anchor: _find_vehicle_ahead(
            anchor, lanelet2vehicle, current_position, min_ahead_distance
        )
        for anchor in anchors
    }


def _find_vehicle_ahead(
    anchor: Anchor,
    lanlet2vehicle: Dict[Lanelet, List[Vehicle]],
    current_position: Point,
    min_ahead_distance: float,
) -> Optional[VehicleAhead]:
    center_line = _anchor2linestring(anchor.lanelets, "centerline", current_position)

    projections = [
        VehicleAhead(vehicle=vehicle, distance=center_line.project(vehicle.position))
        for lanelet in anchor.lanelets
        for vehicle in lanlet2vehicle[lanelet]
    ]

    projections = list(filter(lambda x: x.distance > min_ahead_distance, projections))

    if len(projections) == 0:
        return None

    return list(sorted(projections, key=lambda x: x.distance))[0]
