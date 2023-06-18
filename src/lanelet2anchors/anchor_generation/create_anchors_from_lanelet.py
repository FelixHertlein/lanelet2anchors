from typing import List

from lanelet2.core import LaneletMap
from lanelet2.routing import RoutingGraph

from .anchor import Anchor
from .discover_anchors import discover_anchors
from .sort_anchors import sort_anchors


def create_anchors_for_lanelet(
    lanelet_map: LaneletMap,
    routing_graph: RoutingGraph,
    lanelet_id: int,
    max_length: float = 100,
    distance_method: str = "iou",
) -> List[Anchor]:
    start_lanelet = lanelet_map.laneletLayer[lanelet_id]

    anchors = discover_anchors(routing_graph, start_lanelet, max_length)
    anchors = sort_anchors(anchors, distance_method)

    return [Anchor(lanelets=lanelets) for lanelets in anchors]
