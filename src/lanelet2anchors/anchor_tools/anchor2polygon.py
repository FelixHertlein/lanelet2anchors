from shapely.geometry import Polygon

from lanelet2anchors.anchor_generation.anchor import Anchor

from .anchor2linestring import anchor2linestring


def anchor2polygon(anchor: Anchor) -> Polygon:
    border_left = anchor2linestring(anchor, "left")
    border_right = anchor2linestring(anchor, "right")

    poly = Polygon(list(border_left.coords) + list(reversed(border_right.coords)))
    return poly
